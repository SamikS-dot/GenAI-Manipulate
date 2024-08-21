#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from linkattacher_msgs.srv import AttachLink, DetachLink
import roboticstoolbox as rtb
from spatialmath import SE3
import sys
import time

class KinematicSolver(Node):

    def __init__(self, x=None, y=None, z=None):
        super().__init__('kinematic_solver')
        topic_name = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name, 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.joints = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                       'panda_joint4', 'panda_joint5', 'panda_joint6',
                       'panda_joint7', 'panda_finger_joint1']
        
        self.finger_offset = SE3(0, 0, 0.1)  # Adjust the offset as necessary
        self.goal_positions = self.calculate_goal_joint_pos(x, y, z)
        self.trajectory_active = False
        self.initial_pose_published = False
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ATTACHLINK not available, waiting again...')
        
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /DETACHLINK not available, waiting again...')
        self.get_logger().info('Controller is running and publishing to topic: {}'.format(topic_name))

    def calculate_goal_joint_pos(self, x, y, z):
        target_pose = SE3(x, y, z) * self.finger_offset  # Apply the offset to the target pose
        robotic_arm = rtb.models.URDF.Panda()
        ik_solution = robotic_arm.ikine_LM(target_pose)
        q = ik_solution.q.tolist()
        if len(q) == 7:
            q.append(0.0)  # Ensure 8 positions by appending 0.0 for panda_finger_joint1 if necessary
        return q

    def timer_callback(self):
        if not self.initial_pose_published:
            self.publish_initial_pose()
            self.initial_pose_published = True
        elif not self.trajectory_active:
            self.publish_trajectory(self.goal_positions)
            self.trajectory_active = True
            self.create_timer(5, self.attach_and_lift)  # Wait for the trajectory to complete before attaching

    def publish_initial_pose(self):
        initial_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = initial_pose
        point.time_from_start = Duration(sec=2)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info('Published initial pose: {}'.format(initial_pose))
    def publish_trajectory(self, goal_positions):
        initial_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Ensure 8 joint positions
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        qt = rtb.jtraj(initial_pose, goal_positions, 50)
        for i, q in enumerate(qt.q):
            point = JointTrajectoryPoint()
            point.positions = q.tolist()  # Ensure 8 positions without appending extra element
            duration_sec = 2 * (i + 1) // 50  # Integer part of the duration
            duration_nsec = int((2 * (i + 1) / 50 - duration_sec) * 1e9)  # Nanoseconds part of the duration
            point.time_from_start = Duration(sec=duration_sec, nanosec=duration_nsec)
            trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info('Published trajectory: {}'.format(goal_positions))

    def attach_and_lift(self):
        self.attach_object()
        self.publish_lift_trajectory()
        self.create_timer(5, self.detach_and_finish)  # Wait for the lift trajectory to complete before detaching

    def attach_object(self):
        request_left = AttachLink.Request()
        request_left.model1_name = 'panda'
        request_left.link1_name = 'panda_leftfinger'
        request_left.model2_name = 'block'
        request_left.link2_name = 'link'
        self.get_logger().info('Sending attach request for left finger...')
        future_left = self.attach_client.call_async(request_left)
        future_left.add_done_callback(self.attach_response_callback)

        request_right = AttachLink.Request()
        request_right.model1_name = 'panda'
        request_right.link1_name = 'panda_rightfinger'
        request_right.model2_name = 'block'
        request_right.link2_name = 'link'
        self.get_logger().info('Sending attach request for right finger...')
        future_right = self.attach_client.call_async(request_right)
        future_right.add_done_callback(self.attach_response_callback)

    def publish_lift_trajectory(self):
        lift_goal_positions = self.goal_positions[:]
        lift_goal_positions[2] += 0.1  # Add a height increment to lift the block
        self.publish_trajectory(lift_goal_positions)

    def detach_and_finish(self):
        self.detach_object()
        self.trajectory_active = False
        self.get_logger().info('Finished task.')

    def detach_object(self):
        request_left = DetachLink.Request()
        request_left.model1_name = 'panda'
        request_left.link1_name = 'panda_leftfinger'
        request_left.model2_name = 'block'
        request_left.link2_name = 'link'
        self.get_logger().info('Sending detach request for left finger...')
        future_left = self.detach_client.call_async(request_left)
        future_left.add_done_callback(self.detach_response_callback)

        request_right = DetachLink.Request()
        request_right.model1_name = 'panda'
        request_right.link1_name = 'panda_rightfinger'
        request_right.model2_name = 'block'
        request_right.link2_name = 'link'
        self.get_logger().info('Sending detach request for right finger...')
        future_right = self.detach_client.call_async(request_right)
        future_right.add_done_callback(self.detach_response_callback)

    def attach_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Attachment successful')
            else:
                self.get_logger().info('Attachment failed')
        except Exception as e:
            self.get_logger().info('Service call failed: %r' % (e,))

    def detach_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Detachment successful')
            else:
                self.get_logger().info('Detachment failed')
        except Exception as e:
            self.get_logger().info('Service call failed: %r' % (e,))


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    else:
        print("Usage: kinematic_solver.py <x> <y> <z>")
        sys.exit(1)
    trajectory_publisher_node = KinematicSolver(x, y, z)
    rclpy.spin(trajectory_publisher_node)
    trajectory_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
