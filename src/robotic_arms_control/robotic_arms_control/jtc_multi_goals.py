#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from linkattacher_msgs.srv import AttachLink, DetachLink
import time
import argparse
import roboticstoolbox as rtb
from spatialmath import SE3

robot = rtb.models.Panda()

class TrajectoryTest(Node):

    def __init__(self, x, y, z):
        super().__init__('trajectory_test')
        topic_name = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name, 10)
        self.joints = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                       'panda_joint4', 'panda_joint5', 'panda_joint6',
                       'panda_joint7', 'panda_finger_joint1']
        
        self.x = x
        self.y = y
        self.z = z
        self.goal_positions_list = self.compute_trajectory(self.x, self.y, self.z)
        self.current_goal_index = 0
        self.trajectory_active = False
        self.timer = self.create_timer(3, self.timer_callback)
        self.get_logger().info('Controller is running and publishing to topic: {}'.format(topic_name))

        # Create service client for AttachLink
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ATTACHLINK not available, waiting again...')
        
        # Create service client for DetachLink
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /DETACHLINK not available, waiting again...')

    def timer_callback(self):
        if not self.trajectory_active and self.current_goal_index < len(self.goal_positions_list):
            self.publish_trajectory(self.goal_positions_list[self.current_goal_index])
            self.current_goal_index += 1
            self.trajectory_active = True

    def publish_trajectory(self, goal_positions):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = goal_positions
        point.time_from_start = Duration(sec=2)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info('Published trajectory: {}'.format(goal_positions))
        self.create_timer(3, self.trajectory_complete_callback)  # Wait for the trajectory to complete

    def trajectory_complete_callback(self):
        self.get_logger().info('Completed trajectory {}'.format(self.current_goal_index))
        time.sleep(1)
        self.trajectory_active = False

        # Attach after the first trajectory (moving to the XYZ coordinate)
        if self.current_goal_index == 1:
            time.sleep(3)
            self.attach_object()

        # Detach after the last trajectory
        if self.current_goal_index == 2:
            time.sleep(3)
            self.detach_object()

    def attach_object(self):
        request_left = AttachLink.Request()
        request_left.model1_name = 'bazu'
        request_left.link1_name = 'panda_leftfinger'
        request_left.model2_name = 'box'
        request_left.link2_name = 'link'
        self.get_logger().info('Sending attach request for left finger...')
        future_left = self.attach_client.call_async(request_left)
        future_left.add_done_callback(self.attach_response_callback)

        request_right = AttachLink.Request()
        request_right.model1_name = 'bazu'
        request_right.link1_name = 'panda_rightfinger'
        request_right.model2_name = 'box'
        request_right.link2_name = 'link'
        self.get_logger().info('Sending attach request for right finger...')
        future_right = self.attach_client.call_async(request_right)
        future_right.add_done_callback(self.attach_response_callback)

    def detach_object(self):
        request_left = DetachLink.Request()
        request_left.model1_name = 'bazu'
        request_left.link1_name = 'panda_leftfinger'
        request_left.model2_name = 'box'
        request_left.link2_name = 'link'
        self.get_logger().info('Sending detach request for left finger...')
        future_left = self.detach_client.call_async(request_left)
        future_left.add_done_callback(self.detach_response_callback)

        request_right = DetachLink.Request()
        request_right.model1_name = 'bazu'
        request_right.link1_name = 'panda_rightfinger'
        request_right.model2_name = 'box'
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

    def compute_trajectory(self, x, y, z):
        # Define the desired end-effector pose
        desired_pose = SE3.Trans(x, y, z) * SE3.OA([0, 1, 0], [0, 0, -1])

        # Solve inverse kinematics
        sol = robot.ikine_LM(desired_pose)

        # Get the joint configuration
        q_pickup = sol.q

        q_pickup1 = list(q_pickup) + [0.8]  # Move to pickup location
        q_transport = [0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0, 0.8]  # Transport position

        return [q_pickup1, q_transport]

def main(args=None):
    rclpy.init(args=args)

    # Argument parsing
    parser = argparse.ArgumentParser(description='Publish custom trajectory for the robotic arm.')
    parser.add_argument('x', type=float, help='X coordinate for trajectory computation')
    parser.add_argument('y', type=float, help='Y coordinate for trajectory computation')
    parser.add_argument('z', type=float, help='Z coordinate for trajectory computation')
    args = parser.parse_args()

    # Create the node
    trajectory_publisher_node = TrajectoryTest(args.x, args.y, args.z)

    # Spin the node
    rclpy.spin(trajectory_publisher_node)
    trajectory_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
