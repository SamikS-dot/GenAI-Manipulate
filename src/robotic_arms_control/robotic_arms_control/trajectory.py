#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import roboticstoolbox as rtb
from spatialmath import SE3
import argparse

# Initialize the robot model
robot = rtb.models.Panda()

class TrajectoryTest(Node):

    def __init__(self):
        super().__init__('trajectory_test')
        topic_name = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name, 10)
        self.joints = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                       'panda_joint4', 'panda_joint5', 'panda_joint6',
                       'panda_joint7']
        self.current_goal_index = 0
        self.trajectory_active = False
        self.timer = self.create_timer(3, self.timer_callback)
        self.get_logger().info('Controller is running and publishing to topic: {}'.format(topic_name))

    def timer_callback(self):
        if not self.trajectory_active and self.current_goal_index < len(self.goal_positions_list):
            self.publish_trajectory(self.goal_positions_list[self.current_goal_index])
            self.current_goal_index += 1
            self.trajectory_active = True

    def publish_trajectory(self, goal_positions):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = goal_positions.tolist()
        point.time_from_start = Duration(sec=2)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info('Published trajectory: {}'.format(goal_positions))
        self.create_timer(3, self.trajectory_complete_callback)  # Wait for the trajectory to complete

    def trajectory_complete_callback(self):
        self.get_logger().info('Completed trajectory point {}'.format(self.current_goal_index))
        self.trajectory_active = False

    def compute_trajectory(self, x, y, z):
        # Define the desired end-effector pose
        desired_pose = SE3.Trans(x, y, z) * SE3.OA([0, 1, 0], [0, 0, -1])

        # Solve inverse kinematics
        sol = robot.ikine_LM(desired_pose)

        # Get the joint configuration
        q_pickup = sol.q

        # Generate a trajectory from the current joint configuration to the pickup configuration
        self.goal_positions_list = q_pickup

        # Print joint configuration and end-effector pose for verification
        print("Joint Configuration for Desired Pose:\n", q_pickup)

        return self.goal_positions_list

def main(args=None):
    rclpy.init(args=args)

    # Argument parser for command-line arguments
    parser = argparse.ArgumentParser(description='Move the robot arm to the specified coordinates.')
    parser.add_argument('x', type=float, help='X coordinate')
    parser.add_argument('y', type=float, help='Y coordinate')
    parser.add_argument('z', type=float, help='Z coordinate')
    args = parser.parse_args()

    # Initialize the TrajectoryTest node
    node = TrajectoryTest()

    # Compute the trajectory based on user input
    goal_positions_list = node.compute_trajectory(args.x, args.y, args.z)

    # Set the computed trajectory as the goal positions list
    node.goal_positions_list = goal_positions_list

    # Spin the node
    rclpy.spin(node)

    # Destroy the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
