#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from linkattacher_msgs.srv import AttachLink, DetachLink  # Import the attach and detach service messages
import time
import sys

class TrajectoryTest(Node):

    def __init__(self, box_colors):
        super().__init__('trajectory_test')
        topic_name = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name, 10)
        self.joints = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                       'panda_joint4', 'panda_joint5', 'panda_joint6',
                       'panda_joint7', 'panda_finger_joint1']
        
        self.box_colors = box_colors
        self.current_color_index = 0
        self.box_color = box_colors[self.current_color_index]
        self.goal_positions_list = self.get_goal_positions_list(self.box_color)
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

    def get_goal_positions_list(self, box_color):
        if box_color == 'green':
            return [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8],
                [-0.798, 0.010, 0.016, -2.990, -0.235, 2.886, 0.262, 0.8],
                [-0.798, 0.010, 0.016, -2.990, -0.235, 2.886, 0.262, 0.8],
                [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, -0.7],
                [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, -0.7],
                [0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0, -0.7],
                [0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0, 0.8]
            ]
        elif box_color == 'blue':
            return [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8],
                [-1.488, 0.75, -0.078, -1.701, 0.188, 2.510, 0.767, 0.8],
                [-1.488, 0.75, -0.078, -1.701, 0.188, 2.510, 0.767, -0.7],
                [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, -0.7],
                [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, -0.7],
                [0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0, -0.7],
                [0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0, 0.8]
            ]
        elif box_color == 'red':
            return [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8],
                [0.0, 0.753, -0.016, -1.644, 0.016, 2.407, 0.767, 0.8],
                [0.0, 0.753, -0.016, -1.644, 0.016, 2.407, 0.767, -0.7],
                [0.0, 0.753, -0.016, -1.644, 0.016, 2.407, 0.953, -0.7],
                [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, -0.7],
                [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, -0.7],
                [0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0, -0.7],
                [0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0, 0.8]
            ]
        else:
            self.get_logger().info('Invalid box color specified')
            return []

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

        # Attach after the second trajectory
        if self.current_goal_index == 2:
            time.sleep(3)
            self.attach_object()

        # Detach after the seventh trajectory
        if self.current_goal_index == 7:
            time.sleep(3)
            self.detach_object()

        # If all goals for the current color are completed, move to the next color
        if self.current_goal_index >= len(self.goal_positions_list):
            self.current_color_index += 1
            if self.current_color_index < len(self.box_colors):
                self.box_color = self.box_colors[self.current_color_index]
                self.goal_positions_list = self.get_goal_positions_list(self.box_color)
                self.current_goal_index = 0

    def attach_object(self):
        request_left = AttachLink.Request()
        request_left.model1_name = 'bazu'
        request_left.link1_name = 'panda_leftfinger'
        request_left.model2_name = f'box_{self.box_color}'
        request_left.link2_name = 'link'
        self.get_logger().info('Sending attach request for left finger...')
        future_left = self.attach_client.call_async(request_left)
        future_left.add_done_callback(self.attach_response_callback)

        request_right = AttachLink.Request()
        request_right.model1_name = 'bazu'
        request_right.link1_name = 'panda_rightfinger'
        request_right.model2_name = f'box_{self.box_color}'
        request_right.link2_name = 'link'
        self.get_logger().info('Sending attach request for right finger...')
        future_right = self.attach_client.call_async(request_right)
        future_right.add_done_callback(self.attach_response_callback)

    def detach_object(self):
        request_left = DetachLink.Request()
        request_left.model1_name = 'bazu'
        request_left.link1_name = 'panda_leftfinger'
        request_left.model2_name = f'box_{self.box_color}'
        request_left.link2_name = 'link'
        self.get_logger().info('Sending detach request for left finger...')
        future_left = self.detach_client.call_async(request_left)
        future_left.add_done_callback(self.detach_response_callback)

        request_right = DetachLink.Request()
        request_right.model1_name = 'bazu'
        request_right.link1_name = 'panda_rightfinger'
        request_right.model2_name = f'box_{self.box_color}'
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

    if len(sys.argv) < 2:
        print("Usage: trajectory_test.py <box_color1> <box_color2> ...")
        sys.exit(1)

    box_colors = sys.argv[1:]

    node = TrajectoryTest(box_colors)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
