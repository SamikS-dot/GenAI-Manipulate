#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from linkattacher_msgs.srv import AttachLink, DetachLink
from pynput import keyboard

class TeleoperationNode(Node):

    def __init__(self):
        super().__init__('teleoperation')
        topic_name = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name, 10)
        self.joints = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                       'panda_joint4', 'panda_joint5', 'panda_joint6',
                       'panda_joint7', 'panda_finger_joint1']
        self.goal_positions = [0.0] * 8

        # Create service client for AttachLink
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ATTACHLINK not available, waiting again...')
        
        # Create service client for DetachLink
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /DETACHLINK not available, waiting again...')
        
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        self.get_logger().info('Controller is running and publishing to topic: {}'.format(topic_name))

    def publish_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        trajectory_msg.points.append(point)
        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info('Published trajectory: {}'.format(self.goal_positions))

    def on_press(self, key):
        try:
            if key.char == 'q':
                self.goal_positions[0] += 0.01
            elif key.char == 'a':
                self.goal_positions[0] -= 0.01
            elif key.char == 'w':
                self.goal_positions[1] += 0.01
            elif key.char == 's':
                self.goal_positions[1] -= 0.01
            elif key.char == 'e':
                self.goal_positions[2] += 0.01
            elif key.char == 'd':
                self.goal_positions[2] -= 0.01
            elif key.char == 'r':
                self.goal_positions[3] += 0.01
            elif key.char == 'f':
                self.goal_positions[3] -= 0.01
            elif key.char == 't':
                self.goal_positions[4] += 0.01
            elif key.char == 'g':
                self.goal_positions[4] -= 0.01
            elif key.char == 'y':
                self.goal_positions[5] += 0.01
            elif key.char == 'h':
                self.goal_positions[5] -= 0.01
            elif key.char == 'u':
                self.goal_positions[6] += 0.01
            elif key.char == 'j':
                self.goal_positions[6] -= 0.01
            elif key.char == 'i':
                self.goal_positions[7] += 0.01
            elif key.char == 'k':
                self.goal_positions[7] -= 0.01
            elif key.char == 'b':
                self.attach_object('blue')
            elif key.char == 'g':
                self.attach_object('green')
            elif key.char == 'r':
                self.attach_object('red')
            elif key.char == 'x':
                self.detach_object()

            self.publish_trajectory()

        except AttributeError:
            pass

    def attach_object(self, color):
        request_left = AttachLink.Request()
        request_left.model1_name = 'bazu'
        request_left.link1_name = 'panda_leftfinger'
        request_left.model2_name = f'box_{color}'
        request_left.link2_name = 'link'
        self.get_logger().info(f'Sending attach request for left finger to attach {color} box...')
        future_left = self.attach_client.call_async(request_left)
        future_left.add_done_callback(self.attach_response_callback)

        request_right = AttachLink.Request()
        request_right.model1_name = 'bazu'
        request_right.link1_name = 'panda_rightfinger'
        request_right.model2_name = f'box_{color}'
        request_right.link2_name = 'link'
        self.get_logger().info(f'Sending attach request for right finger to attach {color} box...')
        future_right = self.attach_client.call_async(request_right)
        future_right.add_done_callback(self.attach_response_callback)

    def detach_object(self):
        request_left = DetachLink.Request()
        request_left.model1_name = 'bazu'
        request_left.link1_name = 'panda_leftfinger'
        request_left.model2_name = 'box_blue'  # Replace with the correct color if necessary
        request_left.link2_name = 'link'
        self.get_logger().info('Sending detach request for left finger...')
        future_left = self.detach_client.call_async(request_left)
        future_left.add_done_callback(self.detach_response_callback)

        request_right = DetachLink.Request()
        request_right.model1_name = 'bazu'
        request_right.link1_name = 'panda_rightfinger'
        request_right.model2_name = 'box_blue'  # Replace with the correct color if necessary
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
    node = TeleoperationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
