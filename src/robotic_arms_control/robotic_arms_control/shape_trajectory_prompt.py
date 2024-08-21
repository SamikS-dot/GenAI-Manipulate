#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import openai
import time

# Set your OpenAI API key
openai.api_key = 'Replace with your OpenAI Key'

class TrajectoryTest(Node):

    def __init__(self):
        super().__init__('trajectory_test')
        topic_name = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name, 10)
        self.joints = ['panda_joint1','panda_joint2','panda_joint3',
                       'panda_joint4','panda_joint5','panda_joint6',
                       'panda_joint7','panda_finger_joint1']
        self.goal_positions_list = [[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]]
        self.current_goal_index = 0
        self.trajectory_active = False
        self.timer = self.create_timer(3, self.timer_callback)
        self.get_logger().info('Controller is running and publishing to topic: {}'.format(topic_name))
        

    def timer_callback(self):
        if not self.trajectory_active and self.current_goal_index < len(self.goal_positions_list):
            self.publish_trajectory(self.goal_positions_list[self.current_goal_index])
            self.current_goal_index += 1
            self.trajectory_active = True
            
    
    def get_raised_hand_image(self):
        messages = [
           {"role": "user", "content": "What’s the person doig in this image? Are they raising their hands are performing other actions? Explain what the person is doing using this form as an example, yes they are raising their hands."},
    {"role": "system", "content": "The following message contains an image URL to analyze."},
    {"role": "user", "content": "https://img.freepik.com/premium-photo/greeting-happy-smiling-young-dark-skinned-man-raising-hand-greeting-standing-isolated-light-gray-background_259150-29120.jpg"}
        ]
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=messages,
            max_tokens=300
        )
        response_content = response.choices[0].message['content']
        print(response_content)

        # Check if the response indicates that the person is raising their hands
        if "raising their hand" in response_content or "raised hands" in response_content or "raising their hands" in response_content or "Yes" in response_content:
            print("The person is raising their hands.")
            return True
        else:
            print("The person is not raising their hands.")
            return False

    
    def action_trajectory(self):
        print("Executing hand raise to mimic subject...")
        self.goal_positions_list.append([-0.266, -0.067, -0.517, -0.524, 0.611, 3.752, 1.707, 0.0])

        
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
        time.sleep(2)
        self.trajectory_active = False

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher_node = TrajectoryTest()
    
    # Check if the image shows a person raising their hands
    if trajectory_publisher_node.get_raised_hand_image():
        trajectory_publisher_node.action_trajectory()
    
    rclpy.spin(trajectory_publisher_node)
    trajectory_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
