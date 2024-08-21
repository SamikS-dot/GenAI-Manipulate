import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from linkattacher_msgs.srv import AttachLink, DetachLink
import time
import argparse
import roboticstoolbox as rtb
from spatialmath import SE3
import sys
import termios
import tty

robot = rtb.models.Panda()

class TrajectoryTest(Node):

    def __init__(self, x, y, z, object_name):
        super().__init__('trajectory_test')
        topic_name = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publisher = self.create_publisher(JointTrajectory, topic_name, 10)
        self.joints = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                       'panda_joint4', 'panda_joint5', 'panda_joint6',
                       'panda_joint7', 'panda_finger_joint1']
        
        self.x = x
        self.y = y
        self.z = z
        self.object_name = object_name
        self.goal_positions_list = self.get_goal_positions_list(self.x, self.y, self.z)
        self.current_goal_index = 0
        self.trajectory_active = False
        self.timer = self.create_timer(3, self.timer_callback)
        self.get_logger().info(f'Controller is running and publishing to topic: {topic_name}')

        # Create service client for AttachLink
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ATTACHLINK not available, waiting again...')
        
        # Create service client for DetachLink
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /DETACHLINK not available, waiting again...')

    def get_goal_positions_list(self, x, y, z):
        desired_pose = SE3.Trans(x, y, z) * SE3.OA([0, 1, 0], [0, 0, -1])
        sol = robot.ikine_LM(desired_pose)
        q_pickup = sol.q

        q_pickup1 = list(q_pickup) + [0.8]  
        q_pickup2 = list(q_pickup) + [-0.7]
        
        return [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8],
            q_pickup1,
            q_pickup2,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, -0.7],
            [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, -0.7],
            [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, -0.7],
            [0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0, -0.7],
            [0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0, 0.8]
        ]
    
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
        self.get_logger().info(f'Published trajectory: {goal_positions}')
        self.create_timer(3, self.trajectory_complete_callback)

    def trajectory_complete_callback(self):
        self.get_logger().info(f'Completed trajectory {self.current_goal_index}')
        time.sleep(1)
        self.trajectory_active = False

        if self.current_goal_index == 2:  # After q_pickup1
            self.teleoperation_mode()

        if self.current_goal_index == 2:
            time.sleep(3)
            self.attach_object()

        if self.current_goal_index == 7:
            self.teleoperation_mode()

        if self.current_goal_index == 7:
            time.sleep(3)
            self.detach_object()

    def teleoperation_mode(self):

        print("\nEntering joint control mode.")
        print("Control the robot joints with the following keys:")
        print("  1: Increase Joint 1")
        print("  2: Decrease Joint 1")
        print("  3: Increase Joint 2")
        print("  4: Decrease Joint 2")
        print("  5: Increase Joint 3")
        print("  6: Decrease Joint 3")
        print("  7: Increase Joint 4")
        print("  8: Decrease Joint 4")
        print("  9: Increase Joint 5")
        print("  0: Decrease Joint 5")
        print("  -: Increase Joint 6")
        print("  =: Decrease Joint 6")
        print("  [: Increase Joint 7")
        print("  ]: Decrease Joint 7")
        print("  q: Quit joint control mode and continue")

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                key = sys.stdin.read(1)
                if key == '1':
                    self.goal_positions_list[1][0] += 0.001
                elif key == '2':
                    self.goal_positions_list[1][0] -= 0.001
                elif key == '3':
                    self.goal_positions_list[1][1] += 0.001
                elif key == '4':
                    self.goal_positions_list[1][1] -= 0.001
                elif key == '5':
                    self.goal_positions_list[1][2] += 0.001
                elif key == '6':
                    self.goal_positions_list[1][2] -= 0.001
                elif key == '7':
                    self.goal_positions_list[1][3] += 0.001
                elif key == '8':
                    self.goal_positions_list[1][3] -= 0.001
                elif key == '9':
                    self.goal_positions_list[1][4] += 0.001
                elif key == '0':
                    self.goal_positions_list[1][4] -= 0.001
                elif key == '-':
                    self.goal_positions_list[1][5] += 0.001
                elif key == '=':
                    self.goal_positions_list[1][5] -= 0.001
                elif key == '[':
                    self.goal_positions_list[1][6] += 0.001
                elif key == ']':
                    self.goal_positions_list[1][6] -= 0.001
                elif key == 'q':
                    print("\nExiting joint control mode.")
                    break

                self.get_logger().info(f"New joint positions: {self.goal_positions_list[1]}")
                self.publish_trajectory(self.goal_positions_list[1])

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


    def attach_object(self):
        self.get_logger().info(f'Attaching object: {self.object_name}')
        request_left = AttachLink.Request()
        request_left.model1_name = 'bazu'
        request_left.link1_name = 'panda_leftfinger'
        request_left.model2_name = self.object_name
        request_left.link2_name = 'link'
        self.get_logger().info('Sending attach request for left finger...')
        future_left = self.attach_client.call_async(request_left)
        future_left.add_done_callback(self.attach_response_callback)

        request_right = AttachLink.Request()
        request_right.model1_name = 'bazu'
        request_right.link1_name = 'panda_rightfinger'
        request_right.model2_name = self.object_name
        request_right.link2_name = 'link'
        self.get_logger().info('Sending attach request for right finger...')
        future_right = self.attach_client.call_async(request_right)
        future_right.add_done_callback(self.attach_response_callback)

    def detach_object(self):
        self.get_logger().info(f'Detaching object: {self.object_name}')
        request_left = DetachLink.Request()
        request_left.model1_name = 'bazu'
        request_left.link1_name = 'panda_leftfinger'
        request_left.model2_name = self.object_name
        request_left.link2_name = 'link'
        self.get_logger().info('Sending detach request for left finger...')
        future_left = self.detach_client.call_async(request_left)
        future_left.add_done_callback(self.detach_response_callback)

        request_right = DetachLink.Request()
        request_right.model1_name = 'bazu'
        request_right.link1_name = 'panda_rightfinger'
        request_right.model2_name = self.object_name
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
            self.get_logger().info(f'Service call failed: {e}')

    def detach_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Detachment successful')
            else:
                self.get_logger().info('Detachment failed')
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)

    # Argument parsing
    parser = argparse.ArgumentParser(description='Send XYZ coordinates and object name to the robot.')
    parser.add_argument('x', type=float, help='X coordinate')
    parser.add_argument('y', type=float, help='Y coordinate')
    parser.add_argument('z', type=float, help='Z coordinate')
    parser.add_argument('object_name', type=str, help='Name of the object to pick up')
    args = parser.parse_args()

    # Create the node
    node = TrajectoryTest(args.x, args.y, args.z, args.object_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
