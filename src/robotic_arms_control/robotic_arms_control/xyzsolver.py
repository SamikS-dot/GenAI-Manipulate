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

    def __init__(self, x, y, z, object_name, action):
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
        self.action = action
        self.goal_positions_list = self.get_goal_positions_list(self.x, self.y, self.z)
        self.current_goal_index = 0
        self.trajectory_active = False
        self.timer = self.create_timer(3, self.timer_callback)
        self.get_logger().info(f'Controller is running and publishing to topic: {topic_name}')

        # Create service clients for AttachLink and DetachLink
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ATTACHLINK not available, waiting again...')
        
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        while not self.detach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /DETACHLINK not available, waiting again...')

    def get_goal_positions_list(self, x, y, z):
        desired_pose = SE3.Trans(x, y, z) * SE3.OA([0, 1, 0], [0, 0, -1])
        sol = robot.ikine_LM(desired_pose)
        q_pickup = sol.q

        q_pickup1 = list(q_pickup) + [0.8]  
        q_pickup2 = list(q_pickup) + [-0.7]

        if self.action == "table":
            return [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8],
                q_pickup1,
                q_pickup2,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, 0.7],
                [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, 0.7],
                [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, 0.7],
                [0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0, 0.7],
                [0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0, 0.8]
            ]
        elif self.action == "rearrange":
            return [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8],
                q_pickup1,
                q_pickup2,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, -0.7],
                [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, -0.7],
                [0.0, 0.0, 1.551, 0.0, 0.0, 1.592, 0.0, -0.7],
                [-1.081, 0.60, -.454, -1.936, 0.110, 2.652, 0.987, -0.7],
                [-1.081, 0.60, -.454, -1.936, 0.110, 2.652, 0.987, 0.8]
            ]
        else:
            self.get_logger().info('Unknown action, using default sequence.')
            return []

    # The rest of your methods remain unchanged

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
            time.sleep(3)
            self.detach_object()

    def teleoperation_mode(self):
        print("\nEntering teleoperation mode.")
        print("Control the robot with the following keys:")
        print("  w: Increase X")
        print("  s: Decrease X")
        print("  a: Increase Y")
        print("  d: Decrease Y")
        print("  z: Increase Z")
        print("  x: Decrease Z")
        print("  q: Quit teleoperation mode and continue")

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                key = sys.stdin.read(1)
                if key == 'w':
                    self.x += 0.01
                elif key == 's':
                    self.x -= 0.01
                elif key == 'a':
                    self.y += 0.01
                elif key == 'd':
                    self.y -= 0.01
                elif key == 'z':
                    self.z += 0.01
                elif key == 'x':
                    self.z -= 0.01
                elif key == 'q':
                    print("\nExiting teleoperation mode.")
                    break

                self.get_logger().info(f"New target coordinates: X={self.x}, Y={self.y}, Z={self.z}")
                self.publish_trajectory(self.get_goal_positions_list(self.x, self.y, self.z)[1])

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
    parser = argparse.ArgumentParser(description='Send XYZ coordinates, object name, and action to the robot.')
    parser.add_argument('x', type=float, help='X coordinate')
    parser.add_argument('y', type=float, help='Y coordinate')
    parser.add_argument('z', type=float, help='Z coordinate')
    parser.add_argument('object_name', type=str, help='Name of the object to pick up')
    parser.add_argument('action', type=str, help='Action to perform: table or rearrange')
    args = parser.parse_args()

    # Create the node
    node = TrajectoryTest(args.x, args.y, args.z, args.object_name, args.action)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

