import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import argparse

class DepthImageSubscriber(Node):

    def __init__(self, x, y):
        super().__init__('depth_image_subscriber')
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera_sensor/depth/image_raw',
            self.depth_listener_callback,
            10)
        self.bridge = CvBridge()
        self.depth_image = None
        self.x = x
        self.y = y

    def depth_listener_callback(self, data):
        try:
            # Convert ROS image message to OpenCV image
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")  # Adjust encoding if necessary
            if self.depth_image is None:
                self.get_logger().info("Received a None depth image.")
                return

            # Flip the depth image by 180 degrees
            self.depth_image = cv2.flip(self.depth_image, -1)

            # Normalize depth image for visualization
            depth_normalized = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_colored = cv2.applyColorMap(np.uint8(depth_normalized), cv2.COLORMAP_JET)

            # Display the depth image
            cv2.imshow("Depth Image", depth_colored)
            cv2.waitKey(1)
            self.get_logger().info("Depth image displayed successfully.")

            # Print the depth value at the specified coordinates
            self.print_depth_at_pixel(self.x, self.y)

        except CvBridgeError as e:
            self.get_logger().error('Error processing depth image: %s' % str(e))
        except Exception as e:
            self.get_logger().error('Unexpected error: %s' % str(e))

    def print_depth_at_pixel(self, x, y):
        if self.depth_image is not None:
            if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
                depth_value = self.depth_image[y, x]
                self.get_logger().info(f"Depth at ({x}, {y}): {depth_value:.2f} meters")
            else:
                self.get_logger().error(f"Coordinates ({x}, {y}) are out of bounds")
        else:
            self.get_logger().error("Depth image not received yet")

def main(args=None):
    parser = argparse.ArgumentParser(description='Get depth value at specified pixel coordinates.')
    parser.add_argument('x', type=int, help='The x coordinate')
    parser.add_argument('y', type=int, help='The y coordinate')
    args = parser.parse_args()

    # Convert args to list of strings for rclpy.init()
    rclpy_args = [str(args.x), str(args.y)]

    rclpy.init(args=rclpy_args)
    depth_image_subscriber = DepthImageSubscriber(args.x, args.y)
    
    rclpy.spin(depth_image_subscriber)
    
    depth_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
