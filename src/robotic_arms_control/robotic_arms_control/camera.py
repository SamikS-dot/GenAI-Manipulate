import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.image_subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_listener_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera_sensor/depth/image_raw',
            self.depth_listener_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO("yolo-Weights/yolov8n.pt")
        self.classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                           "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                           "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
                           "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
                           "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
                           "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
                           "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                           "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                           "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
                           "teddy bear", "hair drier", "toothbrush"]
        self.annotations = []  # Initialize annotations list
        self.block_coordinates = {"Red": None, "Green": None, "Blue": None}  # Initialize block coordinates
        self.depth_image = None  # Initialize depth image

    def image_listener_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Flip the image by 180 degrees
            cv_image = cv2.flip(cv_image, -1)
            
            masked_image, annotated_image = self.detect_and_mask_colors(cv_image)
            annotated_image = self.detect_and_annotate_objects(masked_image, annotated_image)
            
            # Display annotations and block coordinates
            annotated_image = self.display_annotations(annotated_image)
            
            # Show the result
            cv2.imshow("Image Window", annotated_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error('Error processing image: %s' % str(e))

    def depth_listener_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
            # Flip the depth image by 180 degrees to match the RGB image
            self.depth_image = cv2.flip(self.depth_image, -1)
            self.get_logger().info("Depth image received and processed")
        except CvBridgeError as e:
            self.get_logger().error('Error processing depth image: %s' % str(e))

    def detect_and_mask_colors(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for green, blue, and red
        green_lower = np.array([35, 100, 100])
        green_upper = np.array([85, 255, 255])
        blue_lower = np.array([100, 100, 100])
        blue_upper = np.array([140, 255, 255])
        red_lower1 = np.array([0, 100, 100])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([160, 100, 100])
        red_upper2 = np.array([180, 255, 255])
        
        # Create masks for each color
        green_mask = cv2.inRange(hsv_image, green_lower, green_upper)
        blue_mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
        red_mask1 = cv2.inRange(hsv_image, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv_image, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        # Combine masks to create a masked image
        combined_mask = cv2.bitwise_or(green_mask, blue_mask)
        combined_mask = cv2.bitwise_or(combined_mask, red_mask)
        masked_image = cv2.bitwise_and(image, image, mask=cv2.bitwise_not(combined_mask))
        
        # Annotate detected color blocks
        annotated_image = image.copy()
        annotated_image = self.find_and_draw_contours(annotated_image, green_mask, "Green")
        annotated_image = self.find_and_draw_contours(annotated_image, blue_mask, "Blue")
        annotated_image = self.find_and_draw_contours(annotated_image, red_mask, "Red")
        
        return masked_image, annotated_image

    def find_and_draw_contours(self, image, mask, color_name):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Calculate the centroid of the contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                # Annotate the image with the color and coordinates
                cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
                # Store block coordinates
                self.block_coordinates[color_name] = (cX, cY)
                
        return image

    def detect_and_annotate_objects(self, masked_image, annotated_image):
        self.annotations = []  # Reset annotations list for each frame

        results = self.model(masked_image)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0].item()
                cls = int(box.cls[0])
                class_name = self.classNames[cls]
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                if confidence > 0.3:  # Set a confidence threshold
                    # Draw bounding box
                    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (255, 0, 255), 2)
                    # Collect annotation information
                    annotation = f"{class_name} {confidence:.2f} ({center_x}, {center_y})"
                    # Add depth information if available
                    if self.depth_image is not None:
                        depth = self.depth_image[center_y, center_x]
                        annotation += f" depth: {depth:.2f}m"
                    else:
                        annotation += " depth: N/A"
                    self.annotations.append(annotation)

        return annotated_image

    def display_annotations(self, image):
        # Display all annotations in the upper right corner
        y_offset = 20
        max_width = 400
        color_names = ["Red", "Green", "Blue"]

        for text in self.annotations:
            cv2.putText(image, text, (image.shape[1] - max_width, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            y_offset += 20

        # Display block coordinates
        y_offset += 20  # Add space before starting block coordinates
        for color in color_names:
            coords = self.block_coordinates.get(color, (None, None))
            if coords != (None, None):
                text = f"{color} Block: ({coords[0]}, {coords[1]})"
                # Add depth information if available
                if self.depth_image is not None:
                    depth = self.depth_image[coords[1], coords[0]]
                    text += f" depth: {depth:.2f}m"
                else:
                    text += " depth: N/A"
                cv2.putText(image, text, (image.shape[1] - max_width, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                y_offset += 20  # Move down for next color

        return image

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
