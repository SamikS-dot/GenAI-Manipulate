import os
import re
from google.cloud import vision
import subprocess
import math
import openai

# Set up Google Cloud credentials
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/rosam/Downloads/causal-relic-430206-p6-ec3ade6f63da.json'

# Initialize Google Cloud Vision client
vision_client = vision.ImageAnnotatorClient()

openai.api_key = 'Replace with your OpenAI Key'

# Camera intrinsics (example values, replace with your actual values)
fx = 554.38
fy = 554.38
cx = 320
cy = 240

# Updated camera pose (example values, replace with your actual values)
camera_position = (1.4, -0.4, 0.2)  # (x, y, z) position of the camera in the world frame
camera_orientation = (0, 0, 0)  # (roll, pitch, yaw) orientation of the camera in radians

def detect_text(path):
    """Detects text in the file."""
    with open(path, "rb") as image_file:
        content = image_file.read()

    image = vision.Image(content=content)
    response = vision_client.text_detection(image=image)
    texts = response.text_annotations

    detected_text = " ".join([text.description for text in texts])
    print(detected_text)
    return detected_text

def extract_objects_info(detected_text):
    """
    Extract the names, probabilities, coordinates, and depths of objects from the detected text.
    
    Parameters:
        detected_text (str): The input text containing the object information.
        
    Returns:
        list: A list of tuples containing the object name, probability, coordinates, and depth.
    """
    # General object detection pattern including depth
    pattern = r'(\w+\s\w+|\w+)\s+(\d+\.\d+)\s+\((\d+),\s*(\d+)\)\s+depth:\s+(\d+\.\d+)m'
    matches = re.findall(pattern, detected_text)
    
    # Block detection pattern
    block_pattern = r'(Red|Green|Blue) Block:\s+\((\d+),\s*(\d+)\)\s+depth:\s+(\d+\.\d+)m'
    block_matches = re.findall(block_pattern, detected_text)
    
    objects_info = []
    
    # Process general object detections
    for match in matches:
        object_name = match[0]
        probability = float(match[1])
        x = int(match[2])
        y = int(match[3])
        depth = float(match[4])
        objects_info.append((object_name, probability, x, y, depth))
    
    # Process block detections with dummy probability value
    for match in block_matches:
        block_name = match[0] + " Block"
        probability = 1.0  # Assuming a probability of 1 for block detections
        x = int(match[1])
        y = int(match[2])
        depth = float(match[3])
        objects_info.append((block_name, probability, x, y, depth))
    
    return objects_info

def calculate_world_coordinates(pixel_x, pixel_y, depth_z, fx, fy, cx, cy, camera_position, camera_orientation):
    """
    Calculate the world coordinates (x, y, z) from pixel coordinates, depth, camera intrinsics, and camera pose.
    
    Parameters:
        pixel_x (int): X coordinate of the pixel.
        pixel_y (int): Y coordinate of the pixel.
        depth_z (float): Depth of the object from the camera in meters.
        fx, fy (float): Focal lengths in pixels along the x and y axes.
        cx, cy (float): Principal point coordinates in pixels.
        camera_position (tuple): The (x, y, z) position of the camera in the world frame.
        camera_orientation (tuple): The (roll, pitch, yaw) orientation of the camera in radians.
        
    Returns:
        tuple: The (x, y, z) coordinates in the world frame.
    """
    # Convert pixel coordinates to camera coordinates
    
    y_camera = (pixel_y - cy) * depth_z / fy
    z_camera = depth_z

    # Adjust for camera orientation (assuming the camera is aligned with the world frame)
    # In a more complex scenario, you would apply rotation transformations based on camera_orientation

    # Convert camera coordinates to world coordinates
    x_world = round(camera_position[0] - z_camera, 1)
    y_world = round(camera_position[1] + y_camera - 0.1, 1)
    z_world = 0.25  # Default z value, adjust if necessary

    
    return x_world, y_world, z_world

def determine_suitable_object_and_action(user_message, objects_info):
    # Prepare the message with the user input and the list of detected objects
    object_list = "\n".join([f"{i}: {obj[0]}" for i, obj in enumerate(objects_info)])
    messages = [
        {"role": "user", "content": f"Given the user's message '{user_message}', determine the most suitable object from the following list for their needs. Return the index of the most suitable object and whether the action should be 'rearrange' or 'table'.\n{object_list}"}
    ]

    # Create a chat completion request using GPT-4 model
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=messages,
        max_tokens=1000
    )

    # Extract the index of the suitable object and the action from the API response
    content = response.choices[0].message['content']
    match = re.search(r'(\d+)\s*(rearrange|table)', content)

    if match:
        suitable_object_index = int(match.group(1))
        action = match.group(2)
    else:
        suitable_object_index = 0  # Default to the first object
        action = 'table'  # Default action

    # Print the chosen action
    print(f"Action chosen: {action}")

    return suitable_object_index, action


def main():
    image_path = '/home/rosam/Downloads/Image Window_screenshot_04.08.2024.png'
    detected_texts = detect_text(image_path)

    if not detected_texts:
        print("No text detected or an error occurred.")
        return

    # Extract object information
    objects_info = extract_objects_info(detected_texts)
    
    if not objects_info:
        print("No objects found in the detected text.")
        return

    # Print the information of each detected object
    for i, obj_info in enumerate(objects_info):
        object_name, probability, pixel_x, pixel_y, depth_z = obj_info
        print(f"{i}: Object: {object_name}, Probability: {probability}, Pixel Coordinates: ({pixel_x}, {pixel_y}), Depth: {depth_z}m")

    user_message = input("Enter your message: ")
    suitable_object_index, action = determine_suitable_object_and_action(user_message, objects_info)
    
    if 0 <= suitable_object_index < len(objects_info):
        selected_object = objects_info[suitable_object_index]
        object_name, probability, pixel_x, pixel_y, depth_z = selected_object
        print(f"Selected {object_name} with probability {probability}, pixel coordinates ({pixel_x}, {pixel_y}), depth {depth_z}m")

        # Calculate world coordinates
        x, y, z = calculate_world_coordinates(pixel_x, pixel_y, depth_z, fx, fy, cx, cy, camera_position, camera_orientation)
        print(f"Calculated world coordinates for {object_name}: x = {x}, y = {y}, z = {z}")

        # Run the ROS 2 script with the extracted coordinates and the action
        result = subprocess.run(['ros2', 'run', 'robotic_arms_control', 'xyzsolver', str(x), str(y), str(z), object_name, action])
        if result.returncode == 0:
            print(f"Trajectory command for {object_name} with action {action} sent successfully.")
        else:
            print(f"Failed to send trajectory command for {object_name} with action {action}.")
    else:
        print("Invalid object selection. Please try again.")

if __name__ == '__main__':
    main()
