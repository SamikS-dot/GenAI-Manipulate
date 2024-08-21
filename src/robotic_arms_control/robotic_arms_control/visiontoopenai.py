import os
import re
import openai
from google.cloud import vision
import subprocess

# Set up Google Cloud credentials
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/rosam/Downloads/causal-relic-430206-p6-ec3ade6f63da.json'

# Initialize Google Cloud Vision client
vision_client = vision.ImageAnnotatorClient()

# Initialize OpenAI client
openai.api_key = 'Replace with your OpenAI Key'

# Camera intrinsics (example values, replace with your actual values)
fx = 554.38
fy = 554.38
cx = 320
cy = 240

# Updated camera pose (example values, replace with your actual values)
camera_position = (1.4, -0.4, 0.2)  # (x, y, z) position of the camera in the world frame
camera_orientation = (0, 0, 0)  # (x, y, z) quaternion representing the camera's orientation

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

def get_world_coordinates_from_openai(pixel_x, pixel_y, depth_z, fx, fy, cx, cy, camera_position, camera_orientation, object_name):
    # Prepare the message with the pixel coordinates, camera intrinsics, and camera pose
    messages = [
        {"role": "user", "content": f"Calculate the world coordinates (x,y,z) for the object located at the following pixel coordinates a distance d (d is represented as the depth) away from a camera with the following camera intrinsics and the knowledge that to calculate the x coordinate you can use depth and subtract it from camera x-pos. x-coordinate = camera x pos - depth. The camera is looking at objects from the side so the v pixel values correspond to the y-axis. "
                                    f"Camera position: {camera_position}, Camera orientation (roll,pitch,yaw): {camera_orientation}. "
                                    f"Return your answer as x,y,z values like this:(x,y-0.4,0.1 (unless the object is a bottle which is 0.2)) with the x-value being camera x pos - depth. z is not necessary as you will always return 0.1 unless the object is a bottle in which case you will return 0.2. Don't add units. Depth is the distance from the camera to the object in the x-direction which means the x-coordinate is simply camera x position - depth. Completely solve the problem to make sure you end up with xyz values. Note that the camera is facing the -x direction. "
                                    f"Pixel coordinates: ({pixel_x}, {pixel_y}), Depth: {depth_z}m, "
                                    f"Camera intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}. Object name: {object_name}"}
    ]

    # Create a chat completion request using GPT-4 model
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=messages,
        max_tokens=4000
    )

    # Extract the calculated world coordinates from the API response
    content = response.choices[0].message['content']
    return content

def extract_last_coordinates(text):
    """
    Extract the last set of coordinates from a text string.
    
    Parameters:
        text (str): The input text containing the coordinates.
        
    Returns:
        tuple: A tuple containing the extracted x, y, and z coordinates.
    """
    # Define a regular expression pattern to match coordinates
    pattern = r'\((-?\d+\.\d+), (-?\d+\.\d+), (-?\d+\.\d+)\)'
    
    # Find all matches of the pattern in the text
    matches = re.findall(pattern, text)
    
    if matches:
        # Take the last match
        last_match = matches[-1]
        x = float(last_match[0])
        y = float(last_match[1])
        z = float(last_match[2])
        return x, y, z
    else:
        raise ValueError("Coordinates not found in the text.")

def determine_suitable_object(user_message, objects_info):
    # Prepare the message with the user input and the list of detected objects
    object_list = "\n".join([f"{i}: {obj[0]}" for i, obj in enumerate(objects_info)])
    messages = [
        {"role": "user", "content": f"Given the user's message '{user_message}', determine the most suitable object from the following list for their needs. Return the index of the most suitable object.\n{object_list}"}
    ]

    # Create a chat completion request using GPT-4 model
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=messages,
        max_tokens=1000
    )

    # Extract the index of the suitable object from the API response
    content = response.choices[0].message['content']
    return int(re.search(r'\d+', content).group())

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
    suitable_object_index = determine_suitable_object(user_message, objects_info)
    
    if 0 <= suitable_object_index < len(objects_info):
        selected_object = objects_info[suitable_object_index]
        object_name, probability, pixel_x, pixel_y, depth_z = selected_object
        print(f"Selected {object_name} with probability {probability}, pixel coordinates ({pixel_x}, {pixel_y}), depth {depth_z}m")

        # Get world coordinates from OpenAI
        world_coordinates = get_world_coordinates_from_openai(pixel_x, pixel_y, depth_z, fx, fy, cx, cy, camera_position, camera_orientation, object_name)
        print(f"World coordinates for {object_name}: {world_coordinates}")
        x, y, z = extract_last_coordinates(world_coordinates)
        print(f"Extracted world coordinates: x = {x}, y = {y}, z = {z}")

        # Run the ROS 2 script with the extracted coordinates
        result = subprocess.run(['ros2', 'run', 'robotic_arms_control', 'xyzsolver', str(x), str(y), str(z), object_name])
        if result.returncode == 0:
            print(f"Trajectory command for {object_name} sent successfully.")
        else:
            print(f"Failed to send trajectory command for {object_name}.")
    else:
        print("Invalid object selection. Please try again.")

if __name__ == '__main__':
    main()
