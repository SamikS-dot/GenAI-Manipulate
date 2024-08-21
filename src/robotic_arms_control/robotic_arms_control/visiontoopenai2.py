import os
import re
import subprocess
import openai
from google.cloud import vision
from flask import Flask, render_template, request, jsonify

# Set up Google Cloud credentials
os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/home/rosam/Downloads/causal-relic-430206-p6-ec3ade6f63da.json'

# Initialize Google Cloud Vision client
vision_client = vision.ImageAnnotatorClient()

# Initialize OpenAI client
openai.api_key = 'Replace with your OpenAI Key'

# Flask app setup
app = Flask(__name__, template_folder='/home/rosam/new_ros2_ws/src/ROS2-Ultimate-guide-for-Custom-Robotic-Arms-and-Panda-7-DOF/robotic_arms_control/robotic_arms_control/templates')

# Camera intrinsics (example values, replace with your actual values)
fx = 554.38
fy = 554.38
cx = 320
cy = 240

# Updated camera pose (example values, replace with your actual values)
camera_position = (1.4, -0.4, 0.2)  # (x, y, z) position of the camera in the world frame
camera_orientation = (0, 0, 0)  # (x, y, z) quaternion representing the camera's orientation

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/chat', methods=['POST'])
def chat():
    user_message = request.json.get('message')
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": user_message}],
        max_tokens=4000
    )
    reply = response.choices[0].message['content']
    return jsonify({'reply': reply})

def detect_text():
    """Detects text in a hardcoded image file."""
    # Path to the hardcoded image file
    image_path = '/home/rosam/Downloads/Image Window_screenshot_04.08.2024.png'
    
    # Open the file and read its content
    with open(image_path, "rb") as image_file:
        content = image_file.read()
    
    # Create an Image object from the content
    image = vision.Image(content=content)
    
    # Perform text detection
    response = vision_client.text_detection(image=image)
    
    # Extract detected text annotations
    texts = response.text_annotations
    return texts


def extract_objects_info(detected_text):
    pattern = r'(\w+\s\w+|\w+)\s+(\d+\.\d+)\s+\((\d+),\s*(\d+)\)\s+depth:\s+(\d+\.\d+)m'
    matches = re.findall(pattern, detected_text)
    
    block_pattern = r'(Red|Green|Blue) Block:\s+\((\d+),\s*(\d+)\)\s+depth:\s+(\d+\.\d+)m'
    block_matches = re.findall(block_pattern, detected_text)
    
    objects_info = []
    
    for match in matches:
        object_name = match[0]
        probability = float(match[1])
        x = int(match[2])
        y = int(match[3])
        depth = float(match[4])
        objects_info.append((object_name, probability, x, y, depth))
    
    for match in block_matches:
        block_name = match[0] + " Block"
        probability = 1.0
        x = int(match[1])
        y = int(match[2])
        depth = float(match[3])
        objects_info.append((block_name, probability, x, y, depth))
    
    return objects_info

def get_world_coordinates_from_openai(pixel_x, pixel_y, depth_z, fx, fy, cx, cy, camera_position, camera_orientation, object_name):
    messages = [
        {"role": "user", "content": f"Calculate the world coordinates (x,y,z) for the object located at the following pixel coordinates a distance d (d is represented as the depth) away from a camera with the following camera intrinsics and the knowledge that to calculate the x coordinate you can use depth and subtract it from camera x-pos. x-coordinate = camera x pos - depth. The camera is looking at objects from the side so the v pixel values correspond to the y-axis. "
                                    f"Camera position: {camera_position}, Camera orientation (roll,pitch,yaw): {camera_orientation}. "
                                    f"Return your answer as x,y,z values like this:(x,y-0.4,0.1 (unless the object is a bottle which is 0.2)) with the x-value being camera x pos - depth. z is not necessary as you will always return 0.1 unless the object is a bottle in which case you will return 0.2. Don't add units. Depth is the distance from the camera to the object in the x-direction which means the x-coordinate is simply camera x position - depth. Completely solve the problem to make sure you end up with xyz values. Note that the camera is facing the -x direction. "
                                    f"Pixel coordinates: ({pixel_x}, {pixel_y}), Depth: {depth_z}m, "
                                    f"Camera intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}. Object name: {object_name}"}
    ]

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=messages,
        max_tokens=4000
    )
    content = response.choices[0].message['content']
    return content

def extract_last_coordinates(text):
    pattern = r'\((-?\d+\.\d+), (-?\d+\.\d+), (-?\d+\.\d+)\)'
    matches = re.findall(pattern, text)
    
    if matches:
        last_match = matches[-1]
        x = float(last_match[0])
        y = float(last_match[1])
        z = float(last_match[2])
        return x, y, z
    else:
        raise ValueError("Coordinates not found in the text.")

def determine_suitable_object(user_message, objects_info):
    object_list = "\n".join([f"{i}: {obj[0]}" for i, obj in enumerate(objects_info)])
    messages = [
        {"role": "user", "content": f"Given the user's message '{user_message}', determine the most suitable object from the following list for their needs. Return the index of the most suitable object.\n{object_list}"}
    ]

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=messages,
        max_tokens=1000
    )
    content = response.choices[0].message['content']
    return int(re.search(r'\d+', content).group())

@app.route('/process_image', methods=['POST'])

@app.route('/process_image', methods=['GET'])
def process_image():
    # Process the hardcoded image
    detected_texts = detect_text()
    objects_info = extract_objects_info(detected_texts)
    
    if not objects_info:
        return jsonify({'error': 'No objects found in the detected text.'})
    
    # Example user message
    user_message = "example_object"
    
    suitable_object_index = determine_suitable_object(user_message, objects_info)
    
    if 0 <= suitable_object_index < len(objects_info):
        selected_object = objects_info[suitable_object_index]
        object_name, probability, pixel_x, pixel_y, depth_z = selected_object

        # Assuming these are defined elsewhere in your code
        fx, fy, cx, cy = 1.0, 1.0, 0.0, 0.0
        camera_position = (0, 0, 0)
        camera_orientation = (0, 0, 0, 1)
        
        world_coordinates = get_world_coordinates_from_openai(pixel_x, pixel_y, depth_z, fx, fy, cx, cy, camera_position, camera_orientation, object_name)
        x, y, z = extract_last_coordinates(world_coordinates)
        
        # Run the ROS 2 command to handle the robotic arm
        result = subprocess.run(['ros2', 'run', 'robotic_arms_control', 'xyzsolver', str(x), str(y), str(z), object_name])
        if result.returncode == 0:
            return jsonify({'message': f"Trajectory command for {object_name} sent successfully.", 'coordinates': {'x': x, 'y': y, 'z': z}, 'objects': [f"{obj[0]} (Probability: {obj[1]}, Pixel: ({obj[2]}, {obj[3]}), Depth: {obj[4]}m)" for obj in objects_info]})
        else:
            return jsonify({'error': f"Failed to send trajectory command for {object_name}."})
    else:
        return jsonify({'error': "Invalid object selection. Please try again."})

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')

