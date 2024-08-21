# run_scripts.py

import subprocess
import re

# Run the first script and capture the output
result = subprocess.run(['python3', 'vision_to_openai.py', 'Green'], capture_output=True, text=True)
output = result.stdout

# Define the regex pattern to extract the coordinates at the end of the output
pattern = re.compile(r'([-?\d.]+),([-?\d.]+),([-?\d.]+)')

# Search for the pattern in the output string
print(output)
match = pattern.search(output)

if match:
    # Extract the coordinates
    x, y, z = match.groups()
    print(f"Extracted coordinates: x = {x}, y = {y}, z = {z}")

    # Run the ROS 2 script with the extracted coordinates
    subprocess.run(['ros2', 'run', 'your_package', 'trajectory_test', str(x), str(y), str(z), 'Green'])
else:
    print("Failed to extract coordinates.")
