<div style="display: flex; justify-content: space-between;">
  <img src="https://github.com/user-attachments/assets/e1db0d14-f7c4-4e46-8c86-119dbb68d517" alt="Image 1" style="width: 45%;"/>
  <img src="https://github.com/user-attachments/assets/f6d16c54-43b4-4265-8099-f7bdd3d6e75d" alt="Image 2" style="width: 45%;"/>
</div>

# GenAI Manipulation

---

**Samik Singh**  
**(https://www.linkedin.com/in/imsamik/)** | **imsamik@gmail.com** | **University of Texas at Austin Robotics**

---

<details>
  <summary>📄 Table of Contents</summary>

  - [About](#about)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Features](#features)
  - [Contributing](#contributing)
  - [License](#license)
  - [Contact](#contact)

</details>

## About

**GenAI Manipulation** is a ROS 2-based project that combines computer vision and generative AI to enable intuitive human-robot interaction. By detecting and classifying objects, the system interprets user requests and determines appropriate robotic actions to assist in various tasks. The goal is to make human-robot interaction effortless by leveraging advanced AI techniques to understand and fulfill user needs.

---
## Installation

All packages in this repository have been developed, executed, and tested on an Ubuntu 22.04 machine with ROS 2 Humble. Follow the steps below to set up your ROS 2 Humble environment and install the necessary robot simulation packages.

### ROS 2 Humble Environment Setup

1. **Install Ubuntu 22.04**: [Download and install Ubuntu 22.04](https://ubuntu.com/desktop).

2. **Install Git**:
    ```bash
    sudo apt update
    sudo apt install git
    ```

3. **Configure Git**:
    ```bash
    git config --global user.name YourUsername
    git config --global user.email YourEmail
    git config --global color.ui true
    git config --global core.editor code --wait # Visual Studio Code is recommended.
    git config --global credential.helper store
    ```

4. **Install ROS 2 Humble**:
    - Follow the installation instructions on the [ROS 2 Humble documentation](https://docs.ros.org/en/humble/Installation.html).

5. **Source ROS 2 Humble Installation**:
    Add the following line to your `.bashrc` file:
    ```bash
    source /opt/ros/humble/setup.bash
    ```

6. **Install MoveIt! 2 for ROS 2 Humble**:
    - Reference: [MoveIt! 2 Humble Documentation](https://moveit.ros.org/documentation/).
    - Install MoveIt! 2 binaries:
      ```bash
      sudo apt install ros-humble-moveit
      ```

7. **Create and Configure the ROS 2 Humble Workspace**:
    - Follow the instructions in the [ROS 2 Humble Workspace Documentation](https://docs.ros.org/en/humble/Creating-A-Workspace.html).
    - Add the following line to your `.bashrc` file to source the workspace:
      ```bash
      source ~/dev_ws/install/local_setup.bash
      ```

8. **Install ROS 2 Packages Required for Robot Simulation and Control**:
    - ROS 2 Control:
      ```bash
      sudo apt install ros-humble-ros2-control
      ```
    - ROS 2 Controllers:
      ```bash
      sudo apt install ros-humble-ros2-controllers
      sudo apt install ros-humble-gripper-controllers
      ```
    - Gazebo-ROS 2:
      ```bash
      sudo apt install gazebo11
      sudo apt install ros-humble-gazebo-ros2-control
      sudo apt install ros-humble-gazebo-ros-pkgs
      ```
    - Xacro:
      ```bash
      sudo apt install ros-humble-xacro
      ```

9. **Import and Install the `ros2_RobotSimulation` Repository**:
    This repository is required to run the ROS 2 packages for robot simulation.
    - Clone and build the repository:
      ```bash
      cd ~/dev_ws/src
      git clone 
      cd ~/dev_ws
      colcon build
      ```

10. **Additional Installations**:

    - **YOLOv8**:
      YOLOv8 is a state-of-the-art object detection model. 
      - **Install YOLOv8 dependencies**:
        ```bash
        pip install torch torchvision torchaudio
        pip install yolov8
        ```

    - **Google Vision Cloud API**:
      Google Vision Cloud API is used for image analysis. It is used to extract labeled text from images
      - **Install Google Cloud Vision client library**:
        ```bash
        pip install google-cloud-vision
        ```
      - **Obtain API Key**:
        1. Create a Google Cloud Project and enable the Vision API.
        2. Set up billing and create credentials.
        3. Download the JSON key file and set the environment variable:
           ```bash
           export GOOGLE_APPLICATION_CREDENTIALS="/path/to/your/keyfile.json"
           ```

    - **OpenAI GPT Model**:
      OpenAI’s GPT model is used for text-based AI tasks. It is used to identify which actions will be most suitable based on the users request.
      - **Install OpenAI Python client**:
        ```bash
        pip install openai
        ```
      - **Obtain API Key**:
        1. Sign up for OpenAI and generate an API key from the [OpenAI API Keys page](https://platform.openai.com/account/api-keys).
        2. Set up your API key in your environment:
           ```bash
           export OPENAI_API_KEY="your-api-key"
           ```

    - **Robotics Toolbox for Forward and Inverse Kinematics**:
      For kinematic computations.
      - **Install Robotics Toolbox**:
        ```bash
        pip install roboticstoolbox
        ```

---

## Usage

To get started with the ROS 2 simulation and control of your Panda robot, follow these steps:

1. **Source Your Workspace**:
   Every time you open a new terminal, make sure to source your workspace setup file:
   ```bash
   source ~/dev_ws/install/local_setup.bash

---
## Features

### Vision Capabilities

This project integrates advanced computer vision tools to enhance object detection and spatial understanding. The vision capabilities are essential for accurate object identification and position mapping in real-world scenarios. Below are the key vision components:

1. **YOLOv8 Object Detection**  
   YOLOv8 is a state-of-the-art object detection model that enables high-accuracy detection and classification of objects in images. We use this identify and annotate images with a comprehensive
   image of all of the objects detected in a workspace

   **Command to run YOLOv8 object detection:**  
   ```bash
   # Replace with your actual command for YOLOv8 object detection
   ros2 run robotic_arms_control camera

2. **Sensors**                                                                                                                                                                              
   We use a rgb camera and a depth camera to identify the distance the camera is from certain objects. Using computer vision based technqiues we can then identify where the objects are in x,y,z
   coordinates
   ```bash
    # Replace with your actual command for YOLOv8 object detection
   ros2 run robotic_arms_control camera_depth

3. **Google Vision Cloud API**                                                                                                                                                                      Google Vision Cloud API allows us to parse text from our annotated images that are taken as a livestream from our camera. Using the information provided we can then feed information into OpenAI in order to deduce which objects should be manipulated and what actions should be taken with them. Each object is given a probability, its (u,v) camera-based coordinates and its distance from our camera.                                                                                                                                                                       
   
   ```bash
    # Replace with your actual command for YOLOv8 object detection
   python3 cloudvision_tester.py

   
   
### Contributing

*Guidelines for contributing to the project.*

### License

*Information about the project's license.*

### Contact

**[Your Name]**  
**[Your Email Address]**  
**[Your LinkedIn Profile]**

