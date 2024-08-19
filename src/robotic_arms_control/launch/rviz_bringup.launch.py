from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the package and the URDF files
    pkg_description = get_package_share_directory('robotic_arms_control')
    panda_urdf_file = os.path.join(pkg_description, "urdf", 'panda_arm.urdf')
    box_urdf_file = os.path.join(pkg_description, "urdf", 'blue_box.urdf')

    # Read the URDF files
    with open(panda_urdf_file, 'r') as file:
        panda_urdf_content = file.read()

    with open(box_urdf_file, 'r') as file:
        box_urdf_content = file.read()

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="JSP",
        output="screen",
    )

    panda_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="RSP_Panda",
        output="screen",
        parameters=[{'robot_description': panda_urdf_content}]
    )

    box_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="RSP_Box",
        output="screen",
        parameters=[{'robot_description': box_urdf_content}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_gui",
        output="screen"
    )

    # Return the LaunchDescription
    return LaunchDescription([
        joint_state_publisher_node,
        panda_robot_state_publisher_node,
        box_robot_state_publisher_node,
        rviz_node
    ])

if __name__ == '__main__':
    generate_launch_description()
