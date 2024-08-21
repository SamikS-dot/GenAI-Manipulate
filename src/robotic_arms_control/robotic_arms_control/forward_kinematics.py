import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import roboticstoolbox as rtb
from spatialmath.base import *
from spatialmath import SE3
import sys

#Find target joint configurations in order to position end-effector near object to be grasped

# Add .672 to x, subtract .57 from y
# box_green--> -0.682 -0.02 0.1274
def main(args=None):
    rclpy.init(args=args)
    robotic_arm = rtb.models.URDF.Panda()

    # Target position for the end-effector
    target_position = [-0.01, -0.59, 0.5]


    # Calculate Inverse Kinematics
    point = SE3(target_position[0], target_position[1], target_position[2])
    ik_solution = robotic_arm.ikine_LM(point)

    if ik_solution.success:
        joint_positions = ik_solution.q
        print(f"Joint positions to reach the target position {target_position}: {joint_positions}")
    else:
        print(f"Could not find a solution for the target position {target_position}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
