import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import roboticstoolbox as rtb
from spatialmath.base import *
from spatialmath import SE3
import sys

def main(args=None):
    rclpy.init(args=args)
    robotic_arm = rtb.models.URDF.Panda()
    robot = rtb.models.Panda()
    print(robot)
   
    # Calculating Forward Kinematics
    fk_tran_m = robotic_arm.fkine([0.0, 0.0, 1.551, -0.99, 0.0, 1.592, 0.0])
    print(fk_tran_m)

    # Extracting the x, y, z coordinates
    position = fk_tran_m.t
    x, y, z = position[0], position[1], position[2]
    print(f"End-Effector Position: x={x}, y={y}, z={z}")

    # Calculating Inverse Kinematics
    point = SE3(0.0, 0.7, 0.5)
    ik_tran_m = robotic_arm.ikine_LM(point)

    #print(ik_tran_m)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
