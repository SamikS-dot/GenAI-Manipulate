from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robotic_arms_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'action'), glob('action/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samiksingh',
    maintainer_email='imsamik@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transforms_pub = robotic_arms_control.transform_publisher:main',
            'jtc_controller_test = robotic_arms_control.joint_controller_test:main',
            'kinematic_solver = robotic_arms_control.kinematic_solver:main',
            'jtc_multi_goals = robotic_arms_control.jtc_multi_goals:main',
            'shape_traj = robotic_arms_control.shape_trajectory_prompt:main',
            'spawn_obj = robotic_arms_control.spawn_object:main',
            'trajectory = robotic_arms_control.trajectory:main',
            'rtb = robotic_arms_control.rtb_ikine_fkine:main',
            'attacher_action = robotic_arms_control.attacher_action:main',
            'bpp = robotic_arms_control.box_pick_and_place:main',
            'camera = robotic_arms_control.camera:main',
            'camera_depth = robotic_arms_control.camera_depth:main',
            'teleop = robotic_arms_control.teleop:main',
            'blocksolver = robotic_arms_control.block_solver:main',
            'xyzsolver = robotic_arms_control.xyzsolver:main',
            'xyzsolver2 = robotic_arms_control.xyzsolver2:main',
            'v2a = robotic_arms_control.visiontoopenai2:main',
            'v1a = robotic_arms_control.visiontoopenai:main',
            'v3a = robotic_arms_control.visiontoapi3:main'

            




        ],
    },
)
