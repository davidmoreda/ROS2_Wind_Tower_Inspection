from setuptools import find_packages, setup

package_name = 'wind_tower_arm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/move_group.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/kinematics.yaml',
            'config/ompl_planning.yaml',
            'config/joint_limits.yaml',
            'config/moveit_controllers.yaml',
            'config/pilz_cartesian_limits.yaml',
            'config/moveit.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmore',
    maintainer_email='dmoreda29@gmail.com',
    description=(
        'MoveIt-based control of the UR5e inspection arm for the wind tower '
        'internal inspection robot.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # arm_inspection_node is added in F3.
        ],
    },
)
