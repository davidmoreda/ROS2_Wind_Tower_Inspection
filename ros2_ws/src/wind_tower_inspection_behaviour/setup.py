from setuptools import find_packages, setup

package_name = 'wind_tower_inspection_behaviour'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'langchain-google-genai',
        'faster-whisper',
        'sounddevice',
        'numpy',
    ],
    zip_safe=True,
    maintainer='dmore',
    maintainer_email='dmoreda29@gmail.com',
    description='Inspection behaviour nodes for wind tower internal inspection.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_controller = wind_tower_inspection_behaviour.mission_controller:main',
            'voice_command_node = wind_tower_inspection_behaviour.voice_command_node:main',
            'random_walk_people = wind_tower_inspection_behaviour.random_walk_people:main',
            'people_collision_sync = wind_tower_inspection_behaviour.people_collision_sync:main',
        ],
    },
)
