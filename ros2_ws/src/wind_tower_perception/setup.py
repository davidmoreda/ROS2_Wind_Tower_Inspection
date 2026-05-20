from setuptools import find_packages, setup

package_name = 'wind_tower_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/perception.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/perception_params.yaml',
            'config/synthetic_dataset.yaml',
            'config/dataset.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmore',
    maintainer_email='dmoreda29@gmail.com',
    description=(
        'Perception layer for wind tower internal inspection: circular-defect '
        'detector (YOLO + HoughCircles fallback), metadata-tagged image capture, '
        'projection of detections to cylindrical (x, theta) and aggregation for '
        'a downstream LLM inspection report.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector = wind_tower_perception.detector_node:main',
            'image_capture = wind_tower_perception.image_capture_node:main',
            'defect_mapper = wind_tower_perception.defect_mapper_node:main',
            'synthetic_capture = wind_tower_perception.synthetic_capture_node:main',
            'generate_synthetic_world = '
            'wind_tower_perception.scripts.generate_synthetic_world:main',
            'train_yolo = wind_tower_perception.scripts.train_yolo:main',
            'generate_inspection_report = '
            'wind_tower_perception.scripts.generate_inspection_report:main',
        ],
    },
)
