from setuptools import find_packages, setup

package_name = 'yolov8_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minsuje',
    maintainer_email='minsuje@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'image_pub = 9_1_image_publisher:main',
        'image_sub = 9_2_image_subscriber:main',
        'data_pub = 9_3_data_publisher:main',
        'data_sub = 9_4_data_subscriber:main',
        'yolo_pub = 9_5_yolo_publisher:main',
        'yolo_sub = 9_6_yolo_subscriber:main',
        'capture_image = rokey_pjt.1_tb4_capture_image:main',
        'cont_cap_image = rokey_pjt.2_tb4_cont_capture_image:main',
        'det_obj = rokey_pjt.3_tb4_yolov8_obj_det:main',
        'det_obj_thread = rokey_pjt.4_tb4_yolov8_obj_det_thread:main',
        'det_obj_track = rokey_pjt.5_tb4_yolov8_obj_det_track:main',
        ],
    },
)
