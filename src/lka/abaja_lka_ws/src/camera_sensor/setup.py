from setuptools import setup

package_name = 'camera_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('lib/' + package_name, []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohit',
    maintainer_email='rohit@todo.todo',
    description='ROS 2 package to publish images from CarMaker RSDS TCP stream using sensor_msgs/Image',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rsds_camera_publisher = camera_sensor.CameraFramePublisher:main',
        ],
    },
)
