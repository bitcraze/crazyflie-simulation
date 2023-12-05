from setuptools import find_packages, setup

package_name = 'webots_ros2_crazyflie'

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
    maintainer='kimberly (Bitcraze AB)',
    maintainer_email='kimberly@bitcraze.io',
    description='ROS2 package for Crazyflie webots simulator,
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
