from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Package and xacro path
    pkg_share = get_package_share_directory("crazyflie_description")
    xacro_file = os.path.join(pkg_share, "urdf", "crazyflie_body.xacro")
    # Process xacro → urdf
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Launch argument for robot name
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="crazyflie",
        description="Name of the robot"
    )

    # Start Gazebo Harmonic (empty world)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ]),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # Load URDF to robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_config}]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", LaunchConfiguration("robot_name"),
            "-string", robot_description_config,
        ],
        output="screen"
    )

    return LaunchDescription([
        robot_name_arg,
        gazebo_launch,
        robot_state_publisher,
        spawn_robot
    ])

