from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument('robot_name', default_value='crazyflie',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value=LaunchConfiguration('robot_name'),
                          description='Robot namespace'),
]


def generate_launch_description():
    pkg_crazyflie_description = get_package_share_directory('crazyflie_description')

    # Paths
    xacro_file = PathJoinSubstitution([pkg_crazyflie_description, 'urdf', 'crazyflie_body.xacro'])
    rviz_config_file = PathJoinSubstitution([pkg_crazyflie_description, 'rviz', 'crazyflie_body.rviz'])

    namespace = LaunchConfiguration('namespace')

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command(['xacro ', xacro_file])},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Joint State Publisher GUI (for revolute joints)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # LaunchDescription
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz2)
    return ld
