import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')
    pkg_digital_twin = FindPackageShare('digital_twin_description').find('digital_twin_description')

    # Gazebo launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='true')
    headless = LaunchConfiguration('headless', default='false')
    gui = LaunchConfiguration('gui', default='true')

    world = LaunchConfiguration('world', default=PathJoinSubstitution([
        FindPackageShare('digital_twin_description'),
        'worlds',
        'simple_world.world'
    ]))

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world,
        description='Choose one of the world files from `/digital_twin_description/worlds`'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control for robot control if true'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Enable headless mode if true'
    )

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable GUI if true'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'gui': gui,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
        }],
    )

    # Joint State Publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_ros2_control_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_gui_cmd)

    # Add nodes and launch files
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)

    return ld