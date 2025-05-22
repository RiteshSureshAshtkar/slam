from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ignition_robot')

    world_path = os.path.join(pkg_share, 'worlds', 'empty_world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'ignbot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r -v4 {world_path}',
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Spawn robot entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'diffbot',
                   '-z', '0.1'],
        output='screen'
    )

    # Delayed spawn to ensure Gazebo is ready
    spawn_entity_delayed = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    # Bridge for topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    # Odometry to TF publisher node
    odom_tf_publisher = Node(
        package='ignition_robot',  # Replace with your actual package name
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # SLAM Toolbox node (delayed to ensure robot is spawned)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'debug_logging': True,
            'throttle_scans': 1,
            'transform_publish_period': 0.02,
            'map_update_interval': 5.0,
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'stack_size_to_use': 40000000
        }]
    )

    # Delayed SLAM start to ensure everything is ready
    slam_toolbox_delayed = TimerAction(
        period=8.0,  # Wait 8 seconds after launch
        actions=[slam_toolbox]
    )

    # Teleop keyboard node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='gnome-terminal --'  # Opens in new terminal
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        bridge,
        odom_tf_publisher,
        slam_toolbox_delayed,  # Use delayed SLAM Toolbox
        teleop_node,
        spawn_entity_delayed,
    ])