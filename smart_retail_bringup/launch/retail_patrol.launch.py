import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to dependencies
    tortoisebot_desc_dir = get_package_share_directory('tortoisebot_description')
    tortoisebot_nav_dir = get_package_share_directory('tortoisebot_navigation')
    tortoisebot_slam_dir = get_package_share_directory('tortoisebot_slam')
    ydlidar_dir = get_package_share_directory('ydlidar')
    smart_retail_dir = get_package_share_directory('smart_retail_bringup')

    # Config files
    default_model_path = os.path.join(tortoisebot_desc_dir, 'models/urdf/tortoisebot_simple.xacro')
    default_rviz_config = os.path.join(tortoisebot_desc_dir, 'rviz/tortoisebot_sensor_display.rviz')
    default_map = os.path.join(smart_retail_dir, 'maps', 'room2.yaml')
    nav2_params = os.path.join(tortoisebot_slam_dir, 'config', 'nav2_params.yaml')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration = LaunchConfiguration('exploration')
    map_file = LaunchConfiguration('map')

    # Nodes
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', LaunchConfiguration('model')])
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ydlidar_dir, 'launch', 'x2_ydlidar_launch.py')),
        condition=PythonExpression(['not ', use_sim_time])
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tortoisebot_nav_dir, 'launch', 'navigation.launch.py')),
        launch_arguments={'params_file': nav2_params}.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tortoisebot_slam_dir, 'launch', 'cartographer.launch.py')),
        launch_arguments={
            'params_file': nav2_params,
            'slam': exploration,
            'use_sim_time': use_sim_time
        }.items()
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_publisher',
        parameters=[{
            'image_size': [160, 120],
            'camera_name': 'camera',
            'frame_id': 'camera_link',
            'framerate': 10.0,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    patrol_manager = Node(
        package='patrol_manager',
        executable='patrol_node.py',
        name='patrol_manager',
        output='screen'
    )

    perception_node = Node(
        package='smart_retail_perception',
        executable='perception_node.py',
        name='retail_perception',
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use sim time'),
        DeclareLaunchArgument('exploration', default_value='True', description='Enable SLAM exploration'),
        DeclareLaunchArgument('model', default_value=default_model_path, description='URDF path'),
        DeclareLaunchArgument('map', default_value=default_map, description='Map file'),

        rviz_node,
        robot_state_publisher,
        joint_state_publisher,
        lidar_launch,
        navigation_launch,
        slam_launch,
        camera_node,
        patrol_manager,
        perception_node
    ])
