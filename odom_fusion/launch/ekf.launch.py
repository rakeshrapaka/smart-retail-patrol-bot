from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/home/smart_retail_bot/src/smart-retail-patrol-bot/odom_fusion/config/ekf.yaml']
        )
    ])
