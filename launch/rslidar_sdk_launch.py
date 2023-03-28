from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='rslidar_sdk_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rslidar_sdk',
                plugin='robosense::lidar::RSLidarDriver',
                name='rslidar_driver',
                parameters=[
                    {"lidar_type": "RSHELIOS"},
                    {"input_type": "pcap"},
                    {"msop_port": 5599},
                    {"difop_port": 6688},
                    {"pcap_path": "/home/alex/robosense/full_campus.pcap"},
                    {"frame_id": "rslidar"}
                    # ... add other parameters as needed
                ]
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])