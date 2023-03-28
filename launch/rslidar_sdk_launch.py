from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        node_name='rslidar_sdk_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rslidar_sdk',
                node_plugin='RSLidarDriver',
                node_name='rslidar_driver',
                parameters=[
                    {"lidar_type": "RSHELIOS"},
                    {"input_type": "pcap"},
                    {"msop_port": 5599},
                    {"difop_port": 6688},
                    {"pcap_path": "/home/alex/robosense/full_campus.pcap"}
                    # ... add other parameters as needed
                ]
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])