from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='rslidar_sdk_container',
        namespace='rs_helios',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rslidar_sdk',
                plugin='robosense::lidar::RSLidarDriver',
                name='rslidar_driver',
                parameters=[
                    {"lidar_type": "RSHELIOS"},
                    {"input_type": "online"},
                    {"msop_port": 2011},
                    {"difop_port": 3011},
                    {"frame_id": "rs_helios_top"}
                    # ... add other parameters as needed
                ]
            ),
            ComposableNode(
                package='rslidar_sdk',
                plugin='robosense::lidar::RSLidarDriver',
                name='rslidar_driver',
                parameters=[
                    {"lidar_type": "RSBP"},
                    {"input_type": "online"},
                    {"msop_port": 2012},
                    {"difop_port": 3012},
                    {"frame_id": "rs_bpearl_left"}
                    # ... add other parameters as needed
                ]
            ),
            ComposableNode(
                package='rslidar_sdk',
                plugin='robosense::lidar::RSLidarDriver',
                name='rslidar_driver',
                parameters=[
                    {"lidar_type": "RSBP"},
                    {"input_type": "online"},
                    {"msop_port": 2010},
                    {"difop_port": 3010},
                    {"frame_id": "rs_bpearl_right"}
                    # ... add other parameters as needed
                ]
            )

        ],
        output='screen',
    )

    return LaunchDescription([container])