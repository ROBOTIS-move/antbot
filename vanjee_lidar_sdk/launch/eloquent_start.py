from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    config_path=get_package_share_directory('vanjee_lidar_sdk')+'/config/config_722.yaml'
    rviz_config=get_package_share_directory('vanjee_lidar_sdk')+'/rviz/rviz2.rviz'
    return LaunchDescription([
        Node(
            package='vanjee_lidar_sdk',
            node_namespace='vanjee_lidar_sdk',
            node_name='vanjee_lidar_sdk_node',
            node_executable='vanjee_lidar_sdk_node',
            output='screen',
            parameters=[{'config_path':config_path}]
        ),
        Node(
            package='rviz2',
            node_namespace='rviz2',
            node_name='rviz2',
            node_executable='rviz2',
            arguments=['-d',rviz_config]
        )
    ])
