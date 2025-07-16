from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    pursuit_pkg_share = get_package_share_directory('ros2_pure_pursuit_controller')
    bcr_bot_pkg_share = get_package_share_directory('bcr_bot')
    
    # Parameter files
    pure_pursuit_params = os.path.join(pursuit_pkg_share, 'config', 'pure_pursuit_params.yaml')
    map_file = os.path.join(bcr_bot_pkg_share, 'config', 'bcr_map.yaml')
    rviz_config = os.path.join(pursuit_pkg_share, 'rviz', 'pure_pursuit_navigation.rviz')
    
    return LaunchDescription([
        # Static transform from map to odom (identity transform for simplicity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),
        
        # Map server to load the bcr_map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}],
        ),
        
        # Map server lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'autostart': True},
                       {'node_names': ['map_server']}]
        ),
        
        # A* planner server
        Node(
            package='a_star_planner',
            executable='a_star_server',
            name='a_star_server',
            output='screen',
        ),
        
        # A* path client (requests path and publishes to /plan)
        Node(
            package='ros2_pure_pursuit_controller',
            executable='a_star_path_client_node',
            name='a_star_path_client',
            output='screen',
        ),
        
        # Pure Pursuit controller
        Node(
            package='ros2_pure_pursuit_controller',
            executable='pure_pursuit_controller_node',
            name='pure_pursuit_controller',
            output='screen',
            parameters=[pure_pursuit_params],
        ),
        
        # RViz for visualization and goal setting
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
