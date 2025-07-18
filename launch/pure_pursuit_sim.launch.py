from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
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
        # Launch dynamic Gazebo world for testing
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bcr_bot_pkg_share, "launch", "ign_dynamic.launch.py")),
        ),
        
        # Dynamic object controller for moving obstacles
        Node(
            package='bcr_bot',
            executable='dynamic_object_controller.py',
            name='dynamic_object_controller',
            output='screen',
        ),
        
        # Odometry noise node to add realistic noise to clean Gazebo odometry
        Node(
            package='bcr_bot',
            executable='odometry_noise_node.py',
            name='odometry_noise_node',
            output='screen',
            parameters=[
                {'linear_noise_stddev': 0.02},        # 2cm standard deviation
                {'angular_noise_stddev': 0.05},       # ~3 degrees standard deviation  
                {'velocity_noise_stddev': 0.01},      # 1cm/s standard deviation
                {'angular_velocity_noise_stddev': 0.02}, # ~1 degree/s standard deviation
            ],
        ),
        # AMCL Lite with dynamic object detection for robot localization
        Node(
            package='ros2_amcl_lite',
            executable='amcl_lite_node',
            name='amcl_lite_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'enable_dynamic_detection': True},
                {'dynamic_detection_threshold': 0.3},
                {'dynamic_confidence_threshold': 0.7},
                {'dynamic_weight_reduction': 0.5},
                {'dynamic_history_size': 5},
                {'max_particles': 1000},  # Reduced from 2000 for better performance
                {'min_particles': 200},   # Reduced from 500 for better performance
                {'z_hit': 0.7},           # Increased for better sensor model
                {'z_short': 0.1},         # Increased for better obstacle handling
                {'z_max': 0.05},
                {'z_rand': 0.15},         # Reduced random noise
                {'sigma_hit': 0.15},      # Reduced for more precise matching
                {'lambda_short': 0.1},
                {'laser_likelihood_max_dist': 2.0},
                {'laser_max_range': 100.0},
                {'laser_min_range': 0.0},
                {'laser_model_type': 'likelihood_field'},
                {'set_initial_pose': True},
                {'initial_pose_x': 0.0},
                {'initial_pose_y': 0.0},
                {'initial_pose_a': 0.0},
                {'base_frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'global_frame_id': 'map'},
                {'scan_topic': 'bcr_bot/scan'},
                {'map_topic': 'map'},
                {'odom_topic': 'bcr_bot/odom'}
            ],
        ),
        
        # Static transform from map to odom (initial transform for map visualization)
        # AMCL will override this with its own dynamic transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        
        # Map server to load the bcr_map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file,
                        'use_sim_time': True}],
        ),
        
        # Map server lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'autostart': True},
                       {'node_names': ['map_server']},
                       {'use_sim_time': True}]
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
