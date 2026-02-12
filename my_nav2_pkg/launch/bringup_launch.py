import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # File paths
    pkg_share = get_package_share_directory('my_nav2_pkg')
    map_file = os.path.join(pkg_share, 'config', 'my_map.yaml')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    ekf_file = os.path.join(pkg_share, 'config', 'ekf_scanodom.yaml')

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
            ),

        # -----------------------------------------------------
        # 1. Load your URDF robot model (robot_state_publisher)
        # -----------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('tracer_description'),
                    'launch',
                    'display.launch.py'
                )
            )
        ),

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the Nav2 parameters file'
        ),

        # Static TF: base_link → laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_laser',
            output='screen',
            arguments=[
                '--x','0.102','--y','0','--z','0.076',
                '--roll','0','--pitch','0','--yaw','3.14159',
                '--frame-id','base_link','--child-frame-id','laser_frame'
            ]
        ),

        # Static TF: base_link → imu_link (adjust pose if needed)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_imu',
            output='screen',
            arguments=[
                '--x','0','--y','0','--z','0',
                '--roll','0','--pitch','0','--yaw','0',
                '--frame-id','base_link','--child-frame-id','imu_link'
            ]
        ),

        # # IMU filter (Madgwick) -> outputs /imu/data
        # Node(
        #     package='imu_filter_madgwick',
        #     executable='imu_filter_madgwick_node',
        #     name='imu_filter',
        #     output='screen',
        #     parameters=[{
        #         'use_mag': False,
        #         'world_frame': 'enu',
        #         'publish_tf': False,
        #         'gain': 0.05
        #     }],
        #     remappings=[
        #         ('imu/data_raw', '/imu/data_raw'),
        #         ('imu/data', '/imu/data')
        #     ]
        # ),

        # Lidar Node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'angle_compensate': True
            }]
        ),

        # Lidar Odometry Node
        Node(
            package='lidar_odometry',
            executable='lidar_odometry_node',
            name='lidar_odometry_node',
            output='screen',
            parameters=[{
                'scan_topic_name': '/scan',
                'odom_topic_name': '/scan_odom',
                'max_correspondence_distance': 1.0,
                'transformation_epsilon': 0.005,
                'maximum_iterations': 30
            }]
        ),
 
        # IMU covariance/bias republisher
        Node(
            package='my_nav2_pkg',
            executable='imu_cov_republisher',
            name='imu_cov_republisher',
            output='screen'
        ),

        # EKF Node for sensor fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'frequency': 50.0,
                'two_d_mode': True,
                'publish_tf': True,

                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',

                # Fuse odom and imu
                'odom0': '/odom',
                'odom0_queue_size': 20,
                'odom0_differential': False,
                'odom0_relative': False,
                'odom0_nodelay': True,
                'odom0_config': [ False, False, False,
                                  False, False, False,
                                  True,  False, False,
                                  False, False, True,
                                  False, False, False ],

                # Lidar odom (pose)
                'odom1': '/scan_odom',
                'odom1_queue_size': 50,
                'odom1_nodelay': True,
                'odom1_differential': False,
                'odom1_relative': False,
                'odom1_config': [ True, True, False,
                                  False, False, True,
                                  False, False, False,
                                  False, False, False,
                                  False, False, False ],

                # IMU: yaw rate only (disable orientation to avoid zero-cov NaNs)
                'imu0': '/imu/data_cov',
                'imu0_queue_size': 50,
                'imu0_nodelay': True,
                'imu0_differential': False,
                'imu0_relative': False,
                'imu0_remove_gravitational_acceleration': False,
                'imu0_config': [ False, False, False,
                                 False, False, False,
                                 False, False, False,
                                 False, False, True,
                                 False, False, False ],

                'sensor_timeout': 0.2,
                'print_diagnostics': True
                
            }]
        ),


        # -------------------------
        # Nav2 Bringup Components
        # -------------------------
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file],
        ),

        # Old ekf node
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[ekf_file],
        # ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'behavior_server'
                ]
            }],
        ),
    ])
