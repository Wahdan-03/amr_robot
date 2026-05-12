import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Paths
    amr_pkg = get_package_share_directory('amr_robot')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    
    nav2_params = os.path.join(amr_pkg, 'config', 'nav2_params.yaml')

    # 2. Arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(amr_pkg, 'maps', 'my_first_map.yaml'),
        description='Full path to the saved map yaml file'
    )
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        map_arg,

        # ── 1. LD19 LiDAR ────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ldlidar_stl_ros2'),
                '/launch/ld19.launch.py'
            ])
        ),

        # ── 2. Arduino Bridge ─────────────────────────────────
        Node(
            package='amr_robot',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
            parameters=[{
                'serial_port': '/dev/arduino_mega',
                'baud_rate': 115200,
                'wheel_base': 0.175,
                'publish_rate': 20.0,
            }]
        ),

        # ── 3. Map Server ─────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False, 'yaml_filename': map_file}]
        ),

        # ── 4. AMCL ───────────────────────────────────────────
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params]
        ),

        # ── 5. Navigation (Planner, Controller, Behavior) ─────
        # Note: We use the single 'navigation_launch.py' but disable its internal manager
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_pkg, '/launch/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': nav2_params,
                'use_lifecycle_mgr': 'false', # We will use our own manager below
                'map_subscribe_transient_local': 'true'
            }.items()
        ),

        # ── 6. SINGLE Lifecycle Manager ───────────────────────
        # This is the "Brain" that starts the nodes in the CORRECT order
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
                    'controller_server', 
                    'planner_server', 
                    'behavior_server', # FIXED: was recoveries_server
                    'bt_navigator', 
                    'waypoint_follower'
                ]
            }]
        ),
    ])
