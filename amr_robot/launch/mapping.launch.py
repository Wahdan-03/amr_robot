"""
mapping.launch.py
─────────────────
Use this launch file to BUILD a map.

Starts:
  1. LD19 LiDAR          → /scan
  2. Arduino Bridge      → /odom, TF odom→base_link, listens /cmd_vel
  3. SLAM Toolbox        → /map  (mapping mode)
  4. Teleop Keyboard     → publishes /cmd_vel  (drive robot to build map)

Usage:
  ros2 launch amr_robot mapping.launch.py

Controls (in the teleop terminal):
  i / , = forward / back
  j / l = rotate left / right
  k     = stop
  q / z = increase / decrease speed

Save map when done:
  ros2 service call /slam_toolbox/save_map \
    slam_toolbox/srv/SaveMap "{name: {data: '/home/wahdan/maps/amr_map'}}"
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    slam_config = os.path.join(
        get_package_share_directory('amr_robot'),
        'config', 'slam_config.yaml'
    )

    return LaunchDescription([

        # ── 1. LD19 LiDAR ────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('ldlidar_stl_ros2'),
                '/launch/ld19.launch.py'
            ])
        ),
        
       # ── Static Transform: base_link to base_laser ────────
        Node(
           package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            # New ROS 2 Humble format:
            arguments=['--x', '0', '--y', '0', '--z', '1.1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'base_laser']
        ),

        # ── 2. Arduino Bridge ─────────────────────────────────
        Node(
            package='amr_robot',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
            parameters=[{
                'serial_port':  '/dev/arduino_mega',
                'baud_rate':    115200,
                'wheel_base':   0.165,    # ← MEASURE: centre-to-centre (metres)
                'publish_rate': 20.0,
            }]
        ),

        # ── 3. SLAM Toolbox (delayed 3s) ─────────────────────
        TimerAction(period=5.0, actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_config,
                    {'use_sim_time': False}
                ],
            )
        ]),
        
        
        # ── 4. Teleop Keyboard (delayed 4s) ──────────────────
        # Drives the robot so the map gets built.
        # Keep the terminal focused to send key commands.
        TimerAction(period=4.0, actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop',
                output='screen',
                prefix='xterm -e',        # opens in its own window via VNC
                remappings=[('/cmd_vel', '/cmd_vel')],
            )
        ]),

    ])
