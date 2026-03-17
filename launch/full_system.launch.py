"""
full_system.launch.py
─────────────────────
Launches all ScanToCAD nodes in one command.

Usage:
  cd ~/ros2_ws
  source install/setup.bash
  ros2 launch ScanToCAD full_system.launch.py

To launch only motion nodes (motors + odometry + pan-tilt), skipping scan:
  ros2 launch ScanToCAD full_system.launch.py scan_nodes:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'scan_nodes',
            default_value='true',
            description='Launch sensor, point cloud, and LCD nodes'
        ),

        # ── Gantry motor controller ───────────────────────────────────────────
        Node(
            package='ScanToCAD',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
        ),

        # ── Odometry + TF publisher ───────────────────────────────────────────
        Node(
            package='ScanToCAD',
            executable='odom_tf_pubs',
            name='odom_tf_pubs',
            output='screen',
        ),

        # ── Pan-tilt controller ───────────────────────────────────────────────
        Node(
            package='ScanToCAD',
            executable='pan_tilt_controller',
            name='pan_tilt_controller',
            output='screen',
        ),

        # ── TOF sensor publisher ──────────────────────────────────────────────
        Node(
            package='ScanToCAD',
            executable='sensor_pub',
            name='sensor_pub',
            output='screen',
            condition=IfCondition(LaunchConfiguration('scan_nodes')),
        ),

        # ── Point cloud publisher ─────────────────────────────────────────────
        Node(
            package='ScanToCAD',
            executable='point_cloud_pub',
            name='point_cloud_pub',
            output='screen',
            condition=IfCondition(LaunchConfiguration('scan_nodes')),
        ),

        # ── LCD display ───────────────────────────────────────────────────────
        Node(
            package='ScanToCAD',
            executable='lcd_display',
            name='lcd_display',
            output='screen',
            condition=IfCondition(LaunchConfiguration('scan_nodes')),
        ),
    ])