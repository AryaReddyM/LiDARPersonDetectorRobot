from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='pdetector_base', executable='esp_comms', output='screen', respawn=True, respawn_delay=3.0,
             parameters=[{
                 'wheel_radius': 0.0325,
                 'track_width': 0.197,
                 'max_wheel_speed': 13.6,
                 'ticks_per_rev': 0.0,
             }]),
        Node(package='pdetector_base', executable='teleop', output='screen', respawn=True),
    ])
