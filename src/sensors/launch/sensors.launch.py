from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(package='sensors', executable='imu_pub',         output='screen'),
        Node(package='sensors', executable='gps_pub',         output='screen'),
        Node(package='sensors', executable='water_sensor',    output='screen'),
        # object_detector publishes both obstacle_detected and the annotated
        # camera/image/compressed feed (replacing camera_pub).
        Node(package='sensors', executable='object_detector', output='screen'),

    ])
