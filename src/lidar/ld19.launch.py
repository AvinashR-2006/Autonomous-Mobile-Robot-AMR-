#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for LDROBOT LD19 D300 LiDAR.
    Publishes /scan topic and broadcasts static transform from base_link to base_laser.
    """
    ldlidar_node = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ldlidar_publisher_ld19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'laser_scan_topic_name': 'scan'},
            {'point_cloud_2d_topic_name': 'pointcloud2d'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyUSB0'},
            {'serial_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0},
            {'range_min': 0.02},
            {'range_max': 12.0}
        ]
    )

    # Static transform: base_link -> base_laser (mounted 18cm above center of chassis)
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld19',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser']
    )

    ld = LaunchDescription()
    ld.add_action(ldlidar_node)
    ld.add_action(base_link_to_laser_tf_node)
    return ld
