#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_planning = get_package_share_directory('autoware_launch')
    map_dir = "/home/autolab/autoware_map/sample-map-planning"

    
    navigation_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_planning, 'launch', 'planning_simulator.launch.xml')
        ),
        launch_arguments={
            'map_path': map_dir,
            'vehicle_model': 'sample_vehicle',
            'sensor_model': 'sample_sensor_kit'
        }.items()
    )

    
    goal_pose_publisher = Node(
        package='my_robot_controller',
        executable='car_nav',
        name='car_nav',
        output='screen'
    )


    ld = LaunchDescription()
    ld.add_action(navigation_node)
    ld.add_action(goal_pose_publisher)

    return ld
