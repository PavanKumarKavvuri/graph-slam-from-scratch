#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    bag_file_path_arg = DeclareLaunchArgument(
        "bag_file_path",
        default_value=" ",
        description="bag file path to play",
    )

    graph_slam_node = Node(
        package="graph-slam-from-scratch",
        executable="graph_slam",
        name="graph_slam_node",
        # parameters=[
        #     {"bag_file_path": LaunchConfiguration("bag_file_path")},
        #     {"spatial_viewpoint_type": LaunchConfiguration("spatial_viewpoint_type")},
        #     {"use_rosbag_tf": LaunchConfiguration("use_rosbag_tf")},
        # ],
        output="screen",
    )

    bag_play_process = ExecuteProcess(
        cmd=['ros2', 
             'bag', 
             'play', 
             LaunchConfiguration('bag_file_path'),
             "--rate", 
             "2.5"
             ],
        output='screen'
    )

    return LaunchDescription(
        [bag_play_process, graph_slam_node, bag_file_path_arg]
    )