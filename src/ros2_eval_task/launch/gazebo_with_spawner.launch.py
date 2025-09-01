#!/usr/bin/env python3
"""
Launch file for Gazebo simulation with model spawner node
This launch file starts:
1. Gazebo simulator with the factory world
2. RViz for visualization
3. Model spawner node for automatic model spawning and image capture
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    """
    Generate the launch description for the complete system
    """
    
    # Get package share directory
    pkg_share = FindPackageShare('ros2_eval_task')
    
    # Path to the world file
    world_path = PathJoinSubstitution([
        pkg_share, 'worlds', 'factory.world'
    ])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Gazebo launch
    # This includes the Gazebo server and client with the specified world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 
                'launch', 
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'false',  # Set to 'true' for debugging
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # RViz configuration file path
    rviz_config = PathJoinSubstitution([
        pkg_share, 'rviz', 'default.rviz'
    ])
    
    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Model spawner node with delayed start
    # This node handles automatic spawning/deletion of models and image capture
    # Using TimerAction to ensure Gazebo is fully loaded before starting
    delayed_spawner = TimerAction(
        period=5.0,  # Wait 5 seconds before starting
        actions=[
            Node(
                package='ros2_eval_task',
                executable='model_spawner_node',
                name='model_spawner_node',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    # Return the launch description with all nodes
    return LaunchDescription([
        declare_use_sim_time,
        gazebo_launch,
        rviz_node,
        delayed_spawner
    ])