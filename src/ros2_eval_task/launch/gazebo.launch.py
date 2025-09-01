from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare('ros2_eval_task')

    world_path = PathJoinSubstitution([
        pkg_share, 'worlds', 'factory.world'
    ])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'fasle'
        }.items()
    )

    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'default.rviz'])

    rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg],
                output='screen'
            )

    return LaunchDescription([gz, rviz_node])
