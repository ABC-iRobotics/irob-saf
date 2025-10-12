from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('ns')
    arms = LaunchConfiguration('arm_names')

    return LaunchDescription([
        # --- Arguments (from ROS1 <arg>) ---
        DeclareLaunchArgument('ns', default_value='saf'),
        DeclareLaunchArgument('arm_names', default_value='[arm_1]'),

        # --- Node (from ROS1 <node>) ---
        Node(
            package='irob_subtask_logic',
            executable='grasp_node',
            namespace=ns,
            name='dummy_grasp',
            output='screen',
            parameters=[{
                'arm_names': arms
            }]
        )
    ])
