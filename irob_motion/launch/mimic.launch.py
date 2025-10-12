from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ns = LaunchConfiguration('ns')
    params_file = PathJoinSubstitution([FindPackageShare('irob_motion'), 'config', 'mimic.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='saf', description='Namespace'),
        Node(
            package='irob_motion',
            executable='mimic_node',
            name='mimic',
            namespace=ns,
            output='screen',
            parameters=[params_file]
        )
    ])
