from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ns = LaunchConfiguration('ns')
    arm_name = LaunchConfiguration('arm_name')
    simulated = LaunchConfiguration('simulated')
    params_file = PathJoinSubstitution([FindPackageShare('irob_motion'), 'config', 'surgeme_server.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='saf', description='Namespace'),
        DeclareLaunchArgument('arm_name', default_value='psm1', description='Arm logical name'),
        DeclareLaunchArgument('simulated', default_value='false', description='Run in simulation'),

        Node(
            package='irob_motion',
            executable='surgeme_server_node',
            name='surgeme_server',
            namespace=ns,
            output='screen',
            parameters=[params_file, {'arm_name': arm_name, 'simulated': simulated}]
        )
    ])
