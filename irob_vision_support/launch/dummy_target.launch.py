from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare configurable arguments
    frame_id = LaunchConfiguration('frame_id')
    rate_hz = LaunchConfiguration('rate_hz')

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'frame_id',
            default_value='PSM1_psm_base_link',
            description='Parent TF frame for dummy target'
        ),
        DeclareLaunchArgument(
            'rate_hz',
            default_value='10.0',
            description='Publishing rate (Hz)'
        ),

        # Dummy target node
        Node(
            package='irob_vision_support',
            executable='dummy_target_publisher_node',
            name='dummy_target_publisher',
            namespace='saf',
            output='screen',
            parameters=[{
                'frame_id': frame_id,
                'rate_hz': rate_hz,
            }]
        ),
    ])
