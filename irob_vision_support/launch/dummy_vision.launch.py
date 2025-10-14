from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='irob_vision_support',
            executable='dummy_image_processor_node',
            name='dummy_vision',
            namespace='saf/vision',
            output='screen',
            remappings=[
                ('marker', 'dummy_target_marker'),
                ('result', 'target'),
            ]
        )
    ])
