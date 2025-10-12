from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('ns')
    img_topic = LaunchConfiguration('image_topic')
    target_topic = LaunchConfiguration('target_topic')
    parent_frame = LaunchConfiguration('parent_frame')
    child_frame = LaunchConfiguration('child_frame')

    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='vision'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_preprocessed'),
        DeclareLaunchArgument('target_topic', default_value='/vision/target'),
        DeclareLaunchArgument('parent_frame', default_value='camera_frame'),
        DeclareLaunchArgument('child_frame', default_value='target_frame'),

        Node(
            package='irob_vision_support',
            executable='vision_server_test_dummy_node',
            namespace=ns,
            name='dummy_vision_server',
            output='screen',
            parameters=[{
                'image_topic': img_topic,
                'target_topic': target_topic,
                'parent_frame': parent_frame,
                'child_frame': child_frame
            }]
        )
    ])
