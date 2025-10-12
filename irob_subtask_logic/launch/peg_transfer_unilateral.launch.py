from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('ns')
    mode = LaunchConfiguration('mode')
    board = LaunchConfiguration('board_descriptor_file')
    offset = LaunchConfiguration('offset_file_arm_1')
    threshold = LaunchConfiguration('on_dist_threshold')
    arms = LaunchConfiguration('arm_names')

    return LaunchDescription([
        # --- Arguments (from <arg> tags) ---
        DeclareLaunchArgument('ns', default_value='saf'),
        DeclareLaunchArgument('mode', default_value='execution',
                              description='execution | calibration | acc_blocks | acc_pegs'),
        DeclareLaunchArgument('board_descriptor_file', default_value='peg_transfer_board.yaml'),
        DeclareLaunchArgument('offset_file_arm_1', default_value='offset_psm_2_uni.yaml'),
        DeclareLaunchArgument('on_dist_threshold', default_value='5.0'),
        DeclareLaunchArgument('arm_names', default_value='[arm_2]'),

        # --- Node (from <node> tag) ---
        Node(
            package='irob_subtask_logic',
            executable='peg_transfer_unilateral_node',
            namespace=ns,
            name='peg_transfer_unilateral',
            output='screen',
            parameters=[{
                'mode': mode,
                'board_descriptor_file': board,
                'offset_file_arm_1': offset,
                'on_dist_threshold': threshold,
                'arm_names': arms
            }]
        )
    ])
