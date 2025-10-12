from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def _arm_node(arm_typ, arm_name, ns, simulated, filename,
              camera_reg_file, instrument_file, pkg_share):
    """Build one Node for a single arm with the same param logic as ROS1."""
    # Parameter files from config/
    dvrk_topics_yaml = PathJoinSubstitution([pkg_share, 'config', 'dvrk_topic_names.yaml'])
    camera_yaml = PathJoinSubstitution([pkg_share, 'config', camera_reg_file])
    instrument_yaml = PathJoinSubstitution([pkg_share, 'config', instrument_file])

    return Node(
        package='irob_robot',
        executable='robot_server_dvrk',  # your node binary
        name=f'robot_server_{arm_typ}',
        namespace=ns,
        output='screen',
        parameters=[
            dvrk_topics_yaml,
            camera_yaml,
            instrument_yaml,
            {
                'arm_typ': arm_typ,           # ROS1: <param name="arm_typ" .../>
                'arm_name': arm_name,         # ROS1: <param name="arm_name" .../>
                'filename': filename,         # ROS1: <param name="filename" .../>
                'simulated': simulated        # extra: allow sim vs real
            }
        ]
    )

def _expand_all_arms(context):
    # Launch args
    ns = LaunchConfiguration('ns').perform(context)
    simulated = LaunchConfiguration('simulated').perform(context)
    all_arms = LaunchConfiguration('all_arms').perform(context)
    arm_typ = LaunchConfiguration('arm_typ').perform(context)
    arm_name = LaunchConfiguration('arm_name').perform(context)
    filename = LaunchConfiguration('filename').perform(context)
    camera_reg_file = LaunchConfiguration('camera_registration_file').perform(context)
    instrument_file = LaunchConfiguration('instrument_info_file').perform(context)

    pkg_share = FindPackageShare('irob_robot')
    nodes = []

    if all_arms.lower() in ('true', '1', 'yes', 'on'):
        # Launch all four arms with sensible default names
        default_map = [
            ('PSM1', 'psm1'),
            ('PSM2', 'psm2'),
            ('ECM',  'ecm'),
            ('MTM',  'mtm'),
        ]
        for typ, default_name in default_map:
            nodes.append(_arm_node(
                arm_typ=typ,
                arm_name=default_name,
                ns=ns,
                simulated=simulated,
                filename=filename,
                camera_reg_file=camera_reg_file,
                instrument_file=instrument_file,
                pkg_share=pkg_share
            ))
    else:
        # Single arm (mirror ROS1 launch args)
        nodes.append(_arm_node(
            arm_typ=arm_typ,
            arm_name=arm_name,
            ns=ns,
            simulated=simulated,
            filename=filename,
            camera_reg_file=camera_reg_file,
            instrument_file=instrument_file,
            pkg_share=pkg_share
        ))
    return nodes

def generate_launch_description():
    return LaunchDescription([
        # Same args as your ROS1 file (plus ns, simulated, all_arms)
        DeclareLaunchArgument('ns', default_value='saf',
                              description='Top-level namespace (ROS1 used <group ns="saf">)'),
        DeclareLaunchArgument('arm_typ', default_value='PSM2',
                              description='Arm type (PSM1, PSM2, ECM, MTM)'),
        DeclareLaunchArgument('arm_name', default_value='arm_1',
                              description='Logical arm name'),
        DeclareLaunchArgument('camera_registration_file', default_value='registration_psm2.yaml',
                              description='Which registration YAML to load from config/'),
        DeclareLaunchArgument('filename', default_value='/home/dvrk_nat/trajectory_1.dat',
                              description='Trajectory/aux file path (carried over from ROS1)'),
        DeclareLaunchArgument('instrument_info_file', default_value='prograsp_forceps.yaml',
                              description='Which instrument YAML to load from config/'),
        DeclareLaunchArgument('simulated', default_value='false',
                              description='Set true to run in simulation mode'),
        DeclareLaunchArgument('all_arms', default_value='false',
                              description='Set true to launch PSM1, PSM2, ECM, MTM together'),

        # Build nodes once all args are known
        GroupAction([
            OpaqueFunction(function=_expand_all_arms)
        ])
    ])
