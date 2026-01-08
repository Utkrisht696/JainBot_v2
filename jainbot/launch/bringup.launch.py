from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('jainbot')

    # Render URDF (xacro -> string)
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([pkg, 'urdf', 'jainbot.urdf.xacro'])
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    controller_params = PathJoinSubstitution([pkg, 'config', 'controllers.yaml'])
    ekf_params = PathJoinSubstitution([pkg, 'config', 'ekf.yaml'])

    # 1) Publish /robot_description (topic + param) → unblocks controller_manager
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'publish_fixed_joints': True, 'use_tf_static': True}],
        output='both'
    )

    # 2) Controller manager (also gets the param directly)
    cm = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params],
        output='both'
    )

    # 3) Spawn controllers after a short delay to avoid race conditions
    jsb = TimerAction(
        period=1.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='both'
        )]
    )

    diff = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='both'
        )]
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params],
        output='both'
    )

    return LaunchDescription([rsp, cm, jsb, diff, ekf])
