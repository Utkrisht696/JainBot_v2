from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('jainbot')
    use_realsense = LaunchConfiguration('use_realsense')

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

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'enable_sync': 'true',
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '',
            'enable_color': 'false',
            'depth_module.global_time_enabled': 'true',
            'align_depth.enable': 'false',
            'pointcloud.enable': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'filters': 'spatial,temporal',
        }.items(),
        condition=IfCondition(use_realsense)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_realsense',
            default_value='true',
            description='Enable RealSense camera launch'
        ),
        rsp,
        cm,
        jsb,
        diff,
        ekf,
        realsense_launch,
    ])
