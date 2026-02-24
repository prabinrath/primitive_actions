from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------------
    # Include cartesian_path_action_server.launch.py
    # (brings up franka, arm controller, move_to_start, error_recovery,
    #  spacemouse, and the cartesian path action server)
    # ------------------------------------------------------------------
    cartesian_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('primitive_actions'),
            'launch', 'cartesian_path_action_server.launch.py',
        ])),
        launch_arguments={
            'namespace':              LaunchConfiguration('namespace'),
            'robot_ip':               LaunchConfiguration('robot_ip'),
            'urdf_path':              LaunchConfiguration('urdf_path'),
            'ee_frame':               LaunchConfiguration('ee_frame'),
            'joint_trajectory_topic': LaunchConfiguration('joint_trajectory_topic'),
            'default_alpha':          LaunchConfiguration('cartesian_alpha'),
            'position_threshold':     LaunchConfiguration('position_threshold'),
            'orientation_threshold':  LaunchConfiguration('orientation_threshold'),
            'waypoint_timeout':       LaunchConfiguration('waypoint_timeout'),
            'control_rate':           LaunchConfiguration('control_rate'),
        }.items(),
    )

    # ------------------------------------------------------------------
    # PrimitiveActionPlanner
    # ------------------------------------------------------------------
    primitive_planner = Node(
        package='primitive_actions',
        executable='primitive_action_planner',
        name='primitive_action_planner',
        output='screen',
        parameters=[{
            'urdf_path':          LaunchConfiguration('urdf_path'),
            'ee_frame':           LaunchConfiguration('ee_frame'),
            'approach_height':    LaunchConfiguration('approach_height'),
            'cartesian_alpha':    LaunchConfiguration('cartesian_alpha'),
            'interp_step':        LaunchConfiguration('interp_step'),
            'interp_step_rad':    LaunchConfiguration('interp_step_rad'),
            'gripper_open_width': LaunchConfiguration('gripper_open_width'),
            'gripper_speed':      LaunchConfiguration('gripper_speed'),
            'gripper_force':      LaunchConfiguration('gripper_force'),
        }],
    )

    return LaunchDescription([

        # --- Forwarded to cartesian_path_action_server.launch.py ---
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Robot namespace. Leave empty for none.'),
        DeclareLaunchArgument(
            'robot_ip', default_value='172.16.0.2',
            description='IP address of the Franka FCI controller.'),
        DeclareLaunchArgument(
            'urdf_path', default_value='',
            description='Absolute path to the robot URDF. '
                        'Defaults to primitive_actions package URDF when empty.'),
        DeclareLaunchArgument(
            'ee_frame', default_value='fr3_hand_tcp',
            description='End-effector frame name in the URDF.'),
        DeclareLaunchArgument(
            'joint_trajectory_topic',
            default_value='/fr3_arm_controller/joint_trajectory',
            description='JointTrajectory topic consumed by the arm controller.'),
        DeclareLaunchArgument(
            'cartesian_alpha', default_value='0.95',
            description='IK smoothing factor (0, 1].'),
        DeclareLaunchArgument(
            'position_threshold', default_value='0.005',
            description='Waypoint convergence position threshold [m].'),
        DeclareLaunchArgument(
            'orientation_threshold', default_value='0.02',
            description='Waypoint convergence orientation threshold [rad].'),
        DeclareLaunchArgument(
            'waypoint_timeout', default_value='30.0',
            description='Per-waypoint timeout [s].'),
        DeclareLaunchArgument(
            'control_rate', default_value='50.0',
            description='IK control loop rate [Hz].'),

        # --- PrimitiveActionPlanner-only ---
        DeclareLaunchArgument(
            'approach_height', default_value='0.10',
            description='Clearance above pick/place target for top-down approach [m].'),
        DeclareLaunchArgument(
            'interp_step', default_value='0.005',
            description='Cartesian step size between interpolated waypoints [m]. '
                        'Smaller = smoother but more waypoints.'),
        DeclareLaunchArgument(
            'interp_step_rad', default_value='0.02',
            description='Orientation step between interpolated waypoints [rad]. '
                        'Smaller = smoother rotation.'),
        DeclareLaunchArgument(
            'gripper_open_width', default_value='0.08',
            description='Gripper width when opening [m].'),
        DeclareLaunchArgument(
            'gripper_speed', default_value='0.1',
            description='Gripper movement speed [m/s].'),
        DeclareLaunchArgument(
            'gripper_force', default_value='30.0',
            description='Gripper grasp force [N].'),

        cartesian_launch,
        primitive_planner,
    ])
