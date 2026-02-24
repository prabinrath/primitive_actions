from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    robot_ip = LaunchConfiguration('robot_ip')

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('primitive_actions'),
        'config', 'fr3_ros_controllers.yaml',
    ])

    franka_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('primitive_actions'),
            'launch', 'franka.launch.py',
        ])),
        launch_arguments={
            'robot_ip': robot_ip,
            'arm_id': 'fr3',
            'load_gripper': 'true',
            'controllers_yaml': controllers_yaml,
            'namespace': namespace,
        }.items(),
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        namespace=namespace,
        arguments=[
            'fr3_arm_controller',
            '--controller-manager-timeout', '60',
            '--controller-manager', '/controller_manager',
        ],
    )

    spacemouse = Node(
        package='spacenav',
        executable='spacenav_node',
        name='spacenav_node',
        output='screen',
        parameters=[{
            'publish_joy': True,
            'publish_twist': True,
            'zero_when_static': True,
        }],
    )

    move_to_start = Node(
        package='primitive_actions',
        executable='franka_move_to_start',
        name='franka_move_to_start',
        output='screen',
        namespace=namespace,
    )

    error_recovery = Node(
        package='primitive_actions',
        executable='franka_error_recovery',
        name='franka_error_recovery',
        output='screen',
        namespace=namespace,
    )

    action_server = Node(
        package='primitive_actions',
        executable='cartesian_path_action_server',
        name='cartesian_path_action_server',
        output='screen',
        parameters=[{
            'urdf_path': LaunchConfiguration('urdf_path'),
            'ee_frame': LaunchConfiguration('ee_frame'),
            'joint_trajectory_topic': LaunchConfiguration('joint_trajectory_topic'),
            'default_alpha': LaunchConfiguration('default_alpha'),
            'position_threshold': LaunchConfiguration('position_threshold'),
            'orientation_threshold': LaunchConfiguration('orientation_threshold'),
            'waypoint_timeout': LaunchConfiguration('waypoint_timeout'),
            'control_rate': LaunchConfiguration('control_rate'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Robot namespace. Leave empty for none.'),
        DeclareLaunchArgument(
            'robot_ip', default_value='172.16.0.2',
            description='IP address of the Franka FCI controller.'),
        DeclareLaunchArgument(
            'urdf_path', default_value='',
            description='Absolute path to the robot URDF for the IK solver. '
                        'Defaults to factr_teleop package URDF when empty.'),
        DeclareLaunchArgument(
            'ee_frame', default_value='fr3_hand_tcp',
            description='End-effector frame name in the URDF.'),
        DeclareLaunchArgument(
            'joint_trajectory_topic',
            default_value='/fr3_arm_controller/joint_trajectory',
            description='JointTrajectory topic for the arm controller.'),
        DeclareLaunchArgument(
            'default_alpha', default_value='0.95',
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

        franka_bringup,
        arm_controller_spawner,
        spacemouse,
        move_to_start,
        error_recovery,
        action_server,
    ])
