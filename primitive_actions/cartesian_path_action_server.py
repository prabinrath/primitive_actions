#!/usr/bin/env python3

# ---------------------------------------------------------------------------
# Cartesian Path Action Server
#
# Accepts a geometry_msgs/PoseArray (EE waypoints in the robot base frame)
# and executes the path using IK (placo) + joint trajectory commands.
# ---------------------------------------------------------------------------

import time
import threading

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from primitive_actions.action import CartesianPathFollow

import placo
from scipy.spatial.transform import Rotation
import os


class CartesianPathActionServer(Node):
    """
    Action server: cartesian_path_follow (primitive_actions/CartesianPathFollow)

    Receives a list of EE poses (PoseArray), iterates through each waypoint
    and drives the robot there using IK, publishing JointTrajectory commands.
    """

    # Server-side defaults (overridable via action goal fields)
    DEFAULT_LINEAR_SPEED = 0.1   # m/s
    DEFAULT_ANGULAR_SPEED = 0.5  # rad/s
    DEFAULT_ALPHA = 0.9          # IK smoothing factor

    # Convergence thresholds
    POSITION_THRESHOLD = 0.005   # 5 mm
    ORIENTATION_THRESHOLD = 0.01  # ~0.57 deg

    # Per-waypoint safety timeout
    WAYPOINT_TIMEOUT = 30.0      # seconds

    # Control loop rate
    CONTROL_RATE = 50.0          # Hz

    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__('cartesian_path_action_server')

        # --- Declare parameters ---
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('ee_frame', 'fr3_hand_tcp')
        self.declare_parameter('joint_names', [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
            'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
        ])
        self.declare_parameter('neutral_positions', [
            0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785,
        ])
        self.declare_parameter(
            'joint_trajectory_topic', '/fr3_arm_controller/joint_trajectory')

        self.joint_names: list = self.get_parameter('joint_names').value
        self.neutral_positions: list = self.get_parameter('neutral_positions').value
        self.default_ee_frame: str = self.get_parameter('ee_frame').value
        self.joint_trajectory_topic: str = \
            self.get_parameter('joint_trajectory_topic').value

        # --- Load URDF and set up placo IK solver ---
        urdf_path: str = self.get_parameter('urdf_path').value
        if not urdf_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('primitive_actions')
                urdf_path = os.path.join(
                    pkg_share, 'urdf', 'franka_fr3_kinematics.urdf')
            except Exception:
                self.get_logger().fatal(
                    'urdf_path parameter is empty and primitive_actions package '
                    'is not found. Please pass the urdf_path parameter:\n'
                    '  ros2 run primitive_actions cartesian_path_action_server '
                    '--ros-args -p urdf_path:=/absolute/path/to/robot.urdf')
                raise RuntimeError('urdf_path not set')
        self.get_logger().info(f'Loading URDF: {urdf_path}')

        self.robot = placo.RobotWrapper(
            urdf_path, placo.Flags.collision_as_visual)
        self.solver = self.robot.make_solver()
        self.solver.mask_fbase(True)
        self.solver.enable_joint_limits(True)
        self.solver.dt = 1.0 / self.CONTROL_RATE

        # Initialise robot to neutral posture
        for name, pos in zip(self.joint_names, self.neutral_positions):
            self.robot.set_joint(name, pos)
        self.robot.update_kinematics()

        # Frame task – target is updated per waypoint at execution time
        self.frame_task = self.solver.add_frame_task(
            self.default_ee_frame, np.eye(4))
        self.frame_task.configure(
            self.default_ee_frame, 'soft',
            self.DEFAULT_LINEAR_SPEED, self.DEFAULT_ANGULAR_SPEED)

        # Nullspace regularisation – pull joints toward neutral
        self.joints_task = self.solver.add_joints_task()
        self.joints_task.set_joints(
            {n: p for n, p in zip(self.joint_names, self.neutral_positions)})
        self.joints_task.configure('nullspace', 'soft', 1e-5)
        self.solver.add_regularization_task(1e-6)

        # Mutex protecting access to the latest joint state
        self._js_lock = threading.Lock()
        self._latest_js: JointState | None = None

        # Only one goal can be active at a time; this lock guards solver state
        self._exec_lock = threading.Lock()

        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10,
            callback_group=cb_group)

        self._pub = self.create_publisher(
            JointTrajectory, self.joint_trajectory_topic, 10)

        self._action_server = ActionServer(
            self,
            CartesianPathFollow,
            'cartesian_path_follow',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cb_group)

        self.get_logger().info('CartesianPathActionServer is ready.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _js_cb(self, msg: JointState):
        with self._js_lock:
            self._latest_js = msg

    def _goal_cb(self, goal_request):
        if len(goal_request.path.poses) == 0:
            self.get_logger().warn('Rejected goal: path is empty.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        self.get_logger().info('Cancel requested.')
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _sync_robot_to_js(self, js: JointState):
        """Copy latest joint positions into the placo robot model."""
        for name in self.joint_names:
            if name in js.name:
                self.robot.set_joint(name, js.position[js.name.index(name)])

    def _current_joint_positions(self, js: JointState) -> list[float]:
        return [
            js.position[js.name.index(n)]
            for n in self.joint_names if n in js.name
        ]

    @staticmethod
    def _pose_to_T(pose: Pose) -> np.ndarray:
        T = np.eye(4)
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        q = (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w)
        T[:3, :3] = Rotation.from_quat(q).as_matrix()
        return T

    @staticmethod
    def _T_to_pose(T: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x = float(T[0, 3])
        pose.position.y = float(T[1, 3])
        pose.position.z = float(T[2, 3])
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        pose.orientation.x, pose.orientation.y = float(q[0]), float(q[1])
        pose.orientation.z, pose.orientation.w = float(q[2]), float(q[3])
        return pose

    @staticmethod
    def _rotation_angle_error(R_current: np.ndarray,
                               R_target: np.ndarray) -> float:
        R_err = R_current.T @ R_target
        trace = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
        return abs(np.arccos(trace))

    def _publish_joint_command(self, positions: list[float]):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=0, nanosec=int(0.1 * 1e9))
        msg.points = [pt]
        self._pub.publish(msg)

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def _execute_cb(self, goal_handle):
        goal = goal_handle.request
        waypoints = goal.path.poses
        ee_frame = goal.ee_frame if goal.ee_frame else self.default_ee_frame
        lin_speed = (goal.linear_speed
                     if goal.linear_speed > 0 else self.DEFAULT_LINEAR_SPEED)
        ang_speed = (goal.angular_speed
                     if goal.angular_speed > 0 else self.DEFAULT_ANGULAR_SPEED)
        alpha = (goal.alpha
                 if 0.0 < goal.alpha <= 1.0 else self.DEFAULT_ALPHA)
        dt = 1.0 / self.CONTROL_RATE

        self.get_logger().info(
            f'Goal accepted: {len(waypoints)} waypoints | '
            f'ee_frame={ee_frame} | v={lin_speed:.2f}m/s '
            f'ω={ang_speed:.2f}rad/s | α={alpha:.2f}')

        # Only one execution at a time (protects the shared placo solver)
        if not self._exec_lock.acquire(blocking=False):
            goal_handle.abort()
            return CartesianPathFollow.Result(
                success=False, message='Another goal is already executing.')

        try:
            return self._run_path(
                goal_handle, waypoints, ee_frame,
                lin_speed, ang_speed, alpha, dt)
        finally:
            self._exec_lock.release()

    def _run_path(self, goal_handle, waypoints, ee_frame,
                  lin_speed, ang_speed, alpha, dt):

        # Reconfigure frame task for this goal
        self.frame_task.configure(ee_frame, 'soft', lin_speed, ang_speed)

        # Reset solver dt to default at the start of each new execution
        self.solver.dt = 1.0 / self.CONTROL_RATE

        # Wait up to 5 s for the first joint state
        deadline = self.get_clock().now().nanoseconds / 1e9 + 5.0
        while True:
            with self._js_lock:
                js = self._latest_js
            if js is not None:
                break
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                goal_handle.abort()
                return CartesianPathFollow.Result(
                    success=False,
                    message='Timed out waiting for /joint_states')
            time.sleep(0.05)

        # Seed the placo model with the actual current robot state
        self._sync_robot_to_js(js)
        self.robot.update_kinematics()

        feedback = CartesianPathFollow.Feedback()
        feedback.total_waypoints = len(waypoints)

        for wp_idx, wp_pose in enumerate(waypoints):

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return CartesianPathFollow.Result(
                    success=False,
                    message=f'Cancelled before waypoint {wp_idx}')

            T_target = self._pose_to_T(wp_pose)
            self.frame_task.T_world_frame = T_target

            wp_start = time.perf_counter()
            last_loop = wp_start

            while True:
                # --- Cancel check ---
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return CartesianPathFollow.Result(
                        success=False,
                        message=f'Cancelled at waypoint {wp_idx}')

                now = time.perf_counter()

                # --- Timeout guard ---
                if now - wp_start > self.WAYPOINT_TIMEOUT:
                    goal_handle.abort()
                    return CartesianPathFollow.Result(
                        success=False,
                        message=f'Timeout at waypoint {wp_idx}')

                # --- Sync placo with real robot ---
                with self._js_lock:
                    js = self._latest_js
                self._sync_robot_to_js(js)

                # --- Adaptive dt (skip first iteration, keep existing solver.dt) ---
                elapsed = now - last_loop
                if last_loop != wp_start:
                    self.solver.dt = elapsed
                last_loop = now

                # --- Solve IK and apply to placo state ---
                self.robot.update_kinematics()
                self.solver.solve(True)

                ik_positions = [
                    self.robot.get_joint(n) for n in self.joint_names]

                # --- Exponential smoothing toward IK solution ---
                curr_positions = self._current_joint_positions(js)
                if len(curr_positions) == len(ik_positions):
                    cmd = [
                        alpha * ik + (1.0 - alpha) * cur
                        for ik, cur in zip(ik_positions, curr_positions)
                    ]
                else:
                    cmd = ik_positions

                self._publish_joint_command(cmd)

                # --- Convergence check ---
                T_ee = self.robot.get_T_world_frame(ee_frame)
                pos_err = float(np.linalg.norm(T_ee[:3, 3] - T_target[:3, 3]))
                ang_err = self._rotation_angle_error(
                    T_ee[:3, :3], T_target[:3, :3])

                # --- Feedback ---
                feedback.current_waypoint = wp_idx
                feedback.current_ee_pose = self._T_to_pose(T_ee)
                goal_handle.publish_feedback(feedback)

                if (pos_err < self.POSITION_THRESHOLD
                        and ang_err < self.ORIENTATION_THRESHOLD):
                    self.get_logger().info(
                        f'  Waypoint {wp_idx + 1}/{len(waypoints)} '
                        f'reached  pos={pos_err * 1e3:.1f}mm  '
                        f'ang={np.degrees(ang_err):.2f}°')
                    break

                time.sleep(dt)

        goal_handle.succeed()
        return CartesianPathFollow.Result(
            success=True,
            message=f'Path completed: {len(waypoints)} waypoints executed.')


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CartesianPathActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
