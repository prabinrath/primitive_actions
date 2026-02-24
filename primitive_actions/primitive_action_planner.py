#!/usr/bin/env python3

# ---------------------------------------------------------------------------
# Primitive Action Planner
#
# Provides two action servers:
#   - /pick  (primitive_actions/Pick)
#   - /place (primitive_actions/Place)
#
# Both primitives use a top-down end-effector approach: the gripper Z-axis
# always points straight down; only yaw (rotation around world Z) is exposed
# as an orientation input.
#
# Internally, each primitive drives the robot by chaining calls to the
# CartesianPathFollow action server and the Franka gripper Grasp action.
#
# Pick sequence:
#   1. Open gripper
#   2. Top-down approach  →  (x, y, z + approach_height)
#   3. Descend            →  (x, y, z)
#   4. Close gripper
#   5. Retract            →  (x, y, z + approach_height)
#
# Place sequence:
#   1. Top-down approach  →  (x, y, z + approach_height)
#   2. Descend            →  (x, y, z)
#   3. Open gripper
#   4. Retract            →  (x, y, z + approach_height)
#
# Chaining:
#   Primitives are serialised by a single execution lock, so successive
#   action requests queue naturally – each request is accepted immediately
#   but processed in order.
# ---------------------------------------------------------------------------

import os
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState
from franka_msgs.action import Grasp
from scipy.spatial.transform import Rotation, Slerp
import placo

from primitive_actions.action import CartesianPathFollow, Pick, Place


class PrimitiveActionPlanner(Node):
    """
    Action servers: /pick and /place

    Both servers use the CartesianPathFollow action server for motion and the
    Franka gripper interface for grasping/releasing objects.

    Orientation convention (top-down):
        The end-effector Z-axis points straight down (-Z world).
        Yaw rotates the gripper around the world Z-axis.
        Roll and pitch are always zero relative to the world.
    """

    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__('primitive_action_planner')

        # --- Parameters ---
        self.declare_parameter('approach_height',    0.10)  # set via launch file
        self.declare_parameter('ee_frame',           'fr3_hand_tcp')
        self.declare_parameter('cartesian_alpha',    0.9)
        self.declare_parameter('linear_speed',       0.0)
        self.declare_parameter('angular_speed',      0.0)
        self.declare_parameter('gripper_open_width', 0.08)
        self.declare_parameter('gripper_speed',      0.1)
        self.declare_parameter('gripper_force',      30.0)
        self.declare_parameter('interp_step',        0.01)   # m per waypoint
        self.declare_parameter('interp_step_rad',     0.05)   # rad per waypoint
        self.declare_parameter('urdf_path',          '')
        self.declare_parameter('joint_names', [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
            'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
        ])
        self.declare_parameter('neutral_positions', [
            0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785,
        ])

        self._approach_height    = self.get_parameter('approach_height').value
        self._ee_frame           = self.get_parameter('ee_frame').value
        self._alpha              = self.get_parameter('cartesian_alpha').value
        self._linear_speed       = self.get_parameter('linear_speed').value
        self._angular_speed      = self.get_parameter('angular_speed').value
        self._gripper_open_width = self.get_parameter('gripper_open_width').value
        self._gripper_speed      = self.get_parameter('gripper_speed').value
        self._gripper_force      = self.get_parameter('gripper_force').value
        self._interp_step: float     = self.get_parameter('interp_step').value
        self._interp_step_rad: float = self.get_parameter('interp_step_rad').value
        self._joint_names: list  = self.get_parameter('joint_names').value
        self._neutral_positions: list = self.get_parameter('neutral_positions').value

        # --- Placo FK model (read-only; used only to compute current EE pose) ---
        urdf_path: str = self.get_parameter('urdf_path').value
        if not urdf_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('primitive_actions')
                urdf_path = os.path.join(
                    pkg_share, 'urdf', 'franka_fr3_kinematics.urdf')
            except Exception:
                self.get_logger().fatal(
                    'urdf_path parameter is empty and primitive_actions '
                    'package not found. Pass: --ros-args -p urdf_path:=...')
                raise RuntimeError('urdf_path not set')
        self.get_logger().info(f'Loading URDF: {urdf_path}')
        self._fk_robot = placo.RobotWrapper(
            urdf_path, placo.Flags.collision_as_visual)
        for name, pos in zip(self._joint_names, self._neutral_positions):
            self._fk_robot.set_joint(name, pos)
        self._fk_robot.update_kinematics()

        # Joint state cache (updated by subscription)
        self._js_lock = threading.Lock()
        self._latest_js: JointState | None = None

        # Serialise pick/place executions so they do not overlap.
        # Successive goal requests are accepted immediately but run in order.
        self._exec_lock = threading.Lock()

        cb_group = ReentrantCallbackGroup()

        # --- Joint state subscriber ---
        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10,
            callback_group=cb_group)

        # --- CartesianPathFollow client ---
        self._cartesian_client = ActionClient(
            self, CartesianPathFollow, 'cartesian_path_follow',
            callback_group=cb_group)

        # --- Franka gripper client ---
        self._gripper_client = ActionClient(
            self, Grasp, '/franka_gripper/grasp',
            callback_group=cb_group)

        # --- Pick action server ---
        self._pick_server = ActionServer(
            self,
            Pick,
            'pick',
            execute_callback=self._execute_pick,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cb_group,
        )

        # --- Place action server ---
        self._place_server = ActionServer(
            self,
            Place,
            'place',
            execute_callback=self._execute_place,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cb_group,
        )

        self.get_logger().info('PrimitiveActionPlanner ready.')
        self.get_logger().info(
            f'  default approach_height : {self._approach_height:.3f} m')
        self.get_logger().info(
            f'  ee_frame                : {self._ee_frame}')

    # ------------------------------------------------------------------
    # Common action server callbacks
    # ------------------------------------------------------------------

    def _goal_cb(self, _goal_request):
        """Accept all incoming goals immediately; execution is serialised
        internally by the _exec_lock."""
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Orientation helper
    # ------------------------------------------------------------------

    @staticmethod
    def _top_down_pose(x: float, y: float, z: float, yaw: float) -> Pose:
        """
        Build a Pose with a top-down end-effector orientation.

        The orientation is constructed as:
            R = Rz(yaw) · Rx(π)

        where Rx(π) flips the EE Z-axis to point straight down (-Z world),
        and Rz(yaw) rotates the gripper about the world Z-axis.

        Only yaw differs between poses; roll and pitch are always zero.
        """
        # Intrinsic ZX: first rotate body frame around Z by yaw,
        # then around (new) X by π  →  EE z-column = [0, 0, -1]ᵀ
        R_mat = Rotation.from_euler('ZX', [yaw, np.pi]).as_matrix()
        q = Rotation.from_matrix(R_mat).as_quat()  # (x, y, z, w)

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])
        return pose

    # ------------------------------------------------------------------
    # Joint state callback
    # ------------------------------------------------------------------

    def _js_cb(self, msg: JointState):
        with self._js_lock:
            self._latest_js = msg

    # ------------------------------------------------------------------
    # FK + interpolation helpers
    # ------------------------------------------------------------------

    def _get_current_T_ee(self, timeout: float = 5.0) -> np.ndarray:
        """
        Return the current 4×4 EE transform via FK.

        Blocks until a joint state is available, or raises RuntimeError on
        timeout.
        """
        deadline = time.perf_counter() + timeout
        while True:
            with self._js_lock:
                js = self._latest_js
            if js is not None:
                break
            if time.perf_counter() > deadline:
                raise RuntimeError('Timed out waiting for /joint_states')
            time.sleep(0.05)

        for name in self._joint_names:
            if name in js.name:
                self._fk_robot.set_joint(
                    name, js.position[js.name.index(name)])
        self._fk_robot.update_kinematics()
        return self._fk_robot.get_T_world_frame(self._ee_frame).copy()

    def _interpolate_path(self, target_pose: Pose) -> list[Pose]:
        """
        Build a linearly-interpolated path from the *current* EE pose
        (obtained via FK) to *target_pose*.

        Position  : linear lerp in Cartesian space.
        Orientation: SLERP between start and target quaternions.

        The number of waypoints is computed adaptively so that consecutive
        waypoints are separated by *interp_step* metres, giving a uniform
        EE-space velocity regardless of segment length.

        Returns
        -------
        list[Pose]  – waypoints from (exclusive) start to (inclusive) target.
        """
        # --- Start: current FK pose ---
        T_start = self._get_current_T_ee()
        p_start = T_start[:3, 3]
        R_start = Rotation.from_matrix(T_start[:3, :3])

        # --- Target ---
        p_end = np.array([
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z,
        ])
        R_end = Rotation.from_quat([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
        ])

        def make_pose(p: np.ndarray, r: Rotation) -> Pose:
            q = r.as_quat()
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = \
                float(p[0]), float(p[1]), float(p[2])
            pose.orientation.x, pose.orientation.y = float(q[0]), float(q[1])
            pose.orientation.z, pose.orientation.w = float(q[2]), float(q[3])
            return pose

        # --- Phase 1: rotate in place (p_start fixed, R_start → R_end) ---
        R_err = R_start.inv() * R_end
        angle = float(np.abs(R_err.magnitude()))
        n_rot = max(1, int(np.ceil(angle / self._interp_step_rad)))
        slerp = Slerp([0.0, 1.0], Rotation.concatenate([R_start, R_end]))
        rot_ts = np.linspace(0.0, 1.0, n_rot + 1)[1:]
        poses = [make_pose(p_start, slerp(t)) for t in rot_ts]

        # --- Phase 2: translate (R_end fixed, p_start → p_end) ---
        dist = float(np.linalg.norm(p_end - p_start))
        n_pos = max(1, int(np.ceil(dist / self._interp_step)))
        pos_ts = np.linspace(0.0, 1.0, n_pos + 1)[1:]
        poses += [make_pose((1.0 - t) * p_start + t * p_end, R_end) for t in pos_ts]

        return poses

    # ------------------------------------------------------------------
    # Blocking action client helpers
    # ------------------------------------------------------------------

    def _send_cartesian_path(self, poses: list[Pose]) -> bool:
        """
        Send a CartesianPathFollow goal and block until it completes.
        Uses threading.Event internally so the call does not deadlock
        inside a MultiThreadedExecutor callback.
        Returns True on success, False on failure / server unavailable.
        """
        if not self._cartesian_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'cartesian_path_follow server not available.')
            return False

        goal = CartesianPathFollow.Goal()
        goal.path = PoseArray()
        goal.path.poses = poses
        goal.ee_frame = self._ee_frame
        goal.linear_speed = self._linear_speed
        goal.angular_speed = self._angular_speed
        goal.alpha = self._alpha

        done_event = threading.Event()
        result_holder: list[bool] = [False]

        def on_result(future):
            try:
                result_holder[0] = future.result().result.success
            except Exception as exc:
                self.get_logger().error(f'Cartesian path result error: {exc}')
                result_holder[0] = False
            finally:
                done_event.set()

        def on_goal_accepted(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().warn('CartesianPathFollow goal rejected.')
                done_event.set()
                return
            gh.get_result_async().add_done_callback(on_result)

        self._cartesian_client.send_goal_async(goal).add_done_callback(
            on_goal_accepted)

        # Timeout: per-waypoint timeout × number of waypoints, minimum 30 s
        path_timeout = max(30.0, len(poses) * 35.0)
        if not done_event.wait(timeout=path_timeout):
            self.get_logger().error('Cartesian path timed out waiting for result.')
            return False
        return result_holder[0]

    def _gripper_command(self, open_gripper: bool) -> bool:
        """
        Send a gripper Grasp goal and block until it completes.
        open_gripper=True  → open to full width (pre-grasp / release)
        open_gripper=False → close with configured force (grasp)
        The gripper server may return success=False when opening (no object
        at full width); we treat any non-exception return as success.
        """
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                '/franka_gripper/grasp server not available.')
            return False

        goal = Grasp.Goal()
        if open_gripper:
            goal.width = float(self._gripper_open_width)
            goal.speed = float(self._gripper_speed)
            goal.force = 5.0
            goal.epsilon.inner = float(self._gripper_open_width)
            goal.epsilon.outer = float(self._gripper_open_width)
        else:
            goal.width = 0.0
            goal.speed = float(self._gripper_speed)
            goal.force = float(self._gripper_force)
            goal.epsilon.inner = float(self._gripper_open_width)
            goal.epsilon.outer = float(self._gripper_open_width)

        done_event = threading.Event()
        result_holder: list[bool] = [False]

        def on_result(future):
            try:
                # For an open command the server may report failure (no object
                # detected at the requested width), which is fine – we ignore
                # that distinction and consider the command sent successfully.
                result_holder[0] = True
            except Exception as exc:
                self.get_logger().error(f'Gripper result error: {exc}')
                result_holder[0] = False
            finally:
                done_event.set()

        def on_goal_accepted(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().warn('Gripper goal rejected.')
                done_event.set()
                return
            gh.get_result_async().add_done_callback(on_result)

        self._gripper_client.send_goal_async(goal).add_done_callback(
            on_goal_accepted)

        if not done_event.wait(timeout=15.0):
            self.get_logger().error('Gripper command timed out waiting for result.')
            return False
        return result_holder[0]

    # ------------------------------------------------------------------
    # Pick
    # ------------------------------------------------------------------

    def _execute_pick(self, goal_handle):
        """Serialised entry point for Pick goals."""
        # Block until any ongoing primitive finishes, then take the lock.
        self._exec_lock.acquire()
        try:
            return self._run_pick(goal_handle)
        finally:
            self._exec_lock.release()

    def _run_pick(self, goal_handle):
        goal = goal_handle.request
        x, y, z = goal.x, goal.y, goal.z
        yaw = goal.yaw
        h = goal.approach_height if goal.approach_height > 0.0 \
            else self._approach_height

        feedback = Pick.Feedback()

        def publish(stage: str):
            feedback.stage = stage
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'[Pick ] {stage}')

        def abort(msg: str):
            goal_handle.abort()
            return Pick.Result(success=False, message=msg)

        # 1. Open gripper ---------------------------------------------------
        publish('opening_gripper')
        if not self._gripper_command(open_gripper=True):
            return abort('Failed to open gripper.')

        # 2. Move to approach pose ------------------------------------------
        publish('approaching')
        approach_pose = self._top_down_pose(x, y, z + h, yaw)
        if not self._send_cartesian_path(self._interpolate_path(approach_pose)):
            return abort('Failed to reach approach pose.')

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Pick.Result(success=False, message='Cancelled.')

        # 3. Descend to pick position ----------------------------------------
        publish('descending')
        pick_pose = self._top_down_pose(x, y, z, yaw)
        if not self._send_cartesian_path(self._interpolate_path(pick_pose)):
            return abort('Failed to reach pick pose.')

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Pick.Result(success=False, message='Cancelled.')

        # 4. Close gripper (grasp) -------------------------------------------
        publish('grasping')
        if not self._gripper_command(open_gripper=False):
            return abort('Failed to close gripper.')
        # Brief pause to let the gripper fully settle before retract
        time.sleep(0.3)

        # 5. Retract to approach pose ----------------------------------------
        publish('retracting')
        retract_pose = self._top_down_pose(x, y, z + h, yaw)
        if not self._send_cartesian_path(self._interpolate_path(retract_pose)):
            return abort('Failed to retract after pick.')

        goal_handle.succeed()
        return Pick.Result(success=True, message='Pick completed successfully.')

    # ------------------------------------------------------------------
    # Place
    # ------------------------------------------------------------------

    def _execute_place(self, goal_handle):
        """Serialised entry point for Place goals."""
        self._exec_lock.acquire()
        try:
            return self._run_place(goal_handle)
        finally:
            self._exec_lock.release()

    def _run_place(self, goal_handle):
        goal = goal_handle.request
        x, y, z = goal.x, goal.y, goal.z
        yaw = goal.yaw
        h = goal.approach_height if goal.approach_height > 0.0 \
            else self._approach_height

        feedback = Place.Feedback()

        def publish(stage: str):
            feedback.stage = stage
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'[Place] {stage}')

        def abort(msg: str):
            goal_handle.abort()
            return Place.Result(success=False, message=msg)

        # 1. Move to approach pose (gripper already holding object) ----------
        publish('approaching')
        approach_pose = self._top_down_pose(x, y, z + h, yaw)
        if not self._send_cartesian_path(self._interpolate_path(approach_pose)):
            return abort('Failed to reach approach pose.')

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Place.Result(success=False, message='Cancelled.')

        # 2. Descend to place position ----------------------------------------
        publish('descending')
        place_pose = self._top_down_pose(x, y, z, yaw)
        if not self._send_cartesian_path(self._interpolate_path(place_pose)):
            return abort('Failed to reach place pose.')

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Place.Result(success=False, message='Cancelled.')

        # 3. Open gripper (release object) ------------------------------------
        publish('releasing')
        if not self._gripper_command(open_gripper=True):
            return abort('Failed to open gripper.')
        # Allow the gripper to fully open before retracting
        time.sleep(0.3)

        # 4. Retract to approach pose -----------------------------------------
        publish('retracting')
        retract_pose = self._top_down_pose(x, y, z + h, yaw)
        if not self._send_cartesian_path(self._interpolate_path(retract_pose)):
            return abort('Failed to retract after place.')

        goal_handle.succeed()
        return Place.Result(success=True, message='Place completed successfully.')


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PrimitiveActionPlanner()
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
