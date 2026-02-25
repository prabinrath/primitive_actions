#!/usr/bin/env python3
"""
Test script: send a circular EE path to the cartesian_path_follow action server.

The circle:
  - Radius  : 5 cm (configurable via --radius)
  - Plane   : X-Z (vertical), starting tangent direction is -Z (downward)
  - Centre  : p0 + [radius, 0, 0]  →  first motion is straight down
  - Samples : 360 waypoints (1° per step) at constant orientation (start pose)

Usage (after sourcing the workspace):
    ros2 run primitive_actions test_cartesian_circle
    ros2 run primitive_actions test_cartesian_circle --ros-args -p radius:=0.15
"""

import sys
import os
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation

import placo

from primitive_actions.action import CartesianPathFollow


class CircleTestClient(Node):

    def __init__(self):
        super().__init__('cartesian_circle_test_client')

        self.declare_parameter('radius', 0.05)
        self.declare_parameter('n_samples', 360)
        self.declare_parameter('ee_frame', 'fr3_hand_tcp')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('linear_speed', 0.1)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('alpha', 0.95)

        self._latest_js = None
        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)

        self._action_client = ActionClient(
            self, CartesianPathFollow, 'cartesian_path_follow')

    def _js_cb(self, msg: JointState):
        self._latest_js = msg

    # ------------------------------------------------------------------
    def _wait_for_js(self, timeout=5.0):
        deadline = time.perf_counter() + timeout
        while self._latest_js is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.perf_counter() > deadline:
                raise RuntimeError('Timed out waiting for /joint_states')

    def _get_current_ee_pose(self, ee_frame: str, urdf_path: str):
        """FK via placo to obtain the current EE pose as a 4×4 matrix."""
        joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
            'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
        ]
        robot = placo.RobotWrapper(urdf_path, placo.Flags.collision_as_visual)
        js = self._latest_js
        for name in joint_names:
            if name in js.name:
                robot.set_joint(name, js.position[js.name.index(name)])
        robot.update_kinematics()
        return robot.get_T_world_frame(ee_frame)  # 4×4 numpy array

    # ------------------------------------------------------------------
    def _build_circle_path(self, T0: np.ndarray, radius: float,
                           n_samples: int) -> PoseArray:
        """
        Generate a full circle in the X-Z plane relative to the base frame.

        Starting position  : T0[:3, 3]
        Starting direction : -Z  (downward)
        Orientation        : constant = T0[:3, :3]

        Parametric form (theta: 0 → 2π):
            p(θ) = p0 + r·[1-cos(θ), 0, -sin(θ)]
        At θ=0        → p0          (start)
        At θ=π/2      → p0+[r,0,-r] (right and down)
        At θ=π        → p0+[2r,0,0] (full right)
        At θ=3π/2     → p0+[r,0, r] (right and up)
        At θ=2π       → p0          (back to start)
        """
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]
        q0 = Rotation.from_matrix(R0).as_quat()  # [x, y, z, w]

        thetas = np.linspace(0, 2 * np.pi, n_samples, endpoint=False)

        path = PoseArray()
        path.header.frame_id = 'base'

        for theta in thetas:
            pose = Pose()
            p = p0 + radius * np.array([1 - np.cos(theta), 0.0, -np.sin(theta)])
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])
            pose.orientation.x = float(q0[0])
            pose.orientation.y = float(q0[1])
            pose.orientation.z = float(q0[2])
            pose.orientation.w = float(q0[3])
            path.poses.append(pose)

        return path

    # ------------------------------------------------------------------
    def run(self):
        radius = self.get_parameter('radius').value
        n_samples = int(self.get_parameter('n_samples').value)
        ee_frame = self.get_parameter('ee_frame').value
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value
        alpha = self.get_parameter('alpha').value

        urdf_path: str = self.get_parameter('urdf_path').value
        if not urdf_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('primitive_actions')
                urdf_path = os.path.join(
                    pkg_share, 'urdf', 'franka_fr3_kinematics.urdf')
            except Exception:
                self.get_logger().fatal(
                    'urdf_path parameter is empty and primitive_actions is not found. '
                    'Pass: --ros-args -p urdf_path:=/path/to/robot.urdf')
                raise RuntimeError('urdf_path not set')

        # --- 1. Get current EE pose via FK ---
        self.get_logger().info('Waiting for /joint_states ...')
        self._wait_for_js()
        T0 = self._get_current_ee_pose(ee_frame, urdf_path)
        self.get_logger().info(
            f'Current EE position: {T0[:3, 3].round(4).tolist()}')

        # --- 2. Build circle path ---
        path = self._build_circle_path(T0, radius, n_samples)
        self.get_logger().info(
            f'Circle: radius={radius}m  samples={n_samples}  '
            f'plane=X-Z  start_dir=-Z (downward)')

        # --- 3. Wait for action server ---
        self.get_logger().info('Waiting for action server ...')
        self._action_client.wait_for_server()

        # --- 4. Send goal ---
        goal = CartesianPathFollow.Goal()
        goal.path = path
        goal.ee_frame = ee_frame
        goal.linear_speed = linear_speed
        goal.angular_speed = angular_speed
        goal.alpha = alpha
        goal.open_loop = True

        self.get_logger().info('Sending goal ...')
        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted. Executing ...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'SUCCESS: {result.message}')
        else:
            self.get_logger().error(f'FAILED: {result.message}')

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        p = fb.current_ee_pose.position
        self.get_logger().info(
            f'  wp {fb.current_waypoint + 1}/{fb.total_waypoints}  '
            f'EE=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]',
            throttle_duration_sec=1.0)


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CircleTestClient()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
