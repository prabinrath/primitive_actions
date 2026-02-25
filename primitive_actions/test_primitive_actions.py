#!/usr/bin/env python3
"""
Test script: pick then place using the primitive_action_planner servers.

Pick:
    x=0.4, y=-0.2, z=0.05, yaw=0
Place:
    x=0.4, y=0.2,  z=0.05, yaw=pi/2

Usage:
    ros2 run primitive_actions test_primitive_actions
"""

import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from primitive_actions.action import Pick, Place


class TestPrimitiveActions(Node):

    def __init__(self):
        super().__init__('test_primitive_actions')
        self._pick_client  = ActionClient(self, Pick,  'pick')
        self._place_client = ActionClient(self, Place, 'place')

    # ------------------------------------------------------------------

    def _send_pick(self, x, y, z, yaw, approach_height=0.0):
        self.get_logger().info(
            f'Waiting for /pick server ...')
        self._pick_client.wait_for_server()

        goal = Pick.Goal()
        goal.x   = float(x)
        goal.y   = float(y)
        goal.z   = float(z)
        goal.yaw = float(yaw)
        goal.approach_height = float(approach_height)

        self.get_logger().info(
            f'Sending Pick  x={x}  y={y}  z={z}  yaw={yaw:.4f}  approach_height={approach_height}')
        future = self._pick_client.send_goal_async(
            goal, feedback_callback=self._pick_feedback_cb)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Pick goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f'Pick SUCCESS: {result.message}')
        else:
            self.get_logger().error(f'Pick FAILED:  {result.message}')
        return result.success

    def _send_place(self, x, y, z, yaw, approach_height=0.0):
        self.get_logger().info(
            f'Waiting for /place server ...')
        self._place_client.wait_for_server()

        goal = Place.Goal()
        goal.x   = float(x)
        goal.y   = float(y)
        goal.z   = float(z)
        goal.yaw = float(yaw)
        goal.approach_height = float(approach_height)

        self.get_logger().info(
            f'Sending Place x={x}  y={y}  z={z}  yaw={yaw:.4f}  approach_height={approach_height}')
        future = self._place_client.send_goal_async(
            goal, feedback_callback=self._place_feedback_cb)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Place goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.success:
            self.get_logger().info(f'Place SUCCESS: {result.message}')
        else:
            self.get_logger().error(f'Place FAILED:  {result.message}')
        return result.success

    # ------------------------------------------------------------------

    def _pick_feedback_cb(self, feedback_msg):
        self.get_logger().info(
            f'[Pick ] stage: {feedback_msg.feedback.stage}')

    def _place_feedback_cb(self, feedback_msg):
        self.get_logger().info(
            f'[Place] stage: {feedback_msg.feedback.stage}')

    # ------------------------------------------------------------------

    def run(self):
        ok = self._send_pick(x=0.4, y=-0.2, z=0.05, yaw=0.0, approach_height=0.3)
        if not ok:
            self.get_logger().error('Aborting: pick failed.')
            return

        ok = self._send_place(x=0.4, y=0.2, z=0.05, yaw=math.pi / 2, approach_height=0.3)
        if not ok:
            self.get_logger().error('Place failed.')
            return

        self.get_logger().info('Pick-and-place sequence complete.')


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TestPrimitiveActions()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
