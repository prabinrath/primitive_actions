#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy
from franka_msgs.action import ErrorRecovery
from action_msgs.srv import CancelGoal
from controller_manager_msgs.srv import (
    LoadController, ConfigureController, SwitchController)
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from threading import Thread
import time


class FrankaErrorRecovery(Node):
    def __init__(self):
        super().__init__('franka_error_recovery')

        self.last_recovery_time = 0.0

        self.create_subscription(
            Joy, '/spacenav/joy', self.joy_callback, 1)
        self.create_subscription(
            Joy, '/right_hand_inputs', self.quest_callback, 1)

        self.error_recovery_client = ActionClient(
            self, ErrorRecovery, '/action_server/error_recovery')
        self.cartesian_cancel_client = self.create_client(
            CancelGoal, 'cartesian_path_follow/_action/cancel_goal')
        self.pick_cancel_client = self.create_client(
            CancelGoal, 'pick/_action/cancel_goal')
        self.place_cancel_client = self.create_client(
            CancelGoal, 'place/_action/cancel_goal')
        self.load_controller_client = self.create_client(
            LoadController, '/controller_manager/load_controller')
        self.configure_controller_client = self.create_client(
            ConfigureController, '/controller_manager/configure_controller')
        self.switch_controller_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.set_param_client = self.create_client(
            SetParameters,
            '/franka_robot_state_broadcaster/set_parameters')

        self.get_logger().info(
            'Franka error recovery ready. '
            'Press ESC on SpaceMouse or btn_up on Quest to recover.')

    def joy_callback(self, msg):
        if len(msg.buttons) < 27:
            return
        # Button 22 (ESC) on SpaceMouse
        if msg.buttons[22] == 1:
            self.error_recovery_and_respawn()

    def quest_callback(self, msg):
        if len(msg.buttons) < 2:
            return
        # btn_up for error recovery
        if msg.buttons[0] == 1:
            self.error_recovery_and_respawn()

    def error_recovery_and_respawn(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_recovery_time < 5.0:
            return
        self.last_recovery_time = current_time

        def recovery_thread():
            self.get_logger().info('Starting error recovery...')
            for client, name in [
                (self.cartesian_cancel_client, 'cartesian_path_follow'),
                (self.pick_cancel_client, 'pick'),
                (self.place_cancel_client, 'place'),
            ]:
                if client.service_is_ready():
                    self.get_logger().info(f'Cancelling all {name} goals...')
                    future = client.call_async(CancelGoal.Request())
                    rclpy.spin_until_future_complete(
                        self, future, timeout_sec=5.0)
            goal = ErrorRecovery.Goal()
            future = self.error_recovery_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            self.get_logger().info(
                'Waiting 3 s before respawning controllers...')
            time.sleep(3.0)

            controllers = [
                'joint_state_broadcaster',
                'franka_robot_state_broadcaster',
                'fr3_arm_controller',
            ]

            for ctrl in controllers:
                self.get_logger().info(f'Loading {ctrl}...')
                load_req = LoadController.Request()
                load_req.name = ctrl
                future = self.load_controller_client.call_async(load_req)
                rclpy.spin_until_future_complete(
                    self, future, timeout_sec=10.0)

                if ctrl == 'franka_robot_state_broadcaster':
                    self.get_logger().info('Setting arm_id parameter...')
                    param = Parameter()
                    param.name = 'arm_id'
                    param.value = ParameterValue(
                        type=ParameterType.PARAMETER_STRING,
                        string_value='fr3')
                    set_req = SetParameters.Request()
                    set_req.parameters = [param]
                    future = self.set_param_client.call_async(set_req)
                    rclpy.spin_until_future_complete(
                        self, future, timeout_sec=10.0)

                config_req = ConfigureController.Request()
                config_req.name = ctrl
                future = self.configure_controller_client.call_async(
                    config_req)
                rclpy.spin_until_future_complete(
                    self, future, timeout_sec=10.0)

            switch_req = SwitchController.Request()
            switch_req.activate_controllers = controllers
            switch_req.deactivate_controllers = []
            switch_req.strictness = SwitchController.Request.BEST_EFFORT
            switch_req.activate_asap = True
            future = self.switch_controller_client.call_async(switch_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            self.get_logger().info('All controllers respawned.')

        Thread(target=recovery_thread, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = FrankaErrorRecovery()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
