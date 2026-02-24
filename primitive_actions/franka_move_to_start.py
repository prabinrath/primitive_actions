#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState, Joy
from franka_msgs.action import Move
from std_msgs.msg import Empty
from builtin_interfaces.msg import Duration
import numpy as np
from threading import Lock


class FrankaMoveToStart(Node):
    def __init__(self):
        super().__init__('franka_move_to_start')

        self.home_pose = [0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/4]
        self.joint_names = [f'fr3_joint{i}' for i in range(1, 8)]

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/fr3_arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(
            self, Move, '/franka_gripper/move')

        self.create_subscription(
            Joy, '/spacenav/joy', self.joy_callback, 1)
        self.get_logger().info('MoveToStart: using spacemouse interface')

        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(
            Empty, '/franka_reset', self.reset_callback, 1)

        self.current_joint_positions = None
        self.lock = Lock()
        self.last_reset_time = 0.0

        self.get_logger().info('Waiting for action servers...')
        self.gripper_client.wait_for_server()
        self.arm_client.wait_for_server()
        self.get_logger().info('Action servers available.')

    def joint_state_callback(self, msg):
        positions = []
        try:
            for name in self.joint_names:
                if name in msg.name:
                    positions.append(msg.position[msg.name.index(name)])
            if len(positions) == 7:
                with self.lock:
                    self.current_joint_positions = np.array(positions)
        except ValueError:
            pass

    def joy_callback(self, msg):
        if len(msg.buttons) < 27:
            return
        # R button on the right side of the SpaceMouse
        if msg.buttons[4] == 1:
            self.send_reset_command()

    def reset_callback(self, msg):
        self.send_reset_command()

    def send_reset_command(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_reset_time < 2.0:
            return
        self.last_reset_time = current_time

        move_goal = Move.Goal()
        move_goal.width = 0.08
        move_goal.speed = 0.1
        self.gripper_client.send_goal_async(move_goal)

        with self.lock:
            if self.current_joint_positions is None:
                self.get_logger().warn('Current joint state not received yet.')
                return
            current_pos = self.current_joint_positions.copy()

        target = np.array(self.home_pose)
        distance = np.linalg.norm(current_pos - target)

        speed = 0.5  # rad/s
        duration_sec = max(2.0, distance / speed)

        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.home_pose
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec - int(duration_sec)) * 1e9))
        traj.points = [point]
        traj.header.stamp = self.get_clock().now().to_msg()
        goal_msg.trajectory = traj

        self.get_logger().info(
            f'Sending reset. Distance: {distance:.2f} rad  '
            f'Duration: {duration_sec:.2f} s')
        self.arm_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrankaMoveToStart()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
