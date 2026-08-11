#!/usr/bin/env python3
"""
mission_control_demo.py

Standalone demo/test client for robot_task_server — sends a fixed sequence:
  navigate to Cargo Bay -> grab the toolkit -> navigate to Engine Room -> fix it

This plays the same role as the "Sequence" node in the Week 9 Behavior Tree
labs (do A, then B, then C, stop on first failure) but written as a plain
Python script instead of standing up BehaviorTree.CPP + Groot2 — same
end result. It's also exactly the pattern LLM node should follow: parse the instruction, then send this same
sequence of RobotTask goals.

Coordinates below are placeholders — swap in the real module coordinates
once the Gazebo world's map is available.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from astronaut_robot.action import RobotTask

MODULES = {
    'command_module': (0.0, 0.0),
    'cargo_bay': (3.0, 1.5),
    'engine_room': (5.0, -2.0),
}


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control_demo')
        self.client = ActionClient(self, RobotTask, 'robot_task')

    def send_task(self, task_type, target_name, x=0.0, y=0.0):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('robot_task_server not available')
            return False

        goal = RobotTask.Goal()
        goal.task_type = task_type
        goal.target_name = target_name
        goal.x = x
        goal.y = y

        print(f'[INFO] Sending task: {task_type} -> {target_name}')
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print('[ERROR] Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        print(f'[{"SUCCESS" if result.success else "FAILED"}] {result.message}')
        return result.success


def main():
    rclpy.init()
    node = MissionControl()

    x, y = MODULES['cargo_bay']
    if node.send_task('navigate', 'cargo_bay', x, y):
        if node.send_task('grab', 'toolkit'):
            x, y = MODULES['engine_room']
            if node.send_task('navigate', 'engine_room', x, y):
                node.send_task('fix', 'engine_panel')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
