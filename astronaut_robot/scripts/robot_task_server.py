#!/usr/bin/env python3
"""
robot_task_server.py

Single action server exposing RobotTask, handling:
  - "navigate": forwards to Nav2's NavigateToPose action (real navigation,
     owned by your Gazebo/Nav2 teammate's stack)
  - "grab":     sends a scripted joint trajectory to reach + close the
     gripper, then calls an attach service (Gazebo DetachableJoint bridge)
  - "fix":      sends a scripted joint trajectory to reach + touch a panel,
     then calls a part-state service to flip its visual state

This is the one node LLM code should call. It follows the
same shape as reach_location_server.cpp from the Week 9 BT lab (goal in,
feedback while running, success/failure result) and the send_joint_goal()
pattern from panda_swing_client.py (Week 11) for the arm moves — just
without needing full MoveIt planning, since the arm only ever reaches a
couple of hand-picked poses.


"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from astronaut_robot.action import RobotTask


# --- Hand-picked joint poses (degrees->radians done inline), same idea as
# swing_left / home_center in panda_swing_client.py. Tune these by dragging
# the arm in RViz first and reading off the joint_states values, same as
# the class exercise. Order: [shoulder_joint, elbow_joint, gripper_joint]
ARM_HOME    = [0.0, 0.0, 0.0]
ARM_REACH   = [0.9, -1.3, 0.0]   # lean forward, elbow bent down toward object
ARM_GRASP   = [0.9, -1.3, 0.035] # same reach pose, gripper closed
ARM_LIFT    = [0.3, -0.5, 0.035] # lifted, still holding
ARM_TOUCH   = [0.9, -1.3, 0.0]   # reach toward a panel, gripper open (for "fix")


class RobotTaskServer(Node):
    def __init__(self):
        super().__init__('robot_task_server')
        cb_group = ReentrantCallbackGroup()

        self._arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
            callback_group=cb_group)
        self._nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=cb_group)

        # Placeholders — confirm real names/types with the Gazebo teammate
        self._attach_client = self.create_client(Trigger, '/manipulator/attach_object')
        self._part_state_client = self.create_client(Trigger, '/manipulator/set_part_fixed')

        self._server = ActionServer(
            self, RobotTask, 'robot_task', self.execute_callback,
            callback_group=cb_group)

        self.get_logger().info('robot_task_server ready.')

    # ---------- arm helper: send one waypoint and block until done ----------
    def send_arm_goal(self, joint_positions, time_from_start_sec=1.5):
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('arm_controller action server not available')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['shoulder_joint', 'elbow_joint', 'gripper_joint']
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(time_from_start_sec)
        goal.trajectory.points = [point]

        send_future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

    # ---------- navigate ----------
    def do_navigate(self, x, y):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            return False, 'Nav2 action server not available'

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_future = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False, 'Nav2 rejected the goal'

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True, 'Arrived'

    # ---------- grab ----------
    def do_grab(self, target_name):
        self.send_arm_goal(ARM_REACH)
        self.send_arm_goal(ARM_GRASP)  # close gripper near the object

        if self._attach_client.wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            future = self._attach_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
        else:
            self.get_logger().warn('attach service not found — skipping physical attach')

        self.send_arm_goal(ARM_LIFT)
        return True, f'Grabbed {target_name}'

    # ---------- fix ----------
    def do_fix(self, target_name):
        self.send_arm_goal(ARM_TOUCH)

        if self._part_state_client.wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            future = self._part_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
        else:
            self.get_logger().warn('part-state service not found — skipping state change')

        self.send_arm_goal(ARM_HOME)
        return True, f'Fixed {target_name}'

    # ---------- dispatch ----------
    def execute_callback(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(f'Task received: {req.task_type} -> {req.target_name}')

        feedback = RobotTask.Feedback()
        feedback.status = f'starting {req.task_type}'
        goal_handle.publish_feedback(feedback)

        if req.task_type == 'navigate':
            success, msg = self.do_navigate(req.x, req.y)
        elif req.task_type == 'grab':
            success, msg = self.do_grab(req.target_name)
        elif req.task_type == 'fix':
            success, msg = self.do_fix(req.target_name)
        else:
            success, msg = False, f'Unknown task_type: {req.task_type}'

        result = RobotTask.Result()
        result.success = success
        result.message = msg
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = RobotTaskServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
