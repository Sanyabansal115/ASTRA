#!/usr/bin/env python3
"""
astra_navigator.py - THE LLM HOOK POINT.

Turns a location NAME into a real Nav2 goal, so the LLM never has to know
coordinates, frames, quaternions, or action APIs.

Three ways to drive it
----------------------
1) Topic (simplest for the LLM - just publish a string):
     ros2 topic pub --once /astra/go_to std_msgs/msg/String "{data: 'engine_room'}"

2) Command line, for testing without any LLM:
     ros2 run astra_robot astra_navigator --location engine_room

3) Import it from Python (e.g. inside a LangChain tool):
     from astra_robot.astra_navigator import LOCATIONS, resolve_location

Status is published on /astra/status (std_msgs/String) so the LLM can
report back to the user: "navigating", "arrived", "failed: ...".

"""

import os
import sys
import math
import argparse
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import yaml
from ament_index_python.packages import get_package_share_directory

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def _load_locations():
    """Read locations.yaml from the installed share directory."""
    share = get_package_share_directory('astra_robot')
    path = os.path.join(share, 'config', 'locations.yaml')
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('locations', {}), data.get('aliases', {})


LOCATIONS, ALIASES = _load_locations()


def resolve_location(name):
    """
    Map free-ish text to a canonical location key.
    Returns the key, or None if there's no confident match.

    Deliberately conservative: it does NOT guess. If the LLM sends something
    unrecognised we return None so the agent can ask the user to clarify -
    exactly the behaviour the project workflow doc requires.
    """
    if not name:
        return None
    key = name.strip().lower().replace(' ', '_').replace('-', '_')

    if key in LOCATIONS:
        return key
    if key in ALIASES:
        return ALIASES[key]

    # Allow "go to the engine room" style input. Longest candidates first so
    # 'medical_bay' wins over 'medical' and a short alias doesn't accidentally
    # match inside a longer phrase.
    candidates = sorted(list(LOCATIONS.keys()) + list(ALIASES.keys()),
                        key=len, reverse=True)
    for cand in candidates:
        if cand in key:
            return ALIASES.get(cand, cand)
    return None


def yaw_to_quaternion(yaw):
    """Minimal yaw -> (z, w); roll and pitch are always 0 for a ground robot."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class AstraNavigator(Node):
    def __init__(self):
        # use_sim_time MUST be true. Nav2 and Gazebo run on simulated time; if
        # this node stamps goal poses with wall-clock time, the TF lookup for
        # that stamp fails and Nav2 rejects or mis-transforms the goal. This
        # node is usually launched with `ros2 run` (no launch file to set the
        # param for us), so we set it here explicitly.
        super().__init__(
            'astra_navigator',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        cb = ReentrantCallbackGroup()

        self._client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=cb)

        self._status_pub = self.create_publisher(String, '/astra/status', 10)
        self.create_subscription(
            String, '/astra/go_to', self._on_command, 10, callback_group=cb)

        # Serialises goals: if the LLM fires two commands quickly, the second
        # waits rather than racing Nav2 with overlapping goals.
        self._nav_lock = threading.Lock()

        self.get_logger().info(
            'astra_navigator ready. Known locations: %s'
            % ', '.join(sorted(LOCATIONS.keys())))
        self._publish_status('ready')

    # ------------------------------------------------------------------ util
    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _on_command(self, msg):
        self.go_to(msg.data)

    # ------------------------------------------------------------- main call
    def go_to(self, name, timeout_sec=300.0):
        """
        Blocking. Returns True on arrival. Safe to call from a subscription
        callback because it waits on an Event, never on a nested spin.
        """
        key = resolve_location(name)
        if key is None:
            warn = (f"unknown location: '{name}'. Known: "
                    f"{', '.join(sorted(LOCATIONS.keys()))}")
            self.get_logger().warn(warn)
            self._publish_status(f'failed: {warn}')
            return False

        with self._nav_lock:
            return self._send_and_wait(key, timeout_sec)

    def _send_and_wait(self, key, timeout_sec):
        x, y, yaw = LOCATIONS[key]
        self.get_logger().info(f'Navigating to {key} at ({x}, {y})')
        self._publish_status(f'navigating: {key}')

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 navigate_to_pose server unavailable')
            self._publish_status('failed: Nav2 not running')
            return False

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qz, qw = yaw_to_quaternion(float(yaw))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose

        done = threading.Event()
        outcome = {'accepted': False, 'status': None}

        def on_result(result_future):
            res = result_future.result()
            outcome['status'] = res.status if res is not None else None
            done.set()

        def on_goal_response(goal_future):
            handle = goal_future.result()
            if handle is None or not handle.accepted:
                outcome['accepted'] = False
                done.set()
                return
            outcome['accepted'] = True
            handle.get_result_async().add_done_callback(on_result)

        self._client.send_goal_async(goal).add_done_callback(on_goal_response)

        if not done.wait(timeout=timeout_sec):
            self.get_logger().error(f'Timed out navigating to {key}')
            self._publish_status(f'failed: timeout going to {key}')
            return False

        if not outcome['accepted']:
            self.get_logger().error('Nav2 rejected the goal')
            self._publish_status(f'failed: goal rejected for {key}')
            return False

        if outcome['status'] == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Arrived at {key}')
            self._publish_status(f'arrived: {key}')
            self._task_reaction(key)
            return True

        self.get_logger().warn(
            f'Navigation to {key} ended with status {outcome["status"]}')
        self._publish_status(f'failed: could not reach {key}')
        return False

    # ------------------------------------------------------- task reactions
    def _task_reaction(self, key):
        """
        The 'task reaction' the workflow doc asks for - a visible response on
        arrival. Kept as a status message + log line so there is nothing extra
        to break during the demo. Extend here for per-room behaviour.
        """
        reactions = {
            'engine_room': 'Running engine diagnostics... repair complete.',
            'medical_bay': 'Medical supplies checked and restocked.',
            'cargo_bay': 'Cargo manifest scanned.',
            'airlock': 'EVA prep sequence complete. Airlock ready.',
            'sleeping_quarters': 'Entering rest mode.',
            'command_module': 'Docked at command station. Awaiting orders.',
        }
        text = reactions.get(key, 'Task complete.')
        self.get_logger().info(f'[TASK] {text}')
        self._publish_status(f'task: {text}')


def spin_in_background(node):
    """Start a MultiThreadedExecutor on a daemon thread; return (executor, thread)."""
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--location', '-l', default=None,
                        help='Navigate here immediately, then exit.')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = AstraNavigator()
    executor, _thread = spin_in_background(node)
    code = 0

    try:
        if args.location:
            code = 0 if node.go_to(args.location) else 1
        else:
            # Service mode: idle while the background executor handles
            # /astra/go_to commands from the LLM.
            idle = threading.Event()
            while rclpy.ok():
                idle.wait(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(code)


if __name__ == '__main__':
    main()
