#!/usr/bin/env python3
"""ASTRA - LLM navigation agent (Mistral AI + LangChain + Nav2).

High-level command interface for the spacecraft robot. It listens on the
``/prompt`` topic for natural-language instructions ("Go to the medical bay"),
lets a Mistral model decide which tool to call, and the navigation tools send a
``NavigateToPose`` goal to the Nav2 action server that the rest of the team's
stack already provides.

    /prompt (std_msgs/String)
        -> LangChain AgentExecutor + ChatMistralAI
        -> tool: navigate_to_room / navigate_to_coordinates / list_rooms / get_robot_pose
        -> NavigateToPose action goal (frame: map)
        -> Nav2 (bt_navigator -> planner -> controller -> /cmd_vel)

The LLM never plans a path itself - Nav2 keeps doing all planning, control and
obstacle avoidance, exactly as the project brief requires.

Follows the Week 12-13 lab pattern (prompt topic + LangChain tools + Mistral),
extended with a semantic room table so destinations can be named instead of
typed as raw coordinates.
"""

from __future__ import annotations

import math
import os
import queue
import sys
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

import tf2_ros

from astra_llm_agent.rooms import RoomTable, quaternion_to_yaw, yaw_to_quaternion

# LangChain / Mistral are installed with pip into the workspace virtualenv, not
# with rosdep, so give a useful message instead of a bare ImportError traceback.
try:
    from langchain.agents import AgentExecutor, create_tool_calling_agent
    from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
    from langchain_core.tools import tool
    try:
        from langchain_mistralai import ChatMistralAI
    except ImportError:  # older langchain-mistralai releases
        from langchain_mistralai.chat_models import ChatMistralAI
except ImportError as exc:  # pragma: no cover - depends on the local venv
    raise SystemExit(
        f"\nMissing LangChain/Mistral dependency: {exc}\n"
        "Activate the workspace virtualenv and install them:\n"
        '    pip install "langchain<1.0.0" "langchain-mistralai<1.0.0"\n'
        "If you launched with `ros2 launch`, remember to export the venv path:\n"
        "    export PYTHONPATH=$PYTHONPATH:$HOME/<workspace>/.venv/lib/python3.12/site-packages\n"
    ) from exc


SYSTEM_PROMPT = """You are ASTRA, the navigation assistant of an autonomous \
robot operating inside a simulated spacecraft.

You control the robot ONLY through the tools you are given. Nav2 does the path \
planning and obstacle avoidance; your job is to turn what the astronaut says \
into the right tool call.

The spacecraft has exactly these modules:
{room_list}

Rules:
1. If the user asks the robot to go somewhere named, call `navigate_to_room` \
with the matching key from the list above.
2. If the user gives raw coordinates, call `navigate_to_coordinates`.
3. If the user asks what places exist or where the robot can go, call `list_rooms`.
4. If the user asks where the robot is right now, call `get_robot_pose`.
5. NEVER claim the robot is moving unless you actually called a navigation \
tool and it reported success. If a tool reports an error, say so plainly.
6. If the destination is unclear or is not one of the modules above, do not \
guess: call `list_rooms` and ask the user which one they mean.
7. Answer in one or two short sentences.
"""


class LlmNavAgent(Node):
    """ROS 2 node: natural-language prompts in, Nav2 goals out."""

    def __init__(self) -> None:
        super().__init__("astra_llm_nav_agent")

        # ------------------------------------------------------------ params
        share_dir = _package_share_dir()
        self.declare_parameter("rooms_file", os.path.join(share_dir, "config", "rooms.yaml"))
        self.declare_parameter("env_file", os.path.join(share_dir, "config", "openai.env"))
        # The free tier 429s on mistral-large-latest after ~2 calls, so the
        # default is the small model; override with -p model:=... if you have
        # a paid key.
        self.declare_parameter("model", "mistral-small-latest")
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("prompt_topic", "/prompt")
        self.declare_parameter("response_topic", "/llm_response")
        self.declare_parameter("nav_action", "navigate_to_pose")
        self.declare_parameter("map_frame", "map")
        # astra_robot's URDF and nav2_params use base_footprint as the base
        # frame; the TF chain is map -> odom -> base_footprint -> base_link.
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("server_timeout", 5.0)

        rooms_file = self.get_parameter("rooms_file").value
        env_file = self.get_parameter("env_file").value
        model = self.get_parameter("model").value
        temperature = float(self.get_parameter("temperature").value)
        prompt_topic = self.get_parameter("prompt_topic").value
        response_topic = self.get_parameter("response_topic").value
        nav_action = self.get_parameter("nav_action").value
        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("robot_base_frame").value
        self._server_timeout = float(self.get_parameter("server_timeout").value)

        # ------------------------------------------------------- room table
        try:
            self.rooms = RoomTable.from_file(rooms_file)
        except ValueError as exc:
            raise SystemExit(f"\nCould not load the room table: {exc}\n")
        self.get_logger().info(
            f"Loaded {len(self.rooms.keys())} destinations from {rooms_file}: "
            f"{', '.join(self.rooms.keys())}"
        )

        # ----------------------------------------------------- ROS interfaces
        # Reentrant group so Nav2 feedback/result callbacks keep being served
        # while the LLM worker thread is waiting on the Mistral API.
        self._cb_group = ReentrantCallbackGroup()
        self._nav_client = ActionClient(
            self, NavigateToPose, nav_action, callback_group=self._cb_group
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._response_pub = self.create_publisher(String, response_topic, 10)
        self.create_subscription(
            String, prompt_topic, self._on_prompt, 10, callback_group=self._cb_group
        )

        # Goal bookkeeping (touched from both the executor and worker threads).
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self._goal_label = ""
        self._feedback_counter = 0

        # ----------------------------------------------------------- the LLM
        self._agent_executor = self._build_agent(model, temperature, env_file)

        # Prompts are handled on a worker thread: an LLM round-trip takes
        # seconds and must not block the executor (which is what delivers the
        # Nav2 result callbacks and TF updates).
        self._work_queue: "queue.Queue[Optional[str]]" = queue.Queue()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"ASTRA LLM navigation agent ready (model={model}). "
            f"Send prompts on {prompt_topic}, replies appear on {response_topic}."
        )

    # ==================================================================== LLM
    def _build_agent(self, model: str, temperature: float, env_file: str) -> AgentExecutor:
        api_key = self._load_api_key(env_file)
        if not api_key:
            raise SystemExit(
                "\nNo Mistral API key found. Set it in ONE of these ways:\n"
                "  1. export MISTRAL_API_KEY=your_key_here        (recommended)\n"
                "  2. echo 'MISTRAL_API_KEY=your_key_here' > ~/.astra_llm/openai.env\n"
                "  3. cp config/openai.env.example config/openai.env, fill in\n"
                f"     MISTRAL_API_KEY= and rebuild the package (reads {env_file})\n"
            )
        # ChatMistralAI also reads the environment, so keep both in sync.
        os.environ["MISTRAL_API_KEY"] = api_key

        llm = ChatMistralAI(model=model, temperature=temperature, api_key=api_key)

        prompt = ChatPromptTemplate.from_messages(
            [
                ("system", SYSTEM_PROMPT.format(room_list=self.rooms.describe())),
                ("human", "{input}"),
                MessagesPlaceholder(variable_name="agent_scratchpad"),
            ]
        )

        tools = self._build_tools()
        agent = create_tool_calling_agent(llm, tools, prompt)
        return AgentExecutor(
            agent=agent,
            tools=tools,
            verbose=True,           # prints "Invoking: `navigate_to_room` ..."
            max_iterations=4,       # never loop forever on a confused model
            handle_parsing_errors=True,
        )

    def _load_api_key(self, env_file: str) -> str:
        """Resolve the Mistral key: environment -> home file -> packaged file."""
        key = os.environ.get("MISTRAL_API_KEY", "").strip()
        if key:
            self.get_logger().info("Mistral API key taken from the environment.")
            return key

        home_env = os.path.expanduser("~/.astra_llm/openai.env")
        for path, source in ((home_env, "~/.astra_llm/openai.env"), (env_file, env_file)):
            key = _read_env_value(path, "MISTRAL_API_KEY")
            if key:
                self.get_logger().info(f"Mistral API key loaded from {source}.")
                if os.path.abspath(path) == os.path.abspath(env_file):
                    self.get_logger().warn(
                        "The key came from config/openai.env - that file is git-ignored "
                        "on purpose; never commit it or rename it to the .example file."
                    )
                return key
        return ""

    def _build_tools(self):
        """Tool definitions handed to the LLM.

        The docstrings ARE the tool descriptions the model sees, so they are
        written for the model rather than for a developer.
        """
        node = self

        @tool
        def navigate_to_room(room: str) -> str:
            """Send the robot to a named module of the spacecraft.

            Use this for any request like "go to the medical bay" or "navigate
            to the command module". The `room` argument should be one of the
            module keys, for example: medical_bay, command_module, cargo_bay,
            engine_room, airlock, sleeping_quarters.
            """
            match = node.rooms.resolve(room)
            if match is None:
                return (
                    f"Unknown destination '{room}'. Valid destinations are: "
                    f"{', '.join(node.rooms.keys())}."
                )
            ok, message = node._send_nav_goal(match.x, match.y, match.yaw, match.label)
            if not ok:
                return message
            return (
                f"{message} Destination {match.label} at x={match.x:.2f}, y={match.y:.2f}. "
                "Nav2 is planning the path now."
            )

        @tool
        def navigate_to_coordinates(x: float, y: float) -> str:
            """Send the robot to raw map coordinates, in metres.

            Use this only when the user gives explicit numbers, for example
            "move to x=2.0 and y=1.5". For named places use navigate_to_room.
            """
            try:
                goal_x, goal_y = float(x), float(y)
            except (TypeError, ValueError):
                return f"'{x}' and '{y}' are not valid numbers; give coordinates in metres."
            if not (math.isfinite(goal_x) and math.isfinite(goal_y)):
                return "Coordinates must be finite numbers, in metres."

            ok, message = node._send_nav_goal(
                goal_x, goal_y, 0.0, f"x={goal_x:.2f}, y={goal_y:.2f}"
            )
            if not ok:
                return message
            return f"{message} Destination x={goal_x:.2f}, y={goal_y:.2f}."

        @tool
        def list_rooms() -> str:
            """List every module the robot can navigate to, with its coordinates.

            Use this when the user asks where the robot can go, or when a
            requested destination does not exist.
            """
            return "The robot can navigate to these modules:\n" + node.rooms.describe()

        @tool
        def get_robot_pose() -> str:
            """Report where the robot currently is on the map.

            Use this for questions like "where is the robot?" or "where am I?".
            """
            return node._describe_pose()

        return [navigate_to_room, navigate_to_coordinates, list_rooms, get_robot_pose]

    # ================================================================ prompts
    def _on_prompt(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text:
            self.get_logger().warn("Ignoring an empty prompt.")
            return
        self.get_logger().info(f"Prompt received: {text}")
        self._work_queue.put(text)

    def _worker_loop(self) -> None:
        while True:
            text = self._work_queue.get()
            if text is None:          # shutdown sentinel
                break
            try:
                result = self._agent_executor.invoke({"input": text})
                answer = str(result.get("output", "")).strip() or "(no answer produced)"
            except Exception as exc:  # network error, bad key, rate limit, ...
                answer = f"The LLM request failed: {exc}"
                self.get_logger().error(answer)
            self.get_logger().info(f"Result: {answer}")
            self._publish(answer)

    def _publish(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._response_pub.publish(msg)

    # ================================================================== Nav2
    def _send_nav_goal(self, x: float, y: float, yaw: float, label: str) -> Tuple[bool, str]:
        """Send one NavigateToPose goal. Returns (accepted_for_sending, message).

        Deliberately non-blocking: the tool returns as soon as the goal is on
        its way so the agent can answer another prompt while the robot drives.
        Acceptance and arrival are reported later through the callbacks below.
        """
        if not self._nav_client.wait_for_server(timeout_sec=self._server_timeout):
            return False, (
                "The Nav2 action server 'navigate_to_pose' is not available. "
                "Is the navigation stack running?"
            )

        with self._goal_lock:
            previous = self._goal_label if self._goal_handle is not None else ""
        if previous:
            self.get_logger().warn(f"Replacing the goal in progress ({previous}) with {label}.")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(f"Sending Nav2 goal -> {label} (x={x:.2f}, y={y:.2f})")
        self._feedback_counter = 0
        with self._goal_lock:
            self._goal_label = label
        try:
            send_future = self._nav_client.send_goal_async(
                goal, feedback_callback=self._on_feedback
            )
        except Exception as exc:  # pragma: no cover - rclpy transport failure
            return False, f"Could not send the goal to Nav2: {exc}"
        send_future.add_done_callback(self._on_goal_response)
        return True, "Navigation goal sent to Nav2."

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Nav2 rejected the goal request: {exc}")
            return

        if not goal_handle.accepted:
            # Leave any goal already in flight alone - only this request failed.
            with self._goal_lock:
                label = self._goal_label
            message = f"Nav2 rejected the goal for {label}."
            self.get_logger().error(message)
            self._publish(message)
            return

        with self._goal_lock:
            self._goal_handle = goal_handle
            label = self._goal_label
        self.get_logger().info(f"Nav2 accepted the goal for {label}.")
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        # Nav2 publishes feedback at a high rate; log roughly every 20th one.
        self._feedback_counter += 1
        if self._feedback_counter % 20 != 1:
            return
        remaining = feedback_msg.feedback.distance_remaining
        with self._goal_lock:
            label = self._goal_label
        self.get_logger().info(f"Navigating to {label} - {remaining:.2f} m remaining.")

    def _on_result(self, future) -> None:
        with self._goal_lock:
            label = self._goal_label
            self._goal_handle = None

        try:
            status = future.result().status
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Could not read the navigation result: {exc}")
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            message = f"Arrived at {label}."
        elif status == GoalStatus.STATUS_CANCELED:
            message = f"Navigation to {label} was cancelled (a new goal may have replaced it)."
        else:
            message = (
                f"Navigation to {label} failed (status {status}). "
                "The goal may be unreachable or outside the map."
            )
        self.get_logger().info(message)
        self._publish(message)

    # =================================================================== pose
    def _describe_pose(self) -> str:
        """Look up map -> base_link in TF and describe it in plain language."""
        try:
            if not self._tf_buffer.can_transform(
                self._map_frame, self._base_frame, Time(),
                timeout=Duration(seconds=1.0),
            ):
                return (
                    f"The robot's position is not available yet: no transform from "
                    f"'{self._map_frame}' to '{self._base_frame}'. Is Nav2/AMCL running "
                    "and has the initial pose been set in RViz?"
                )
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time()
            )
        except Exception as exc:
            return f"Could not read the robot's position from TF: {exc}"

        t = transform.transform.translation
        r = transform.transform.rotation
        heading = math.degrees(quaternion_to_yaw(r.z, r.w, r.x, r.y))

        text = f"The robot is at x={t.x:.2f}, y={t.y:.2f} facing {heading:.0f} degrees."
        nearest = self.rooms.nearest(t.x, t.y)
        if nearest is not None:
            distance = math.hypot(nearest.x - t.x, nearest.y - t.y)
            text += f" Nearest module: {nearest.label} ({distance:.2f} m away)."
        return text

    # =============================================================== shutdown
    def shutdown(self) -> None:
        self._work_queue.put(None)


def _package_share_dir() -> str:
    """Installed share directory, falling back to the source tree if needed."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return get_package_share_directory("astra_llm_agent")
    except Exception:  # pragma: no cover - running straight from source
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _read_env_value(path: str, key: str) -> str:
    """Read KEY=value from a .env style file. Returns '' if absent or empty."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                name, _, value = line.partition("=")
                if name.strip() == key:
                    return value.strip().strip("'\"")
    except OSError:
        return ""
    return ""


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LlmNavAgent()
    except SystemExit as exc:      # missing key / bad rooms.yaml: no traceback
        print(str(exc), file=sys.stderr)
        rclpy.try_shutdown()
        return

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
