#!/usr/bin/env python3
"""Small interactive console for talking to the ASTRA LLM navigation agent.

Convenience wrapper around

    ros2 topic pub --once /prompt std_msgs/msg/String "{data: '...'}"

so the demo can be driven by typing sentences. Replies published by the agent
on /llm_response are printed as they arrive.

    ros2 run astra_llm_agent prompt_cli
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


BANNER = """
=== ASTRA - natural language navigation ===
Type an instruction and press Enter. 'quit' or Ctrl-D to exit.
Examples:
  Go to the medical bay
  Navigate to the command module
  Go to the cargo bay
  Where can you go?
  Where is the robot?
"""


class PromptCli(Node):
    def __init__(self) -> None:
        super().__init__("astra_prompt_cli")
        self.declare_parameter("prompt_topic", "/prompt")
        self.declare_parameter("response_topic", "/llm_response")

        self._pub = self.create_publisher(
            String, self.get_parameter("prompt_topic").value, 10
        )
        self.create_subscription(
            String, self.get_parameter("response_topic").value, self._on_response, 10
        )

    def _on_response(self, msg: String) -> None:
        print(f"\n[ASTRA] {msg.data}\n> ", end="", flush=True)

    def send(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PromptCli()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(BANNER)
    try:
        while rclpy.ok():
            try:
                text = input("> ").strip()
            except EOFError:
                break
            if not text:
                continue
            if text.lower() in {"quit", "exit", "q"}:
                break
            node.send(text)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
