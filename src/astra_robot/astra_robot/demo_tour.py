#!/usr/bin/env python3
"""
demo_tour.py - drives the robot to every module in sequence.

Good for the demo video and for proving all six locations are reachable
(a "Definition of done" checklist item). Run after Nav2 is up:

    ros2 run astra_robot demo_tour

Or pick a subset / order:

    ros2 run astra_robot demo_tour --route airlock engine_room cargo_bay
"""

import argparse
import sys

import rclpy

from astra_robot.astra_navigator import (
    AstraNavigator, LOCATIONS, spin_in_background,
)

DEFAULT_ROUTE = [
    'sleeping_quarters',
    'airlock',
    'engine_room',
    'medical_bay',
    'cargo_bay',
    'command_module',
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--route', nargs='*', default=DEFAULT_ROUTE,
                        help='Ordered location names to visit.')
    args, _ = parser.parse_known_args()

    unknown = [r for r in args.route if r not in LOCATIONS]
    if unknown:
        print(f'Unknown locations: {unknown}')
        print(f'Known: {sorted(LOCATIONS.keys())}')
        sys.exit(1)

    rclpy.init()
    node = AstraNavigator()
    executor, _thread = spin_in_background(node)

    results = []
    try:
        for name in args.route:
            node.get_logger().info(f'=== Tour leg: {name} ===')
            ok = node.go_to(name)
            results.append((name, ok))
            if not ok:
                node.get_logger().warn(f'Leg failed: {name}; continuing.')
    except KeyboardInterrupt:
        pass
    finally:
        print('\n===== TOUR SUMMARY =====')
        for name, ok in results:
            print(f'  {name:20s} {"OK" if ok else "FAILED"}')
        failed = [n for n, ok in results if not ok]
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
