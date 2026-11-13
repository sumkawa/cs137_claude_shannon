#!/usr/bin/env python3
"""Frontier exploration node.

Listens to /map, /state and /nav_success, picks the closest valid frontier
state using the same heuristics as HW4 Problem 2, and sends it to the
navigator on /cmd_nav. Repeats until no frontier states are left.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.signal import convolve2d

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from asl_tb3_msgs.msg import TurtleBotState
from asl_tb3_lib.grids import StochOccupancyGrid2D


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        # parameters
        self.declare_parameter("window_size", 13)       # frontier window side length [cells]
        self.declare_parameter("min_goal_dist", 0.5)    # ignore frontiers closer than this [m]
        self.declare_parameter("fail_radius", 0.5)      # skip frontiers near goals that failed [m]

        self.occupancy = None
        self.state = None
        self.goal_active = False
        self.done = False
        self.failed_goals = []

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.state_sub = self.create_subscription(TurtleBotState, "/state", self.state_callback, 10)
        self.nav_success_sub = self.create_subscription(Bool, "/nav_success", self.nav_success_callback, 10)
        self.cmd_nav_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)

        # sends the first goal once map and state have arrived
        self.timer = self.create_timer(1.0, self.timer_callback)

    # ---------------- callbacks ----------------
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data,
        )

    def state_callback(self, msg: TurtleBotState) -> None:
        self.state = msg

    def nav_success_callback(self, msg: Bool) -> None:
        if not msg.data and self.last_goal is not None:
            self.get_logger().warn("Navigation failed, trying a different frontier")
            self.failed_goals.append(self.last_goal)
        self.goal_active = False
        self.send_next_goal()

    def timer_callback(self) -> None:
        if not self.goal_active and not self.done:
            self.send_next_goal()

    # ---------------- exploration ----------------
    def explore(self) -> np.ndarray:
        """Frontier states using the HW4 Problem 2 heuristics."""
        window_size = self.get_parameter("window_size").value
        kernel = np.ones((window_size, window_size))
        n = float(window_size ** 2)

        # ROS map data: -1 unknown, 0-100 occupancy probability in percent
        probs = self.occupancy.probs
        unknown = (probs == -1).astype(float)
        occupied = (probs >= 50).astype(float)
        free = ((probs >= 0) & (probs < 50)).astype(float)

        n_unknown = convolve2d(unknown, kernel, mode="same", boundary="fill", fillvalue=0)
        n_occupied = convolve2d(occupied, kernel, mode="same", boundary="fill", fillvalue=0)
        n_free = convolve2d(free, kernel, mode="same", boundary="fill", fillvalue=0)

        valid = (n_unknown / n >= 0.2) & (n_occupied == 0) & (n_free / n >= 0.3)
        rows, cols = np.where(valid)
        return self.occupancy.grid2state(np.stack([cols, rows], axis=1))

    def send_next_goal(self) -> None:
        if self.occupancy is None or self.state is None:
            return

        frontier = self.explore()
        current = np.array([self.state.x, self.state.y])

        # drop frontiers that are too close, or near goals that already failed
        if len(frontier) > 0:
            dists = np.linalg.norm(frontier - current, axis=1)
            keep = dists >= self.get_parameter("min_goal_dist").value
            for g in self.failed_goals:
                keep &= np.linalg.norm(frontier - g, axis=1) >= self.get_parameter("fail_radius").value
            frontier, dists = frontier[keep], dists[keep]

        if len(frontier) == 0:
            self.get_logger().info("No frontier states left: exploration finished")
            self.done = True
            self.last_goal = None
            return

        goal_xy = frontier[np.argmin(dists)]
        self.last_goal = goal_xy
        goal = TurtleBotState(x=float(goal_xy[0]), y=float(goal_xy[1]), theta=float(self.state.theta))
        self.cmd_nav_pub.publish(goal)
        self.goal_active = True
        self.get_logger().info(f"New frontier goal: ({goal.x:.2f}, {goal.y:.2f}), "
                               f"{len(frontier)} frontier states remaining")

    last_goal = None


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()