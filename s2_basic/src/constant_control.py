#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import Int64, Bool, String
from geometry_msgs.msg import Twist


class Heartbeat(Node):
    def __init__(self) -> None:
		# initialize base class (must happen before everything else)
        super().__init__("heartbeat")  # initialize base class

		# a heartbeat counter
        self.hb_counter = 0

		# create publisher with
		#   self.create_publisher(<msg type>, <topic>, <qos>)
        self.hb_pub = self.create_publisher(Int64, "/heartbeat", 10)

        # create a 0.2 sec timer
        self.constant_control_timer = self.create_timer(0.2, self.constant_control_callback)
        self.constant_control_pub = self.create_publisher(String, "/constant_control", 10)

        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # create a timer
		#   self.create_timer(<second>, <callback>)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)

		# create subscription with
		#   self.create_subscription(<msg type>, <topic>, <callback>, <qos>)
        self.motor_sub = self.create_subscription(Bool, "/health/motor",
                                                  self.health_callback, 10)
        self.kill_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)

    def hb_callback(self) -> None:
        """ heartbeat callback triggered by the timer """
        # construct heartbeat message
        msg = Int64()
        msg.data = self.hb_counter

        # publish heartbeat counter
        self.hb_pub.publish(msg)

		# counter increment
        self.hb_counter += 1

    def kill_callback(self, msg) -> None:
        if msg.data:
            self.constant_control_timer.cancel()
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.twist_pub.publish(stop_msg)
            self.get_logger().info("kill received")

    def constant_control_callback(self) -> None:
        msg = Twist()
        msg.linear.x = 0.8
        msg.angular.z = 2.0

        self.twist_pub.publish(msg)

    def health_callback(self, msg: Bool) -> None:
        """ sensor health callback triggered by subscription """
        if not msg.data:
            self.get_logger().fatal("Heartbeat stopped")
            self.hb_timer.cancel()


if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = Heartbeat()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context
