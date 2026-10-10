#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import Int64, Bool, String

class Coordinate:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Twist:
    def __init__(self):
        self.angular = Coordinate(0, 0, 0)
        self.linear = Coordinate(0, 0, 0)

class Heartbeat(Node):
    def __init__(self) -> None:
				# initialize base class (must happen before everything else)
        super().__init__("heartbeat")  # initialize base class
				
				# a heartbeat counter
				self.hb_msg = "sending constant control"
                self.twist_msg = Twist()

				# create publisher with
				#   self.create_publisher(<msg type>, <topic>, <qos>)

        self.hb_pub = self.create_publisher(String, "/heartbeat", 10)

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 9)
        
        # create a timer
				#   self.create_timer(<second>, <callback>)
        self.hb_timer = self.create_timer(1.0, self.hb_callback)

        self.twist_pub_timer = self.create_timer(0.5, self.twist_pub_callback)

				# create subscription with
				#   self.create_subscription(<msg type>, <topic>, <callback>, <qos>)
        self.motor_sub = self.create_subscription(String, "/health/motor",
                                                  self.health_callback, 10)

        self.twist_sub = self.create_subscription(String, "/health/motor",
                                                  self.twist_listener, 10)

    def hb_callback(self) -> None:
				""" heartbeat callback triggered by the timer """
        # construct heartbeat message
				msg = String()
        msg.data = self.hb_msg
        # publish heartbeat counter
        self.hb_pub.publish(msg)
		# counter increment

    def hb_pub_callback(self) -> None:
                """ heartbeat callback triggered by the timer """
        # construct heartbeat message
                msg = Twist()
        msg.data = self.twist_msg
        # publish heartbeat counter
        self.twist_pub.publish(msg)
        # counter increment

    def twist_listener(self, msg:Twist):
        if msg.data:
            print(msg.data)


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