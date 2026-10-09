#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import Int64, Bool

from geometry_msgs.msg import Twist


class ConstantControl(Node):
    def __init__(self) -> None:
				# initialize base class (must happen before everything else)
        super().__init__("constant_control")  # initialize base class
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.c_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.motor_sub = self.create_subscription(Bool, "/kill",
                                                  self.kill_callback, 10)


    def timer_callback(self) -> None:
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        self.c_pub.publish(msg)
        self.get_logger().info("sending constant control...")

    def kill_callback(self, msg: Bool) -> None:
        if msg.data:
            self.timer.cancel()
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.c_pub.publish(stop)
				
        

if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = ConstantControl()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context