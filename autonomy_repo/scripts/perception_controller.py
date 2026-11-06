#!/usr/bin/env python3

import rclpy
from asl_tb3_lib.control import BaseController
from asl_tb3_msgs.msg import TurtleBotControl
from std_msgs.msg import Bool

STOP_DURATION = 5.0
OMEGA = 0.5
IGNORE_DURATION = 3.0

class PerceptionController(BaseController):
    def __init__(self) -> None:
        super().__init__("perception_controller")
        self.declare_parameter("active", True)

        self.stop_time = None
        self.resume_time = None
        self.detector_sub = self.create_subscription(Bool, "detector_bool", self.detector_callback, 10)
    @property
    def active(self) -> bool:
        return self.get_parameter("active").value
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9
    def detector_callback(self, msg: Bool) -> None:
        if not msg.data or not self.active:
            return
        # if self.resume_time is not None and (self.now_sec() - self.resume_time) < IGNORE_DURATION:
        #     return
        self.set_parameters([rclpy.Parameter("active", value=False)])
        
    def compute_control(self) -> TurtleBotControl:
        control = TurtleBotControl()
        now = self.now_sec()

        if self.active:
            self.stop_time = None
            control.omega = OMEGA
            return control

        if self.stop_time is None:
            self.stop_time = now

        if now - self.stop_time < STOP_DURATION:
            control.omega = 0.0
        else:
            self.get_logger().info("Resuming, ignoring detections for 3s")
            self.set_parameters([rclpy.Parameter("active", value=True)])
            self.stop_time = None
            self.resume_time = now
            control.omega = OMEGA
        
        return control

if __name__ == "__main__":
    rclpy.init()
    node = PerceptionController()
    rclpy.spin(node)
    rclpy.shutdown()