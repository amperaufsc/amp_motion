#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fs_msgs.msg import ControlCommand
from signals.signal_control import SignalsController
from longitudinal_control.PID_controller import PIDController

class LowLevelControl(Node):
    def __init__(self):
        super().__init__('low_level_control')
        self.subscription = self.create_subscription(ControlCommand, '/control_command', self.control_callback, 10)
        self.pwm = SignalsController()
        self.get_logger().info("Funciona")

    def control_callback(self, control: ControlCommand.steering):
        self.pwm.change_duty(control)
        self.get_logger().info("Repetição")

def main(args=None):
    rclpy.init()
    low_level_control = LowLevelControl()
    rclpy.spin(low_level_control)
    rclpy.shutdown()


if __name__ == '__main__':
    main()