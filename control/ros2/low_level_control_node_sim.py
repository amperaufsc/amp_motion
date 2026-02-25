#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fs_msgs.msg import ControlCommand
from signals.signal_control import SignalsController
import can
from longitudinal_control.PID_controller import PIDController

class LowLevelControlSim(Node):
    def __init__(self):
        super().__init__('low_level_control')
        self.subscription = self.create_subscription(ControlCommand, '/control_command', self.control_callback, 10)
        self.kp = 100
        self.ki = 30
        self.kd = 10
        self.k  = 1
        self.t  = 0.1
        self.Tt = 10 
        self.angle = 0.0
        self.signals = SignalsController(15, 13, 18)
        self.angular_velocity = 0
        self.pid = PIDController(self.kp, self.ki, self.kd, self.t, self.Tt, 100.0,-100.0)
        self.get_logger().info("Funciona")

    def control_callback(self, control: ControlCommand):
        for _ in range(int(1 /self.t)):
            pwm = self.pid.update_signal(1, self.angle)
            self.signals.steer(pwm)
            voltage = pwm * 0.05
            self.angular_velocity += voltage * self.k * self.t
            self.angle += self.angular_velocity
            self.get_logger().info(f"oi {self.angle} | {control.steering} | {pwm}")

def main(args=None):
    rclpy.init()
    low_level_control = LowLevelControlSim()
    rclpy.spin(low_level_control)
    rclpy.shutdown()


if __name__ == '__main__':
    main()