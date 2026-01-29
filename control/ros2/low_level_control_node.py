#!/usr/bin/env python3
import rclpy
import can
from can_bus1.can_reader import StateCanReader
from rclpy.node import Node
from fs_msgs.msg import ControlCommand
from signals.signal_control import SignalsController
from longitudinal_control.PID_controller import PIDController

class LowLevelControl(Node):
    def __init__(self):
        super().__init__('low_level_control')
        self.subscription = self.create_subscription(ControlCommand, '/control_command', self.control_callback, 10)
        self.pwm = SignalsController()
        self.kp = 20 
        self.ki = 10 
        self.kd = 10 
        self.t  = 
        self.Tt =  
        self.limit = 90
        self.angle = 0.0
        self.angular_velocity = 0
        self.pid = PIDController(self.kp, self.ki, self.kd, self.t, self.Tt, self.limit, 100.0, -100.0)
        self.can = StateCanReader()
        self.get_logger().info("Funciona")

    def control_callback(self, control: ControlCommand.steering):
        measure =self.can.can_reader["Estercamento_Atuador"]
        pwm = self.pid.update_signal(control, measure)
        self.pwm.steer(pwm)
        self.get_logger().info("Repetição")

def main(args=None):
    rclpy.init()
    low_level_control = LowLevelControl()
    rclpy.spin(low_level_control)
    rclpy.shutdown()


if __name__ == '__main__':
    main()