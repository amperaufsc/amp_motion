#!/usr/bin/env python3
import rclpy
import can
from can_classes.can_reader import StateCanReader
from rclpy.node import Node
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Int16
from signals.signal_control import SignalsController
from longitudinal_control.PIDT_controller import PIDController

class LowLevelControl(Node):
    def __init__(self):
        super().__init__('low_level_control')
        self.subscription = self.create_subscription(ControlCommand, '/control_command', self.control_callback, 10)

        self.publisher = self.create_publisher(Int16, 'control', 10)

        self.left = 15
        self.right = 13
        self.pwm = 18

        self.kp = 100
        self.ki = 30
        self.kd = 10
        self.k  = 1
        self.kt = 10 
        self.t  = 0.1
        self.max_signal = 100
        self.min_signal = -100

        self.signals = SignalsController(self.left, self.right, self.pwm)
 
        self.pid = PIDController(self.kp, self.ki, self.kd, self.t, self.kt, self.max_signal, self.min_signal)   
        
        self.can = StateCanReader()

        self.get_logger().info("Funciona")

    def control_callback(self, reference: ControlCommand):
        try:
            #message = self.can.can_listener.read_message()
            #measure = self.can.can_reader(message)*2/255 - 1
            #pwm = self.pid.update_signal(reference.steering, measure)
            self.signals.steer(reference.steering)
        except Exception as e:
            self.get_logger().info(f"{e}")
        #if message != None:
            self.get_logger().info(f"A mensagem é: {0} <-> {reference.steering} <-> {0}")

def main(args=None):
    rclpy.init()
    low_level_control = LowLevelControl()
    try:
        rclpy.spin(low_level_control)
    except:
        low_level_control.signals.shutdown()

    finally:
        low_level_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()