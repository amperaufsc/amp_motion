#!/usr/bin/env python3
import rclpy
import can
from can_classes.can_reader import StateCanReader
from rclpy.node import Node
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Int16, Float32
from signals.signal_control import SignalsController
from longitudinal_control.PIDT_controller import PIDController


class LowLevelControl(Node):
    def __init__(self):
        super().__init__('low_level_control')
        
        self.subscription = self.create_subscription(ControlCommand, '/control_command', self.control_callback, 10)

        self.pub_sensor = self.create_publisher(Float32, 'sensor', 10)
        self.pub_control_overshoot = self.create_publisher(Float32, 'control_overshoot', 10)
        self.pub_control = self.create_publisher(Float32, 'control', 10)
        self.pub_min = self.create_publisher(Int16, 'min', 10)
        self.pub_max = self.create_publisher(Int16, 'max', 10)

        self.sensor = Float32()
        self.control = Float32()
        self.lcontrol = Float32()
        self.control_overshoot = Float32()
        self.min_signal = Int16()
        self.max_signal = Int16()
        
        self.left = 15
        self.right = 13
        self.pwm = 18

        self.kp = 0.75
        self.ki = 0.20
        self.kd = 0.0005
        self.k  = 1
        self.kt = 0.0005
        self.t  = 0.1
        self.max_signal.data = 100
        self.min_signal.data = -100
        
        self.signals = SignalsController(self.left, self.right, self.pwm)
 
        self.pid = PIDController(
            self.kp, self.ki, self.kd, self.t, self.kt, self.max_signal.data, self.min_signal.data)   
        
        self.can = StateCanReader()

        self.get_logger().info("Funciona")

    def control_callback(self, reference: ControlCommand):
        try:
            message = self.can.can_listener.read_message()
            data = self.can.can_reader(message)                
            self.sensor.data = data*2/255 - 1

            self.control_overshoot.data, self.control.data = self.pid.update_signal(reference.steering, self.sensor.data)

            self.pub_sensor.publish(self.sensor)
            self.pub_control.publish(self.control)
            self.pub_control_overshoot.publish(self.control_overshoot)
            self.pub_min.publish(self.min_signal)
            self.pub_max.publish(self.max_signal)
            if data > 155:
                self.lcontrol.data = max(0.0, self.control.data)
                self.signals.steer(self.lcontrol.data)
            elif data < 100:
                self.lcontrol.data = min(0.0, self.control.data)
                self.signals.steer(self.lcontrol.data)
            else:
                self.signals.steer(self.control.data)


            
        except Exception as e:
            self.get_logger().info(f"{e}")
        if message != None:
            self.get_logger().info(f"A mensagem é: {self.control.data} <-> {reference.steering} <-> {data} <-> {(data, self.sensor.data)}")
            

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
        low_level_control.can.can_listener.bus.shutdown()


if __name__ == '__main__':
    main()