#!/usr/bin/env python3
import rclpy
from rclpy.timer import Timer
import can
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from can_classes.can_reader import StateCanReader
from rclpy.node import Node
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Float32
from signals.signal_control import SignalsController
from longitudinal_control.PIDT_controller import PIDController


class LowLevelControl(Node):
    def __init__(self):
        super().__init__('low_level_control')
        group = MutuallyExclusiveCallbackGroup()
        self.timer = Timer(callback=self.timer_callback, timer_period_ns=100000, callback_group=group, clock=)
        
        self.subscription = self.create_subscription(ControlCommand, '/control_command', self.control_callback, 10)

        self.pub_sensor = self.create_publisher(Float32, 'sensor/value', 10)
        self.pub_control_overshoot = self.create_publisher(Float32, 'control/actual_value', 10)
        self.pub_control = self.create_publisher(Float32, 'control/anti_windup_value', 10)
        self.pub_control_error = self.create_publisher(Float32, 'control/error', 10)
        self.pub_min = self.create_publisher(Float32, 'control/min_value', 10)
        self.pub_max = self.create_publisher(Float32, 'control/max_value', 10)

        self.sensor = Float32()
        self.control = Float32()
        self.control_overshoot = Float32()
        self.error = Float32()
        self.min_signal = Float32()
        self.max_signal = Float32()
        
        self.left = 15
        self.right = 13
        self.pwm = 18

        self.kp = 75.0
        self.ki = 0.20
        self.kd = 0
        self.k  = 1
        self.kt = 5.0
        self.t  = 0.01
        self.max_signal.data = 100.0
        self.min_signal.data = -100.0
        self.sensor_max = 120.0
        self.sensor_min = 40.0
        
        self.signals = SignalsController(self.left, self.right, self.pwm)
 
        self.pid = PIDController(
            self.kp, self.ki, self.kd, self.t, self.kt, self.max_signal.data, self.min_signal.data)   
        
        self.can = StateCanReader()

        self.get_logger().info("Funciona")

    def control_callback(self, reference: ControlCommand):
        self.control_reference = reference.steering
                
    def timer_callback(self):
        try:
            message = self.can.can_listener.read_message()
            data = self.can.can_reader(message)                
            self.sensor.data = float(((200*(data - self.sensor_min)/(self.sensor_max - self.sensor_min)) - 100))
            
            self.control_overshoot.data, self.control.data, self.error.data = self.pid.update_signal(self.control_reference, self.sensor.data)

            self.pub_sensor.publish(self.sensor)
            self.pub_control.publish(self.control)
            self.pub_control_overshoot.publish(self.control_overshoot)
            self.pub_min.publish(self.min_signal)
            self.pub_max.publish(self.max_signal)
            self.pub_control_error.publish(self.error)
                
            if data > 100:
                limited_control = min(0.0, self.control.data)
                self.signals.steer(limited_control)
            elif data < 60:
                limited_control = max(0.0, self.control.data)
                self.signals.steer(limited_control)      
            else:
                self.signals.steer(self.control.data)
                
                
        except Exception as e:
            self.get_logger().info(f"{e}")
        if message == None:
            self.signals.steer(0)
        else:
            self.get_logger().info(f'''
Control: {self.control.data, self.control_overshoot.data, self.error.data}
Reference: {self.control_reference}\nSensor: {self.sensor.data}''')
        self.sensor = Float32()        

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