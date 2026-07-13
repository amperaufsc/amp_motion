#!/usr/bin/env python3
import rclpy
import can
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from can_classes.can_reader import StateCanReader
from rclpy.node import Node
from fs_msgs.msg import ControlCommand, GoSignal
from std_msgs.msg import Float32
from signals.signal_control import SignalsController
from longitudinal_control.PIDT_controller import PIDController
from longitudinal_control.PD_controller import PDController


class LowLevelControl(Node):
    def __init__(self):
        super().__init__('low_level_control')    
        self.subscription = self.create_subscription(ControlCommand, '/control', self.control_callback, 10)
        self.subscription = self.create_subscription(GoSignal, '/as_amp/mission_selected/go', self.go_callback, 10)     

        self.pub_sensor = self.create_publisher(Float32, 'sensor/value', 10)
        self.pub_control_overshoot = self.create_publisher(Float32, 'control/actual_value', 10)
        self.pub_control = self.create_publisher(Float32, 'control/anti_windup_value', 10)
        self.pub_control_error   = self.create_publisher(Float32, 'control/error', 10)
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

        self.ts  = 1/50
        self.kp = 50.0
        self.kd = 0.0*self.ts
        self.bias = 15.0                #valor do bias
        self.max_signal.data = 60.0     #limite superior do saturador
        self.min_signal.data = -60.0    #limite inferior do saturador
        self.sensor_max = 187.0       #max é na esquerda
        self.sensor_min = 93.0        #min é na direita
        #Quando o sinal de controle for positivo, as rodas esterçam para a esquerda do piloto

        self.control_setPoint = 0
        
        self.signals = SignalsController(self.left, self.right, self.pwm)
 
        self.pd = PDController(
            self.kp, self.kd, self.ts, self.bias, self.max_signal.data, self.min_signal.data)   
        
        self.can = StateCanReader()

        self.timer = self.create_timer(1/50, self.timer_callback)

        self.initialized = 0


    def control_callback(self, setPoint: ControlCommand):
        self.control_setPoint = -setPoint.steering
                
    def timer_callback(self):
        try:
            if self.initialized < 3:
                return
            message = self.can.can_listener.read_message()
            data = self.can.can_reader(message)        
            self.sensor.data = float(((2*(data - self.sensor_min)/(self.sensor_max - self.sensor_min)) - 1))
            
            self.control_overshoot.data, self.control.data, self.error.data = self.pd.update_signal(self.control_setPoint, self.sensor.data)

            self.pub_sensor.publish(self.sensor)
            self.pub_control.publish(self.control)
            self.pub_control_overshoot.publish(self.control_overshoot)
            self.pub_min.publish(self.min_signal)
            self.pub_max.publish(self.max_signal)
            self.pub_control_error.publish(self.error)
                
            if data > (self.sensor_max -4):
                limited_control = min(0.0, self.control.data)
                self.signals.steer(limited_control)
            elif data < (self.sensor_min +4):
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
setPoint: {self.control_setPoint}\nSensor: {self.sensor.data}, {data}''')
        self.sensor = Float32()        

    def go_callback(self, message):
        self.initialized += 1

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
