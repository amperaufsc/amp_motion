#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import Transition, TransitionEvent, State
from launch_ros.events.lifecycle import ChangeState
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from fs_msgs.msg import Motion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from rclpy.duration import Duration
import numpy as np
from fs_msgs.msg import ControlCommand
from lateral_control.kls_lateral_motion_controller import KLS_Lateral_Motion_Controller
from include.kls_lateral_motion_controller_gains import KLS_Lateral_Motion_Controller_Gains
from include.vehicle_parameters import Vehicle_Parameters
from include.vehicle_state import Vehicle_State
from longitudinal_control.longitudinal_controller import Longitudinal_Controller
from longitudinal_control.PID_controller import PIDController
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from lifecycle_msgs.srv import GetState, ChangeState

import sys

class ControlNode(LifecycleNode):
    def __init__(self):
        super().__init__('control_node')
        #self.declare_parameter('T', 0.5)
        T = 0.01   #float(self.get_parameter('T').value)

        self.get_logger().info('Control started')

        #self.reference_path = 0
        self.speed_reference = 2.5
        self.received_path= False
        self.received_odom = False
        self.closest_index = 0
        self.timer = None 
        self.transition_failure = self.create_publisher(TransitionEvent, '/EbsNOTMissionFinished', 10)
    
    def change_own_state(self, transition_id): #obrigado chatgpt
        client = self.create_client(ChangeState, f'/{self.node_name}/change_state')

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Serviço de mudança de estado não está disponível.")
            return

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Transição feita com sucesso: {future.result().success}")
        else:
            self.get_logger().error("Falha ao fazer a transição de estado.")       
    
    def on_failure(self):
        event_failure = TransitionEvent()
        self.transition_failure.publish(event_failure)
        
    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info('SHUTDOWN Control')
        return TransitionCallbackReturn.SUCCESS
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring Control...')

        try:
        # Declaração dos parâmetros
            self.declare_parameter('Kp', 0.05)
            self.declare_parameter('Ki', 0.01)
            self.declare_parameter('Kd', 0.0)
            self.declare_parameter('T', 0.01)

        # Recuperação dos parâmetros
            Kp = float(self.get_parameter('Kp').value)
            Ki = float(self.get_parameter('Ki').value)
            Kd = float(self.get_parameter('Kd').value)
            T = 0.01 
            self.timer_period = 0.01
            self.timer = None

            self.speed_reference = 2.5
            vehicle_parameters = Vehicle_Parameters(1, np.radians(35), np.radians(-35))
            kls_lateral_motion_controller_gains = KLS_Lateral_Motion_Controller_Gains(2.5, 1.0)
            self.control = KLS_Lateral_Motion_Controller(vehicle_parameters, kls_lateral_motion_controller_gains)
            self.longitudinal_controller = Longitudinal_Controller(Kp, Ki, Kd, T, self.speed_reference)

            self.PID_Controller = PIDController(Kp, Ki, Kd, T)

            # Criando os publishers
            self.publisher_ = self.create_publisher(ControlCommand, 'control', 10)
            self.speed_publisher_ = self.create_publisher(Float32, '/speed', 10)
            self.erro_ant_publisher_ = self.create_publisher(Float32, '/erro_ant', 10)
            self.eh_publisher_ = self.create_publisher(Float32, '/eh', 10)
            self.ey_publisher_ = self.create_publisher(Float32, '/ey', 10)
            self.path_publisher_ = self.create_publisher(Path, '/reference_path', 10)

            self.subscription = self.create_subscription(Motion, '/path', self.motion_callback, 10)
            self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
            
            # Publisher resposnavel pela ativação dos leds do carro baseado em seu estado
            self.led_pub = self.create_publisher(String, '/AMP/as_status_indicator', 10)
            self.led_msg = String()

            self.get_logger().info('Configuração Control concluída com sucesso.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
                self.get_logger().error(f'Erro durante on_configure: {e}')
                return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating ControlNode...')
    
        event = TransitionEvent()
        #self.transition_event_pub.publish(event)

        try:

            # Criando o timer
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            self.get_logger().info('ControlNode ativado com sucesso.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Erro durante on_activate: {e}')
            return TransitionCallbackReturn.FAILURE
        
    def motion_callback(self, motion_msg):
        self.path = []
        self.timestamp = []
        self.speed_profile = []
        for odom in motion_msg.odom:
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            speed = odom.twist.twist.linear.x
            time_stamp = odom.header.stamp
            time_stamp_float = time_stamp.sec + time_stamp.nanosec * 1e-9
            self.path.append([x, y]) 
            self.timestamp.append([time_stamp_float])
            self.speed_profile.append(speed)
            
        self.path = np.array(self.path)
        self.timestamp = np.array(self.timestamp)
        self.speed_profile = np.array(self.speed_profile)

        self.received_path = True
        self.get_logger().info('Path')
    
    def odom_callback(self, msg):

        self.get_logger().info('Odom Received: ')
        self.received_odom = True
        car_position_x = msg.pose.pose.position.x
        car_position_y = msg.pose.pose.position.y
        self.odom_timestamp = msg.header.stamp
        self.odom_time_stamp_float = self.odom_timestamp.sec + self.odom_timestamp.nanosec * 1e-9
        #self.car_pose = Vehicle_Pose(car_position_x, car_position_y, yaw)
        
        self.position = np.array([car_position_x, car_position_y])
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        yaw = np.arctan2(2*(orientation_q.w*orientation_q.z - orientation_q.x*orientation_q.y), 1 - 2*((orientation_q.y)**2 + (orientation_q.z)**2))
        
        self.body_linear_velocity_x = msg.twist.twist.linear.x
        self.body_linear_velocity_y = msg.twist.twist.linear.y
        
        self.vehicle_state = Vehicle_State(self.position[0], self.position[1], yaw, self.body_linear_velocity_x, self.body_linear_velocity_y, 0)

    def timer_callback(self):
        try:    
            if self.received_odom:                
                speed = ((self.body_linear_velocity_x)**2 + (self.body_linear_velocity_y)**2)**0.5
                    #self.get_logger().info('Speed: "%f"' %speed)
                speed_msg = Float32()
                speed_msg.data = speed

                erro_ant = self.longitudinal_controller.erro
                erro_ant_msg = Float32()
                erro_ant_msg.data = erro_ant

                eh = self.control.eh
                eh_msg = Float32()
                eh_msg.data = eh

                ey = self.control.ey
                ey_msg = Float32()
                ey_msg.data = ey


                if self.received_path:
                    self.closest_index += np.argmin([np.linalg.norm(i) for i in (self.path[self.closest_index:self.closest_index + 10] - self.position)])

                    self.reference_path = self.path[self.closest_index:]

                    if self.closest_index < len(self.speed_profile):
                        self.speed_reference = self.speed_profile[self.closest_index + 1]
                        
                    self.longitudinal_controller = Longitudinal_Controller(0.05, 0.01, 0.0, 0.01, self.speed_reference)

                    self.get_logger().info('speed_reference: "%f"' %self.speed_reference)

                        
                    self.path_publishing(self.reference_path)
                    steering_command = - self.control.update_steering_angle_control_signal(self.reference_path, self.vehicle_state)
                    throttle_command, brake_command = self.longitudinal_controller.update_torque_control_signal(self.path, self.vehicle_state)
                        #self.get_logger().debug('Steering: "%f"' %steering_command)
                        #self.get_logger().debug('Throttle: "%f"' %throttle_command)

                    msg = ControlCommand()
                    msg.steering = steering_command
                    msg.throttle = throttle_command
                    msg.brake = brake_command 

                        #self.get_logger().info("Control received")

                        
                    self.publisher_.publish(msg)
                    self.speed_publisher_.publish(speed_msg)
                    self.erro_ant_publisher_.publish(erro_ant_msg)
                    self.eh_publisher_.publish(eh_msg)
                    self.ey_publisher_.publish(ey_msg)
        except:
            event = TransitionEvent()

            self.transition_failure.publish(event) # Muda o estado do carro pra Emergency
            self.led_msg.data = 'as_emergency'
            self.led_pub.publish(self.led_msg) # Publica mensagem de Emergency pra ativação do LED
            
            sys.exit(1) # Mata o processo com chamada de sistema
            


            #    else:
            #        self.get_logger().warn("Path not received")
            #else:
            #    self.get_logger().warn("Odom not received")

    def path_publishing(self, np_array_path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "fsds/map"
        poses = []
        for i, point in enumerate(np_array_path):
            pose = PoseStamped()
            #pose.header.stamp = path_msg.header.stamp + dt_duration
            pose.header.frame_id = "fsds/map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            #self.get_logger().info('x: "%f"' %point[0])
            #self.get_logger().info('y: "%f"' %point[1])
            poses.append(pose)
            
        #self.get_logger().info('Publishing')
        path_msg.poses = poses 
        self.path_publisher_.publish(path_msg)

    def change_own_state(self, transition_id): #obrigado chatgpt
        client = self.create_client(ChangeState, f'/{self.node_name}/change_state')

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Serviço de mudança de estado não está disponível.")
            sys.exit(1)
            return

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Transição feita com sucesso: {future.result().success}")
        else:
            self.get_logger().error("Falha ao fazer a transição de estado.")    


def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
