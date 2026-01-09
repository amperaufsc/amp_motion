#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
import numpy as np
from fs_msgs.msg import ControlCommand
from lateral_control.kls_lateral_motion_controller import KLS_Lateral_Motion_Controller
from include.kls_lateral_motion_controller_gains import KLS_Lateral_Motion_Controller_Gains
from include.vehicle_parameters import Vehicle_Parameters
from include.vehicle_state import Vehicle_State
from longitudinal_control.longitudinal_controller import Longitudinal_Controller
from longitudinal_control.PID_controller import PIDController
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from lifecycle_msgs.msg import Transition, TransitionEvent, State
from lifecycle_msgs.srv import GetState, ChangeState
from fs_msgs.msg import Motion
import sys

class ControlNode(LifecycleNode):
    def __init__(self):
        super().__init__('control_publisher')

        #self.declare_parameter('T', 0.5)
        self.index = 0
        T = 0.01#float(self.get_parameter('T').value)

        self.get_logger().info('Control started')

        self.received_path= False
        self.received_odom = False
        self.closest_index = 0

        
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
            self.longitudinal_controller = Longitudinal_Controller(Kp, Ki, Kd, T, 0) # MUDAR PARA REFERENCIA DE VELOCIDADE CERTA PORRAAAAAAAAAAAAA

            self.PID_Controller = PIDController(Kp, Ki, Kd, T)

            # Criando os publishers
            self.publisher_ = self.create_publisher(ControlCommand, 'control', 10)
            self.speed_publisher_ = self.create_publisher(Float32, '/speed', 10)
            self.erro_ant_publisher_ = self.create_publisher(Float32, '/erro_ant', 10)
            self.eh_publisher_ = self.create_publisher(Float32, '/eh', 10)
            self.ey_publisher_ = self.create_publisher(Float32, '/ey', 10)
            self.path_publisher_ = self.create_publisher(Path, '/reference_path', 10)

            self.subscription = self.create_subscription(Path, 'path', self.path_callback, 10)
            self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
            
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
            T = 0.01
            self.timer = self.create_timer(T, self.timer_callback)
            self.get_logger().info('ControlNode ativado com sucesso.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Erro durante on_activate: {e}')
            return TransitionCallbackReturn.FAILURE
            
    def path_callback(self, path_msg):
        #self.get_logger().info('Path Received: ')
        self.path = []
        self.timestamp = []
        for poses in path_msg.poses:
            x = poses.pose.position.x
            y = poses.pose.position.y 
            time_stamp = poses.header.stamp
            time_stamp_float = time_stamp.sec + time_stamp.nanosec * 1e-9
            self.path.append([x, y]) 
            self.timestamp.append([time_stamp_float])
        self.path = np.array(self.path)
        self.timestamp = np.array(self.timestamp)

        self.received_path = True
        self.get_logger().info('Path')
    
    def odom_callback(self, msg):
        # self.get_logger().info('Odom Received: ')
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
                #self.closest_index = np.argmin(np.abs(self.timestamp - self.odom_time_stamp_float))
                self.closest_index = np.argmin([np.linalg.norm(i) for i in (self.path[self.closest_index:self.closest_index + 10] - self.position)]) + self.closest_index
                self.get_logger().info('time: "%d"' %self.closest_index)
                self.reference_path = self.path[self.closest_index:]
                #self.path = self.path[closest_arg:]
                #self.path = self.path[closest_time_index + 90:]
                #self.reference_path = self.path[closest_time_index:]
                #self.get_logger().info('time: "%d"' %closest_time_index)
                
                
                self.path_publishing(self.reference_path)
                steering_command = - self.control.update_steering_angle_control_signal(self.reference_path, self.vehicle_state)
                throttle_command, brake_command = self.longitudinal_controller.update_torque_control_signal(self.path, self.vehicle_state)
                self.get_logger().debug('Steering: "%f"' %steering_command)
                self.get_logger().debug('Throttle: "%f"' %throttle_command)


                msg = ControlCommand()
                msg.steering = steering_command
                msg.throttle = throttle_command
                msg.brake = brake_command 

                self.publisher_.publish(msg)
                self.speed_publisher_.publish(speed_msg)
                self.erro_ant_publisher_.publish(erro_ant_msg)
                self.eh_publisher_.publish(eh_msg)
                self.ey_publisher_.publish(ey_msg)
            else:
                self.get_logger().warn("Path not received")
        #else:
            #self.get_logger().warn("Odom not received")

    def path_publishing(self, np_array_path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        poses = []
        for i, point in enumerate(np_array_path):
            pose = PoseStamped()
            #pose.header.stamp = path_msg.header.stamp + dt_duration
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            poses.append(pose)
            
        #self.get_logger().info('Publishing')
        path_msg.poses = poses
        self.path_publisher_.publish(path_msg)

def main(args=None):
    rclpy.init()
    control_publisher = ControlNode()
    rclpy.spin(control_publisher)
    control_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()