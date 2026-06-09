#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
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
from lifecycle_msgs.msg import Transition, TransitionEvent, State
from lifecycle_msgs.srv import GetState, ChangeState
from fs_msgs.msg import Motion
import sys
from std_msgs.msg import String

class ControlNode(LifecycleNode):
    def __init__(self):
        super().__init__('control_node')
        self.get_logger().info("CONTROL NODE INICIALIZADO")
        
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:

            self.subscription = self.create_subscription(Path, 'path', self.path_callback, 10)
            self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

            self.publisher_ = self.create_publisher(ControlCommand, 'control', 10)
            self.speed_publisher_ = self.create_publisher(Float32, '/speed', 10)
            self.erro_ant_publisher_ = self.create_publisher(Float32, '/erro_ant', 10)
            self.eh_publisher_ = self.create_publisher(Float32, '/eh', 10)
            self.ey_publisher_ = self.create_publisher(Float32, '/ey', 10)
            self.path_publisher_ = self.create_publisher(Path, 'reference_path', 10)
            #self.declare_parameter('T', 0.5)
            self.index = 0

            self.declare_parameter('Kp', 0.05)
            self.declare_parameter('Ki', 0.01)
            self.declare_parameter('Kd', 0.0)
            self.declare_parameter('T', 0.1)

            Kp = float(self.get_parameter('Kp').value)
            Ki = float(self.get_parameter('Ki').value)
            Kd = float(self.get_parameter('Kd').value)
            
            self.get_logger().info('Kp:"%f"' %Kp)
            self.get_logger().info('Ki:"%f"' %Ki)
            self.get_logger().info('Kd:"%f"' %Kd)

            vehicle_parameters = Vehicle_Parameters(1, np.radians(35), np.radians(-35))
            kls_lateral_motion_controller_gains = KLS_Lateral_Motion_Controller_Gains(1.5, 1.2)
            self.control = KLS_Lateral_Motion_Controller(vehicle_parameters, kls_lateral_motion_controller_gains)
            T = 0.01#float(self.get_parameter('T').value)
            self.longitudinal_controller = Longitudinal_Controller(Kp, Ki, Kd, T, 1)
            self.PID_Controller = PIDController(Kp, Ki, Kd, T)
            self.received_path= False
            self.received_odom = False
            self.closest_index = 0

            self.transition_pub_emergency = self.create_publisher(TransitionEvent, "EbsNOTMissionFinished", 10)
                            # Publisher resposnavel pela ativação dos leds do carro baseado em seu estado
            self.led_pub = self.create_publisher(String, '/AMP/as_status_indicator', 10)
            self.led_msg = String()

            self.get_logger().info("CONTROL CONFIGURADO")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            import traceback
            self.get_logger().error(f"FALHA AO CONFIGURAR O CONTROL: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            T = 0.01#float(self.get_parameter('T').value)
            self.timer = self.create_timer(T, self.timer_callback)
            self.get_logger().info("CONTROL ATIVADO")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().info("FALHA AO ATIVAR O CONTROL")
            return TransitionCallbackReturn.FAILURE
    
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self.get_logger().info("CONTROL DESATIVADO")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().info("FALHA AO DESATIVAR O CONTROL")
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
        try:
            if self.received_odom:                
                #speed = ((self.body_linear_velocity_x)**2 + (self.body_linear_velocity_y)**2)**0.5
                speed = self.body_linear_velocity_x
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
                    self.closest_index = np.argmin([np.linalg.norm(i) for i in (self.path[self.closest_index:self.closest_index + 10] - self.position)])
                    self.get_logger().info('index: "%f"' %self.closest_index)

                    self.reference_path = self.path[self.closest_index:]
                    if self.closest_index < len(self.speed_profile):
                        self.speed_reference = self.speed_profile[self.closest_index]
    #                 '''if self.closest_index < len(self.speed_profile):
    #                     self.speed_reference = self.speed_profile[self.closest_index + 1]'''
            
            
    # '''175-control-bia-antunes
                    
    #                 self.speed_reference = 4.0
                    
    #                 self.longitudinal_controller = Longitudinal_Controller(0.8, 0.08, 0.0, 0.01, self.speed_reference)'''

                    
                    self.speed_reference = 4.0
                    self.longitudinal_controller = Longitudinal_Controller(0.05, 0.01, 0.0, 0.1, self.speed_reference)


                    #self.get_logger().info('speed_reference: "%f"' %self.speed_reference)
                    self.get_logger().info('speed_reference: "%f"' %self.speed_reference)
                    
                    '''
                    
                    - - -----  OLD VERSION OF CONTROL TIMER_CALLBACK ----- - - 
                    
                    self.closest_index = np.argmin(np.abs(self.timestamp - self.odom_time_stamp_float))
                    self.closest_index = np.argmin([np.linalg.norm(i) for i in (self.path[self.closest_index:self.closest_index + 10] - self.position)]) + self.closest_index
                    self.get_logger().info('time: "%d"' %self.closest_index)
                    self.reference_path = self.path[self.closest_index:]
                    self.path = self.path[closest_arg:]
                    self.path = self.path[closest_time_index + 90:]
                    self.reference_path = self.path[closest_time_index:]
                    self.get_logger().info('time: "%d"' %closest_time_index)
                    '''
                    
                    self.path_publishing(self.reference_path)
                    steering_command = - self.control.update_steering_angle_control_signal(self.reference_path, self.vehicle_state)
                    throttle_command, brake_command = self.longitudinal_controller.update_torque_control_signal(self.path, self.vehicle_state)

                    #self.get_logger().debug('Steering: "%f"' %steering_command)
                    self.get_logger().debug('Throttle: "%f"' %throttle_command)
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
            else:
                #self.get_logger().warn("Odom not received")
                a="string so pro else nao dar merda e o print n ficar enchendo meu saco.pode tirar"
        except Exception as e:
            event = TransitionEvent()
            self.transition_pub_emergency(event) # Muda o estado do carro pra Emergency
            self.led_msg.data = 'as_emergency'
            self.led_pub.publish(self.led_msg) # Publica mensagem de Emergency pra ativação do LED
            sys.exit(1) # Mata o processo com chamada de sistema

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