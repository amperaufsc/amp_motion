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
import yaml

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.subscription = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.publisher_ = self.create_publisher(ControlCommand, 'control', 10)
        self.speed_publisher_ = self.create_publisher(Float32, '/speed', 10)
        self.erro_ant_publisher_ = self.create_publisher(Float32, '/erro_ant', 10)
        self.eh_publisher_ = self.create_publisher(Float32, '/eh', 10)
        self.ey_publisher_ = self.create_publisher(Float32, '/ey', 10)
        self.path_publisher_ = self.create_publisher(Path, 'reference_path', 10)

        # difinição dos parametros que estão no arquivo yaml (control/config/control_parameters.yaml)
        self.declare_parameter('Kp', 0.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.0)
        self.declare_parameter('Key', 0.0)
        self.declare_parameter('Keh', 0.0)
        self.declare_parameter('speed', 0.0)
        self.declare_parameter('sampling_period', 0.01)
        self.declare_parameter('throttle_max', 1.0)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.Key = self.get_parameter('Key').value
        self.Keh = self.get_parameter('Keh').value
        self.speed = self.get_parameter('speed').value
        self.T = self.get_parameter('sampling_period').value
        self.throttle_max = self.get_parameter('throttle_max').value

        self.timer = self.create_timer(float(self.T), self.timer_callback)
        self.index = 0
        self.get_logger().info('Control started')
        
        vehicle_parameters = Vehicle_Parameters(1, np.radians(35), np.radians(-35)) #axle_length - steering_up_limit - steering_down_limit

        kls_lateral_motion_controller_gains = KLS_Lateral_Motion_Controller_Gains(self.Key, self.Keh) #lateral error gain - orientation error gain
        self.control = KLS_Lateral_Motion_Controller(vehicle_parameters, kls_lateral_motion_controller_gains)
        self.longitudinal_controller = Longitudinal_Controller(self.Kp, self.Ki, self.Kd, self.T, self.speed, max_signal=self.throttle_max)
        #self.PID_Controller = PIDController(Kp, Ki, Kd, T)
        self.received_path= False
        self.received_odom = False
        self.closest_index = 0
        
        
    def path_callback(self, path_msg):
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
        self.received_odom = True
        car_position_x = msg.pose.pose.position.x
        car_position_y = msg.pose.pose.position.y
        self.odom_timestamp = msg.header.stamp
        self.odom_time_stamp_float = self.odom_timestamp.sec + self.odom_timestamp.nanosec * 1e-9
        
        self.position = np.array([car_position_x, car_position_y])

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        yaw = np.arctan2(2*(orientation_q.w*orientation_q.z - orientation_q.x*orientation_q.y), 1 - 2*((orientation_q.y)**2 + (orientation_q.z)**2))
        
        self.body_linear_velocity_x = msg.twist.twist.linear.x
        self.body_linear_velocity_y = msg.twist.twist.linear.y
        
        self.vehicle_state = Vehicle_State(self.position[0], self.position[1], yaw, self.body_linear_velocity_x, self.body_linear_velocity_y, 0)
    

    def timer_callback(self):
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
                #Procurar apenas em uma janela (20) ao redor do último ponto conhecido.
                # A pista e' um LOOP fechado: a janela e a referencia dao a
                # volta no array (modulo N). Sem isso, no fechamento da volta
                # reference_path encolhia ate 1 ponto -> IndexError no KLS e o
                # no morria com o ultimo comando latched no sim.
                n = len(self.path)
                idxs = np.arange(self.closest_index - 20, self.closest_index + 20) % n
                segment = self.path[idxs] # pega apenas os pontos dentro dessa janela
                distancias = np.linalg.norm(segment - self.position, axis=1) # calcula as distancias em relação ao carro

                self.closest_index = int(idxs[np.argmin(distancias)]) # indice global do mais proximo

                ref_idxs = np.arange(self.closest_index, self.closest_index + 60) % n
                self.reference_path = self.path[ref_idxs] # 60 pts (~18 m) a frente, com wrap

                # KLS consertado sai CCW-positivo (steering>0 = esquerda), mesma
                # convencao do EUFS/carro real -> SEM negacao. (A negacao era
                # necessaria so no FSDS, onde comando negativo = esquerda.)
                steering_command = self.control.update_steering_angle_control_signal(self.reference_path, self.vehicle_state)
                throttle_command, brake_command = self.longitudinal_controller.update_torque_control_signal(self.path, self.vehicle_state)
                # anti-kick de largada: na partida o erro de velocidade e'
                # maximo e o PID pede throttle alto de uma vez (o carro rabeia)
                throttle_command = min(throttle_command, self.throttle_max)

                self.get_logger().debug('Throttle: "%f"' %throttle_command)
                self.get_logger().debug('Steering: "%f"' %steering_command)
                self.get_logger().debug('Throttle: "%f"' %throttle_command)

                msg = ControlCommand()
                msg.steering = steering_command
                msg.throttle = throttle_command
                msg.brake = brake_command 

                self.path_publishing(self.reference_path)
                self.publisher_.publish(msg)
                self.speed_publisher_.publish(speed_msg)
                self.erro_ant_publisher_.publish(erro_ant_msg)
                self.eh_publisher_.publish(eh_msg)
                self.ey_publisher_.publish(ey_msg)
            else:
                self.get_logger().warn("Path not received")
        else:
            self.get_logger().warn("Odom not received")


    def path_publishing(self, np_array_path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "fsds/map"
        poses = []
        for i, point in enumerate(np_array_path):
            pose = PoseStamped()
            pose.header.frame_id = "fsds/map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            poses.append(pose)
            
        path_msg.poses = poses
        self.path_publisher_.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    control_publisher = ControlNode()
    rclpy.spin(control_publisher)
    control_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()