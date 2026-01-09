#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from nav_msgs.msg import Odometry
from fs_msgs.msg import Motion, Track, GoSignal
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from rclpy.duration import Duration
from rclpy.time import Time
import numpy as np
from scipy.interpolate import interp1d
import sensor_msgs_py.point_cloud2 as pc2
from bayesian_inference.bayesian_inference_planner import Bayesian_Inference_Planner, Bayesian_Inference_Gains, Vehicle_Pose
from bayesian_inference.speed_profile import SpeedProfile
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from lifecycle_msgs.msg import Transition, TransitionEvent, State
from lifecycle_msgs.srv import GetState, ChangeState
import sys


class PathNode(LifecycleNode):
    def __init__(self):
        super().__init__('path_node')

        # State variables
        self.track_received = False
        self.odom_received = False
        self.obstacle_numpy_array = []
        self.car_pose = Vehicle_Pose(0, 0, 0)
        self.go_msg = GoSignal()

        self.position = np.array([0, 0])
        self.timer = None
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
             
    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("SHUTDOWN PathNode")
        return TransitionCallbackReturn.SUCCESS
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring PathNode...')

        try:
        # Declaração dos parâmetros
            self.declare_parameter('max_angle_change_gain', 5.0)
            self.declare_parameter('std_dvt_track_width_gain', 0.0)
            self.declare_parameter('std_dvt_left_right_cones', 0.0)
            self.declare_parameter('max_wrong_color_gain', 20.0)
            self.declare_parameter('sqd_diff_path_len_sensor_range', 0.0)
            self.declare_parameter('T', 0.01)
            self.declare_parameter('max_acceleration', 0.5)
            self.declare_parameter('braking_acceleration', 0.5)
            self.declare_parameter('lateral_acceleration', 0.5)
            self.declare_parameter('max_speed', 4.0)
            self.declare_parameter('frame_id', 'fsds/map')

        # Recuperação dos parâmetros
            max_angle_change_gain = float(self.get_parameter('max_angle_change_gain').value)
            std_dvt_track_width_gain = float(self.get_parameter('std_dvt_track_width_gain').value)
            std_dvt_left_right_cones = float(self.get_parameter('std_dvt_left_right_cones').value)
            max_wrong_color_gain = float(self.get_parameter('max_wrong_color_gain').value)
            sqd_diff_path_len_sensor_range = float(self.get_parameter('sqd_diff_path_len_sensor_range').value)
            T = float(self.get_parameter('T').value)
            max_acceleration = float(self.get_parameter('max_acceleration').value)
            braking_acceleration = float(self.get_parameter('braking_acceleration').value)
            lateral_acceleration = float(self.get_parameter('lateral_acceleration').value)
            max_speed = float(self.get_parameter('max_speed').value)
            frame_id = self.get_parameter('frame_id').value

            # Inicialização do planner e speed profile
            gains = Bayesian_Inference_Gains(
                max_angle_change_gain,
                std_dvt_track_width_gain,
                std_dvt_left_right_cones,
                max_wrong_color_gain,
                sqd_diff_path_len_sensor_range
            )
            self.planner = Bayesian_Inference_Planner(gains)
            self.speed_profile = SpeedProfile(max_acceleration, braking_acceleration, lateral_acceleration, max_speed)
            self.timer_period = T
            self.frame_id = frame_id
            
            # Publishers do nó
            self.publisher_path = self.create_lifecycle_publisher(Motion, 'path', 10)
            self.publisher_pointcloud = self.create_lifecycle_publisher(PointCloud2, 'pointcloud', 10)
            self.transition_pub_driving = self.create_publisher(TransitionEvent, '/R2D', 10)
            self.transition_pub_emergency = self.create_lifecycle_publisher(TransitionEvent, 'EbsNOTMissionFinished', 10)
            # Criando os publishers
            self.publisher_path = self.create_lifecycle_publisher(Motion, 'path', 10)
            self.publisher_pointcloud = self.create_lifecycle_publisher(PointCloud2, 'track_pointcloud', 10)

            # Criando subscribers
            self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
            self.track_sub = self.create_subscription(Track, '/track_pub', self.track_callback, 10)
            self.go_sub = self.create_subscription(GoSignal, 'go', self.go_callback, 10)

            # Publisher resposnavel pela ativação dos leds do carro baseado em seu estado
            self.led_pub = self.create_publisher(String, '/AMP/as_status_indicator', 10)
            self.led_msg = String()

            self.get_logger().info('Configuração PATH concluída com sucesso.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
                self.get_logger().error(f'Erro durante on_configure: {e}')
                return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating PathNode...')

        try:
            # Criando o timer
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
            self.get_logger().info('PathNode ativado com sucesso.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Erro durante on_activate: {e}')
            return TransitionCallbackReturn.FAILURE

    def interpolator(self, path):
        distance = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
        distance = np.insert(distance, 0, 0) / distance[-1]
        interpolator = interp1d(distance, path, kind="cubic", axis=0)
        return interpolator(np.linspace(0, 1, 1000))

    def track_callback(self, msg):
        self.get_logger().info('Track Received')
        obstacle_numpy_array = []
        self.track_pointcloud_msg = msg

        for cone in msg.track:
            x = cone.location.x
            y = cone.location.y
            if cone.color == 0:
                color = 0
                obstacle = np.array([x, y, 1, color])
                obstacle_numpy_array.append(obstacle)
            elif cone.color == 1:
                color = 4
                obstacle = np.array([x, y, 1, color])
                obstacle_numpy_array.append(obstacle)

        self.obstacle_numpy_array = np.array(obstacle_numpy_array)
        self.track_received = True

    def odom_callback(self, msg):

        self.trajectory = []
        if self.track_received:
            orientation_q = msg.pose.pose.orientation
            self.yaw = np.arctan2(2 * (orientation_q.w * orientation_q.z - orientation_q.x * orientation_q.y), 1 - 2 * ((orientation_q.y)**2 + (orientation_q.z)**2))
            self.car_position_x = msg.pose.pose.position.x
            self.car_position_y = msg.pose.pose.position.y
            self.position = np.array([self.car_position_x, self.car_position_y])
            self.trajectory.append(self.position)

            self.car_pose = Vehicle_Pose(self.car_position_x, self.car_position_y, self.yaw)
            self.odom_timestamp = msg.header.stamp
            self.odom_time_stamp_float = self.odom_timestamp.sec + self.odom_timestamp.nanosec * 1e-9
            self.odom_msg = msg
            self.odom_received = True

    def go_callback(self, msg):
        event_driving = TransitionEvent()
        self.transition_pub_driving.publish(event_driving)
        self.get_logger().info('SIM EU QUERO DAR MEU CU')
        self.go_msg = msg

    def publish_path(self, np_array_path):
        path_msg = Motion()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "fsds/map"
        poses = []

        interpolated_path = self.interpolator(np_array_path)
        car_speed = self.speed_profile.compute_speed_profile(interpolated_path)
        dt_seconds = np.zeros(len(car_speed))

        for i, point in enumerate(interpolated_path):
            if i >= 1:
                odom = Odometry()
                if car_speed[i] != 0 and i < len(car_speed) - 1:
                    d = np.linalg.norm(point - interpolated_path[i - 1])
                    t = (2 * d) / (car_speed[i] + car_speed[i + 1])
                    if t > 0:
                        dt_seconds[i] = t + dt_seconds[i - 1]
                        np_dt_seconds = np.array(dt_seconds)
                        dt_duration = Duration(seconds=np_dt_seconds[i])
                        original_time = Time.from_msg(path_msg.header.stamp)
                        new_time = original_time + dt_duration
                        odom.header.stamp = new_time.to_msg()

                odom.header.frame_id = "fsds/map"
                odom.pose.pose.position.x = point[0]
                odom.pose.pose.position.y = point[1]
                odom.twist.twist.linear.x = car_speed[i]
                poses.append(odom)

        self.get_logger().info('Path Publishing')
        path_msg.odom = poses
        self.publisher_path.publish(path_msg)

    def timer_callback(self):
        try:

            if not self.track_received:
                return

            self.publisher_pointcloud.publish(self.track_to_pointcloud())

            if self.go_msg.mission == "trackdrive":
                local_cones = [cone for cone in self.obstacle_numpy_array if np.linalg.norm(cone[:2] - self.position) <= 20]
                local_cones_array = np.array(local_cones)
                np_array_path = self.planner.get_interpolated_path(local_cones_array, self.car_pose)
                self.publish_path(np_array_path)

            elif self.go_msg.mission == "acceleration":
                path = np.array([[0, 0], [75, 0]])
                self.publish_path(self.interpolator(path))

            elif self.go_msg.mission == "brake-test":
                path = np.array([[0, 0], [75, 0]])
                self.publish_path(self.interpolator(path))

            elif self.go_msg.mission == "skidpad":
                self.get_logger().info('skidpad received')
                path = np.genfromtxt("/home/carlosmello/ws/src/as_amp/path_planning/ros2/skidpad.csv", delimiter=';', skip_header=1, dtype=float, invalid_raise=False)
                path = [sublist[::-1] for sublist in path]
                for sublist in path:
                    sublist[1] *= -1
                skidpad_path = np.array(path) + np.array([15, 0])
                self.publish_path(self.interpolator(skidpad_path))

            elif self.go_msg.mission == "auto-cross":
                waypoints = self.planner.get_waypoints(self.obstacle_numpy_array, self.car_pose)
                np_array_path = self.planner.get_interpolated_path(self.obstacle_numpy_array, waypoints[-1])
                self.publish_path(np_array_path)
        except:
            event_emergency = TransitionEvent()

            self.transition_pub_emergency.publish(event_emergency) # Muda o estado do carro pra Emergency
            self.led_msg.data = 'as_emergency' 
            self.led_pub.publish(self.led_msg) # Publica mensagem de Emergency pra ativação do LED
            
            sys.exit(1) # Mata o processo com chamada de sistema


    def track_to_pointcloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.get_parameter('frame_id').value
        points = [[cone.location.x, cone.location.y, cone.location.z] for cone in self.track_pointcloud_msg.track]
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        return pc2.create_cloud(header, fields, points)
    
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
    node = PathNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

