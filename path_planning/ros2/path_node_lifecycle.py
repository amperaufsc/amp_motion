#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from fs_msgs.msg import GoSignal
import numpy as np
from scipy.interpolate import interp1d
from rclpy.duration import Duration
from fs_msgs.msg import Track
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from bayesian_inference.bayesian_inference_planner import Bayesian_Inference_Planner
from bayesian_inference.bayesian_inference_planner import Bayesian_Inference_Gains
from bayesian_inference.bayesian_inference_planner import Vehicle_Pose
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from lifecycle_msgs.msg import Transition, TransitionEvent, State
import sys


class PathNode(LifecycleNode):


    def __init__(self):
        super().__init__('path_node')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
            self.subscription = self.create_subscription(Track, 'track_pub', self.track_callback, 10)
            self.subscription = self.create_subscription(GoSignal, 'go', self.go_callback, 10)
            self.publisher_ = self.create_publisher(Path, 'path', 10)
            self.publisher_pointcloud = self.create_publisher(PointCloud2, 'track_pointcloud',10)
            
            self.led_pub = self.create_publisher(String, '/AMP/as_status_indicator', 10)
            self.led_msg = String()
            self.transition_pub_emergency = self.create_publisher(TransitionEvent, "EbsNOTMissionFinished", 10)
            self.transition_pub_driving = self.create_publisher(TransitionEvent, '/R2D', 10)
            
            self.declare_parameter('max_angle_change_gain', 5.0)
            self.declare_parameter('std_dvt_track_width_gain', 0.0)
            self.declare_parameter('std_dvt_left_right_cones', 0.0)
            self.declare_parameter('max_wrong_color_gain', 20.0)
            self.declare_parameter('sqd_diff_path_len_sensor_range', 0.0)
            self.declare_parameter('frame_id', 'frame_id')
            self.declare_parameter('T', 0.01)
            self.T = float(self.get_parameter('T').value)

            max_angle_change_gain = float(self.get_parameter('max_angle_change_gain').value)
            std_dvt_track_width_gain = float(self.get_parameter('std_dvt_track_width_gain').value)
            std_dvt_left_right_cones = float(self.get_parameter('std_dvt_left_right_cones').value)
            max_wrong_color_gain = float(self.get_parameter('max_wrong_color_gain').value)
            sqd_diff_path_len_sensor_range = float(self.get_parameter('sqd_diff_path_len_sensor_range').value)
            frame_id = self.get_parameter('frame_id').value
            
            
            self.get_logger().info('max_angle_change_gain:"%f"' %max_angle_change_gain)
            self.get_logger().info('std_dvt_track_width_gain:"%f"' %std_dvt_track_width_gain)
            self.get_logger().info('std_dvt_left_right_cones:"%f"' %std_dvt_left_right_cones)

            gains = Bayesian_Inference_Gains(max_angle_change_gain, std_dvt_track_width_gain, std_dvt_left_right_cones, max_wrong_color_gain, sqd_diff_path_len_sensor_range)
            self.planner = Bayesian_Inference_Planner(gains)

            self.track_received = False
            self.odom_received = False
            self.obstacle_numpy_array = []
            self.car_position_x = []
            self.car_position_y = []
            self.yaw = []
            self.car_pose = Vehicle_Pose(self.car_position_x, self.car_position_y, self.yaw)
            self.go_msg = GoSignal()
            self.time_stamp = self.get_clock().now().to_msg()
            self.index = 0
            self.position = np.array([0, 0])

            self.get_logger().info("PathNode configurado com sucesso")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().info("Falha ao configurar o PathNode")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        try:
            self.timer = self.create_timer(self.T, self.timer_callback)

            self.get_logger().info("PathNode ativado com sucesso")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().info("Falha ao ativar PathNode")
            return TransitionCallbackReturn.FAILURE
    
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("SHUTDOWN PathNode")

        return TransitionCallbackReturn.SUCCESS

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
        # self.get_logger().info('X: "%f"' % msg.pose.pose.position.x)
        if self.track_received:
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            self.yaw = np.arctan2(2*(orientation_q.w*orientation_q.z - orientation_q.x*orientation_q.y), 1 - 2*((orientation_q.y)**2 + (orientation_q.z)**2))
            self.car_position_x = msg.pose.pose.position.x
            self.car_position_y = msg.pose.pose.position.y
            self.position = np.array([self.car_position_x, self.car_position_y])
            self.car_pose = Vehicle_Pose(self.car_position_x, self.car_position_y, self.yaw)
            self.odom_timestamp = msg.header.stamp
            self.odom_time_stamp_float = self.odom_timestamp.sec + self.odom_timestamp.nanosec * 1e-9
            self.odom_msg = msg
            self.odom_received = True

    def go_callback(self, msg):
        event_driving = TransitionEvent()
        self.transition_pub_driving.publish(event_driving)
        self.go_msg = msg
        
    def path_publishing(self, np_array_path):
        path_msg = Path()
        path_msg.header.stamp = self.time_stamp
        path_msg.header.frame_id = "fsds/map"
        poses = []
        for i, point in enumerate(np_array_path):
            if i >= 1:
                pose = PoseStamped()
                dt_seconds = np.linalg.norm(point - np_array_path[i - 1])/2.5
                
                dt_duration = Duration(seconds = dt_seconds*i)
                original_time = Time.from_msg(path_msg.header.stamp)
                new_time = original_time + dt_duration
                pose.header.stamp = new_time.to_msg()
                #pose.header.stamp = path_msg.header.stamp + dt_duration
                pose.header.frame_id = "fsds/map"
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                poses.append(pose)
            
        self.get_logger().info('Publishing')
        path_msg.poses = poses
        self.publisher_.publish(path_msg)
            
    def timer_callback(self):
        try:
            if self.track_received:
                    pointcloud=self.track_to_pointcloud()
                    self.publisher_pointcloud.publish(pointcloud)
                    
            if self.go_msg.mission == "trackdrive" and self.track_received:
                self.get_logger().info('oi')
                #waypoints = self.planner.get_waypoints(self.obstacle_numpy_array, self.car_pose)
                local_cones = []
                for cone in self.obstacle_numpy_array:
                    if np.linalg.norm(cone[:2] - self.position) <= 20:
                        local_cones.append(cone)


                local_cones_array = np.array(local_cones)
                np_array_path = self.planner.get_interpolated_path(local_cones_array, self.car_pose)
                
                self.path_publishing(np_array_path)
                self.get_logger().info('Waypoints')
        

            if self.go_msg.mission == "acceleration":
                first_position = np.array([0, 0])
                final_position = np.array([75, 0])
                path = np.array([first_position, final_position])
                distance = np.cumsum( np.sqrt(np.sum( np.diff(path, axis=0)**2, axis=1 )))
                distance = np.insert(distance, 0, 0)/distance[-1]
                interpolator =  interp1d(distance, path, kind="slinear", axis=0)
                self.path_publishing(interpolator(np.linspace(0,1,50)))
                    
            '''if self.go_msg.mission == "brake-test":
                first_position = np.array([0, 0])
                final_position = np.array([75, 0])
                path = np.array([first_position, final_position])
                distance = np.cumsum( np.sqrt(np.sum( np.diff(path, axis=0)**2, axis=1 )) )
                distance = np.insert(distance, 0, 0)/distance[-1]
                interpolator =  interp1d(distance, path, kind="slinear", axis=0)
                self.path_publishing(interpolator(np.linspace(0,1,100)))'''
            
            if self.go_msg.mission == "skidpad":
                self.get_logger().info('skidpad received')

                path = np.genfromtxt("/home/carlosmello/ws/src/as_amp/path_planning/ros2/skidpad.csv",
                                delimiter = ';',
                                skip_header = 1,
                                dtype = float,
                                invalid_raise = False)
                
            
                path = [sublist[::-1] for sublist in path]
                for sublist in path:
                    sublist[1] *= -1
                skidpad_path = path + np.array([15, 0])
                distance = np.cumsum(np.sqrt(np.sum( np.diff(skidpad_path, axis=0)**2, axis=1 )) )
                distance = np.insert(distance, 0, 0)/distance[-1]
                interpolator =  interp1d(distance, skidpad_path, kind="slinear", axis=0)
                self.path_publishing(interpolator(np.linspace(0, 1, 100)))
                
            '''
            if self.go_msg.mission == "auto-cross":
                waypoints = self.planner.get_waypoints(self.obstacle_numpy_array, self.car_pose)
                np_array_path = self.planner.get_interpolated_path(self.obstacle_numpy_array, waypoints[-1])
                
                np_array_path = self.planner.get_interpolated_path(self.obstacle_numpy_array, waypoints[-1])
                self.path_publishing(np_array_path)'''
        except Exception as e:
            event = TransitionEvent()
            self.transition_pub_emergency(event) # Muda o estado do carro pra Emergency
            self.led_msg.data = 'as_emergency'
            self.led_pub.publish(self.led_msg) # Publica mensagem de Emergency pra ativação do LED
            sys.exit(1) # Mata o processo com chamada de sistema

    def track_to_pointcloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "/fsds/map"  # Ajuste para o frame de referência correto

        points = []
        for cone in self.track_pointcloud_msg.track:  # Supondo que track_msg.tracks é a lista de rastreamentos
            x = cone.location.x
            y = cone.location.y
            z = cone.location.z
            points.append([x, y, z])

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            
        ]
    
        pointcloud_msg = pc2.create_cloud(header, fields, points)
        return pointcloud_msg
      
def main():
   rclpy.init()
   path_node = PathNode()
   rclpy.spin(path_node)
   path_node.destroy_node()
   rclpy.shutdown()



if __name__ == '__main__':
   main()