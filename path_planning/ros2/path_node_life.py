#!/usr/bin/env python3
import rclpy
import os
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from fs_msgs.msg import GoSignal
from scipy.interpolate import interp1d
from rclpy.duration import Duration
from fs_msgs.msg import Track
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from bayesian_inference.bayesian_inference_planner import Bayesian_Inference_Planner
from bayesian_inference.bayesian_inference_planner import Bayesian_Inference_Gains
from bayesian_inference.bayesian_inference_planner import Vehicle_Pose
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory


"""

Reset é "dividido" em deactivate e clean_up

Ciclo Comum:
unconfigured -> inactive -> active -> inactive -> clean_up
Ciclo Emergêncial(Problema no active):
unconfigured -> inactive -> active -> finalized

Transições:
    - on_configure  : Inicializar variáveis, declarar parâmetros e começar a escutar
                    as subscriptions
    - on_activate   : Cria os publishers e inicia o timer, começar a pública o path
    - on_deactivate : Destroí os publishers e o timer
    - on_cleanup    : Destroí subscriptions e reseta variáveis
    - on_shutdown   : Clean-up só que podendo ser acessado de qualquer estado, MATA o nó
"""


class PathNode(LifecycleNode):

    def __init__(self):
        super().__init__('path_node')

        #Reservando espaço pra usar depois
        self.get_logger().info('Unconfigured.')
        self._sub_odom = None
        self._sub_track = None
        self._sub_go = None
        self._publisher_ = None
        self._publisher_concatenated = None
        self._publisher_pointcloud = None
        self._timer = None


    #Lifecycle Callbacks

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        #Escuta para ter as informações mas não publica
        self.get_logger().info('Configuring PathNode...')

        self.declare_parameter('max_angle_change_gain', 5.0)
        self.declare_parameter('std_dvt_track_width_gain', 0.0)
        self.declare_parameter('std_dvt_left_right_cones', 0.0)
        self.declare_parameter('max_wrong_color_gain', 20.0)
        self.declare_parameter('sqd_diff_path_len_sensor_range', 0.0)
        self.declare_parameter('T', 0.01)
        self.declare_parameter('frame_id', 'frame_id')

        max_angle_change_gain = float(self.get_parameter('max_angle_change_gain').value)
        std_dvt_track_width_gain = float(self.get_parameter('std_dvt_track_width_gain').value)
        std_dvt_left_right_cones = float(self.get_parameter('std_dvt_left_right_cones').value)
        max_wrong_color_gain = float(self.get_parameter('max_wrong_color_gain').value)
        sqd_diff_path_len_sensor_range = float(self.get_parameter('sqd_diff_path_len_sensor_range').value)
        self._T = float(self.get_parameter('T').value)
        self._frame_id = self.get_parameter('frame_id').value

        self.get_logger().info('max_angle_change_gain:"%f"' % max_angle_change_gain)
        self.get_logger().info('std_dvt_track_width_gain:"%f"' % std_dvt_track_width_gain)
        self.get_logger().info('std_dvt_left_right_cones:"%f"' % std_dvt_left_right_cones)

        gains = Bayesian_Inference_Gains(
            max_angle_change_gain,
            std_dvt_track_width_gain,
            std_dvt_left_right_cones,
            max_wrong_color_gain,
            sqd_diff_path_len_sensor_range,
        )
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
        self.obstacle_global = []

        self._sub_odom = self.create_subscription(
            Odometry, '/fsds/testing_only/odom', self.odom_callback, 10)
        self._sub_track = self.create_subscription(
            Track, '/fsds/testing_only/track', self.track_callback, 10)
        self._sub_go = self.create_subscription(
            GoSignal, '/fsds/signal/go', self.go_callback, 10)

        return TransitionCallbackReturn.SUCCESS


    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        #Começa a publicar e o timer
        self.get_logger().info('Activating PathNode...')

        #Lifecycle publisher pode ser desativado e ativado
        self._publisher_ = self.create_lifecycle_publisher(Path, 'path', 10)
        self._publisher_concatenated = self.create_lifecycle_publisher(Path, 'path_concatenated', 10)
        self._publisher_pointcloud = self.create_lifecycle_publisher(PointCloud2, 'track_pointcloud', 10)

        self._timer = self.create_timer(self._T, self.timer_callback)

        return super().on_activate(state)


    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        #Contrário do on_activate
        self.get_logger().info('Deactivating PathNode...')

        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

        if self._publisher_ is not None:
            self.destroy_lifecycle_publisher(self._publisher_)
            self._publisher_ = None

        if self._publisher_concatenated is not None:
            self.destroy_lifecycle_publisher(self._publisher_concatenated)
            self._publisher_concatenated = None

        if self._publisher_pointcloud is not None:
            self.destroy_lifecycle_publisher(self._publisher_pointcloud)
            self._publisher_pointcloud = None

        return super().on_deactivate(state)


    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        # Terminar de "limpar" o nó  
        # Não precisa limpar os publishers pq é necessário passar pelo deactivate
        self.get_logger().info('Cleaning up PathNode...')

        if self._sub_odom is not None:
            self.destroy_subscription(self._sub_odom)
            self._sub_odom = None

        if self._sub_track is not None:
            self.destroy_subscription(self._sub_track)
            self._sub_track = None

        if self._sub_go is not None:
            self.destroy_subscription(self._sub_go)
            self._sub_go = None

        self.track_received = False
        self.odom_received = False
        self.obstacle_numpy_array = []
        self.car_position_x = []
        self.car_position_y = []
        self.yaw = []
        self.go_msg = GoSignal()
        self.position = np.array([0, 0])
        self.obstacle_global = []

        return TransitionCallbackReturn.SUCCESS


    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Called when the node is shutting down from any state.
        Release everything that may still be alive.
        """
        self.get_logger().info('Shutting down PathNode...')

        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

        for pub_attr in ('_publisher_', '_publisher_concatenated', '_publisher_pointcloud'):
            pub = getattr(self, pub_attr, None)
            if pub is not None:
                self.destroy_lifecycle_publisher(pub)
                setattr(self, pub_attr, None)

        for sub_attr in ('_sub_odom', '_sub_track', '_sub_go'):
            sub = getattr(self, sub_attr, None)
            if sub is not None:
                self.destroy_subscription(sub)
                setattr(self, sub_attr, None)

        return TransitionCallbackReturn.SUCCESS

    
    def track_callback(self, msg):
        self.get_logger().info('Track Received')
        obstacle_list = []
        self.track_pointcloud_msg = msg
        for cone in msg.track:
            x = cone.location.x
            y = cone.location.y
            if cone.color == 0:
                color = 0
                obstacle = np.array([x, y, 1, color])
                obstacle_list.append(obstacle)
            
            elif cone.color == 1 or cone.color == 4:
                color = 4
                obstacle = np.array([x, y, 1, color])
                obstacle_list.append(obstacle)
        
        self.obstacle_numpy_array = np.array(obstacle_list)
        
        self.track_received = True
    

    def odom_callback(self, msg):
        # self.get_logger().info('X: "%f"' % msg.pose.pose.position.x)
        self.get_logger().info('Odom Received')
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
        self._publisher_.publish(path_msg)


    def path_publishing_concatenated(self, np_array_path):
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
        self._publisher_concatenated.publish(path_msg)


    def timer_callback(self):
        if self.track_received:
                pointcloud=self.track_to_pointcloud()
                self._publisher_pointcloud.publish(pointcloud)
        else:
            self.get_logger().info('Track not Received')
                
        if self.go_msg.mission == "trackdrive" and self.track_received:
            self.get_logger().info('oi')

            self.local_cones = []
            for cone in self.obstacle_numpy_array:
                if np.linalg.norm(cone[:2] - self.position) <= 20:
                    self.local_cones.append(cone)    

            self.obstacle_global_array = np.array(self.local_cones)
            
            obstacle_global_array = np.array(self.obstacle_numpy_array)

            self.get_logger().info('cones: "%s"' %self.obstacle_global_array[:2])
            np_array_path,np_array_path_concatenated = self.planner.get_interpolated_path(self.obstacle_numpy_array, self.car_pose)
            
            self.path_publishing(np_array_path)
            self.path_publishing_concatenated(np_array_path_concatenated)
            self.get_logger().info('Waypoints')
    

        if self.go_msg.mission == "acceleration":
            first_position = np.array([0, 0])
            final_position = np.array([75, 0])
            path = np.array([first_position, final_position])
            distance = np.cumsum( np.sqrt(np.sum( np.diff(path, axis=0)**2, axis=1 )))
            distance = np.insert(distance, 0, 0)/distance[-1]
            interpolator =  interp1d(distance, path, kind="slinear", axis=0)
            self.path_publishing(interpolator(np.linspace(0,1,50)))
                
        if self.go_msg.mission == "brake-test":
            first_position = np.array([0, 0])
            final_position = np.array([75, 0])
            path = np.array([first_position, final_position])
            distance = np.cumsum( np.sqrt(np.sum( np.diff(path, axis=0)**2, axis=1 )) )
            distance = np.insert(distance, 0, 0)/distance[-1]
            interpolator =  interp1d(distance, path, kind="slinear", axis=0)
            self.path_publishing(interpolator(np.linspace(0,1,100)))
          
        if self.go_msg.mission == "skidpad":
            self.get_logger().info('skidpad received')

            pkg_share_dir = get_package_share_directory('path_planning')

            skidpad_csv_path = os.path.join(pkg_share_dir, 'ros2', 'skidpad.csv')

            path = np.genfromtxt(skidpad_csv_path,
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
            
        
        if self.go_msg.mission == "auto-cross":
            waypoints = self.planner.get_waypoints(self.obstacle_numpy_array, self.car_pose)
            np_array_path = self.planner.get_interpolated_path(self.obstacle_numpy_array, waypoints[-1])
            
            np_array_path = self.planner.get_interpolated_path(self.obstacle_numpy_array, waypoints[-1])
            self.path_publishing(np_array_path)


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