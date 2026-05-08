#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from fs_msgs.msg import Motion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32
from fs_msgs.msg import GoSignal
import numpy as np
from scipy.interpolate import interp1d
from rclpy.duration import Duration
from fs_msgs.msg import Track
from fs_msgs.msg import Track
from fs_msgs.msg import Motion 
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from bayesian_inference.bayesian_inference_planner import Bayesian_Inference_Planner
from bayesian_inference.bayesian_inference_planner import Bayesian_Inference_Gains
from bayesian_inference.bayesian_inference_planner import Vehicle_Pose
from bayesian_inference.speed_profile import SpeedProfile
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import matplotlib.pyplot as plt
import os
from ament_index_python.packages import get_package_share_directory


class PathNode(Node):

    # Creating the subscription and publisher of the path_planning node

    def __init__(self):
        super().__init__('path_node')
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(Track, 'track', self.track_callback, 10)
        self.subscription = self.create_subscription(GoSignal, 'go', self.go_callback, 10)
        self.publisher_motion = self.create_publisher(Motion, 'motion', 10)
        self.publisher_path = self.create_publisher(Path, 'path', 10)
        self.publisher_pointcloud = self.create_publisher(PointCloud2, 'track_pointcloud', 10)

        # Declaring the parameters that will later be defined in the path_planning.launcher.py file

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
        self.declare_parameter('frame_id', 'frame_id')

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
        
        # Printing the speed_profile parameters 

        self.get_logger().info('max_acceleration:"%f"' %max_acceleration)
        self.get_logger().info('braking_acceleration:"%f"' %braking_acceleration)
        self.get_logger().info('lateral_acceleration:"%f"' %lateral_acceleration)
        self.get_logger().info('max_speed:"%f"' %max_speed)

        # Declaring the weight of each Bayesan_Inference feature, according to it's relevance in the final path sum.

        gains = Bayesian_Inference_Gains(max_angle_change_gain, std_dvt_track_width_gain, std_dvt_left_right_cones, max_wrong_color_gain, sqd_diff_path_len_sensor_range)
        self.planner = Bayesian_Inference_Planner(gains) #vehicle_speed = centripetal_speed(path)
        
        # Setting the variable speed_profile 
        self.speed_profile = SpeedProfile(max_acceleration, braking_acceleration, lateral_acceleration, max_speed)
        
        self.timer = self.create_timer(T, self.timer_callback)

        # Initializing odometry and track messages as false until they are received from the simulator

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

    # Function that interpolates the path.

    def interpolator(self, path):
            distance = np.cumsum(np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)))
            distance = np.insert(distance, 0, 0)/distance[-1]
            interpolator =  interp1d(distance, path, kind="cubic", axis=0)
            interpolated_path = interpolator(np.linspace(0, 1, 1000))
            return interpolated_path
    
    # Function that iterates and provides track data.

    def track_callback(self, msg):
        self.get_logger().info('Track Received')
        obstacle_numpy_array = []
        self.track_pointcloud_msg = msg

        # Iterates through the message and stores cone data (position, color, etc.)

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

        # Declaring that the track has been received

        self.track_received = True

    # Function that iterates and provides odometry data.

    def odom_callback(self, msg):
        self.trajectory = []
        if self.track_received:
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

            # Using quaternions to calculate yaw
            self.yaw = np.arctan2(2*(orientation_q.w*orientation_q.z - orientation_q.x*orientation_q.y), 1 - 2*((orientation_q.y)**2 + (orientation_q.z)**2))
            self.car_position_x = msg.pose.pose.position.x
            self.car_position_y = msg.pose.pose.position.y
            self.position = np.array([self.car_position_x, self.car_position_y])
            self.trajectory.append(self.position)
            
            self.car_pose = Vehicle_Pose(self.car_position_x, self.car_position_y, self.yaw)
            self.odom_timestamp = msg.header.stamp
            self.odom_time_stamp_float = self.odom_timestamp.sec + self.odom_timestamp.nanosec * 1e-9
            
            self.odom_msg = msg

            # Declaring that odometry has been received
            self.odom_received = True
            

    # Publishing the go_msg that contains the mission the car will perform

    def go_callback(self, msg):
        self.go_msg = msg
    
    # Function that publishes the path message.

    def path_publishing(self, np_array_path):
        
        # path_msg is a message of type Motion, meaning it's a list of odometries 
        # (Motion.msg was created in the fsds package)

        path_msg = Motion()
        path_msg.header.stamp = self.time_stamp
        path_msg.header.frame_id = "fsds/map"
        poses = [] 

        # Interpolating the path

        interpolated_path = self.interpolator(np_array_path)

        # Calculating vehicle speed according to the speed_profile

        car_speed = self.speed_profile.compute_speed_profile(interpolated_path)
        dt_seconds = np.zeros(len(car_speed))
        for i, point in enumerate(interpolated_path):
            if i >= 1:
                odom = Odometry()
                if car_speed[i] != 0 and i > 1:
                        d = (np.linalg.norm(point - interpolated_path[i - 1]))
                        t = (2*d)/(car_speed[i] + car_speed[i+1])
                        if t > 0: 
                            dt_seconds[i] = t + dt_seconds[i - 1]
                            np_dt_seconds = np.array(dt_seconds)
                    
                            dt_duration = Duration(seconds = np_dt_seconds[i]*i)
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
        self.publisher_motion.publish(path_msg)
          
    def timer_callback(self):
        # Publishing the pointcloud extracted from the track

        if self.track_received:
                pointcloud=self.track_to_pointcloud()
                self.publisher_pointcloud.publish(pointcloud)

                
        # If the mission is "autocross", generate the path based on nearby cones and publish it

        if self.go_msg.mission == "autocross" and self.track_received:
            self.get_logger().info('oi')
            local_cones = []
            for cone in self.obstacle_numpy_array:
                if np.linalg.norm(cone[:2] - self.position) <= 20:
                    local_cones.append(cone)

            local_cones_array = np.array(local_cones)
            np_array_path = self.planner.get_interpolated_path(local_cones_array, self.car_pose)
            
            self.path_publishing(np_array_path)
            #speed_profile = self.speed_profile.compute_speed_profile(np_array_path)


        # If the mission is "acceleration", create a simple straight path and publish it

        if self.go_msg.mission == "acceleration":
            first_position = np.array([0, 0])
            final_position = np.array([75, 0])
            path = np.array([first_position, final_position])
            interpolated_path = self.interpolator(path)
            self.path_publishing((interpolated_path))


        # If the mission is "brake-test", create a simple straight path and publish it     
         
        if self.go_msg.mission == "brake-test":
            first_position = np.array([0, 0])
            final_position = np.array([75, 0])
            path = np.array([first_position, final_position])
            interpolated_path = self.interpolator(path)
            self.path_publishing((interpolated_path))


        # If the mission is "skidpad", load a predefined path from a CSV file and publish it

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
            interpolated_skidpad = self.interpolator(skidpad_path)
            self.path_publishing(interpolated_skidpad)
            

        # If the mission is "auto-cross", generate waypoints based on cones and publish the interpolated path

        if self.go_msg.mission == "trackdrive" and self.track_received and self.odom_received:
                self.waypoints = self.planner.get_waypoints(self.obstacle_numpy_array, self.car_pose)
                self.final_waypoints_list = self.planner.get_waypoints(self.obstacle_numpy_array, self.car_pose)[1]

                if self.planner.completed_lap(self.car_pose) and self.completed:
                    for i in range(len(self.final_waypoints_list)):
                        print(self.final_waypoints_list[i])
                        self.Xlist.append(self.final_waypoints_list[i][0])
                        self.Ylist.append(self.final_waypoints_list[i][1])

                    self.completed = False
                    self.get_logger().info('Volta completa')
                    plt.scatter(self.Xlist, self.Ylist)
                    plt.tight_layout()
                    plt.show()
                
                local_cones = []
                for cone in self.obstacle_numpy_array:
                    if np.linalg.norm(cone[:2] - self.position) <= 4:
                        local_cones.append(cone)


                local_cones_array = np.array(local_cones)
                np_array_path,np_array_path_concatenated = self.planner.get_interpolated_path(self.obstacle_numpy_array, self.car_pose)
                
                self.path_publishing(np_array_path)
                self.path_publishing_concatenated(np_array_path_concatenated)

        # Converts the track cones into a PointCloud2 message for visualization
        
    def track_to_pointcloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "/fsds/map" 

        points = []
        for cone in self.track_pointcloud_msg.track: 
            x = cone.location.x
            y = cone.location.y
            z = cone.location.z
            points.append([x, y, z])
                    
        fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
            
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