
from scipy.spatial import Delaunay
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

class Vehicle_Pose:
    def __init__(self,x_position, y_position, yaw_position):
        self.x_position_track_reference_frame = x_position
        self.y_position_track_reference_frame = y_position
        self.yaw_position_track_reference_frame = yaw_position
        
class Bayesian_Inference_Gains:
    def __init__(self, max_angle_change_gain, std_dvt_track_width_gain, 
                       std_dvt_left_right_cones, max_wrong_color_gain, sqd_diff_path_len_sensor_range):
        self.max_angle_change_gain = max_angle_change_gain 
        self.std_dvt_track_width_gain = std_dvt_track_width_gain
        self.std_dvt_left_right_cones = std_dvt_left_right_cones
        self.max_wrong_color_gain = max_wrong_color_gain
        self.sqd_diff_path_len_sensor_range = sqd_diff_path_len_sensor_range
        pass

class TreeNode:
    def __init__(self,key, mean_point_coord,parent = None):
        self.key = key
        self.parent = parent
        self.children = []
        self.cost = 0
        self.parentNumber = 0
        if parent:
            self.parentNumber = parent.parentNumber+1
        self.max_angle_change = 0
        self.wrong_color = 0
        self.mean_point_coord = mean_point_coord
        self.path_sqd_lenght = 0
        self.dist_cones = 0
        self.right_cone = []
        self.left_cone = []

class Tree:
    def __init__(self, root: TreeNode):
        self.root = root
        self.leaves = [root]
        self.visited = [root]

    def insertNode(self, node: TreeNode):
        self.visited.append(node)
        if node.parent in self.leaves:
            self.leaves.remove(node.parent)
        self.insertLeaf(node)
        node.parent.children.append(node)
        node.parentNumber = node.parent.parentNumber + 1

    def removeNode(self, node:TreeNode):
        node.parent.children.remove(node)
        if node in self.leaves:
            self.leaves.remove(node)
    
    def insertLeaf(self, node: TreeNode):
        if self.leaves:
            index = len(self.leaves)
            for i,leaf in enumerate(self.leaves):
                if leaf.cost > node.cost:
                    index = i
                    break
            if index == len(self.leaves):
                self.leaves.append(node)
            else:
                self.leaves = self.leaves[:index] + [node] + self.leaves[index:]
        else:
            self.leaves.append(node)


class Bayesian_Inference_Planner:
    def __init__(self, gains: Bayesian_Inference_Gains, beam_width = 3, iterations_number = 10):
        self.beam_width = beam_width
        self.iterations_number = iterations_number
        self.gains = gains
        self.waypoints = []
        self.lap_threshold_distance = 0.5
        self.lap_completed = False
        self.distance_threshold = 1.0
        self.global_path = [[0,0]]
        self.limit_size = 20

    def update_map(self, global_map):
        self.global_map = global_map

    def update_vehicle_pose(self, vehicle_pose):
        self.vehicle_pose = vehicle_pose

    def triangulate(self, vertices):
        cones = vertices
        cones = np.append(cones,[self.vehicle_pose], axis=0)
        return Delaunay(cones).simplices
    
    def get_mean_point(self, side, vertex):
        return np.array((vertex[side[0]]+vertex[side[1]])/2)

    def get_possible_paths_from_origin(self,tri, origin_index):
        possible_paths = tri[np.where(tri == origin_index)[:1]]
        possible_paths = [np.delete(i, np.where(i == origin_index)) for i in possible_paths]
        return possible_paths
    
    def calculate_max_angle(self, node, Car_Orientation):
        v1 = node.mean_point_coord - node.parent.mean_point_coord
        if node.parentNumber > 1:
            v2 = node.parent.mean_point_coord - node.parent.parent.mean_point_coord
        else:
            v2 = Car_Orientation
        u_v1 = v1/np.linalg.norm(v1)
        u_v2 = v2/np.linalg.norm(v2)
        prod = np.dot(u_v1,u_v2)
        if prod > 1:
            prod = 1
        elif prod < -1:
            prod = -1
        ang = np.arccos(prod)
        node.max_angle_change = ang if ang > node.parent.max_angle_change else node.parent.max_angle_change

    def calculate_path_lenght(self, node:TreeNode):
        pass

    def calculate_max_wrong_color(self, node:TreeNode):
        node.wrong_color = node.parent.wrong_color
        if node.left_cone[3] == 4:
            node.wrong_color += 1
        if node.right_cone[3] == 0:
            node.wrong_color += 1

    def split_right_and_left_cones(self, node:TreeNode, cones):
        Pi = node.parent.mean_point_coord
        Pf = node.mean_point_coord
        v = Pf - Pi
        if node.key[0] == len(cones):
            cone1 = cones[node.key[1]]
        else:
            cone1 = cones[node.key[0]]

        if node.key[1] == len(cones):
            cone2 = cones[node.key[0]]
        else:
            cone2 = cones[node.key[1]]

        if v[1] >= 0 and v[0] < 0 or v[1] < 0 and v[0] < 0:
            if cone1[1] > (v[1]/v[0]*(cone1[0]-Pi[0])+Pi[1]):
                node.left_cone = cone2
                node.right_cone = cone1
            else:
                node.left_cone = cone1
                node.right_cone = cone2
        else:
            if v[0]*(cone1[0]-Pi[0])+Pi[1] != 0.0 and cone1[1] > (v[1]/v[0]*(cone1[0]-Pi[0])+Pi[1]):
                node.left_cone = cone1
                node.right_cone = cone2
            else:
                node.left_cone = cone2
                node.right_cone = cone1
    
    def calculate_var_dist_cones(self, node:TreeNode):
        if node.parentNumber > 1:
            dist_cones_pattern = 2.5
            desv_left = np.linalg.norm(node.left_cone[:2]-node.parent.left_cone[:2]) - dist_cones_pattern
            desv_right = np.linalg.norm(node.right_cone[:2]-node.parent.right_cone[:2]) - dist_cones_pattern
            desv_left = desv_left*desv_left
            desv_right = desv_right*desv_right
            var_desv = desv_right + desv_left
            node.dist_cones = node.parent.dist_cones + var_desv

    def calculate_cost(self, node:TreeNode, Car_Orientation, cones):
        self.calculate_max_angle(node, Car_Orientation)
        self.split_right_and_left_cones(node, cones)
        self.calculate_max_wrong_color(node)
        self.calculate_var_dist_cones(node)
        node.cost = self.gains.max_angle_change_gain*node.max_angle_change + self.gains.max_wrong_color_gain*node.wrong_color
    
    def get_possible_paths_from_node(self,node:TreeNode, tri):
        possible_tri = tri[np.where(tri == node.key[0])[0]]
        possible_tri = possible_tri[np.where(possible_tri == node.key[1])[0]]

        if node.parentNumber == 1:
            indexes = np.where(possible_tri == node.parent.key)[0]
            possible_tri = np.delete(possible_tri,indexes, axis=0)
            a = np.delete(possible_tri,np.where(possible_tri == node.key[0])[1])
            b = np.delete(possible_tri,np.where(possible_tri == node.key[1])[1])
            possible_paths = np.array([a,b])
        else:
            a = 1 if node.parent.key[0] in node.key else 0
            indexes = np.where(possible_tri == node.parent.key[a])[0]
            if not(np.delete(possible_tri,indexes, axis=0).any()):
                return np.array([])
            possible_tri = np.delete(possible_tri,indexes, axis=0)[0]
            a = np.delete(possible_tri,np.where(possible_tri == node.key[0])[0])
            b = np.delete(possible_tri,np.where(possible_tri == node.key[1])[0])
            possible_paths = np.array([a,b])

        return possible_paths
    
    def get_path_from_tree(self, tree, node:TreeNode):
        path = []
        while node != tree.root:
            path.append(node.mean_point_coord)
            node = node.parent
        path.append(tree.root.mean_point_coord)
        path = np.array(path)
        return path[::-1]
    
    def plot_paths(self,tree, cones, v, tri, Car_Position, Car_Orientation):
        blue = np.array([i for i in cones if i[3]==0])
        yellow = np.array([i for i in cones if i[3]==2])

        plt.cla()
        for i,leaf in enumerate(tree.leaves):
            path = self.get_path_from_tree(tree, leaf)
            if i == 0:
                plt.plot(path[:,0],path[:,1],c="r")
            else:
                plt.plot(path[:,0],path[:,1],c="black")
            plt.text(path[:,0][-1],path[:,1][-1],str(round(leaf.wrong_color,2)),horizontalalignment ='center')
        plt.axis('equal')
        plt.scatter(Car_Position[0], Car_Position[1], color="k")
        plt.quiver(Car_Position[0], Car_Position[1], Car_Orientation[0], Car_Orientation[1], color="k")
        plt.scatter(blue[:,0], blue[:,1])
        plt.scatter(yellow[:,0], yellow[:,1])
        #plt.triplot(v[:, 0], v[:, 1], tri, c="black", linewidth = 0.5)
        plt.draw()
        plt.pause(0.05)

    def completed_lap(self, path):
        if len(path) < 2:  #pois se não vai dar "completed_path" no primeiro ponto
            return False 
    
        start_point = path[0]
        end_point = path[-1]
        distance = np.linalg.norm(end_point - start_point)

        if distance < self.lap_threshold_distance:
            return True
        
        else:
            return False

    def plan_path(self, obstacle_numpy_array, Vehicle_Pose: Vehicle_Pose):
        cones_array = obstacle_numpy_array

        Car_Position = np.array([Vehicle_Pose.x_position_track_reference_frame, Vehicle_Pose.y_position_track_reference_frame])
        Car_Orientation = np.array([np.cos(Vehicle_Pose.yaw_position_track_reference_frame),np.sin(Vehicle_Pose.yaw_position_track_reference_frame)])
        vertices = np.vstack([cones_array[:,:2], Car_Position])
        origin = len(vertices)-1

        tri  = Delaunay(vertices).simplices

        root = TreeNode(origin, vertices[origin])
        tree = Tree(root)

        for option in self.get_possible_paths_from_origin(tri,origin):
            newnode = TreeNode(option,self.get_mean_point(option,vertices), parent=root)
            self.calculate_cost(newnode, Car_Orientation, cones_array)
            tree.insertNode(newnode)
        
        for i in range(self.iterations_number):
            queue = tree.leaves[:self.beam_width].copy()
            for leaf in tree.leaves[self.beam_width:]:
                tree.removeNode(leaf)
            for node in queue:
                paths = self.get_possible_paths_from_node(node, tri)
                if not(paths.any()):
                    break
                for path in paths:
                    newnode = TreeNode(path,self.get_mean_point(path,vertices), parent=node)
                    self.calculate_cost(newnode, Car_Orientation, cones_array)
                    if  newnode.max_angle_change < np.pi/2:
                        tree.insertNode(newnode)
        self.path = self.get_path_from_tree(tree, tree.leaves[0])
        
        return self.path

    def get_waypoints(self, obstacle_numpy_array, Vehicle_Pose: Vehicle_Pose):
        if not self.lap_completed:
            mean_points = self.plan_path(obstacle_numpy_array, Vehicle_Pose)
            if self.completed_lap(mean_points):
                self.lap_completed = True
                self.waypoints = mean_points 
            self.waypoints_array = np.array(self.waypoints)

        return self.waypoints_array
    
    def continuous_path(self, obstacle_numpy_array, Vehicle_Pose: Vehicle_Pose):
        Car_Position = np.array([Vehicle_Pose.x_position_track_reference_frame, Vehicle_Pose.y_position_track_reference_frame])
        new_path = self.plan_path(obstacle_numpy_array, Vehicle_Pose)
        new_path = new_path[1:] 

        if len(self.global_path) < 2:
            for point in new_path:
                self.global_path = np.append(self.global_path, [point], axis=0)
            return self.global_path

        point_last = self.global_path[-1]      # Último ponto
        point_second_last = self.global_path[-2]  # Penúltimo ponto
        current_angle = np.arctan2(point_last[1] - point_second_last[1], point_last[0] - point_second_last[0])

        for i, point in enumerate(new_path):
            candidate_angle = np.arctan2(point[1] - point_last[1], point[0] - point_last[0])
            angle_diff = abs(candidate_angle - current_angle)

            if angle_diff > np.pi:
                angle_diff = 2 * np.pi - angle_diff

            if angle_diff < np.deg2rad(20):
                self.global_path = np.append(self.global_path, [point], axis=0)
            else:
                break 

        return self.global_path

    def generate_closed_path(self, obstacle_numpy_array, initial_pose: Vehicle_Pose, distance_threshold=0.5, max_iterations=100):
        """
        Gera um caminho fechado de waypoints que retorna próximo ao ponto inicial.
        
        Args:
            obstacle_numpy_array: np.ndarray com os cones.
            initial_pose: Posição inicial do veículo (referência para retorno).
            distance_threshold: Distância para considerar que retornou ao ponto inicial.
            max_iterations: Limite de iterações para evitar loop infinito.

        Returns:
            Lista de waypoints que formam o caminho fechado.
        """
        self.waypoints = []  # Limpa os waypoints antigos
        current_pose = initial_pose

        for _ in range(max_iterations):
            new_points = self.get_waypoints(obstacle_numpy_array, current_pose)

            # Atualiza pose atual com o último ponto gerado
            if len(new_points) == 0 or len(new_points[-1]) == 0:
                break
            
            last_waypoint = new_points[-1][-1]  # último ponto da última chamada
            current_pose = Vehicle_Pose()
            current_pose.x_position_track_reference_frame = last_waypoint[0]
            current_pose.y_position_track_reference_frame = last_waypoint[1]
            current_pose.yaw_position_track_reference_frame = 0.0  # ou calcula baseado em vetor tangente

            # Verifica se retornou ao ponto inicial
            dx = current_pose.x_position_track_reference_frame - initial_pose.x_position_track_reference_frame
            dy = current_pose.y_position_track_reference_frame - initial_pose.y_position_track_reference_frame
            distance_to_start = np.hypot(dx, dy)

            if distance_to_start <= distance_threshold:
                break

        # Junta todos os waypoints em uma única lista
        closed_path = [point for segment in self.waypoints for point in segment]
        return closed_path    

    def get_interpolated_path(self, obstacle_numpy_array, Vehicle_Pose: Vehicle_Pose):
        #interpola os pontos do path normal e concatenado e retorna os dois
        
        normal_path = self.plan_path(obstacle_numpy_array, Vehicle_Pose)
        concatenated_path = self.continuous_path(obstacle_numpy_array, Vehicle_Pose)

        if len(normal_path) > self.limit_size:
            normal_path = normal_path[(-self.limit_size):]

        if len(concatenated_path) > self.limit_size:
            concatenated_path = concatenated_path[(-self.limit_size):]
        
        distance_normalpath = np.cumsum( np.sqrt(np.sum( np.diff(normal_path, axis=0)**2, axis=1 )) )
        distance_normalpath = np.insert(distance_normalpath, 0, 0)/distance_normalpath[-1]
        interpolator_normalpath =  interp1d(distance_normalpath, normal_path, kind="slinear", axis=0)

        distance_concatpath = np.cumsum( np.sqrt(np.sum( np.diff(concatenated_path, axis=0)**2, axis=1 )) )
        distance_concatpath = np.insert(distance_concatpath, 0, 0)/distance_concatpath[-1]
        interpolator_concatpath =  interp1d(distance_concatpath, concatenated_path, kind="slinear", axis=0)

        return interpolator_normalpath(np.linspace(0,1,100)), interpolator_concatpath(np.linspace(0,1,100))
    
'''<3 Diva <3
    oiiii yasmin'''
