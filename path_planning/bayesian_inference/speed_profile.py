import numpy as np

class SpeedProfile:
  def __init__(self, max_acceleration, braking_acceleration, lateral_acceleration, max_speed):
      self.max_acceleration = max_acceleration
      self.braking_acceleration = braking_acceleration
      self.lateral_acceleration = lateral_acceleration
      self.max_speed = max_speed
    
  def path_curvature(self, path):
      x_position = []
      y_position = []
      for point in path:
          x_position.append(point[0])
          y_position.append(point[1])
      x_position_array = np.array(x_position)
      y_position_array = np.array(y_position)
    
      x_first_derivative = np.gradient(x_position_array)
      x_second_derivative = np.gradient(x_first_derivative)
      y_first_derivative = np.gradient(y_position_array)
      y_second_derivative = np.gradient(y_first_derivative)
      curvature = (x_first_derivative * y_second_derivative - x_second_derivative * y_first_derivative) / ((x_first_derivative**2 + y_first_derivative**2)**1.5)
      return curvature
  
  def centripetal_speed(self, path):
      vehicle_speed = np.zeros(len(path))
      for i in range(len(path)):
        if self.path_curvature(path)[i] != 0:
            R = 1/self.path_curvature(path)
            vehicle_speed[i] += ((self.lateral_acceleration*abs(R[i]))**0.5)
        if vehicle_speed[i] > self.max_speed:
            vehicle_speed[i] = self.max_speed
      return vehicle_speed
    
  def speed_profile(self, path, final_speed = 0):
      speed_profile = np.zeros(len(path)) 
      acceleration = self.braking_acceleration
      #speed_profile[-1] = final_speed
      for i in range(len(path) - 2, -1, -1): 
           l = np.linalg.norm(path[i] - path[i + 1]) 
           speed_squared = speed_profile[i + 1]**2 - 2 * acceleration * l
           speed_profile[i] = abs(speed_squared)**0.5
      if speed_profile[i] > self.max_speed:
          speed_profile[i] = self.max_speed
      return speed_profile

  def max_braking_distance(self, path):
      vehicle_speed = self.speed_profile(path, self.braking_acceleration)
      braking_distance = np.zeros(len(path))
      for i in range(len(path)):
            braking_distance[i] += (abs((vehicle_speed[i]**2 - vehicle_speed[i-1]**2) / (2 * self.braking_acceleration)))*3
      return braking_distance
  
  def forward_pass(self, path, centripetal_speed):
      speed = np.zeros(1)
      speed = np.insert(speed, 0, centripetal_speed)
      for i in range(len(path) - 1):
           l = np.linalg.norm(path[i] - path[i + 1])
           speed[i] += np.sqrt(abs((speed[i+1]**2 + 2 * self.max_acceleration * l)))
           if speed[i] > self.max_speed:
            speed[i] = self.max_speed
      return speed[:len(speed) - 1]
  
  def backward_pass(self, path, centripetal_speed):
      speed = np.zeros(1)
      speed = np.insert(speed, 0, centripetal_speed)
      braking_distance = self.max_braking_distance(path)
      for i in range(len(path) - 1):
           speed[i - 1] += np.sqrt(abs(speed[i]**2 + 2 * self.max_acceleration * braking_distance[i]))
           if speed[i - 1] > self.max_speed:
              speed[i - 1] = self.max_speed
      return speed[:len(speed)-1]
  
  def compute_speed_profile(self, path):
      centripetal_speed = self.centripetal_speed(path)
      speed = np.full(len(path), self.max_speed)
      if self.path_curvature(path)[-1] != 0: 
            forward_profile = self.forward_pass(path, centripetal_speed)
            backward_profile = self.backward_pass(path, centripetal_speed)
      elif self.path_curvature(path)[-1] == 0:
            forward_profile = self.forward_pass(path, speed)
            backward_profile = self.backward_pass(path, speed)

      speed_profile = []
      for i in range(len(forward_profile)):
            if forward_profile[i] < backward_profile[i]:
                speed_profile.append(forward_profile[i])
            else:
                speed_profile.append(backward_profile[i])
      speed_profile = np.array(speed_profile)
      #speed_profile[0] = 0
      speed_profile[-1] = 0
      return speed_profile
