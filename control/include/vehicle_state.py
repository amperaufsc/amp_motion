class Vehicle_State:
    def __init__(self, global_x_position, global_y_position, yaw, body_x_linear_velocity, body_y_linear_velocity, body_z_angular_velocity):
        self.x_position = global_x_position
        self.y_position = global_y_position
        self.yaw = yaw
        self.x_velocity = body_x_linear_velocity
        self.y_velocity = body_y_linear_velocity
        self.z_velocity = 0