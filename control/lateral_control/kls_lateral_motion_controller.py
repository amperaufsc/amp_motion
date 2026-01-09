import numpy as np

from include.vehicle_parameters import Vehicle_Parameters
from include.kls_lateral_motion_controller_gains import KLS_Lateral_Motion_Controller_Gains

class KLS_Lateral_Motion_Controller:
    def __init__(self, vehicle_parameters: Vehicle_Parameters, controller_gains: KLS_Lateral_Motion_Controller_Gains, look_ahead_horizon = 3):
        self.vehicle_parameters = vehicle_parameters
        self.controller_gains = controller_gains
        self.look_ahead_horizon = look_ahead_horizon
        self.car_axle_length = vehicle_parameters.axle_length
        self.steering_up_limit = vehicle_parameters.steering_up_limit
        self.steering_down_limit = vehicle_parameters.steering_down_limit
        self.eh = 0.0
        self.ey = 0.0

    def get_cross_error(self, position, reference_trajectory, reference_point_orientation):
        Prf = reference_trajectory[0] - position
        Trf = reference_point_orientation
        return np.cross(Prf,Trf)/np.linalg.norm(Trf)
    
    '''
    def get_cross_error(self, position, reference_trajectory, reference_point_orientation):
        Prf = reference_trajectory[0] - position
        Trf = reference_point_orientation
        return (Prf[0]*Trf[1]-Prf[1]*Trf[0])/np.linalg.norm(Trf)
    '''
    def get_heading_error(self, vehicle_state, reference_point_orientation):
        Trf = reference_point_orientation
        v = np.array([vehicle_state.x_velocity , vehicle_state.y_velocity])
        Trf_norm = np.linalg.norm(Trf)
        v_norm = np.linalg.norm(v)
        print(Trf_norm, v_norm)
        return np.arcsin(round(np.cross(Trf,v)/(Trf_norm * v_norm),4))
    
    def get_krp(self, reference_trajectory, vehicle_state):
        points = reference_trajectory[:self.look_ahead_horizon]
        x = points[:,0]
        y = points[:,1]
        a,b,c = np.polyfit(x,y,2)
        x_rp = reference_trajectory[0][0] - vehicle_state.x_position

        krp = 2*a/(1+(2*a*x_rp+b)**2)**(3/2)
        return krp

    def update_steering_angle_control_signal(self, reference_trajectory, measured_state):
        car_position = [measured_state.x_position, measured_state.y_position]
        reference_point_orientation = reference_trajectory[1]-reference_trajectory[0]
        reference_point_orientation = reference_point_orientation/np.linalg.norm(reference_point_orientation)

        krp = self.get_krp(np.array(reference_trajectory), measured_state)
        ey = self.get_cross_error(car_position, reference_trajectory, reference_point_orientation)
        eh = self.get_heading_error(measured_state, reference_point_orientation)
        Kh = self.controller_gains.orientation_error_gain
        Ky = self.controller_gains.lateral_position_error_gain
        L = self.car_axle_length
        vx = np.linalg.norm([measured_state.x_velocity, measured_state.y_velocity])
        steering_angle = 0
        if vx >= 0.8:
            steering_angle = np.arctan(L*(-Kh*np.sin(eh) - Kh*Ky*ey/vx + krp*np.cos(eh)/(1-krp*ey)))

        if steering_angle < self.steering_down_limit:
            steering_angle = self.steering_down_limit
        elif steering_angle > self.steering_up_limit:
            steering_angle = self.steering_up_limit
            
        self.ey=ey
        self.eh=np.rad2deg(eh)
        return steering_angle/self.steering_up_limit