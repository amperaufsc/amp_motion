import numpy as np

#by Hessmann
#gabriel.hessmann.r@gmail.com
#git GabrielH2003

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
        return (Prf[0]*Trf[1]-Prf[1]*Trf[0])/np.linalg.norm(Trf)

    def get_heading_error(self, vehicle_state, reference_trajectory, reference_point_orientation):
        yaw = vehicle_state.yaw
        py = reference_point_orientation[1]
        px = reference_point_orientation[0]
        reference_orientation = np.arctan2(py,px) 
        return np.arctan2(np.sin(reference_orientation - yaw), np.cos(reference_orientation - yaw))   
    
    def get_krp(self, reference_trajectory, vehicle_state):
        # curvatura ASSINADA por 3 pontos (Menger): independente de frame.
        n = min(self.look_ahead_horizon, len(reference_trajectory))
        if n < 3:
            return 0.0            # fim do path: sem pontos p/ curvatura
        p1, p2, p3 = (np.asarray(reference_trajectory[0], float),
                      np.asarray(reference_trajectory[n // 2], float),
                      np.asarray(reference_trajectory[n - 1], float))
        a = p2 - p1
        b = p3 - p1
        cross = a[0] * b[1] - a[1] * b[0]
        d12 = np.linalg.norm(a); d13 = np.linalg.norm(b)
        d23 = np.linalg.norm(p3 - p2)
        denom = d12 * d13 * d23
        if denom < 1e-9:
            return 0.0
        krp = 2.0 * cross / denom          
        return float(np.clip(krp, -0.5, 0.5))   
                                                

    def update_steering_angle_control_signal(self, reference_trajectory, measured_state):
        car_position = [measured_state.x_position, measured_state.y_position]
        reference_point_orientation = reference_trajectory[1]-reference_trajectory[0] 
        reference_point_orientation = reference_point_orientation/np.linalg.norm(reference_point_orientation)

        krp = self.get_krp(np.array(reference_trajectory), measured_state)
        ey = self.get_cross_error(car_position, reference_trajectory, reference_point_orientation)
        eh = self.get_heading_error(measured_state, reference_trajectory, reference_point_orientation)
        Kh = self.controller_gains.orientation_error_gain
        Ky = self.controller_gains.lateral_position_error_gain
        L = self.car_axle_length
        vx = np.linalg.norm([measured_state.x_velocity, measured_state.y_velocity])
        steering_angle = 0
        if vx >= 0.8:
            # FIX de sinal: eh = ref - yaw, entao yaw a DIREITA da tangente da
            # eh>0 e o carro precisa esterca ESQUERDA (+Kh*sin(eh)). O -Kh
            # original desestabilizava o heading enquanto o termo de ey
            # estabilizava (paridades opostas -> ciclo-limite inevitavel).
            # Convencao final: steering_angle > 0 = esquerda (CCW).
            steering_angle = np.arctan(L*(Kh*np.sin(eh) - Kh*Ky*ey/vx + krp*np.cos(eh)/(1-krp*ey)))

        if steering_angle < self.steering_down_limit:
            steering_angle = self.steering_down_limit
        elif steering_angle > self.steering_up_limit:
            steering_angle = self.steering_up_limit
            
        self.ey=ey
        self.eh=np.rad2deg(eh)
        return steering_angle/self.steering_up_limit