from longitudinal_control.PID_controller import PIDController


class Longitudinal_Controller:
    def __init__(self, Kp, Ki, Kd, T, reference):
        self.reference = reference
        self.controller = PIDController(Kp, Ki, Kd, T)
        self.erro = 0.0

    def update_torque_control_signal(self, reference_trajectory, measured_state):
        speed =((measured_state.x_velocity)**2.0 + (measured_state.y_velocity)**2.0)**0.5
        sinal_controler = self.controller.update_signal(self.reference, speed)
        self.erro = self.controller.erro_ant
        brake, throttle = 0.0 , 0.0

        if sinal_controler <= 0.0:
            brake = (-sinal_controler)

        if sinal_controler > 0.0:
            throttle = sinal_controler

        return throttle, brake