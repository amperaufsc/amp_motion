from longitudinal_control.PID_controller import PIDController


class Longitudinal_Controller:
    def __init__(self, Kp, Ki, Kd, T, reference, max_signal=1.0):
        self.reference = reference
        # max_signal = limite REAL de throttle: o anti-windup do PID precisa
        # enxergar o mesmo teto que o atuador, senao o integrador enche ate o
        # teto interno (1.0) durante a aceleracao e a velocidade passa longe
        # da referencia enquanto ele desenrola
        self.controller = PIDController(Kp, Ki, Kd, T, max_signal=max_signal)
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