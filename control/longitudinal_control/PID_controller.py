class PIDController:

    def __init__(self, Kp, Ki, Kd, T, max_signal=1.0, min_signal=-1.0):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.T = T
        self.I_ant = 0.0
        self.erro_ant = 0.0
        self.max_signal = max_signal
        self.min_signal = min_signal

    def update_signal (self, reference, measure):
        erro = reference - measure
        P = self.kp*erro
        I = self.I_ant + self.ki*self.T*(erro + self.erro_ant)
        D = self.kd*((erro-self.erro_ant)/self.T)
        sinal_controle = P + I + D
        self.erro_ant = erro

        sinal_saturado = sinal_controle
        if sinal_saturado > self.max_signal:
            sinal_saturado = self.max_signal
        if sinal_saturado < self.min_signal:
            sinal_saturado = self.min_signal

        # anti-windup (back-calculation): a saida e' saturada mas o integrador
        I += sinal_saturado - sinal_controle
        self.I_ant = I

        return sinal_saturado