class PIDController:

    def __init__(self, Kp, Ki, Kd, T, Tt, max_signal=1.0, min_signal=-1.0):
        self.kp = Kp
        self.ki = Ki
        self.kd = Kd
        self.T = T
        self.Tt = Tt
        self.D_ant = 0.0
        self.I_ant = 0.0
        self.Tt_ant = 0.0
        self.erro_ant = 0.0
        self.max_signal = max_signal
        self.min_signal = min_signal

    def update_signal (self, reference, measure):
        erro = reference - measure
        P = self.kp*erro
        I = self.I_ant + self.ki*self.T*(erro + self.erro_ant)
        D = self.kd*((erro-self.erro_ant)/self.T)
        sinal_controle = P + I + D
        if sinal_controle >= self.max_signal:
            Tt = (self.Tt*self.T*(self.max_signal - sinal_controle))
            I += Tt
        elif sinal_controle < self.min_signal:
            Tt = (self.Tt*self.T*(self.min_signal - sinal_controle))
            I += Tt
        self.erro_ant = erro
        self.I_ant = I
        self.D_ant = D 

        sinal_controle_depois = sinal_controle

        if sinal_controle_depois >= self.max_signal:
            sinal_controle_depois = self.max_signal
        if sinal_controle_depois <= self.min_signal:
            sinal_controle_depois = self.min_signal
        
        return float(sinal_controle), float(sinal_controle_depois), float(erro)