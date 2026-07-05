class PDController:

    def __init__(self, kp, kd, ts, bias, max_signal=1.0, min_signal=-1.0):
        self.kp = kp
        self.kd = kd
        self.ts = ts
        self.bia = bias
        self.max_signal = max_signal
        self.min_signal = min_signal
        self.previous_measure = 0

    def update_signal (self, reference, measure):
        erro = reference - measure
        P = self.kp*erro
        D = -self.kd*(measure - self.previous_measure)/self.ts
        self.previous_measure = measure
        sinal_controle = (P + D)
        if sinal_controle > 0.5:
            sinal_controle += self.bia
        if sinal_controle < -0.5:
            sinal_controle -= self.bia
        sinal_controle_limitado = max(self.min_signal, min(sinal_controle, self.max_signal) )
        
        return float(sinal_controle), float(sinal_controle_limitado), float(erro)