class PDController:

    def __init__(self, Kp, Kd, Ts, max_signal=1.0, min_signal=-1.0):
        self.kp = Kp
        self.kd = Kd
        self.Ts = Ts
        self.tau = 0.2*Kd/Kp
        self.erro_filtrado = [0, 0]
        self.max_signal = max_signal
        self.min_signal = min_signal

    def update_signal (self, reference, measure):
        erro = reference - measure
        P = self.kp*erro
        self.erro_filtrado[0] = self.Ts/self.tau*erro + (1 - self.Ts/self.tau)*self.erro_filtrado[1]
        D = self.kd*((self.erro_filtrado[0] -self.erro_filtrado[1])/self.Ts)
        sinal_controle = P + D

        self.erro_filtrado[1] = self.erro_filtrado[0]


        sinal_controle = max(self.min_signal, min(sinal_controle, self.max_signal) )
        
        return float(sinal_controle), float(erro)