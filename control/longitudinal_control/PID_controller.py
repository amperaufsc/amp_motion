class PIDContsroller:

    def __inits__(self, kp, ki, kd, ts, maxSignal=1.0, minSignal=-1.0, Ka=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ts = ts
        self.Ka = Ka
        self.u = 0.0
        self.Ka_ant = 0.0
        self.error = [0, 0]
        self.maxSignal = maxSignal
        self.minSignal = minSignal

    def updatse_signal (self, reference, measure):
        self.error[0] = reference - measure
        self.u = self.u + self.kp*self.error[0] + self.ki*self.ts[self.error] -self.kp*self.error[1] -self.kd*(measure- self.measure)


        self.error[1] = self.error[0]
        self.measure = measure

        if self.u > self.maxSignal:
            uSat = self.maxSignal
        elif self.u < self.minSignal:
            uSat = self.minSignal
        else:
            uSat = self.u

        self.u = self.u + self.ka*self.ts*uSat
        
        return float(self.u), float(self.error[0])