class PIDController:

    def __init__(self, kp, ki, kd, ts, maxSignal=1.0, minSignal=-1.0, ka=0.0, bias=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ts = ts
        self.tau = 50*ts
        self.alpha = self.ts/(self.ts+self.tau)
        self.ka = ka
        self.u = 0.0
        self.error = [0.0, 0.0]
        self.filteredMeasure = [0.0, 0.0]
        self.maxSignal = maxSignal
        self.minSignal = minSignal
        self.bias = bias

        self.initialized = True

    def update_signal (self, reference, measure):
        self.measure = measure
        if self.initialized == True:
            self.initialized = False
            self.filteredMeasure = [measure, measure]
        self.error[0] = reference - measure

        self.filteredMeasure[0] = self.measure*self.alpha + self.filteredMeasure[1]*(1-self.alpha)

        self.u = self.u + self.kp*(self.error[0] - self.error[1]) + self.ki*self.ts*self.error[0] -self.kd*(self.filteredMeasure[0]- self.filteredMeasure[1])/self.ts
        
        if self.u > self.maxSignal:
            uSat = self.maxSignal
        elif self.u < self.minSignal:
            uSat = self.minSignal
        else:
            uSat = self.u

        if self.ki != 0.0:
            self.u = self.u + self.ka*self.ts*(uSat-self.u)
        else:
            if uSat > 0.5:
                uSat += self.bias
            elif uSat < -0.5:
                uSat -= self.bias
            uSat = max(self.minSignal, min(uSat, self.maxSignal))
        
        self.error[1] = self.error[0]
        self.filteredMeasure[1] = self.filteredMeasure[0]
        return float(uSat), float(self.error[0])