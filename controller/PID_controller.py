class PIDController:
    def __init__(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D

        self.error = 0.0
        self.prev_error = 0.0
        self.accum_error = 0.0

        self.control = 0.0
    
    def update(self, cur_val, target):
        self.prev_error = self.error
        self.error = target - cur_val
        self.accum_error -= self.error
        self.diff_error = self.error - self.prev_error

        self.control = self.P * self.error +\
                       self.I * self.accum_error +\
                       self.D * self.diff_error
    
    def reset(self):
        self.error = 0.0
        self.prev_error = 0.0
        self.accum_error = 0.0
        
    def getControl(self):
        return self.control


    
    