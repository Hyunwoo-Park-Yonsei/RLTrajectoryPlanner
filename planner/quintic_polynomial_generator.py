import numpy as np

class QuinticPolynomialGenerator:
    def __init__(self, init_d, init_d_dot, init_d_ddot, target_d, T, target_d_dot = 0):
        self.init_d = init_d
        self.init_d_dot = init_d_dot
        self.init_d_ddot = init_d_ddot
        self.target_d = target_d
        self.target_d_dot = target_d_dot
        self.T = T

        self.a0 = self.init_d
        self.a1 = self.init_d_dot
        self.a2 = 0.5 * self.init_d_ddot
        self.a3 = 0
        self.a4 = 0
        self.a5 = 0
    
    def calculate(self):
        A = np.array([[  (self.T ** 3),      (self.T ** 4),      (self.T ** 5)],
                    [3 * (self.T ** 2),  4 * (self.T ** 3),  5 * (self.T ** 4)],
                    [       6 * self.T, 12 * (self.T ** 2), 20 * (self.T ** 3)]])
        # x (a3, a4, a5)
        B = np.array([self.target_d - (self.init_d + self.init_d_dot * self.T + 0.5 * self.init_d_ddot * (self.T ** 2)),
                      self.target_d_dot -(self.init_d_dot + self.init_d_ddot * self.T),
                      -(self.init_d_ddot)])
        x = np.linalg.inv(A).dot(B)
        # x = B.dot(np.linalg.inv(A)))

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

        
        
    def getParams(self):
        return (self.a0, self.a1, self.a2, self.a3, self.a4, self.a5)
