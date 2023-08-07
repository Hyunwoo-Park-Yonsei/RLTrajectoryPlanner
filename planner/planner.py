from planner.quintic_polynomial_generator import QuinticPolynomialGenerator
from visualizer.visualizer import Visualizer

class OptimalTrajectoryPlanner:
    def __init__(self):
        self.visualizer = Visualizer()

    def test(self):
        init_d = 0
        init_d_dot = 0
        init_d_ddot = 0
        target_d = 4
        T = 10

        # lateral planning
        # if low speed t(horizontal axis) should be in meter
        # if high speed t(horizontal axis) is time
        
        # longitudinal planning
        # t(horizontal axis) is meter
        quinticPolynomialGenerator = QuinticPolynomialGenerator(init_d, init_d_dot, init_d_ddot, target_d, T)
        quinticPolynomialGenerator.calculate()
        params = quinticPolynomialGenerator.getParams()

        
        self.visualizer.polynomial(params, T)



