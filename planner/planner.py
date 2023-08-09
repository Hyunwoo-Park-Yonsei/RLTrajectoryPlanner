from planner.quintic_polynomial_generator import QuinticPolynomialGenerator
from visualizer.visualizer import Visualizer
import math

class OptimalTrajectoryPlanner:
    def __init__(self, ego_lat_pos, ego_lat_speed, ego_lat_acc, ego_long_speed, ego_long_acc):
        self.visualizer = Visualizer()
        
        self.ego_lat_pos = ego_lat_pos
        self.ego_lat_speed = ego_lat_speed
        self.ego_lat_acc = ego_lat_acc

        self.ego_long_speed = ego_long_speed
        self.ego_long_acc = ego_long_acc

        self.init_speed = math.sqrt(ego_lat_speed ** 2 + ego_long_speed ** 2)
     
        self.lane_width = 3.5 # m
        self.low_speed_ths = 10/3.6 # 10 km/h
        self.planning_horizon = 5 # second
        self.dt = 0.5
        self.jerk_weight = 1.0
        self.length_weight = 1.0
        self.lateral_error_weight = 1.0

        
    
    def lateralPlanning(self, target_d):
        target_s = self.low_speed_ths * self.planning_horizon
        if self.init_speed > self.low_speed_ths:
            target_s = self.init_speed * self.planning_horizon


        quinticPolynomialGenerator = QuinticPolynomialGenerator(self.ego_lat_pos, self.ego_lat_speed, self.ego_lat_acc, target_d, target_s)
        quinticPolynomialGenerator.calculate()
        params = quinticPolynomialGenerator.getParams()

        total_jerk, total_length, total_lateral_error = 0, 0, 0
        ds =  max(self.dt * self.init_speed, self.dt * self.low_speed_ths)
        # for s in range(0, planning_dist, ds):
        s = 0
        while s < target_s:
            jerk, length, lateral_error = 0, 0, 0
            # jerk
            if s < target_s - ds * 2:
                # s_list = [s_0, s_1, s_2, s_3]
                # s_0 = s
                # s_1 = s + self.dt * init_speed
                # s_2 = s + self.dt * init_speed * 2
                # s_3 = s + self.dt * init_speed * 3
                s_list = []
                for i in range(4):
                    s_list.append(s + ds * i)
                # d_list = [d_0, d_1, d_2, d_3]
                d_list = []
                for s_ in s_list:
                    d_list.append(params[0] + params[1] * s_ + params[2] * (s_ ** 2) + params[3] * (s_ ** 3) + params[4] * (s_ ** 4) + params[5] * (s_ ** 5))
                
                # jerk = {a_1 - a_0} / dt
                #      = {(v_2 - v_1) - (v_1 - v_0)} /dt^2 = {v_2 - 2 v_1 + v_0} /dt^2
                #      = {(p_3 - p_2) - 2 (p_2 - p_1) + (p_1 - p_0)} /dt^3
                #      = {p_3 - 3 p_2 + 3 p_1 - p_0} /dt^3
                long_jerk = (s_list[3] - 3 * s_list[2] + 3 * s_list[1] - s_list[0]) / (self.dt ** 3)
                lat_jerk = (d_list[3] - 3 * d_list[2] + 3 * d_list[1] - d_list[0]) / (self.dt ** 3)
                jerk = math.sqrt(long_jerk ** 2 + lat_jerk ** 2)    
                total_jerk += jerk
            # length
            if s < target_s:
                s_0 = s
                s_1 = s + ds
                d_0 = params[0] + params[1] * s_0 + params[2] * (s_0 ** 2) + params[3] * (s_0 ** 3) + params[4] * (s_0 ** 4) + params[5] * (s_0 ** 5)
                d_1 = params[0] + params[1] * s_1 + params[2] * (s_1 ** 2) + params[3] * (s_1 ** 3) + params[4] * (s_1 ** 4) + params[5] * (s_1 ** 5)
                lat_length = abs(d_1 - d_0)
                long_length = ds
                print("s",s,"lat length", lat_length, "long length", long_length, "length",math.sqrt(lat_length ** 2 + long_length ** 2))
                total_length += math.sqrt(lat_length ** 2 + long_length ** 2)

            # lateral error
            d = params[0] + params[1] * s + params[2] * (s ** 2) + params[3] * (s ** 3) + params[4] * (s ** 4) + params[5] * (s ** 5)
            lateral_error = (target_d - d) ** 2
            total_lateral_error += lateral_error
            s += ds
        
        cost = self.jerk_weight * total_jerk + self.length_weight * total_length + self.lateral_error_weight * total_lateral_error
        print("jerk : ", total_jerk, "total_length : ", total_length, "total_lateral_error : ", total_lateral_error)
        return params, cost


    def test(self):

        target_d = 4
        T = 10

        # lateral planning
        # if low speed t(horizontal axis) should be in meter
        # if high speed t(horizontal axis) is time
        
        # longitudinal planning
        # t(horizontal axis) is meter

        ''' 
        for test 

        # quinticPolynomialGenerator = QuinticPolynomialGenerator(init_d, init_d_dot, init_d_ddot, target_d, T)
        # quinticPolynomialGenerator.calculate()
        # params = quinticPolynomialGenerator.getParams()

        '''
        
        # for lateralPlanning() test
        params, cost = self.lateralPlanning(target_d)
        planning_dist = max(self.planning_horizon * self.init_speed, self.planning_horizon * self.low_speed_ths)
        print("cost : ", cost)
        
        self.visualizer.polynomial(params, planning_dist)



