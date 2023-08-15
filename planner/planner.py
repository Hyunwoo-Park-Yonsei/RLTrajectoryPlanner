from planner.quintic_polynomial_generator import QuinticPolynomialGenerator
from visualizer.visualizer import Visualizer
import math

class OptimalTrajectoryPlanner:
    def __init__(self, ego_lat_pos, ego_lat_speed, ego_lat_acc, ego_long_speed, ego_long_acc, oa_pos, oa_speed, oa_heading, ego_lane_idx):
        self.visualizer = Visualizer()
        
        self.ego_lat_pos = ego_lat_pos
        self.ego_lat_speed = ego_lat_speed
        self.ego_lat_acc = ego_lat_acc

        self.ego_long_speed = ego_long_speed
        self.ego_long_acc = ego_long_acc

        self.init_speed = math.sqrt(ego_lat_speed ** 2 + ego_long_speed ** 2)
     
        self.lane_width = 3.5 # m
        self.low_speed_ths = 10/3.6 # 10 km/h
        self.planning_horizon = 10 # second
        self.dt = 0.5
        self.jerk_weight = 1.0
        self.length_weight = 1.0
        self.lateral_error_weight = 1.0
        self.longitudinal_error_weight = 1.0
        self.time_weight = 10.
        
        self.target_d = [- 4 - self.ego_lat_pos, - self.ego_lat_pos, 4 - self.ego_lat_pos]
        default_target_s = ego_long_speed * self.planning_horizon
        self.target_s = [default_target_s * 0.5, default_target_s, default_target_s * 1.5]

        self.long_control_time = 2 # s

        self.control_target_d = 0.0
        self.control_target_s = 0.0

        self.oa_pos = oa_pos
        self.oa_speed = oa_speed
        self.oa_heading = oa_heading

        self.ego_lane_idx = ego_lane_idx

        self.lat_param = None
        self.long_param = None
        self.exit_flag = False
        
    
    def lateralPlanning(self, target_d):
        target_s = self.low_speed_ths * self.planning_horizon
        if self.init_speed > self.low_speed_ths:
            target_s = self.init_speed * self.planning_horizon
        # print("target_s", target_s, "target_d", target_d, "init speed,",self.init_speed)
        
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
                # print("s",s,"lat length", lat_length, "long length", long_length, "length",math.sqrt(lat_length ** 2 + long_length ** 2))
                total_length += math.sqrt(lat_length ** 2 + long_length ** 2)

            # lateral error
            d = params[0] + params[1] * s + params[2] * (s ** 2) + params[3] * (s ** 3) + params[4] * (s ** 4) + params[5] * (s ** 5)
            lateral_error = (target_d - d) ** 2
            total_lateral_error += lateral_error
            s += ds
        
        cost = self.jerk_weight * total_jerk + self.length_weight * total_length + self.lateral_error_weight * total_lateral_error
        # print("jerk : ", total_jerk, "total_length : ", total_length, "total_lateral_error : ", total_lateral_error)
        return params, cost

    def longitudinalPlanning(self, target_s):
        # s(t)
        target_s = self.low_speed_ths * self.planning_horizon
        if self.init_speed > self.low_speed_ths:
            target_s = self.init_speed * self.planning_horizon
        
        quinticPolynomialGenerator = QuinticPolynomialGenerator(self.ego_lat_pos, self.ego_lat_speed, self.ego_lat_acc, target_s, self.planning_horizon)
        quinticPolynomialGenerator.calculate()
        params = quinticPolynomialGenerator.getParams()

        total_jerk, total_longitudinal_error = 0, 0
        dt =  self.dt
        # for s in range(0, planning_dist, ds):
        t = 0
        while t < self.planning_horizon:
            jerk, time, longitudinal_error = 0, 0, 0
            # jerk
            if t < self.planning_horizon - dt * 2:
                # s_list = [s_0, s_1, s_2, s_3]
                # s_0 = s
                # s_1 = s + self.dt * init_speed
                # s_2 = s + self.dt * init_speed * 2
                # s_3 = s + self.dt * init_speed * 3
                t_list = []
                for i in range(4):
                    t_list.append(t + dt * i)
                # d_list = [d_0, d_1, d_2, d_3]
                s_list = []
                for t_ in t_list:
                    s_list.append(params[0] + params[1] * t_ + params[2] * (t_ ** 2) + params[3] * (t_ ** 3) + params[4] * (t_ ** 4) + params[5] * (t_ ** 5))
                
                # jerk = {a_1 - a_0} / dt
                #      = {(v_2 - v_1) - (v_1 - v_0)} /dt^2 = {v_2 - 2 v_1 + v_0} /dt^2
                #      = {(p_3 - p_2) - 2 (p_2 - p_1) + (p_1 - p_0)} /dt^3
                #      = {p_3 - 3 p_2 + 3 p_1 - p_0} /dt^3
                long_jerk = (t_list[3] - 3 * t_list[2] + 3 * t_list[1] - t_list[0]) / (self.dt ** 3)
                lat_jerk = (s_list[3] - 3 * s_list[2] + 3 * s_list[1] - s_list[0]) / (self.dt ** 3)
                jerk = math.sqrt(long_jerk ** 2 + lat_jerk ** 2)    
                total_jerk += jerk

            # longitudinal error
            s = params[0] + params[1] * t + params[2] * (t ** 2) + params[3] * (t ** 3) + params[4] * (t ** 4) + params[5] * (t ** 5)
            longitudinal_error = (target_s - s) ** 2
            total_longitudinal_error += longitudinal_error
            t += dt
        
        cost = self.jerk_weight * total_jerk + self.time_weight * self.planning_horizon + self.longitudinal_error_weight * total_longitudinal_error

        return params, cost
        
    def isCollision(self, lat_param, long_param):
        for t in range(0, self.planning_horizon):
            s = long_param[0] + long_param[1] * t + long_param[2] * (t ** 2) + long_param[3] * (t ** 3) + long_param[4] * (t ** 4) + long_param[5] * (t ** 5) 
            d = lat_param[0] + lat_param[1] * s + lat_param[2] * (s ** 2) + lat_param[3] * (s ** 3) + lat_param[4] * (s ** 4) + lat_param[5] * (s ** 5)
            for oa_idx in range(len(self.oa_pos)):
                oa_ds = self.oa_speed[oa_idx] * math.cos(self.oa_heading[oa_idx])
                oa_dd = self.oa_speed[oa_idx] * math.sin(self.oa_heading[oa_idx])
                oa_predicted_pos_s = self.oa_pos[oa_idx][0] + oa_ds * t 
                oa_predicted_pos_d = self.oa_pos[oa_idx][1] + oa_dd * t 
                if math.sqrt((s - oa_predicted_pos_s)**2 + (d - oa_predicted_pos_d)**2) < 4:
                    # print("ego :[",s,d,"]oa_predicted_pos :[",oa_predicted_pos_s,oa_predicted_pos_d,"]")
                    return True
        return False

    
    def selectTrajectory(self, lat_params, lat_costs, lat_target_d, long_params, long_costs):
        while True:
            if len(lat_costs) == 0:
                return None, None
            min_cost = 1e9
            min_lat_idx = 0
            for i in range(len(lat_costs)):
                if lat_costs[i] < min_cost:
                    min_cost = lat_costs[i]
                    min_lat_idx = i
            
            min_cost = 1e9
            min_long_idx = 0
            for i in range(len(long_costs)):
                if long_costs[i] < min_cost:
                    min_cost = long_costs[i]
                    min_long_idx = i

            if self.isCollision(lat_params[min_lat_idx], long_params[min_long_idx]):
                del lat_params[min_lat_idx]
                del lat_costs[min_lat_idx]
                del long_params[min_long_idx]
                del long_costs[min_long_idx]
                # print("Collision!!!!!!!!!!!")
            else:
                return lat_params[min_lat_idx], long_params[min_long_idx]
                

    def plan(self):
        lat_costs = []
        lat_params = []
        lat_target_d = []

        default_lat_param, _ = self.lateralPlanning(self.target_d[1])
        for target_d in self.target_d:
            if target_d < 0 and self.ego_lane_idx == 0:
                continue
            if target_d > 0 and self.ego_lane_idx == 4:
                continue
            param, cost = self.lateralPlanning(target_d)
            lat_costs.append(cost)
            lat_params.append(param)
            lat_target_d.append(target_d)
            
        
        long_costs = []
        long_params = []
        default_long_param = []
        for target_s in self.target_s:
            param, cost = self.longitudinalPlanning(target_s)
            long_costs.append(cost)
            long_params.append(param)
            if target_s == self.target_s[0]:
                default_long_param = param

        lat_param,long_param = self.selectTrajectory(lat_params, lat_costs, lat_target_d, long_params, long_costs)

        if lat_param == None:
            lat_param = default_lat_param
            long_param = default_long_param
            print("No Available Trajectory")
            print(lat_param)
        
        ################################################
        s = self.low_speed_ths * self.planning_horizon
        if self.init_speed > self.low_speed_ths:
            s = self.init_speed * self.planning_horizon
        ################################################
        print("Chose Trajectory D", lat_param[0] + lat_param[1] * s + lat_param[2] * (s ** 2) + lat_param[3] * (s ** 3) + lat_param[4] * (s ** 4) + lat_param[5] * (s ** 5) )
        s = self.planning_horizon
        print("Chose Trajectory S", long_param[0] + long_param[1] * s + long_param[2] * (s ** 2) + long_param[3] * (s ** 3) + long_param[4] * (s ** 4) + long_param[5] * (s ** 5) )

        t = self.long_control_time
        self.control_target_s = long_param[0] + long_param[1] * t + long_param[2] * (t ** 2) + long_param[3] * (t ** 3) + long_param[4] * (t ** 4) + long_param[5] * (t ** 5) 
        
        s = self.control_target_s
        self.control_target_d = lat_param[0] + lat_param[1] * s + lat_param[2] * (s ** 2) + lat_param[3] * (s ** 3) + lat_param[4] * (s ** 4) + lat_param[5] * (s ** 5) 

        self.lat_param = lat_param
        self.long_param = long_param

        planning_dist = max(self.planning_horizon * self.init_speed, self.planning_horizon * self.low_speed_ths)
        print(self.ego_lat_pos, self.ego_lat_speed, self.ego_lat_acc)
        # self.visualizer.polynomial(lat_param, planning_dist)
        
    
    def getControlPoint(self):
        return self.control_target_d, self.control_target_s
    
    def getPurePursuitControl(self):
        print("control s d",self.control_target_s,self.control_target_d)
        
        yaw = math.atan2(self.ego_lat_speed,self.ego_long_speed)
        theta = math.atan2(self.control_target_d,self.control_target_s)
        alpha = theta - yaw
        L = 5 # length between rear axis front axis
        l_d = math.sqrt(self.control_target_s**2 + self.control_target_d**2)
        sigma = math.atan(2 * L * math.sin(alpha)/l_d)
        print(2 * L * math.sin(alpha)/l_d)
        print("alpha",alpha,"sigma",sigma,"yaw",yaw,"theta",theta)
        return sigma

    def getStanleyControl(self):
        print("control s",self.control_target_s,"control d",self.control_target_d)
        
        yaw = math.atan2(self.ego_lat_speed,self.ego_long_speed)
        theta = math.atan2(self.control_target_d,self.control_target_s)
        alpha = theta - yaw

        l_d = math.sqrt(self.control_target_s**2 + self.control_target_d**2)
        lateral_error = l_d * math.sin(alpha)
        lateral_weight = 1
        lateral_control = math.atan2(lateral_weight * lateral_error, self.ego_long_speed)


        s_0 = self.control_target_s - 0.1
        s_1 = self.control_target_s + 0.1
        d_0 = self.lat_param[0] + self.lat_param[1] * s_0 + self.lat_param[2] * (s_0 ** 2) + self.lat_param[3] * (s_0 ** 3) + self.lat_param[4] * (s_0 ** 4) + self.lat_param[5] * (s_0 ** 5) 
        d_1 = self.lat_param[0] + self.lat_param[1] * s_1 + self.lat_param[2] * (s_1 ** 2) + self.lat_param[3] * (s_1 ** 3) + self.lat_param[4] * (s_1 ** 4) + self.lat_param[5] * (s_1 ** 5) 
        yaw_error = math.atan2(d_1-d_0,s_1-s_0) - yaw
        heading_control = yaw_error
        print("lateral_control", lateral_control, "heading_control", heading_control)
        print("s_0",s_0,"s_1",s_1,"d_0",d_0,"d_1", d_1, "angle", math.atan2(d_1-d_0,s_1-s_0), "yaw", yaw)
        # if self.exit_flag:
        #     exit()
        return (lateral_control + heading_control) * 0.05
    

        
        

    def test(self):
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
        target_d = -0.2874811844872065
        params, cost = self.lateralPlanning(target_d)
        planning_dist = max(self.planning_horizon * self.init_speed, self.planning_horizon * self.low_speed_ths)
        # print("cost : ", cost)
        self.visualizer.polynomial(params, planning_dist)



