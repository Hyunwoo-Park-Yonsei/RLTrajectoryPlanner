from planner.quintic_polynomial_generator import QuinticPolynomialGenerator
from visualizer.visualizer import Visualizer
import math

class OptimalTrajectoryPlanner:
    def __init__(self, max_lane_idx, ego_lane_idx):
        self.visualizer = Visualizer()
        
        self.ego_lat_pos = 0
        self.ego_lat_speed = 0
        self.ego_lat_acc = 0

        self.ego_long_speed = 0
        self.ego_long_acc = 0

        self.init_speed = 0
     
        self.lane_width = 3.5 # m
        self.low_speed_ths = 10/3.6 # 10 km/h
        self.planning_horizon = 10 # second
        self.dt = 0.5
        self.jerk_weight = 1.0
        self.length_weight = 10.0
        self.lateral_error_weight = 1.0
        self.longitudinal_error_weight = 1.0
        self.lane_change_weight = -1.0
        self.time_weight = 10.0
        
        self.target_d = [- 4, 0, 4]
        self.target_s = []
        self.target_v = []
        self.target_a = []

        self.long_control_time = 5 # second

        self.control_target_d = 0.0
        self.control_target_s = 0.0
        self.control_target_v = 0.0

        self.oa_pos = 0
        self.oa_speed = 0
        self.oa_heading = 0

        self.lat_param = None
        self.long_param = None
        self.exit_flag = False

        self.selected_lane = ego_lane_idx
        self.prev_selected_idx = ego_lane_idx
        self.ego_lane_idx = 0
        self.prev_ego_lane_idx = 0

        self.is_lane_change = False
        self.max_lane_idx = max_lane_idx
        self.lat_control_weight = 0.1

        self.safe_dist = 15 # m
        self.target_speed = 7 # m/s
    
    def update(self, ego_lat_pos, ego_lat_speed, ego_lat_acc, ego_long_speed, ego_long_acc, oa_pos, oa_speed, oa_heading, ego_lane_idx):
        
        self.ego_lat_pos = ego_lat_pos
        self.ego_lat_speed = ego_lat_speed
        self.ego_lat_acc = ego_lat_acc

        self.ego_long_speed = ego_long_speed
        self.ego_long_acc = ego_long_acc

        self.init_speed = math.sqrt(ego_lat_speed ** 2 + ego_long_speed ** 2)
        
        self.target_d = [- 4 - self.ego_lat_pos, - self.ego_lat_pos, 4 - self.ego_lat_pos]

        self.oa_pos = oa_pos
        self.oa_speed = oa_speed
        self.oa_heading = oa_heading

        self.ego_lane_idx = ego_lane_idx
        print("ego_lat_pos", ego_lat_pos, "ego_lat_speed", ego_lat_speed, "ego_long_speed", ego_long_speed, "ego_lane_idx",ego_lane_idx)

        self.target_s = [self.ego_long_speed * self.planning_horizon]
        self.target_v = [self.ego_long_speed]
        self.target_a = [0,0]
        
        self.leading_vehicle_idx = self.findLeadingVehicleIdx()
        print("oa_pos",oa_pos)
        print("oa_speed",oa_speed)
        print("oa_heading",oa_heading)
        if self.leading_vehicle_idx >= 0:
            self.target_s.append(self.oa_pos[self.leading_vehicle_idx][0] + self.oa_speed[self.leading_vehicle_idx] * self.planning_horizon - self.safe_dist)
            self.target_v.append(self.oa_speed[self.leading_vehicle_idx])
            self.target_a.append(0)
        else:
            a = (self.target_speed - self.ego_long_speed) / self.planning_horizon
            target_s = self.ego_long_speed * self.planning_horizon + 0.5 * a * (self.planning_horizon ** 2)
            self.target_s.append(target_s)
            self.target_v.append(self.target_speed)
            self.target_a.append(0)

    
    def lateralPlanning(self, target_d):
        target_s = self.low_speed_ths * self.planning_horizon
        if self.init_speed > self.low_speed_ths:
            target_s = self.init_speed * self.planning_horizon
        # #print("target_s", target_s, "target_d", target_d, "init speed,",self.init_speed)
        
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
                # #print("s",s,"lat length", lat_length, "long length", long_length, "length",math.sqrt(lat_length ** 2 + long_length ** 2))
                total_length += math.sqrt(lat_length ** 2 + long_length ** 2)

            # lateral error
            d = params[0] + params[1] * s + params[2] * (s ** 2) + params[3] * (s ** 3) + params[4] * (s ** 4) + params[5] * (s ** 5)
            lateral_error = (target_d - d) ** 2
            total_lateral_error += lateral_error
            s += ds
        
        cost = self.jerk_weight * total_jerk + self.length_weight * total_length + self.lateral_error_weight * total_lateral_error + self.lane_change_weight * abs(target_d)
        # #print("jerk : ", total_jerk, "total_length : ", total_length, "total_lateral_error : ", total_lateral_error)
        return params, cost

    def longitudinalPlanning(self, target_v, target_s):
        # s(t)
        if self.init_speed < self.low_speed_ths:
            target_v = self.init_speed

        quinticPolynomialGenerator = QuinticPolynomialGenerator(0, self.ego_long_speed, self.ego_long_acc, target_s, self.planning_horizon, target_v)
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
            s_0 = params[0] + params[1] * t + params[2] * (t ** 2) + params[3] * (t ** 3) + params[4] * (t ** 4) + params[5] * (t ** 5)
            t_ = t + dt
            s_1 = params[0] + params[1] * t_ + params[2] * (t_ ** 2) + params[3] * (t_ ** 3) + params[4] * (t_ ** 4) + params[5] * (t_ ** 5)
            longitudinal_error = (self.target_speed - (s_1-s_0)/dt) ** 2
            total_longitudinal_error += longitudinal_error
            t += dt
        
        cost = self.jerk_weight * total_jerk + self.longitudinal_error_weight * total_longitudinal_error
        print("total_jerk", total_jerk, "total_longitudinal_error", total_longitudinal_error, "cost", cost)

        return params, cost
        
    def isCollision(self, lat_param, long_param):
        for t in range(0, self.planning_horizon):
            s = long_param[0] + long_param[1] * t + long_param[2] * (t ** 2) + long_param[3] * (t ** 3) + long_param[4] * (t ** 4) + long_param[5] * (t ** 5) 
            d = lat_param[0] + lat_param[1] * s + lat_param[2] * (s ** 2) + lat_param[3] * (s ** 3) + lat_param[4] * (s ** 4) + lat_param[5] * (s ** 5)
            if d + self.ego_lat_pos < 0 and d + self.ego_lat_pos > self.max_lane_idx * 4:
                # #print("ego :[",s,d,"]")
                return True
            for oa_idx in range(len(self.oa_pos)):
                oa_ds = self.oa_speed[oa_idx] * math.cos(self.oa_heading[oa_idx])
                oa_dd = self.oa_speed[oa_idx] * math.sin(self.oa_heading[oa_idx])
                oa_predicted_pos_s = self.oa_pos[oa_idx][0] + oa_ds * t 
                oa_predicted_pos_d = self.oa_pos[oa_idx][1] + oa_dd * t 
                if (math.sqrt((s - oa_predicted_pos_s)**2 + (d - oa_predicted_pos_d)**2) < 3) or\
                   (math.sqrt((s - oa_predicted_pos_s + 1.5)**2 + (d - oa_predicted_pos_d)**2) < 3) or\
                   (math.sqrt((s - oa_predicted_pos_s - 1.5)**2 + (d - oa_predicted_pos_d)**2) < 3):
                    # #print("oa", self.oa_pos[oa_idx][0], self.oa_pos[oa_idx][1] )
                    # #print("ego :[",s,d,"]oa_predicted_pos :[",oa_predicted_pos_s,oa_predicted_pos_d,"]")
                    return True
        # #print("NO COllision")
        return False

    
    def selectTrajectory(self, lat_params, lat_costs, long_params, long_costs):
        traj_costs = []
        traj_idxes = []
        for i in range(len(lat_params)):
            if len(lat_params[i]) == 0:
                continue
            # #print("lat_param Not zero")
            s = self.low_speed_ths * self.planning_horizon
            if self.init_speed > self.low_speed_ths:
                s = self.init_speed * self.planning_horizon
            # #print("D", lat_params[i][0] + lat_params[i][1] * s + lat_params[i][2] * (s ** 2) + lat_params[i][3] * (s ** 3) + lat_params[i][4] * (s ** 4) + lat_params[i][5] * (s ** 5) )
            for j in range(len(long_params)):
                ## temp code
                # j = len(long_params)//2
                ## ~temp code
                if not self.isCollision(lat_params[i], long_params[j]):
                    traj_costs.append(lat_costs[i]+long_costs[j])
                    traj_idxes.append([i,j])
                else:
                    traj_costs.append(2e10)
                    traj_idxes.append([i,j])
                # else:
                    # #print("lat costs",lat_costs[i])
        
        min_cost = 1e20
        min_idx = -1
        for i in range(len(traj_costs)):
            if traj_costs[i] < min_cost:
                min_idx = i
                min_cost = traj_costs[i]      
        if min_idx < 0:
            return None, None, None
        else:
            print("min idxes lat:",traj_idxes[min_idx][0],"long:",traj_idxes[min_idx][1])
            print("traj_idxes")
            print(traj_idxes)
            print("traj_costs")
            print(traj_costs)
            return lat_params[traj_idxes[min_idx][0]], long_params[traj_idxes[min_idx][1]], traj_idxes[min_idx][0] - len(self.target_d) // 2 + self.ego_lane_idx



        
            # if len(lat_costs) <= count:
            #     return None, None, None
            # min_cost = 1e20
            # min_lat_idx = 0
            # for i in range(len(lat_costs)):
            #     if lat_costs[i] < min_cost:
            #         min_cost = lat_costs[i]
            #         min_lat_idx = i
            
            # min_cost = 1e9
            # min_long_idx = 0
            # for i in range(len(long_costs)):
            #     if long_costs[i] < min_cost:
            #         min_cost = long_costs[i]
            #         min_long_idx = i
            # if lat_costs[min_lat_idx] >= 1e10:
            #     #print("every path is colliding")
            #     return None, None, None
            # try:
            #     if self.isCollision(lat_params[min_lat_idx], long_params[min_long_idx]):
            #         lat_costs[min_lat_idx] += 1e10
            #         long_costs[min_long_idx] += 1e10
            #         #print("Collision!!!!!!!!!!!")
            #     else:
            #         return lat_params[min_lat_idx], long_params[min_long_idx], min_lat_idx
            # except:
            #     #print("Something Got Wrong")
            #     #print(lat_params)
            #     #print(min_lat_idx)
            #     #print(lat_costs)
            #     #print(long_params)
            #     #print(min_long_idx)
            #     #print(long_costs)
            #     exit()
            # count += 1
                

    def plan(self):
        print("self.prev_selected_idx", self.prev_selected_idx)
        lat_costs = [0,0,0]
        lat_params = []
        lat_target_d = []

        if self.is_lane_change:
            print("is lanechanging")
            print("self.ego_lane_idx * 4 + self.ego_lat_pos : ",self.ego_lane_idx * 4 + self.ego_lat_pos)
            print("self.selected_lane * 4 : ", self.selected_lane* 4)
            if abs(self.ego_lane_idx * 4 + self.ego_lat_pos - self.selected_lane * 4) < 5:
                if abs(self.ego_lane_idx * 4 + self.ego_lat_pos - self.selected_lane * 4) < 0.5:
                    self.is_lane_change = False
                    print("lane change finish")
                elif abs(self.ego_lane_idx * 4 + self.ego_lat_pos - self.selected_lane * 4) < 2:
                    print("lane change processing1")
                    for i in range(len(lat_costs)):
                        if i != 1:
                            lat_costs[i] += 4e10
                        else:
                            print("selected dir",i)
                else:
                    print("lane change processing2")
                    for i in range(len(lat_costs)):
                        if i != self.selected_lane - self.ego_lane_idx + 1:
                            lat_costs[i] += 4e10
                        else:
                            print("selected dir",i)
            else:
                print("Something going wrong")
        else:
            print("Not lanechanging")
                

        default_lat_param, _ = self.lateralPlanning(self.target_d[1])
        for i in range(len(self.target_d)):
            # #print("target d ", self.target_d[i], "ego_lane_idx", self.ego_lane_idx)
            param, cost = self.lateralPlanning(self.target_d[i])
            if self.target_d[i] < -2 and self.ego_lane_idx == 0:
                # #print("Out of lane")
                param = []
                cost = 3e10
            if self.target_d[i] >2 and self.ego_lane_idx == self.max_lane_idx-1:
                # #print("Out of lane")
                param = []
                cost = 3e10
            lat_costs[i] += cost
            lat_params.append(param)
            lat_target_d.append(self.target_d[i])
            
        
        long_costs = []
        long_params = []
        default_long_param = []
        for i in range(len(self.target_v)):
            param, cost = self.longitudinalPlanning(self.target_v[i],self.target_s[i])
            long_costs.append(cost)
            long_params.append(param)
            if self.target_v[i] == self.target_v[len(self.target_v)-1]:
                default_long_param = param
        lat_param, long_param, selected_idx = self.selectTrajectory(lat_params, lat_costs, long_params, long_costs)
        self.selected_lane = selected_idx
        print("selected idx ", self.selected_lane, "self.prev_selected_idx", self.prev_selected_idx)
        
        #print("selected idx ", self.selected_lane, "self.prev_selected_idx", self.prev_selected_idx)
        #print(lat_param)
        

        if not self.is_lane_change and self.selected_lane != self.prev_selected_idx:
            self.is_lane_change = True
            # exit()
        # elif self.selected_lane != self.prev_selected_idx:
        #     exit()

        if lat_param == None:
            lat_param = default_lat_param
            long_param = default_long_param
            print("No Available Trajectory")
            #print(lat_param)
            exit()
        
        ################################################
        s = self.low_speed_ths * self.planning_horizon
        if self.init_speed > self.low_speed_ths:
            s = self.init_speed * self.planning_horizon
        ################################################
        #print("Chose Trajectory D", lat_param[0] + lat_param[1] * s + lat_param[2] * (s ** 2) + lat_param[3] * (s ** 3) + lat_param[4] * (s ** 4) + lat_param[5] * (s ** 5) )
        s = self.planning_horizon
        #print("Chose Trajectory S", long_param[0] + long_param[1] * s + long_param[2] * (s ** 2) + long_param[3] * (s ** 3) + long_param[4] * (s ** 4) + long_param[5] * (s ** 5) )

        t = self.long_control_time
        t_ = t + self.dt
        self.control_target_s = long_param[0] + long_param[1] * t + long_param[2] * (t ** 2) + long_param[3] * (t ** 3) + long_param[4] * (t ** 4) + long_param[5] * (t ** 5) 
        s_ = long_param[0] + long_param[1] * t_ + long_param[2] * (t_ ** 2) + long_param[3] * (t_ ** 3) + long_param[4] * (t_ ** 4) + long_param[5] * (t_ ** 5) 
        self.control_target_v = (s_ - self.control_target_s) / self.dt
        s = self.control_target_s
        self.control_target_d = lat_param[0] + lat_param[1] * s + lat_param[2] * (s ** 2) + lat_param[3] * (s ** 3) + lat_param[4] * (s ** 4) + lat_param[5] * (s ** 5) 

        self.lat_param = lat_param

        # planning_dist = max(self.planning_horizon * self.init_speed, self.planning_horizon * self.low_speed_ths)
        # #print(self.ego_lat_pos, self.ego_lat_speed, self.ego_lat_acc)
        # self.visualizer.polynomial(lat_param, planning_dist)
        print("4self.prev_selected_idx", self.prev_selected_idx)
        self.prev_selected_idx = self.selected_lane        
        print("5self.prev_selected_idx", self.prev_selected_idx)
    
    def getControlPoint(self):
        return self.control_target_d, self.control_target_v
    
    def getPurePursuitControl(self):
        #print("control s d",self.control_target_s,self.control_target_d)
        
        yaw = math.atan2(self.ego_lat_speed,self.ego_long_speed)
        theta = math.atan2(self.control_target_d,self.control_target_s)
        alpha = theta - yaw
        L = 5 # length between rear axis front axis
        l_d = math.sqrt(self.control_target_s**2 + self.control_target_d**2)
        sigma = math.atan(2 * L * math.sin(alpha)/l_d)
        #print(2 * L * math.sin(alpha)/l_d)
        #print("alpha",alpha,"sigma",sigma,"yaw",yaw,"theta",theta)
        return sigma

    def getStanleyControl(self):
        #print("control s",self.control_target_s,"control d",self.control_target_d)
        
        yaw = math.atan2(self.ego_lat_speed,self.ego_long_speed)
        theta = math.atan2(self.control_target_d,self.control_target_s)
        alpha = theta - yaw

        l_d = math.sqrt(self.control_target_s**2 + self.control_target_d**2)
        lateral_error = l_d * math.sin(alpha)
        lateral_weight = 0.3
        lateral_control = math.atan2(lateral_weight * lateral_error, self.ego_long_speed)


        s_0 = self.control_target_s - 0.1
        s_1 = self.control_target_s + 0.1
        d_0 = self.lat_param[0] + self.lat_param[1] * s_0 + self.lat_param[2] * (s_0 ** 2) + self.lat_param[3] * (s_0 ** 3) + self.lat_param[4] * (s_0 ** 4) + self.lat_param[5] * (s_0 ** 5) 
        d_1 = self.lat_param[0] + self.lat_param[1] * s_1 + self.lat_param[2] * (s_1 ** 2) + self.lat_param[3] * (s_1 ** 3) + self.lat_param[4] * (s_1 ** 4) + self.lat_param[5] * (s_1 ** 5) 
        yaw_error = math.atan2(d_1-d_0,s_1-s_0) - yaw
        heading_control = yaw_error
        #print("lateral_control", lateral_control, "heading_control", heading_control)
        #print("s_0",s_0,"s_1",s_1,"d_0",d_0,"d_1", d_1, "angle", math.atan2(d_1-d_0,s_1-s_0), "yaw", yaw)
        return (lateral_control + heading_control) * self.lat_control_weight
    

        
        

    def latTest(self):
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
        # #print("cost : ", cost)
        self.visualizer.polynomial(params, planning_dist)

    def longTest(self):
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
        
        #print("target s",self.target_s)
        #print("target v",self.target_v)
        for i in range(len(self.target_v)):
            params, cost = self.longitudinalPlanning(self.target_v[i], self.target_s[i])


            t = self.long_control_time
            t_ = t + self.dt
            self.control_target_s = params[0] + params[1] * t + params[2] * (t ** 2) + params[3] * (t ** 3) + params[4] * (t ** 4) + params[5] * (t ** 5) 
            s_ = params[0] + params[1] * t_ + params[2] * (t_ ** 2) + params[3] * (t_ ** 3) + params[4] * (t_ ** 4) + params[5] * (t_ ** 5) 
            self.control_target_v = (s_ - self.control_target_s) / self.dt
            #print("control_target_v",self.control_target_v)
            self.visualizer.polynomial(params, self.planning_horizon)
        # #print("cost : ", cost)
        
    def findLeadingVehicleIdx(self):
        oa_idx_with_same_lane = []
        # #print("ego pos")
        # #print(self.ego_lat_pos)
        # #print(self.ego_lane_idx)
        for i in range(len(self.oa_pos)):
            # find oa_lane #
            ################
            oa_lane = self.oa_pos[i][1] // 4
            # #print("oa_pos_y",self.oa_pos[i][1])
            # #print("oa_lane",oa_lane)
            if oa_lane == self.ego_lane_idx:
                # #print("same lane")
                oa_idx_with_same_lane.append(i)
        # find closeset oa which is leading vehicle
        min_oa_x = 1e100
        leading_vehicle_idx = -1
        for oa_idx in oa_idx_with_same_lane:
            if min_oa_x > self.oa_pos[oa_idx][0] > 0:
                min_oa_x = self.oa_pos[oa_idx][0]
                leading_vehicle_idx = oa_idx
        # #print("leading_vehicle_idx")
        # #print(leading_vehicle_idx)
        return leading_vehicle_idx

