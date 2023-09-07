from planner.planner import OptimalTrajectoryPlanner

# -0.3203207274139448 -0.04998299574598229 0
# -0.7045295967350116 0.372561637938396 0
# 0.2874811844872065 0.044858350902680456 0
# ego_lat_pos -0.0807302457041672 ego_lat_speed -0.007870999273138706 ego_long_speed 6.944439983853684
ego_lat_pos = -8.63216788e-08
ego_lat_speed =   -2.8634678211593185e-09
ego_lat_acc = 0
ego_long_speed =  6.435185185185182
ego_long_acc = 0
oa_pos = [[-14.507812141584282, 16.00000008632167], [13.944655519007256, 8.000000086321679], [5.005078696793817, 8.63216774445116e-08], [17.18180853264016, 4.000000086321679]]
oa_speed = [6.5178540439939106, 6.328518345981932, 6.140307847017863, 5.9520128672411206]
oa_heading = []
ego_lane_idx = 0
optimalTrajectoryPlanner = OptimalTrajectoryPlanner(5,2)
optimalTrajectoryPlanner.update(ego_lat_pos, ego_lat_speed, ego_lat_acc, ego_long_speed, ego_long_acc, oa_pos, oa_speed, oa_heading, 0)
# optimalTrajectoryPlanner.latTest()
optimalTrajectoryPlanner.longTest()