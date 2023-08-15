from planner.planner import OptimalTrajectoryPlanner

# -0.3203207274139448 -0.04998299574598229 0
# -0.7045295967350116 0.372561637938396 0
# 0.2874811844872065 0.044858350902680456 0
ego_lat_pos = 0.2874811844872065
ego_lat_speed =  0.044858350902680456
ego_lat_acc = 0
ego_long_speed = 0
ego_long_acc = 0
oa_pos = [[]] 
oa_speed = []
oa_heading = []
ego_lane_idx = 0
optimalTrajectoryPlanner = OptimalTrajectoryPlanner(ego_lat_pos, ego_lat_speed, ego_lat_acc, ego_long_speed, ego_long_acc, oa_pos, oa_speed, oa_heading, 48)
optimalTrajectoryPlanner.test()