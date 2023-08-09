from planner.planner import OptimalTrajectoryPlanner

ego_lat_pos = 0
ego_lat_speed = 0
ego_lat_acc = 0
ego_long_speed = 0
ego_long_acc = 0

optimalTrajectoryPlanner = OptimalTrajectoryPlanner(ego_lat_pos, ego_lat_speed, ego_lat_acc, ego_long_speed, ego_long_acc)
optimalTrajectoryPlanner.test()