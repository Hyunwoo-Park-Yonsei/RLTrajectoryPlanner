import threading
import gym
import highway_env
from matplotlib import pyplot as plt
import pprint
from Keyboard import KeyboardEventHandler
import threading
import numpy as np
from datetime import datetime
from highway_env.vehicle.behavior import IDMVehicle
import string
import model
import torch
import utils
from numpy import random
from policy_gradient import Policy
from TD3 import TD3
import torch.nn.functional as F
from controller.PID_controller import PIDController
from planner.planner import OptimalTrajectoryPlanner
import math

f = None

time_steps = 2500
max_action = 0.3
expl_noise = 0.1
batch_size = 256
teraminal_penalty = -100

lat_P = 1
lat_I = 0.15
lat_D = 0
long_P = 1.0
long_I = 1.0
long_D = 1.0
PID_target_time = 0.3

max_timesteps = int(1e6)
def main():
    global f

    generate_data = False
    # mode = "Rule-based, Learning, Evaluation"
    mode = "Rule-based"
    num_of_other_vehicles = 1
    num_of_lanes = 2
    env = gym.make('highway-v0')
    # env.config["show_trajectories"] = True
    env.config["vehicles_count"] = num_of_other_vehicles
    env.config["simulation_frequency"] = 30
    env.config["policy_frequency"] = 30
    env.configure({
        "lanes_count": num_of_lanes,
        "action": {
            "type" : "ContinuousAction"
        },
        "collision_reward": -100,
        "duration": 600,
        "on_road_reward" : 1,
        "off_road_reward" : -5,
        'offroad_terminal': True,
        'high_speed_reward': 0.1,
        'screen_height': 600,
        'screen_width': 2400,
        # "observation": {
        #     "type": "Kinematics",
        #     "vehicles_count": num_of_other_vehicles,
        #     "features": ["presence", "x", "y", "vx", "vy", "heading","lat_off"],
        #     "absolute": True,
        #     "normalize": False,
        #     "order": "sorted"
        # }

        "observation": {
        "type": "OccupancyGrid",
        "vehicles_count": num_of_other_vehicles,
        # "vehicles_count": 15,
        # "features": ["presence", "x", "y", "vx", "vy", "cos_h", "sin_h"],
        "features": ["presence", "x", "y", "lat_off", "heading","ang_off","vx", "vy"],
        "features_range": {
            "x": [-100, 100],
            "y": [-100, 100],
            "vx": [-20, 20],
            "vy": [-20, 20]
        },
        "grid_size": [[-25, 25], [-25, 25]],
        "grid_step": [2.5, 2.5],
        "absolute": False
        }
    })
    if generate_data:
        dir = '/Users/hwpark/Desktop/highway_env/data/'
        date_time = datetime.now()
        dir += date_time.strftime("%Y %m %d %H %M")
        f = open(dir,'w')

    idm_vehicle = None
    obs = None
    

    policy = None
    lat_controller = None
    long_controller = None
    replay_buffer = utils.ReplayBuffer(state_dim = 401, action_dim = 2)

    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    state = env.reset()
    state = state[6].reshape(-1)
    ego = env.road.vehicles[0].position
    ego_lane_idx = np.array(env.road.network.get_closest_lane_index(np.array(ego))[2],np.float32)
    state = np.append(state,[ego_lane_idx])

    if mode == "Rule-based":
        lat_controller = PIDController(lat_P, lat_I, lat_D)
        long_controller = PIDController(long_P, long_I, long_D)

    else:
        policy = TD3(401, 2, 1)
    ego_lane_idx = np.array(env.road.network.get_closest_lane_index(np.array(ego))[2],np.float32)
    optimalTrajectoryPlanner = OptimalTrajectoryPlanner(num_of_lanes,ego_lane_idx)

    for t in range(max_timesteps):
        episode_timesteps += 1

        # acceleration and steering angle

        done = False
        ego = env.road.vehicles[0].position
        ego_heading = env.road.vehicles[0].heading / math.pi
        ego_speed = [env.road.vehicles[0].speed / 3.6 * math.cos(ego_heading), env.road.vehicles[0].speed / 3.6 * math.sin(ego_heading)]
        print("ego_heading", ego_heading)
        print("ego_speed", ego_speed)
        ego_lane_idx = np.array(env.road.network.get_closest_lane_index(np.array(ego))[2],np.float32)
        # print("ego_lane_idx",ego_lane_idx)
        other_agent_heading = []
        other_agent_speed = []
        other_agent_pos = []
        if mode == "Rule-based":
            for i in range(1,len(env.road.vehicles)):
                oa_pos = env.road.vehicles[i].position
                if(math.sqrt((ego[0] - oa_pos[0])**2 + (ego[1] - oa_pos[1])**2) < 50):
                    other_agent_heading.append(env.road.vehicles[i].heading)
                    other_agent_speed.append(env.road.vehicles[i].speed/3.6)
                    other_agent_pos.append([env.road.vehicles[i].position[0] - ego[0], env.road.vehicles[i].position[1] - ego[1]])
        if keyboard_listener.is_space_pressed:
            evt.wait()

        if t < episode_timesteps:
            if mode == "Rule-based":
                print("ego",ego)
                ego_y_pos = ego[1]
                ego_y_pos -= ego_lane_idx * 4
                print("ego_y_pos",ego_y_pos)
                optimalTrajectoryPlanner.update(ego_y_pos, ego_speed[1], 0, ego_speed[0], 0, other_agent_pos, other_agent_speed, other_agent_heading, ego_lane_idx)
                optimalTrajectoryPlanner.plan()

                # control_target_d = optimalTrajectoryPlanner.getPurePursuitControl()
                steer = optimalTrajectoryPlanner.getStanleyControl()
                _, control_target_s = optimalTrajectoryPlanner.getControlPoint()
                accel = (control_target_s - 12.5) * 0.1
                print("control_target_s",control_target_s,"accel",accel)
                # # temp
                # control_target_s = PID_target_time * env.road.vehicles[0].speed 
                # ######
                # print("control_target_d ", control_target_d)
                # lat_controller.update(0,control_target_d)
                # long_controller.update(ego_speed[0],control_target_s)

                # action = [lat_controller.getControl(), long_controller.getControl()]
                action = [0, steer]
            else:
                action = [np.random.normal(0, 0.05, 1),np.random.normal(0, 0.05, 1)]
        else:
            if mode == "Rule-based":
                print("ego",ego)
                ego_y_pos = ego[1]
                ego_y_pos -= ego_lane_idx * 4
                print("ego_y_pos",ego_y_pos)
                optimalTrajectoryPlanner.update(ego_y_pos, ego_speed[1], 0, ego_speed[0], 0, other_agent_pos, other_agent_speed, other_agent_heading, ego_lane_idx)
                optimalTrajectoryPlanner.plan()


                # control_target_d = optimalTrajectoryPlanner.getPurePursuitControl()
                steer = optimalTrajectoryPlanner.getStanleyControl()
                _, control_target_s = optimalTrajectoryPlanner.getControlPoint()
                accel = (control_target_s - 0.2) * 0.1
                print("accel",accel)
                # # temp
                # control_target_s = PID_target_time * env.road.vehicles[0].speed 
                # ######
                # print("control_target_d ", control_target_d)
                # lat_controller.update(0,control_target_d)
                # long_controller.update(ego_speed[0],control_target_s)


                # action = [0, steer + lat_controller.getControl()]
                action = [0, steer]
                # action = [0, steer + lat_controller.getControl()]
            else:
                action = (policy.select_action(np.array(state)) + np.random.normal(0, max_action * expl_noise, size=2)).clip(-max_action, max_action)
        
        # action[0] = action[0]
        # action[1] = action[1]
        print("action",action,"\n")

        state_prime, reward, done, info = env.step(action)
        # done_bool = float(done) if episode_timesteps < env._max_episode_steps else 0         
        done_bool = float(done)
        if(done_bool): reward += teraminal_penalty 
        

        
        
        

        if mode != "Rule-based":
            state_prime = state_prime[6].reshape(-1)
            state_prime = np.append(state_prime,[ego_lane_idx])

            state = state_prime
            replay_buffer.add(state,action,state_prime, reward, done_bool)

            if t >= time_steps:
                policy.train(replay_buffer, batch_size)
        
        episode_reward += reward

        if done:
            exit()
            # +1 to account for 0 indexing. +0 on ep_timesteps since it will increment +1 even if done=True
            print(f"Total T: {t+1} Episode Num: {episode_num+1} Episode T: {episode_timesteps} Reward: {episode_reward:.3f}")
			# Reset environment 
            state, done = env.reset(), False
            lat_controller.reset()
            long_controller.reset()

            state = state[6].reshape(-1)
            ego = env.road.vehicles[0].position
            ego_lane_idx = np.array(env.road.network.get_closest_lane_index(np.array(ego))[2],np.float32)
            state = np.append(state,[ego_lane_idx])

            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1

        env.render()



        if done or keyboard_listener.reset_flag:
            obs = env.reset()
            keyboard_listener.reset_flag = False

        if generate_data:
            f.write(np.array_str(obs))

    evt.clear()
if __name__ == '__main__':
    evt = threading.Event()
    keyboard_listener = KeyboardEventHandler(evt)
    main()
    if f:
        f.close()
