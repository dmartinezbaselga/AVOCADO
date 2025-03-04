#!/usr/bin/env python

#
# actors.py
# AVOCADO library
#
# SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0
#
# SPDX-FileCopyrightText: 2024 University of Zaragoza
# SPDX-License-Identifier: AGPL-3.0-or-later
#
# This file is part of AVOCADO, a derivative work of the RVO2 Library.
# Portions of this file are licensed under the Apache License, Version 2.0,
# and modifications are licensed under the GNU Affero General Public License,
# version 3 or later.
#
# If you use AVOCADO in academic work, please cite:
# Martinez-Baselga, D., Sebastián, E., Montijano, E., Riazuelo, L., Sagüés, C., & Montano, L. (2024). AVOCADO: Adaptive Optimal Collision Avoidance driven by Opinion. arXiv preprint arXiv:2407.00507.
# 
# For details, see the LICENSE file at the root of the repository.
# 
# Contact: diegomartinez@unizar.es
# 			esebastian@unizar.es
# 
#

import avocado
import numpy as np
# import torch
# import torch.nn as nn
# import os
# import gym
# # from pytorchBaselines.a2c_ppo_acktr.model import Policy
# from crowd_nav.configs.config_vecmpc import Config
# from evaluation_mpc import main_config
# from crowd_sim.envs.utils.state import JointState, FullState, ObservableState
# from crowd_nav.policy.vecMPC.controller import vecMPC
# from gym_env.envs.rvo_inter import rvo_inter
# from rl_rvo_nav.policy.policy_rnn_ac import rnn_ac
# import importlib
# from crowd_nav.policy.policy_factory import policy_factory
# from crowd_nav.policy.reward_estimate import Reward_Estimator
# from crowd_sim.envs.utils.robot import Robot
# from crowd_nav.utils.explorer import Explorer


class MultiActor:
    def __init__(self, agent_radius, timestep) -> None:
        self.agent_radius = agent_radius
        self.timestep = timestep
    
    def act(self, agent_positions, other_positions, agent_velocities, other_velocities, agent_goals, max_vel):
        raise NotImplementedError()
    
    def get_desired_vel(self, agent_position, agent_goal, max_vel):
        vel = np.array(agent_goal) - np.array(agent_position)
        return vel / max(1., (np.linalg.norm(vel)/max_vel))
        # return (vel / np.linalg.norm(vel))*max_vel
    
    def get_desired_vel_max(self, agent_position, agent_goal, max_vel):
        vel = np.array(agent_goal) - np.array(agent_position)
        # return vel / max(1., (np.linalg.norm(vel)/max_vel))
        return (vel / np.linalg.norm(vel))*max_vel
    
    def filter_vel(self, vel, max_vel):
        if np.linalg.norm(vel) > max_vel:
            return (vel / np.linalg.norm(vel))*max_vel
        else:
            return vel

class SimpleActor(MultiActor):
    def __init__(self, agent_radius, timestep) -> None:
        super().__init__(agent_radius, timestep)
        
    def act(self, agent_positions, other_positions, agent_velocities, other_velocities, agent_goals, max_vel):
        return np.array([self.filter_vel(self.get_desired_vel(agent_positions[k], agent_goals[k], max_vel) for k in range(len(agent_positions), max_vel))])
    

class AVOCADO_Actor(MultiActor):
    def __init__(self, agent_radius, timestep, a, c, d, kappa, epsilon, delta, 
                 alpha, bias, neighbor_dist=2.5, time_horizon=2.5, max_noise=0.0005, static_obs = []) -> None:
        super().__init__(agent_radius, timestep)
        self.init_sim = False
        self.id_agents = []
        self.id_others = []
        self.a = a
        self.c = c
        self.d = d
        self.kappa = kappa
        self.epsilon = epsilon
        self.delta = delta
        self.alpha = alpha
        self.bias = bias
        self.neighbor_dist = neighbor_dist
        self.time_horizon = time_horizon
        self.max_noise = max_noise
        self.static_obstacles = []
    
    def add_static_obstacles(self, static_obstacles):
        self.static_obstacles = static_obstacles

    def act(self, agent_positions, other_positions, agent_velocities, other_velocities, agent_goals, max_vel):
        if not self.init_sim:
            # RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors,
            # 				 float timeHorizon, float timeHorizonObst, float radius,
            # 				 float maxSpeed, const Vector2 &velocity = Vector2());
            self.sim = avocado.PyRVOSimulator(self.timestep, self.neighbor_dist, 15, self.time_horizon, self.time_horizon, self.agent_radius*1.1, 1)
            for k in range(len(agent_positions)):
                self.id_agents.append(self.sim.addAgent((agent_positions[k,0], agent_positions[k,1]), const_alpha=self.alpha[k%len(self.alpha)], 
                                                        a=self.a, b=self.bias[k%len(self.bias)], c=self.c, d=self.d, 
                                                        kappa=self.kappa, epsilon=self.epsilon, delta=self.delta, max_noise=self.max_noise))
            for k in range(len(other_positions)):
                self.id_others.append(self.sim.addAgent((other_positions[k,0], other_positions[k,1]), const_alpha=0.0))
            for o in self.static_obstacles:
                self.sim.addObstacle(o)
            self.sim.processObstacles()
            self.init_sim = True

        for k in range(len(other_positions)):
            self.sim.setAgentPosition(self.id_others[k], (other_positions[k,0], other_positions[k,1]))
            self.sim.setAgentVelocity(self.id_others[k], (other_velocities[k,0], other_velocities[k,1]))
        for k in range(len(agent_positions)):
            self.sim.setAgentPosition(self.id_agents[k], (agent_positions[k,0], agent_positions[k,1]))
            self.sim.setAgentVelocity(self.id_agents[k], (agent_velocities[k,0], agent_velocities[k,1]))
            vel = self.get_desired_vel(agent_positions[k], agent_goals[k], max_vel)
            self.sim.setAgentPrefVelocity(self.id_agents[k], (vel[0], vel[1]))
        self.sim.doStep()
        new_velocities = []
        for k in range(len(agent_positions)):
            new_velocities.append(self.filter_vel(self.sim.getAgentVelocity(self.id_agents[k]), max_vel))
        return np.array(new_velocities)  

class Simple_ORCA_Actor(AVOCADO_Actor):
    def __init__(self, agent_radius=0.2, timestep=0.1) -> None:
        super().__init__(agent_radius, timestep, 0., 0., 0., 0., 0., 0., [0.5], [0.])

# class DRLActor(MultiActor):
#     def __init__(self, agent_radius, timestep) -> None:
#         super().__init__(agent_radius, timestep)
#         self.initialized = False
  
#     def load_weights(self, num_humans):
#         device = "cpu"
#         model_weights = "intrinsic-rewards-navigation/crowd_nav/data/sarl/best_val.pth"
#         config_file = "intrinsic-rewards-navigation/crowd_nav/data/sarl/config.py"
#         spec = importlib.util.spec_from_file_location('config', config_file)
#         config = importlib.util.module_from_spec(spec)
#         spec.loader.exec_module(config)

#         # configure policy
#         policy_config = config.PolicyConfig(False)
#         self.policy = policy_factory[policy_config.name]()
#         train_config = config.TrainConfig().train
#         if policy_config.use_noisy_net:
#             train_config.exploration_alg = "noisy_net"
#         self.policy.set_exploration_alg(train_config.exploration_alg)
#         reward_estimator = Reward_Estimator()
#         env_config = config.EnvConfig(False)
#         reward_estimator.configure(env_config)
#         self.policy.reward_estimator = reward_estimator
#         # if policy_config.name == "model_predictive_rl":
#         #     if args.planning_depth is not None:
#         #         policy_config.model_predictive_rl.do_action_clip = True
#         #         policy_config.model_predictive_rl.planning_depth = args.planning_depth
#         #     if args.planning_width is not None:
#         #         policy_config.model_predictive_rl.do_action_clip = True
#         #         policy_config.model_predictive_rl.planning_width = args.planning_width
#         #     if args.sparse_search:
#         #         policy_config.model_predictive_rl.sparse_search = True

#         self.policy.configure(policy_config, device)
#         if self.policy.trainable:
#             self.policy.load_model(model_weights)

#         # configure environment
#         env_config = config.EnvConfig(False)

#         # if args.human_num is not None:
#         #     env_config.sim.human_num = args.human_num
#         env_config.sim.human_num = num_humans
#         # env = gym.make('CrowdSim-v0')
#         # env.configure(env_config)

#         # if args.square:
#         #     env.test_scenario = 'square_crossing'
#         # if args.circle:
#         #     env.test_scenario = 'circle_crossing'
#         # if args.test_scenario is not None:
#         #     env.test_scenario = args.test_scenario

#         self.robot = Robot(env_config, 'robot')
#         # env.set_robot(robot)
#         # for continous action
#         # action_dim = env.action_space.shape[0]
#         # max_action = env.action_space.high
#         # min_action = env.action_space.low
#         # print(action_dim, max_action, min_action)
#         # if policy.name == 'TD3RL':
#         #     policy.set_action(action_dim, max_action, min_action)
#         self.robot.time_step = self.timestep
#         self.robot.set_policy(self.policy)
#         env = None
#         # explorer = Explorer(env, robot, device, None, policy_config.use_noisy_net, gamma=0.9)

#         train_config = config.TrainConfig(False)
#         epsilon_end = train_config.train.epsilon_end
#         # if not isinstance(robot.policy, ORCA):
#         #     robot.policy.set_epsilon(epsilon_end)

#         self.policy.set_phase("test")
#         self.policy.set_device(device)

#         # set safety space for ORCA in non-cooperative simulation
#         # if isinstance(robot.policy, ORCA):
#         #     if robot.visible:
#         #         robot.policy.safety_space = args.safety_space
#         #     else:
#         #         robot.policy.safety_space = args.safety_space
#         #     logging.info('ORCA agent buffer: %f', robot.policy.safety_space)

#         self.policy.set_env(env)
    
#     def get_full_state_list_noV(self, agent_position, agent_velocity, agent_goal):
#         return torch.tensor([[[agent_position[0], agent_position[1], 0.3, agent_goal[0], agent_goal[1], 1.0, np.arctan2(agent_velocity[1], agent_velocity[0])]]], device=self.device, dtype=torch.float32)
    
#     def update_other_states(self, other_positions, other_velocities):
#         return other_positions + other_velocities * self.timestep
    
#     def get_distance(self, r1, r2):
#         return ((r1[0] - r2[0])**2 + (r1[1] - r2[1])**2)

#     def generate_ob(self, other_positions, other_velocities, robot_position):     
#         ob = []   
#         dist_to_others = np.array([self.get_distance(robot_position, r) for r in other_positions])
#         sorted_indices = np.argsort(dist_to_others)
#         sorted_other_positions = other_positions[sorted_indices]
#         sorted_other_velocities = other_velocities[sorted_indices]
#         ordered_dist = dist_to_others[sorted_indices]
#         for i in range(min(15, len(other_positions))):
#             if ordered_dist[i] < 2.5:
#                 ob.append(ObservableState(sorted_other_positions[i][0], sorted_other_positions[i][1], sorted_other_velocities[i][0], sorted_other_velocities[i][1], self.agent_radius))
#             else:
#                 break
#         while len(ob) < 2:
#             ob.append(ObservableState(100., sorted_other_positions[0][1], sorted_other_velocities[0][0], sorted_other_velocities[0][1], self.agent_radius))
#         return ob
    
#     def act(self, agent_positions, other_positions, agent_velocities, other_velocities, agent_goals, max_vel):
#         if not self.initialized:
#             self.load_weights(len(agent_positions))
#             self.initialized = True
#         new_velocities = []
#         for i in range(len(agent_positions)):
#             self.robot.set(agent_positions[i][0], agent_positions[i][1], agent_goals[i][0], agent_goals[i][1], agent_velocities[i][0], agent_velocities[i][1],
#                            np.arctan2(agent_velocities[i][1], agent_velocities[i][0]), self.agent_radius, 1.0)
#             ob = self.generate_ob(np.concatenate((agent_positions[:i], agent_positions[i+1:], other_positions)), np.concatenate((agent_velocities[:i], agent_velocities[i+1:], other_velocities)),
#                                   agent_positions[i])
#             action, action_index = self.robot.act(ob)
#             if action_index == 0:
#                 new_velocities.append([0.0, 0.0])
#             else:
#                 new_velocities.append(self.filter_vel([action.vx, action.vy], max_vel))
#         return np.array(new_velocities)

# class StateWithTheta(FullState):
#     def get_theta(self):
#         return np.array([0.0, 0.0])
    
#     def get_theta_dot(self):
#         return np.array([0.0, 0.0])

# class MPCActor(MultiActor):
#     def __init__(self, agent_radius, timestep) -> None:
#         super().__init__(agent_radius, timestep)
#         self.initialized = False

#     def generate_ob(self, agent_position, agent_velocity, agent_goal, other_positions, other_velocities):
#         self_state = StateWithTheta(agent_position[0], agent_position[1], agent_velocity[0], agent_velocity[1], self.agent_radius*1.1, agent_goal[0], agent_goal[1], 1.0, np.arctan2(agent_velocity[1], agent_velocity[0]))
#         others_state = []
#         for i in range(len(other_positions)):
#             others_state.append(ObservableState(other_positions[i][0], other_positions[i][1], other_velocities[i][0], other_velocities[i][1], self.agent_radius*1.1))
#         return JointState(self_state, others_state)
    
#     def act(self, agent_positions, other_positions, agent_velocities, other_velocities, agent_goals, max_vel):
#         if not self.initialized:
#             c = 'sgan'
#             config = Config(c)
#             config.sim.human_num = len(agent_positions) + len(other_positions) - 1
#             config.save_path = os.path.join('final_real')
#             config.exp_name = c    
#             self.policy = vecMPC(config)
#             self.initialized = True
#         new_velocities = []
#         for i in range(len(agent_positions)):
#             ob = self.generate_ob(agent_positions[i], agent_velocities[i], agent_goals[i], np.concatenate((agent_positions[:i], agent_positions[i+1:], other_positions)), np.concatenate((agent_velocities[:i], agent_velocities[i+1:], other_velocities)))
#             action = self.policy.predict(ob)
#             new_velocities.append(self.filter_vel([action.vx, action.vy], max_vel))
#         return np.array(new_velocities)
    
# class RL_RVO_Actor(MultiActor):
#     def __init__(self, agent_radius, timestep) -> None:
#         super().__init__(agent_radius, timestep)
#         neighbors_region=2.5
#         neighbors_num=15
#         vxmax = 1.0
#         vymax = 1.0
#         env_train=False
#         acceler = 100.
#         self.rvo = rvo_inter(neighbors_region, neighbors_num, vxmax, vymax, acceler, env_train, self.agent_radius)
#         self.model_action = self.load_policy("rl_rvo_nav/rl_rvo_nav/pre_trained_model/pre_trained/pre_train_check_point_1000.pt", 1e-05, policy_dict=True)
    
#     def load_policy(self, filename, std_factor=1, policy_dict=False):
#         observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(5,), dtype=np.float32)
#         action_space = gym.spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)
#         state_dim = 6 
#         rnn_input_dim = 8 
#         rnn_hidden_dim = 256 
#         hidden_sizes_ac = (256, 256) 
#         hidden_sizes_v = (256, 256) 
#         activation = nn.ReLU
#         output_activation=nn.Tanh
#         output_activation_v= nn.Identity
#         use_gpu=False
#         rnn_mode="biGRU"
#         if policy_dict == True:
#             model = rnn_ac(observation_space, action_space, state_dim, rnn_input_dim, rnn_hidden_dim, hidden_sizes_ac, 
#                            hidden_sizes_v, activation, output_activation, output_activation_v, use_gpu, rnn_mode)
        
#             check_point = torch.load(filename)
#             model.load_state_dict(check_point['model_state'], strict=True)
#             model.eval()

#         else:
#             model = torch.load(filename)
#             model.eval()

#         # model.train()
#         def get_action(x):
#             with torch.no_grad():
#                 x = torch.as_tensor(x, dtype=torch.float32)
#                 action = model.act(x, std_factor)
#             return action

#         return get_action
    
#     def generate_ob(self, agent_position, agent_velocity, agent_goal, other_positions, other_velocities, max_vel):
#         desired_vel = self.get_desired_vel_max(agent_position, agent_goal, max_vel)
#         propri_obs = np.array([agent_velocity[0], agent_velocity[1], desired_vel[0], desired_vel[1], 
#                                      np.arctan2(agent_velocity[1], agent_velocity[0]), self.agent_radius]) 
        
#         robot_omni_state = np.array([[agent_position[0]], [agent_position[1]], [agent_velocity[0]], [agent_velocity[1]],
#                                      [self.agent_radius], [desired_vel[0]], [desired_vel[1]]])
#         nei_state_list = []
#         for i in range(len(other_positions)):
#             nei_state_list.append(np.array([other_positions[i][0], other_positions[i][1], other_velocities[i][0],
#                                             other_velocities[i][1], self.agent_radius]))
#         action = desired_vel
#         obs_vo_list, _, _, _ = self.rvo.config_vo_inf(robot_omni_state, nei_state_list, [], [], action)
#         if len(obs_vo_list) == 0:
#             exter_obs = np.zeros((8,))
#         else:
#             exter_obs = np.concatenate(obs_vo_list) # vo list
            
#         return np.round(np.concatenate([propri_obs, exter_obs]), 2)
    
#     def act(self, agent_positions, other_positions, agent_velocities, other_velocities, agent_goals, max_vel):
#         new_velocities = []
#         for i in range(len(agent_positions)):
#             ob = self.generate_ob(agent_positions[i], agent_velocities[i], agent_goals[i], np.concatenate((agent_positions[:i], agent_positions[i+1:], other_positions)), 
#                                   np.concatenate((agent_velocities[:i], agent_velocities[i+1:], other_velocities)), max_vel)
#             a_inc = np.round(self.model_action(ob), 2)
#             new_velocities.append(self.filter_vel(np.array(a_inc + agent_velocities[i]), max_vel))
#         return np.array(new_velocities)