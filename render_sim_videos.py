#
# render_sim_videos.py
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

import numpy as np
# from actors import SimpleActor, AVOCADO_Actor, Simple_ORCA_Actor, DRLActor, MPCActor, RL_RVO_Actor
from actors import AVOCADO_Actor
from simple_simulator import CircleSimulator, SquareSimulator

agent_radius = 0.2
AVOCADO_dict = {"a":2., "c":0.2, "d":5, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]}
a = AVOCADO_dict["a"]
c = AVOCADO_dict["c"]
d = AVOCADO_dict["d"]
kappa = AVOCADO_dict["kappa"]
epsilon = AVOCADO_dict["epsilon"]
delta = AVOCADO_dict["delta"]
bias = AVOCADO_dict["bias"]

# Differences 0-0.5-1 ----------------------------------------------------------------------------------------------
# 0-responsibility
# actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100., 0.], a=AVOCADO_dict["a"], c=AVOCADO_dict["c"], d=AVOCADO_dict["d"], kappa=AVOCADO_dict["kappa"], epsilon=AVOCADO_dict["epsilon"], delta=AVOCADO_dict["delta"], bias=AVOCADO_dict["bias"])
# sim = CircleSimulator(2, 1.5, [], actor, orca_vel=1.0, agent_vel=1.0)
# sim.run_simulation(required_metrics=[], visualize=False, save_visualization=True, animate=True, file_name="1-1-0-resp")
# # Cooperative
# actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100., 0.5], a=AVOCADO_dict["a"], c=AVOCADO_dict["c"], d=AVOCADO_dict["d"], kappa=AVOCADO_dict["kappa"], epsilon=AVOCADO_dict["epsilon"], delta=AVOCADO_dict["delta"], bias=AVOCADO_dict["bias"])
# sim = CircleSimulator(2, 1.5, [], actor, orca_vel=1.0, agent_vel=1.0)
# sim.run_simulation(required_metrics=[], visualize=False, save_visualization=True, animate=True, file_name="1-1-coop")
# # Fully-responsibility
# actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100., 1.], a=AVOCADO_dict["a"], c=AVOCADO_dict["c"], d=AVOCADO_dict["d"], kappa=AVOCADO_dict["kappa"], epsilon=AVOCADO_dict["epsilon"], delta=AVOCADO_dict["delta"], bias=AVOCADO_dict["bias"])
# sim = CircleSimulator(2, 1.5, [], actor, orca_vel=1.0, agent_vel=1.0)
# sim.run_simulation(required_metrics=[], visualize=False, save_visualization=True, animate=True, file_name="1-1-fully")
#--------------------------------------------Circles------------------------------------------------------------------------------------------------------------
# finished = False
# agents = 10
# while not finished:
#     actor = Simple_ORCA_Actor(agent_radius, 0.1)
#     sim = CircleSimulator(agents, 2.5, np.arange(1,agents), actor, orca_vel=0.75, agent_vel=1.0)
#     metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, visualize=False, file_name="orca-vs-9")
#     if metrics["collision_rate"]>0:
#         finished = True
# finished = False
# agents = 10
# while not finished:
#     actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
#     sim = CircleSimulator(agents, 2.5, np.arange(1,agents), actor, orca_vel=0.75, agent_vel=1.0)
#     metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="avocado-vs-9")
#     if metrics["collision_rate"]==0:
#         finished = True
# actor = DRLActor(agent_radius, 0.1)
# sim = CircleSimulator(agents, 2.5, np.arange(1,agents), actor, orca_vel=0.75, agent_vel=1.0)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="drl-vs-9")
# actor = MPCActor(agent_radius, 0.1)
# sim = CircleSimulator(agents, 2.5, np.arange(1,agents), actor, orca_vel=0.75, agent_vel=1.0)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="mpc-vs-9")
# actor = RL_RVO_Actor(agent_radius, 0.1)
# sim = CircleSimulator(agents, 2.5, np.arange(1,agents), actor, orca_vel=0.75, agent_vel=1.0)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="rl-rvo-vs-9")
# #--------------------------------------------Col-Circles------------------------------------------------------------------------------------------------------------
# finished = False
# agents = 10
# actor = Simple_ORCA_Actor(agent_radius, 0.1)
# sim = CircleSimulator(agents, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="orca-10-circle")
# finished = False
# agents = 10
# while not finished:
#     actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
#     sim = CircleSimulator(agents, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0)
#     metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="avocado-10-circle")
#     if metrics["collision_rate"]==0:
#         finished = True
# actor = DRLActor(agent_radius, 0.1)
# sim = CircleSimulator(agents, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="drl-10-circle")
# actor = MPCActor(agent_radius, 0.1)
# sim = CircleSimulator(10, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0)
# metrics = sim.run_simulation(required_metrics=["collision_rate", "mean_agent_time"], animate=True, save_visualization=True, file_name="mpc-10-circle")
# actor = RL_RVO_Actor(agent_radius, 0.1)
# sim = CircleSimulator(agents, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="rl-rvo-10-circle")
# #--------------------------------------------Mid-Circles------------------------------------------------------------------------------------------------------------
# finished = False
# agents = 10
# while not finished:
#     actor = Simple_ORCA_Actor(agent_radius, 0.1)
#     sim = CircleSimulator(agents, 2.5, np.arange(0,agents,3), actor, orca_vel=0.75, agent_vel=1.0)
#     metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="orca-10-circle-mid")
#     if metrics["collision_rate"]>0:
#         finished = True
# finished = False
# agents = 10
# while not finished:
#     actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
#     sim = CircleSimulator(agents, 2.5, np.arange(0,agents,3), actor, orca_vel=0.75, agent_vel=1.0)
#     metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="avocado-10-circle-mid")
#     print(metrics["collision_rate"])
#     if metrics["collision_rate"]==0:
#         finished = True
# actor = DRLActor(agent_radius, 0.1)
# sim = CircleSimulator(agents, 2.5, np.arange(0,agents,3), actor, orca_vel=0.75, agent_vel=1.0)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="drl-10-circle-mid")
# actor = MPCActor(agent_radius, 0.1)
# sim = CircleSimulator(agents, 2.5, np.arange(0,agents,3), actor, orca_vel=0.75, agent_vel=1.0)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="mpc-10-circle-mid")
# actor = RL_RVO_Actor(agent_radius, 0.1)
# sim = CircleSimulator(agents, 2.5, np.arange(0,agents,3), actor, orca_vel=0.75, agent_vel=1.0)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="rl-rvo-10-circle-mid")
# #--------------------------------------------Col-head-on------------------------------------------------------------------------------------------------------------
# finished = False
# agents = 10
# actor = Simple_ORCA_Actor(agent_radius, 0.1)
# sim = SquareSimulator(12, 0, 2.5, actor, seed=4)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="orca-12-head-on")
# finished = False
# agents = 10
# while not finished:
#     actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
#     sim = SquareSimulator(12, 0, 2.5, actor, seed=4)
#     metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="avocado-12-head-on")
#     print(metrics["collision_rate"])
#     if metrics["collision_rate"]==0:
#         finished = True
# actor = DRLActor(agent_radius, 0.1)
# sim = SquareSimulator(12, 0, 2.5, actor, seed=4)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="drl-12-head-on")
# actor = MPCActor(agent_radius, 0.1)
# sim = SquareSimulator(12, 0, 2.5, actor, seed=4)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="mpc-12-head-on")
# actor = RL_RVO_Actor(agent_radius, 0.1)
# sim = SquareSimulator(12, 0, 2.5, actor, seed=4)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="rl-rvo-12-head-on")

# #--------------------------------------------7-7-head-on------------------------------------------------------------------------------------------------------------
# finished = False
# agents = 10
# actor = Simple_ORCA_Actor(agent_radius, 0.1)
# sim = SquareSimulator(7, 7, 2.5, actor, seed=17)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="orca-7-7-head-on")
# finished = False
# # agents = 10
# while not finished:
#     actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
#     sim = SquareSimulator(7, 7, 2.5, actor, seed=17)
#     metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="avocado-7-7-head-on")
#     print(metrics["collision_rate"])
#     if metrics["collision_rate"]==0:
#         finished = True
# actor = DRLActor(agent_radius, 0.1)
# sim = SquareSimulator(7, 7, 2.5, actor, seed=17)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="drl-7-7-head-on")
# actor = MPCActor(agent_radius, 0.1)
# sim = SquareSimulator(7, 7, 2.5, actor, seed=17)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="mpc-7-7-head-on")
# actor = RL_RVO_Actor(agent_radius, 0.1)
# sim = SquareSimulator(7, 7, 2.5, actor, seed=17)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="rl-rvo-7-7-head-on")
# #--------------------------------------------1-12-head-on------------------------------------------------------------------------------------------------------------
# finished = False
# agents = 10
# while not finished:
#     actor = Simple_ORCA_Actor(agent_radius, 0.1)
#     sim = SquareSimulator(1, 12, 2.5, actor, seed=17)
#     metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="orca-1-12-head-on")
#     if metrics["collision_rate"]>0:
#         finished = True
# finished = False
# agents = 10
# while not finished:
#     actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
#     sim = SquareSimulator(1, 12, 2.5, actor, seed=17)
#     metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="avocado-1-12-head-on")
#     print(metrics["collision_rate"])
#     if metrics["collision_rate"]==0:
#         finished = True
# actor = DRLActor(agent_radius, 0.1)
# sim = SquareSimulator(1, 12, 2.5, actor, seed=17)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="drl-1-12-head-on")
# actor = MPCActor(agent_radius, 0.1)
# sim = SquareSimulator(1, 12, 2.5, actor, seed=17)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="mpc-1-12-head-on")
# actor = RL_RVO_Actor(agent_radius, 0.1)
# sim = SquareSimulator(1, 12, 2.5, actor, seed=17)
# metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="rl-rvo-1-12-head-on")
# ----------------------------------------------25 AVOCADO -----------------------------------------------------------------

# n_agents = 25
# room_size = max(2.5, (2.3*2*agent_radius*n_agents)/(2*np.pi))
# # n_cooperatives_array = [1, int(np.ceil(n_agents/4.)), int(np.ceil(n_agents/2)), int(np.ceil(3*(n_agents/4))), n_agents]
# n_cooperatives_array = [int(np.ceil(3*(n_agents/4))), n_agents]
# for n_cooperatives in n_cooperatives_array:
#     seed = 0
#     finished = False
#     while not finished:
#         np.random.seed(seed)
#         idx_non_cooperatives = np.random.choice(np.arange(n_agents), size=(n_agents-n_cooperatives), replace=False) 
#         actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
#         sim = CircleSimulator(n_agents=n_agents, circle_radius=room_size, idx_non_cooperative=idx_non_cooperatives, actor=actor)
#         metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=False, file_name="avocado-" + str(n_agents) +
#                                      "-" + str(n_cooperatives) + "-circle")
#         print(metrics["collision_rate"])
#         if metrics["collision_rate"]==0:
#             while not finished:
#                 np.random.seed(seed)
#                 idx_non_cooperatives = np.random.choice(np.arange(n_agents), size=(n_agents-n_cooperatives), replace=False)
#                 actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
#                 sim = CircleSimulator(n_agents=n_agents, circle_radius=room_size, idx_non_cooperative=idx_non_cooperatives, actor=actor)
#                 metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="avocado-" + str(n_agents) +
#                                             "-" + str(n_cooperatives) + "-circle")
#                 print("For video:", metrics["collision_rate"])
#                 if metrics["collision_rate"]==0:
#                     finished = True
#         seed = seed+1

# n_agents = 25
# room_size = agent_radius*n_agents*1.5
# n_cooperatives_array = [1, int(np.ceil(n_agents/4.)), int(np.ceil(n_agents/2)), int(np.ceil(3*(n_agents/4))), n_agents]
# for n_cooperatives in n_cooperatives_array:
#     seed = 0
#     finished = False
#     while not finished:
#         np.random.seed(seed)
#         actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
#         sim = SquareSimulator(n_cooperative=n_cooperatives, n_non_cooperative=n_agents-n_cooperatives, square_width=room_size, actor=actor, seed=seed)
#         metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=False, file_name="avocado-" + str(n_agents) +
#                                      "-" + str(n_cooperatives) + "-square")
#         print(metrics["collision_rate"])
#         if metrics["collision_rate"]==0:
#             while not finished:
#                 np.random.seed(seed)
#                 actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
#                 sim = SquareSimulator(n_cooperative=n_cooperatives, n_non_cooperative=n_agents-n_cooperatives, square_width=room_size, actor=actor, seed=seed)
#                 metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=True, file_name="avocado-" + str(n_agents) +
#                                      "-" + str(n_cooperatives) + "-square")
#                 print("For video:", metrics["collision_rate"])
#                 if metrics["collision_rate"]==0:
#                     finished = True
#         seed = seed+1

#-----------------------------------------------ALPHAS-----------------------------------------------------------------------------------
# # # 0-responsibility
actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100., 0.], a=AVOCADO_dict["a"], c=AVOCADO_dict["c"], d=AVOCADO_dict["d"], kappa=AVOCADO_dict["kappa"], epsilon=AVOCADO_dict["epsilon"], delta=AVOCADO_dict["delta"], bias=AVOCADO_dict["bias"], neighbor_dist=1.5)
sim = CircleSimulator(2, 1.5, [], actor, orca_vel=0.5, agent_vel=0.5, seed=1)
sim.run_simulation(required_metrics=[], visualize=False, save_visualization=True, animate_with_alphas=True, file_name="alphas-1-1-0-resp")
# Cooperative
actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100., 100.], a=AVOCADO_dict["a"], c=AVOCADO_dict["c"], d=AVOCADO_dict["d"], kappa=AVOCADO_dict["kappa"], epsilon=AVOCADO_dict["epsilon"], delta=AVOCADO_dict["delta"], bias=AVOCADO_dict["bias"], neighbor_dist=1.5)
sim = CircleSimulator(2, 1.5, [], actor, orca_vel=0.5, agent_vel=0.5)
sim.run_simulation(required_metrics=[], visualize=False, save_visualization=True, animate_with_alphas=True, file_name="alphas-1-1-coop")
# Fully-responsibility
actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100., 1.], a=AVOCADO_dict["a"], c=AVOCADO_dict["c"], d=AVOCADO_dict["d"], kappa=AVOCADO_dict["kappa"], epsilon=AVOCADO_dict["epsilon"], delta=AVOCADO_dict["delta"], bias=AVOCADO_dict["bias"], neighbor_dist=1.5)
sim = CircleSimulator(2, 1.5, [], actor, orca_vel=0.5, agent_vel=0.5)
sim.run_simulation(required_metrics=[], visualize=False, save_visualization=True, animate_with_alphas=True, file_name="alphas-1-1-fully")
