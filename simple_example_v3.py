#!/usr/bin/env python

#
# quantitative_experiments.py
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

from actors import AVOCADO_Actor
from simple_simulator import CircleSimulator, SquareSimulator, StaticObsCircleSimulator
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import avocado

agent_radius = 0.2
a=0.3
c=0.7
d=2
kappa= 14.15
epsilon= 3.22
delta= 0.57
bias=[0.]
timestep = 0.1
neighbor_dist=2.5
time_horizon=2.5
max_noise=0.0005
# The parameter alpha is a list of alphas, being alpha the cooperation level. If alpha in [0,1], it is fixed.
# If alpha > 1, then it adapts the cooperation level using AVOCADO. The list of alphas is there so that you can directly
# use AVOCADO/non-AVOCADO agents in the same line.


seed = 0
n_agents = 10
circle_radius = 1.5
np.random.seed(seed)
pos_agents = []
goal_agents = []
agent_velocities = []
max_vel = []
agent_artist = []
max_vel = 1.0
colors = ['r', 'g', 'b', 'y', 'm', 'c', 'k']
fig, ax = plt.subplots(1,1, figsize=(12,12))
goals = np.random.permutation(np.arange(n_agents))

for ag in range(int(n_agents)):
    pos_agents.append((circle_radius*np.cos(2*np.pi*(ag+1)/n_agents), circle_radius*np.sin(2*np.pi*(ag+1)/n_agents)))
    agent_velocities.append((0, 0))
    agent = plt.Circle(pos_agents[ag], agent_radius, color=colors[ag%len(colors)])
    agent_artist.append(agent)
    ax.add_artist(agent)
pos_agents = np.array(pos_agents)
agent_velocities = np.array(agent_velocities)
goal_agents = np.array(pos_agents)[goals]
id_agents = []
# You can add other agents not using AVOCADO uncommenting lines referring to others
# id_others = []
sim = avocado.PyRVOSimulator(timestep, neighbor_dist, 15, time_horizon, time_horizon, agent_radius*1.1, 1)
for k in range(len(pos_agents)):
    id_agents.append(sim.addAgent((pos_agents[k,0], pos_agents[k,1]), const_alpha=100, 
                                            a=a, b=bias[k%len(bias)], c=c, d=d, 
                                            kappa=kappa, epsilon=epsilon, delta=delta, max_noise=max_noise))
# for k in range(len(other_positions)):
#     id_others.append(sim.addAgent((other_positions[k,0], other_positions[k,1]), const_alpha=0.0))
# for o in static_obstacles:
#     sim.addObstacle(o)
# sim.processObstacles()

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_xlim(-4, 4)
ax.set_ylim(-4, 4)

def get_desired_vel(agent_position, agent_goal, max_vel):
    vel = np.array(agent_goal) - np.array(agent_position)
    return vel / max(1., (np.linalg.norm(vel)/max_vel))
            
def update(frame):
    time = frame*timestep
    global pos_agents, agent_velocities, goal_agents
    # AVOCADO
    for k in range(len(pos_agents)):
        sim.setAgentPosition(id_agents[k], (pos_agents[k,0], pos_agents[k,1]))
        sim.setAgentVelocity(id_agents[k], (agent_velocities[k,0], agent_velocities[k,1]))
        vel = get_desired_vel(pos_agents[k], goal_agents[k], max_vel)
        sim.setAgentPrefVelocity(id_agents[k], (vel[0], vel[1]))
    # for k in range(len(other_positions)):
    #     sim.setAgentPosition(id_others[k], (other_positions[k,0], other_positions[k,1]))
    #     sim.setAgentVelocity(id_others[k], (other_velocities[k,0], other_velocities[k,1]))
    sim.doStep()
    agent_velocities = []
    for k in range(len(pos_agents)):
        agent_velocities.append(sim.getAgentVelocity(id_agents[k]))
    agent_velocities = np.array(agent_velocities)
    pos_agents = pos_agents + agent_velocities*timestep
    for agent_no in range(n_agents):
        agent_artist[agent_no].center = (pos_agents[agent_no][0], pos_agents[agent_no][1])
    print("Time: ", time)
    return agent_artist

anim = animation.FuncAnimation(fig, update, frames=None, interval=timestep*1000, blit=False)
anim.running = True
plt.show()