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

agent_radius = 0.2
a=0.3
c=0.7
d=2
kappa= 14.15
epsilon= 3.22
delta= 0.57
bias=[0.]
timestep = 0.1
# The parameter alpha is a list of alphas, being alpha the cooperation level. If alpha in [0,1], it is fixed.
# If alpha > 1, then it adapts the cooperation level using AVOCADO. The list of alphas is there so that you can directly
# use AVOCADO/non-AVOCADO agents in the same line.
actor = AVOCADO_Actor(agent_radius, timestep, alpha=[100.], a=a, c=c, d=d, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0.])

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
goal_agents = np.array(pos_agents)[goals]

ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.set_xlim(-4, 4)
ax.set_ylim(-4, 4)
            
def update(frame):
    time = frame*timestep
    global pos_agents, agent_velocities, goal_agents
    # You only need to call this function if you want to use AVOCADO
    # You can add positions and velocities of agents not using AVOCADO (and obstacles) instead of the empty lists
    agent_velocities = actor.act(np.array(pos_agents), [], np.array(agent_velocities), [], np.array(goal_agents), max_vel)
    pos_agents = pos_agents + agent_velocities*timestep
    for agent_no in range(n_agents):
        agent_artist[agent_no].center = (pos_agents[agent_no][0], pos_agents[agent_no][1])
    print("Time: ", time)
    return agent_artist

anim = animation.FuncAnimation(fig, update, frames=None, interval=timestep*1000, blit=False)
anim.running = True
plt.show()