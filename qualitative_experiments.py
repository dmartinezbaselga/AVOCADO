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

import numpy as np
from actors import SimpleActor, AVOCADO_Actor, Simple_ORCA_Actor, DRLActor, MPCActor, RL_RVO_Actor
from simple_simulator import CircleSimulator, SquareSimulator, StaticObsCircleSimulator
import math

agent_radius = 0.2
a=0.3
c=0.7
d=2
kappa= 14.15
epsilon= 3.22
delta= 0.57
bias=[0.]
# #-------------------------------------- Changing bias ------------------------------------------------------------------------------------------
actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=d, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0.])
sim = CircleSimulator(2, 0.75, [0], actor, orca_vel=1.0, agent_vel=1.0)
print(sim.run_simulation(required_metrics=["collision", "sim_time", "mean_agent_time"], visualize=False, save_visualization=True, file_name="1-1-non-col"))
actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=d, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0.])
sim = CircleSimulator(2, 0.75, [], actor, orca_vel=1.0, agent_vel=1.0)
print(sim.run_simulation(required_metrics=["collision", "sim_time", "mean_agent_time"], animate=False, save_visualization=True, file_name="1-1-col"))
actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=d, kappa=kappa, epsilon=epsilon, delta=delta, bias=[-0.9*d])
sim = CircleSimulator(2, 0.75, [0], actor, orca_vel=1.0, agent_vel=1.0)
print(sim.run_simulation(required_metrics=["collision", "sim_time", "mean_agent_time"], animate=False, save_visualization=True, file_name="1-1-non-col-eq-1"))
actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=d, kappa=kappa, epsilon=epsilon, delta=delta, bias=[-0.9*d])
sim = CircleSimulator(2, 0.75, [], actor, orca_vel=1.0, agent_vel=1.0)
print(sim.run_simulation(required_metrics=["collision", "sim_time", "mean_agent_time"], animate=False, save_visualization=True, file_name="1-1-col-eq-1"))
actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=d, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0.9*d])
sim = CircleSimulator(2, 0.75, [0], actor, orca_vel=1.0, agent_vel=1.0)
print(sim.run_simulation(required_metrics=["collision", "sim_time", "mean_agent_time"], animate=False, save_visualization=True, file_name="1-1-non-col-eq1"))
actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=d, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0.9*d])
sim = CircleSimulator(2, 0.75, [], actor, orca_vel=1.0, agent_vel=1.0)
print(sim.run_simulation(required_metrics=["collision", "sim_time", "mean_agent_time"], animate=False, save_visualization=True, file_name="1-1-col-eq1"))
exit()
#--------------------------------------------Circles------------------------------------------------------------------------------------------------------------
finished = False
agents = 10
while not finished:
    actor = Simple_ORCA_Actor(agent_radius, 0.1)
    sim = CircleSimulator(agents, 2.5, np.arange(1,agents), actor, orca_vel=0.75, agent_vel=1.0)
    metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, visualize=False, file_name="orca-vs-9")
    if metrics["collision_rate"]>0:
        finished = True
finished = False
agents = 10
while not finished:
    actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
    sim = CircleSimulator(agents, 2.5, np.arange(1,agents), actor, orca_vel=0.75, agent_vel=1.0)
    metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="avocado-vs-9")
    if metrics["collision_rate"]==0:
        finished = True
actor = DRLActor(agent_radius, 0.1)
sim = CircleSimulator(agents, 2.5, np.arange(1,agents), actor, orca_vel=0.75, agent_vel=1.0)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="drl-vs-9")
actor = MPCActor(agent_radius, 0.1)
sim = CircleSimulator(agents, 2.5, np.arange(1,agents), actor, orca_vel=0.75, agent_vel=1.0)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="mpc-vs-9")
actor = RL_RVO_Actor(agent_radius, 0.1)
sim = CircleSimulator(agents, 2.5, np.arange(1,agents), actor, orca_vel=0.75, agent_vel=1.0)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="rl-rvo-vs-9")
#--------------------------------------------Col-Circles------------------------------------------------------------------------------------------------------------
finished = False
agents = 10
actor = Simple_ORCA_Actor(agent_radius, 0.1)
sim = CircleSimulator(agents, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="orca-10-circle")
finished = False
agents = 10
while not finished:
    actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
    sim = CircleSimulator(agents, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0)
    metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="avocado-10-circle")
    if metrics["collision_rate"]==0:
        finished = True
actor = DRLActor(agent_radius, 0.1)
sim = CircleSimulator(agents, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="drl-10-circle")
actor = MPCActor(agent_radius, 0.1)
sim = CircleSimulator(10, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0)
metrics = sim.run_simulation(required_metrics=["collision_rate", "mean_agent_time"], animate=False, save_visualization=True, file_name="mpc-10-circle")
actor = RL_RVO_Actor(agent_radius, 0.1)
sim = CircleSimulator(agents, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="rl-rvo-10-circle")
#--------------------------------------------Mid-Circles------------------------------------------------------------------------------------------------------------
finished = False
agents = 10
while not finished:
    actor = Simple_ORCA_Actor(agent_radius, 0.1)
    sim = CircleSimulator(agents, 2.5, np.arange(0,agents,3), actor, orca_vel=0.75, agent_vel=1.0)
    metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="orca-10-circle-mid")
    if metrics["collision_rate"]>0:
        finished = True
finished = False
agents = 10
while not finished:
    actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
    sim = CircleSimulator(agents, 2.5, np.arange(0,agents,3), actor, orca_vel=0.75, agent_vel=1.0)
    metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="avocado-10-circle-mid")
    print(metrics["collision_rate"])
    if metrics["collision_rate"]==0:
        finished = True
actor = DRLActor(agent_radius, 0.1)
sim = CircleSimulator(agents, 2.5, np.arange(0,agents,3), actor, orca_vel=0.75, agent_vel=1.0)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="drl-10-circle-mid")
actor = MPCActor(agent_radius, 0.1)
sim = CircleSimulator(agents, 2.5, np.arange(0,agents,3), actor, orca_vel=0.75, agent_vel=1.0)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="mpc-10-circle-mid")
actor = RL_RVO_Actor(agent_radius, 0.1)
sim = CircleSimulator(agents, 2.5, np.arange(0,agents,3), actor, orca_vel=0.75, agent_vel=1.0)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="rl-rvo-10-circle-mid")
#--------------------------------------------Col-head-on------------------------------------------------------------------------------------------------------------
finished = False
agents = 10
actor = Simple_ORCA_Actor(agent_radius, 0.1)
sim = SquareSimulator(12, 0, 2.5, actor, seed=4)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="orca-12-head-on")
finished = False
agents = 10
while not finished:
    actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
    sim = SquareSimulator(12, 0, 2.5, actor, seed=4)
    metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="avocado-12-head-on")
    print(metrics["collision_rate"])
    if metrics["collision_rate"]==0:
        finished = True
actor = DRLActor(agent_radius, 0.1)
sim = SquareSimulator(12, 0, 2.5, actor, seed=4)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="drl-12-head-on")
actor = MPCActor(agent_radius, 0.1)
sim = SquareSimulator(12, 0, 2.5, actor, seed=4)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="mpc-12-head-on")
actor = RL_RVO_Actor(agent_radius, 0.1)
sim = SquareSimulator(12, 0, 2.5, actor, seed=4)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="rl-rvo-12-head-on")

# #--------------------------------------------7-7-head-on------------------------------------------------------------------------------------------------------------
finished = False
agents = 10
while not finished:
    actor = Simple_ORCA_Actor(agent_radius, 0.1)
    sim = SquareSimulator(7, 7, 2.5, actor, seed=17)
    metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="orca-7-7-head-on")
    if metrics["collision_rate"]>0:
        finished = True
finished = False
# agents = 10
while not finished:
    actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
    sim = SquareSimulator(7, 7, 2.5, actor, seed=17)
    metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="avocado-7-7-head-on")
    print(metrics["collision_rate"])
    if metrics["collision_rate"]==0:
        finished = True
actor = DRLActor(agent_radius, 0.1)
sim = SquareSimulator(7, 7, 2.5, actor, seed=17)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="drl-7-7-head-on")
actor = MPCActor(agent_radius, 0.1)
sim = SquareSimulator(7, 7, 2.5, actor, seed=17)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="mpc-7-7-head-on")
actor = RL_RVO_Actor(agent_radius, 0.1)
sim = SquareSimulator(7, 7, 2.5, actor, seed=17)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="rl-rvo-7-7-head-on")
#--------------------------------------------1-12-head-on------------------------------------------------------------------------------------------------------------
finished = False
agents = 10
while not finished:
    actor = Simple_ORCA_Actor(agent_radius, 0.1)
    sim = SquareSimulator(1, 12, 2.5, actor, seed=17)
    metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=True, save_visualization=False, file_name="orca-1-12-head-on")
    if metrics["collision_rate"]>0:
        finished = True
finished = False
agents = 10
while not finished:
    actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
    sim = SquareSimulator(1, 12, 2.5, actor, seed=17)
    metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="avocado-1-12-head-on")
    print(metrics["collision_rate"])
    if metrics["collision_rate"]==0:
        finished = True
actor = DRLActor(agent_radius, 0.1)
sim = SquareSimulator(1, 12, 2.5, actor, seed=17)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="drl-1-12-head-on")
actor = MPCActor(agent_radius, 0.1)
sim = SquareSimulator(1, 12, 2.5, actor, seed=17)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="mpc-1-12-head-on")
actor = RL_RVO_Actor(agent_radius, 0.1)
sim = SquareSimulator(1, 12, 2.5, actor, seed=17)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, save_visualization=True, file_name="rl-rvo-1-12-head-on")

Static obs visualization
finished = False
agents = 10
actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
# actor = Simple_ORCA_Actor(agent_radius, 0.1)
obstacles_static = [[(-1.,-1.), (1.,-1.), (1.,1.), (-1.,1.)]]
radius = 0.5
num_sides = 3
vertices = [
    (radius * math.cos(2 * math.pi * i / num_sides), 
     radius * math.sin(2 * math.pi * i / num_sides))
    for i in range(num_sides)
]
obstacles_static = [[
    (radius * math.cos(2 * math.pi * i / num_sides), 
     radius * math.sin(2 * math.pi * i / num_sides))
    for i in range(num_sides)
],
[
    (radius * math.cos(2 * math.pi * i / num_sides)+1., 
     radius * math.sin(2 * math.pi * i / num_sides)+1.)
    for i in range(num_sides)
],
[
    (radius * math.cos(2 * math.pi * i / num_sides)-1., 
     radius * math.sin(2 * math.pi * i / num_sides)-1.)
    for i in range(num_sides)
]]
actor.add_static_obstacles(obstacles_static)
sim = StaticObsCircleSimulator(obstacles_static, agents, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0, seed=2)
metrics = sim.run_simulation(required_metrics=["collision_rate"], animate=False, visualize=False, save_visualization=True, file_name="static-1")

# Static obs visualization
finished = False
seed = 0
rotation = 0.1
while not finished:
    agents = 4
    actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=15, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0])
    # actor = Simple_ORCA_Actor(agent_radius, 0.1)
    obstacles_static = [[(-1.,-1.), (1.,-1.), (1.,1.), (-1.,1.)]]
    radius = 0.5
    num_sides = 6
    vertices = [
        (radius * math.cos(2 * math.pi * i / num_sides+rotation), 
        radius * math.sin(2 * math.pi * i / num_sides+rotation))
        for i in range(num_sides)
    ]
    obstacles_static = [[
        (radius * math.cos(2 * math.pi * i / num_sides+rotation)+1., 
        radius * math.sin(2 * math.pi * i / num_sides+rotation))
        for i in range(num_sides)
    ],
    [
        (radius * math.cos(2 * math.pi * i / num_sides+rotation), 
        radius * math.sin(2 * math.pi * i / num_sides+rotation)+1.)
        for i in range(num_sides)
    ],
    [
        (radius * math.cos(2 * math.pi * i / num_sides+rotation)-1., 
        radius * math.sin(2 * math.pi * i / num_sides+rotation))
        for i in range(num_sides)
    ],
    [
        (radius * math.cos(2 * math.pi * i / num_sides+rotation), 
        radius * math.sin(2 * math.pi * i / num_sides+rotation)-1.)
        for i in range(num_sides)
    ],]
    actor.add_static_obstacles(obstacles_static)
    sim = StaticObsCircleSimulator(obstacles_static, agents, 2.5, [], actor, orca_vel=0.75, agent_vel=1.0, seed=seed)
    metrics = sim.run_simulation(required_metrics=["success_rate"], animate=False, visualize=False, save_visualization=True, file_name="static-2")
    print(metrics, seed, rotation)
    finished = metrics["success_rate"]==1.
    seed = seed + 1