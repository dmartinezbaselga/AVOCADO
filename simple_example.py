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

agent_radius = 0.2
a=0.3
c=0.7
d=2
kappa= 14.15
epsilon= 3.22
delta= 0.57
bias=[0.]
# The parameter alpha is a list of alphas, being alpha the cooperation level. If alpha in [0,1], it is fixed.
# If alpha > 1, then it adapts the cooperation level using AVOCADO. The list of alphas is there so that you can directly
# use AVOCADO/non-AVOCADO agents in the same line.
actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=d, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0.])
sim = CircleSimulator(10, 3.0, [0,3, 4], actor, orca_vel=1.0, agent_vel=1.0, seed=0)
print(sim.run_simulation(required_metrics=["collision", "sim_time", "mean_agent_time"], visualize=True, animate=True, save_visualization=False, file_name=""))