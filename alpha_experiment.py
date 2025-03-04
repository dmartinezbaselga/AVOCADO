#!/usr/bin/env python

#
# alpha_experiment.py
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
from actors import AVOCADO_Actor
from simple_simulator import CircleSimulator, SquareSimulator
import matplotlib.pyplot as plt
import matplotlib

matplotlib.rcParams['mathtext.fontset'] = 'cm'
matplotlib.rcParams['font.family'] = 'STIXGeneral'
matplotlib.rcParams['font.size'] = 22
timestep = 0.1

def get_alpha(actor_dict, agent_radius):
    np.random.seed(0)
    actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=actor_dict["a"], c=actor_dict["c"], d=actor_dict["d"], kappa=actor_dict["kappa"], epsilon=actor_dict["epsilon"], delta=actor_dict["delta"], bias=actor_dict["bias"],
                            )#max_noise=0.0)#neighbor_dist=5, time_horizon=5, max_noise=0.0)                        #   max_noise=0.0)

    sim = CircleSimulator(n_agents=2, circle_radius=1.75, idx_non_cooperative=[0], actor=actor)
    return sim.run_simulation(required_metrics={"alphas"}, animate=False)["alphas"] 

def compute_alphas(actor_dicts, labels, file_name):
    ### Circle tests
    agent_radius = 0.2
    plt.figure(figsize=(6,5))
    for actor_idx in range(len(labels)):
        print("-------------------------------------------------------------------")
        print("------------------------", labels[actor_idx], "-------------------------------")
        print("-------------------------------------------------------------------")
        # successes = np.zeros((len(n_agents_arr), np.max(np.array(n_agents_arr))))
        alphas = get_alpha(actor_dicts[actor_idx], agent_radius)
        plt.plot(np.arange(len(alphas))*timestep, alphas, label=labels[actor_idx])
    plt.xlabel(r'$\mathrm{t(s)}$')
    plt.ylabel(r'$\mathrm{o_i}$')
    plt.legend()
    # plt.show()
    plt.savefig("images/alphas-" + file_name + ".png", format='png', bbox_inches='tight')

#---------------------------d experiment------------------------------------------
actor_dicts = [{"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]},
               {"a":0.3, "c":0.7, "d":3.5, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]},
               {"a":0.3, "c":0.7, "d":5, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]},
               {"a":0.3, "c":0.7, "d":6.5, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]}]
labels = [r'$\mathrm{d_i=2}$', r'$\mathrm{d_i=3.5}$', r'$\mathrm{d_i=5}$', r'$\mathrm{d_i=6.5}$']
compute_alphas(actor_dicts, labels, "d")
#---------------------------a experiment------------------------------------------
# actor_dicts = [{"a":0.1, "c":0.9, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]},
#                {"a":0.6, "c":0.4, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]},
#                {"a":0.9, "c":0.1, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]}]
# labels = [r'$\mathrm{a_i=0.1}$', r'$\mathrm{a_i=0.3}$', r'$\mathrm{a_i=0.6}$', r'$\mathrm{a_i=0.9}$']
# compute_alphas(actor_dicts, labels, "a")
# #---------------------------kappa experiment------------------------------------------
# actor_dicts = [{"a":0.3, "c":0.7, "d":2, "kappa":5, "epsilon":3.22, "delta":0.57, "bias":[0.]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":10, "epsilon":3.22, "delta":0.57, "bias":[0.]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":15, "epsilon":3.22, "delta":0.57, "bias":[0.]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":20, "epsilon":3.22, "delta":0.57, "bias":[0.]}]
# labels = [r'$\mathrm{\kappa_i=5}$', r'$\mathrm{\kappa_i=10}$', r'$\mathrm{\kappa_i=15}$', r'$\mathrm{\kappa_i=20}$']
# compute_alphas(actor_dicts, labels, "kappa")
# #---------------------------epsilon experiment------------------------------------------
# actor_dicts = [{"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":0.5, "delta":0.57, "bias":[0.]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":2, "delta":0.57, "bias":[0.]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.5, "delta":0.57, "bias":[0.]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":5, "delta":0.57, "bias":[0.]}]
# labels = [r'$\mathrm{\varepsilon_i=0.5}$', r'$\mathrm{\varepsilon_i=2}$', r'$\mathrm{\varepsilon_i=3.5}$', r'$\mathrm{\varepsilon_i=5}$']
# compute_alphas(actor_dicts, labels, "epsilon")
# #---------------------------delta experiment------------------------------------------
# actor_dicts = [{"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.1, "bias":[0.]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.3, "bias":[0.]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.6, "bias":[0.]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.9, "bias":[0.]}]
# labels = [r'$\mathrm{\delta_i=0.1}$', r'$\mathrm{\delta_i=0.3}$', r'$\mathrm{\delta_i=0.6}$', r'$\mathrm{\delta_i=0.9}$']
# compute_alphas(actor_dicts, labels, "delta")
# #---------------------------bias experiment------------------------------------------
# actor_dicts = [{"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[-1.]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[-0.5]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.5]},
#                {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[1.]}]
# labels = [r'$\mathrm{\frac{b_i}{d_i}=-0.5}$', r'$\mathrm{\frac{b_i}{d_i}=-0.25}$', r'$\mathrm{\frac{b_i}{d_i}=0.25}$', r'$\mathrm{\frac{b_i}{d_i}=0.5}$']
# compute_alphas(actor_dicts, labels, "bias")
