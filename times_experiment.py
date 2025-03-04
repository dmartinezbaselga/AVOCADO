#!/usr/bin/env python

#
# times_experiment.py
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
from simple_simulator import CircleSimulator, SquareSimulator
import matplotlib.pyplot as plt
import matplotlib

matplotlib.rcParams['mathtext.fontset'] = 'cm'
matplotlib.rcParams['font.family'] = 'STIXGeneral'
matplotlib.rcParams['font.size'] = 22

AVOCADO1_dict = {"a":0.3, "c":0.7, "d":1, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]}
AVOCADO2_dict = {"a":0.3, "c":0.7, "d":5, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]}
AVOCADO3_dict = {"a":0.3, "c":0.7, "d":1, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.5]}
AVOCADO4_dict = {"a":0.3, "c":0.7, "d":1, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[-0.5]}

def simulate_times(n_agents, n_cooperatives, actor_name, agent_radius, room_size, n_timesteps):
    times = []
    while len(times) < n_timesteps:
        np.random.seed(len(times))
        idx_non_cooperatives = np.random.choice(np.arange(n_agents), size=(n_agents-n_cooperatives), replace=False)  
        if actor_name == "avocado1":
            actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=AVOCADO1_dict["a"], c=AVOCADO1_dict["c"], d=AVOCADO1_dict["d"], kappa=AVOCADO1_dict["kappa"], epsilon=AVOCADO1_dict["epsilon"], delta=AVOCADO1_dict["delta"], bias=AVOCADO1_dict["bias"])
        elif actor_name == "avocado2":
            actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=AVOCADO2_dict["a"], c=AVOCADO2_dict["c"], d=AVOCADO2_dict["d"], kappa=AVOCADO2_dict["kappa"], epsilon=AVOCADO2_dict["epsilon"], delta=AVOCADO2_dict["delta"], bias=AVOCADO2_dict["bias"])
        elif actor_name == "avocado3":
            actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=AVOCADO3_dict["a"], c=AVOCADO3_dict["c"], d=AVOCADO3_dict["d"], kappa=AVOCADO3_dict["kappa"], epsilon=AVOCADO3_dict["epsilon"], delta=AVOCADO3_dict["delta"], bias=AVOCADO3_dict["bias"])
        elif actor_name == "avocado4":
            actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=AVOCADO4_dict["a"], c=AVOCADO4_dict["c"], d=AVOCADO4_dict["d"], kappa=AVOCADO4_dict["kappa"], epsilon=AVOCADO4_dict["epsilon"], delta=AVOCADO4_dict["delta"], bias=AVOCADO4_dict["bias"])
        elif actor_name == "orca":
            actor = Simple_ORCA_Actor(agent_radius, 0.1)
        elif actor_name == "drl":
            actor = DRLActor(agent_radius, 0.1)
        elif actor_name == "mpc":
            actor = MPCActor(agent_radius, 0.1)
        elif actor_name == "rl-rvo":
            actor = RL_RVO_Actor(agent_radius, 0.1)
        sim = CircleSimulator(n_agents=n_agents, circle_radius=room_size, idx_non_cooperative=idx_non_cooperatives, actor=actor)
        times.extend(sim.get_times(n_timesteps - len(times)))
    times = np.array(times)
    print(len(times), np.mean(times), np.std(times))
    return np.mean(times), np.std(times)

def compute_times():
    ### Circle tests
    actor_name_array = ["orca", "mpc", "rl-rvo", "drl", "avocado1", "avocado2", "avocado3", "avocado4"]
    # n_agents_arr = [5, 15, 25]
    n_agents_arr = np.arange(5,25)
    mean_times = np.full((len(actor_name_array), len(n_agents_arr)), np.nan)
    std_times = np.full((len(actor_name_array), len(n_agents_arr)), np.nan)
    agent_radius = 0.2
    for actor_idx in range(len(actor_name_array)):
    # for actor_name in ["orca"]:
        print("-------------------------------------------------------------------")
        print("------------------------", actor_name_array[actor_idx], "-------------------------------")
        print("-------------------------------------------------------------------")
        # successes = np.zeros((len(n_agents_arr), np.max(np.array(n_agents_arr))))
        for n_agents_idx in range(len(n_agents_arr)):
            n_agents = n_agents_arr[n_agents_idx]
            room_size = max(2.5, (2.3*2*agent_radius*n_agents)/(2*np.pi))
            mean_time, std_time = simulate_times(n_agents, 1, actor_name_array[actor_idx], agent_radius, room_size, 1000)
            mean_times[actor_idx, n_agents_idx] = mean_time*1000.0
            std_times[actor_idx, n_agents_idx] = std_time*1000.0
    np.savetxt('data/mean_times.out', mean_times, fmt='%1.2e')
    np.savetxt('data/std_times.out', std_times, fmt='%1.2e')

def write_latex_table():
    # Use np.loadtxt with appropriate parameters
    mean_times = np.loadtxt('data/mean_times.out', dtype=float)
    std_times = np.loadtxt('data/std_times.out', dtype=float)
    actor_name_array = ["orca", "mpc", "rl-rvo", "drl", "avocado1", "avocado2", "avocado3", "avocado4"]
    plotting_names = ["ORCA", "MPC", "DRL", "RL-RVO", "AVOCADO1 (ours)", "AVOCADO2 (ours)", "AVOCADO3 (ours)", "AVOCADO4 (ours)"]
    n_agents_arr = [5, 15, 25]
    print("\\begin{table*}[]")
    print("\\centering")
    print("\\begin{tabular}{c|ccc}")
    print("N-agents & 5 & 15 & 25 \\\\")
    print("\\hline")
    for actor_idx in range(len(actor_name_array)):
        print("\\textbf{" + plotting_names[actor_idx] + "} & $" + str(mean_times[actor_idx][0]) + " \\pm " + str(std_times[actor_idx][0]) + 
              "$ & $" + str(mean_times[actor_idx][1]) + " \\pm " + str(std_times[actor_idx][1]) + 
              "$ & $" + str(mean_times[actor_idx][2]) + " \\pm " + str(std_times[actor_idx][2]) + "$ \\\\")
    print("\\end{tabular}")
    print("\\caption{Computational time (ms)}")
    print("\\label{tab:comp-times}")
    print("\\end{table*}")

def plot_times():
    mean_times = np.loadtxt('data/mean_times.out', dtype=float)
    std_times = np.loadtxt('data/std_times.out', dtype=float)
    actor_name_array = ["orca", "mpc", "rl-rvo", "drl", "avocado1"]
    plotting_names = ["ORCA", "T-MPC", "SARL", "RL-RVO", "AVOCADO (ours)"]
    N_values = np.arange(5,25)
    plt.figure(figsize=(10,6))
    for actor_idx in range(len(actor_name_array)):
        plt.plot(N_values, mean_times[actor_idx], label=plotting_names[actor_idx], marker='o', linestyle='-')
        plt.fill_between(N_values, mean_times[actor_idx] - std_times[actor_idx], mean_times[actor_idx] + std_times[actor_idx], alpha=0.2)
    plt.yscale('log')
    plt.xlabel(r'$\mathsf{N}$', fontweight ='bold')
    plt.ylabel(r'Computational Cost (ms)')
    plt.grid(True, which='both', linestyle='--', linewidth=1.0)
    plt.legend(loc='upper left')
    plt.savefig("images/times.png", format='png', bbox_inches='tight')
# compute_times()
plot_times()
# write_latex_table()
