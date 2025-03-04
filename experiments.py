#!/usr/bin/env python

#
# experiments.py
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
from actors import SimpleActor, AVOCADO_Actor, Simple_ORCA_Actor #, DRLActor, MPCActor, RL_RVO_Actor
from simple_simulator import CircleSimulator, SquareSimulator
from multiprocessing import Process, shared_memory
from pathlib import Path

n_episodes = 128

AVOCADO1_dict = {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]}
AVOCADO2_dict = {"a":0.3, "c":0.7, "d":5, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[0.]}
AVOCADO3_dict = {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[1.]}
AVOCADO4_dict = {"a":0.3, "c":0.7, "d":2, "kappa":14.15, "epsilon":3.22, "delta":0.57, "bias":[-1.]}

def simulate_circle(process_id, n_agents, n_cooperatives, actor_name, agent_radius, room_size, success_episode, time_episode, roughness_episode, path_len_episode):
    for seed in range(round((n_episodes/8)*process_id), round((n_episodes/8)*(process_id+1))):
        np.random.seed(seed)
        idx_non_cooperatives = np.random.choice(np.arange(n_agents), size=(n_agents-n_cooperatives), replace=False)  
        if actor_name == "avocado1":
            actor = AVOCADO_Actor(agent_radius, 0.1, max_noise=0.0001, alpha=[100.], a=AVOCADO1_dict["a"], c=AVOCADO1_dict["c"], d=AVOCADO1_dict["d"], kappa=AVOCADO1_dict["kappa"], epsilon=AVOCADO1_dict["epsilon"], delta=AVOCADO1_dict["delta"], bias=AVOCADO1_dict["bias"])
        elif actor_name == "avocado2":
            actor = AVOCADO_Actor(agent_radius, 0.1, max_noise=0.0001, alpha=[100.], a=AVOCADO2_dict["a"], c=AVOCADO2_dict["c"], d=AVOCADO2_dict["d"], kappa=AVOCADO2_dict["kappa"], epsilon=AVOCADO2_dict["epsilon"], delta=AVOCADO2_dict["delta"], bias=AVOCADO2_dict["bias"])
        elif actor_name == "avocado3":
            actor = AVOCADO_Actor(agent_radius, 0.1, max_noise=0.0001, alpha=[100.], a=AVOCADO3_dict["a"], c=AVOCADO3_dict["c"], d=AVOCADO3_dict["d"], kappa=AVOCADO3_dict["kappa"], epsilon=AVOCADO3_dict["epsilon"], delta=AVOCADO3_dict["delta"], bias=AVOCADO3_dict["bias"])
        elif actor_name == "avocado4":
            actor = AVOCADO_Actor(agent_radius, 0.1, max_noise=0.0001, alpha=[100.], a=AVOCADO4_dict["a"], c=AVOCADO4_dict["c"], d=AVOCADO4_dict["d"], kappa=AVOCADO4_dict["kappa"], epsilon=AVOCADO4_dict["epsilon"], delta=AVOCADO4_dict["delta"], bias=AVOCADO4_dict["bias"])
        elif actor_name == "orca":
            actor = Simple_ORCA_Actor(agent_radius, 0.1)
        elif actor_name == "drl":
            actor = DRLActor(agent_radius, 0.1)
        elif actor_name == "mpc":
            actor = MPCActor(agent_radius, 0.1)
        elif actor_name == "rl-rvo":
            actor = RL_RVO_Actor(agent_radius, 0.1)
        sim = CircleSimulator(n_agents=n_agents, circle_radius=room_size, idx_non_cooperative=idx_non_cooperatives, actor=actor)
        visualize = False
        metrics = sim.run_simulation(required_metrics=["success_rate", "times", "roughness", "path_lens"], animate=visualize)
        success_episode[seed] = metrics["success_rate"]
        time_episode[seed] = metrics["times"]
        roughness_episode[seed] = metrics["roughness"]
        path_len_episode[seed] = metrics["path_lens"]

def simulate_square(process_id, n_cooperatives, n_non_cooperatives, actor_name, agent_radius, room_size, success_episode, time_episode, roughness_episode, path_len_episode):
    for seed in range(round((n_episodes/8)*process_id), round((n_episodes/8)*(process_id+1))):
        np.random.seed(seed)
        if actor_name == "avocado1":
            actor = AVOCADO_Actor(agent_radius, 0.1, max_noise=0.0001, alpha=[100.], a=AVOCADO1_dict["a"], c=AVOCADO1_dict["c"], d=AVOCADO1_dict["d"], kappa=AVOCADO1_dict["kappa"], epsilon=AVOCADO1_dict["epsilon"], delta=AVOCADO1_dict["delta"], bias=AVOCADO1_dict["bias"])
        elif actor_name == "avocado2":
            actor = AVOCADO_Actor(agent_radius, 0.1, max_noise=0.0001, alpha=[100.], a=AVOCADO2_dict["a"], c=AVOCADO2_dict["c"], d=AVOCADO2_dict["d"], kappa=AVOCADO2_dict["kappa"], epsilon=AVOCADO2_dict["epsilon"], delta=AVOCADO2_dict["delta"], bias=AVOCADO2_dict["bias"])
        elif actor_name == "avocado3":
            actor = AVOCADO_Actor(agent_radius, 0.1, max_noise=0.0001, alpha=[100.], a=AVOCADO3_dict["a"], c=AVOCADO3_dict["c"], d=AVOCADO3_dict["d"], kappa=AVOCADO3_dict["kappa"], epsilon=AVOCADO3_dict["epsilon"], delta=AVOCADO3_dict["delta"], bias=AVOCADO3_dict["bias"])
        elif actor_name == "avocado4":
            actor = AVOCADO_Actor(agent_radius, 0.1, max_noise=0.0001, alpha=[100.], a=AVOCADO4_dict["a"], c=AVOCADO4_dict["c"], d=AVOCADO4_dict["d"], kappa=AVOCADO4_dict["kappa"], epsilon=AVOCADO4_dict["epsilon"], delta=AVOCADO4_dict["delta"], bias=AVOCADO4_dict["bias"])
        elif actor_name == "orca":
            actor = Simple_ORCA_Actor(agent_radius, 0.1)
        elif actor_name == "drl":
            actor = DRLActor(agent_radius, 0.1)
        elif actor_name == "mpc":
            actor = MPCActor(agent_radius, 0.1)
        elif actor_name == "rl-rvo":
            actor = RL_RVO_Actor(agent_radius, 0.1)
        sim = SquareSimulator(n_cooperative=n_cooperatives, n_non_cooperative=n_non_cooperatives, square_width=room_size, actor=actor, seed=seed)
        visualize = False
        metrics = sim.run_simulation(required_metrics=["success_rate", "times", "roughness", "path_lens"], animate=visualize)
        success_episode[seed] = metrics["success_rate"]
        time_episode[seed] = metrics["times"]
        roughness_episode[seed] = metrics["roughness"]
        path_len_episode[seed] = metrics["path_lens"]

def multithread_simulation_circle(actor_name, n_agents, n_agents_idx, n_cooperatives, n_cooperatives_idx, agent_radius, room_size, successes, times, roughness, path_len,
                           std_times, std_roughness, std_path_len):
    print(actor_name, n_agents, n_cooperatives)
    processes = []
    success_episode = np.zeros((n_episodes,), dtype=np.float32)
    time_episode = np.zeros((n_episodes,n_cooperatives), dtype=np.float32)
    roughness_episode = np.zeros((n_episodes,n_cooperatives), dtype=np.float32)
    path_len_episode = np.zeros((n_episodes,n_cooperatives), dtype=np.float32)
    shm1 = shared_memory.SharedMemory(create=True, size=success_episode.nbytes)
    shm2 = shared_memory.SharedMemory(create=True, size=time_episode.nbytes)
    shm3 = shared_memory.SharedMemory(create=True, size=roughness_episode.nbytes)
    shm4 = shared_memory.SharedMemory(create=True, size=path_len_episode.nbytes)
    success_episode = np.ndarray((n_episodes,), dtype=np.float32, buffer=shm1.buf)
    time_episode = np.ndarray((n_episodes,n_cooperatives), dtype=np.float32, buffer=shm2.buf)
    roughness_episode = np.ndarray((n_episodes,n_cooperatives), dtype=np.float32, buffer=shm3.buf)
    path_len_episode = np.ndarray((n_episodes,n_cooperatives), dtype=np.float32, buffer=shm4.buf)
    for idx in range(8):
        processes.append(Process(target=simulate_circle, 
                                args=(idx, n_agents, n_cooperatives, actor_name, agent_radius, room_size, 
                                        success_episode, time_episode, roughness_episode, path_len_episode)))
        processes[idx].start()
    for idx in range(8):
        processes[idx].join()

    shm1.unlink()
    shm2.unlink()
    shm3.unlink()
    shm4.unlink()
    time_episode = np.array(time_episode)[~np.isnan(time_episode)]
    roughness_episode = np.array(roughness_episode)[~np.isnan(roughness_episode)]
    path_len_episode = np.array(path_len_episode)[~np.isnan(path_len_episode)]

    successes[n_agents_idx, n_cooperatives_idx] = np.array(success_episode).mean()
    if len(time_episode) > 0:
        times[n_agents_idx, n_cooperatives_idx] = np.array(time_episode).mean()
        std_times[n_agents_idx, n_cooperatives_idx] = np.array(time_episode).std()
        roughness[n_agents_idx, n_cooperatives_idx] = np.array(roughness_episode).mean()
        std_roughness[n_agents_idx, n_cooperatives_idx] = np.array(roughness_episode).std()
        path_len[n_agents_idx, n_cooperatives_idx] = np.array(path_len_episode).mean()
        std_path_len[n_agents_idx, n_cooperatives_idx] = np.array(path_len_episode).std()
    Path("data").mkdir(exist_ok=True)
    np.savetxt('data/' + actor_name + '-success-circle.out', successes, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-times-circle.out', times, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-std-times-circle.out', std_times, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-roughness-circle.out', roughness, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-std-roughness-circle.out', std_roughness, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-path_len-circle.out', path_len, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-std-path_len-circle.out', std_path_len, fmt='%1.4e')
    return successes, times, roughness, path_len, std_times, std_roughness, std_path_len

def multithread_simulation_square(actor_name, n_agents, n_agents_idx, n_cooperatives, n_cooperatives_idx, agent_radius, room_size, successes, times, roughness, path_len,
                           std_times, std_roughness, std_path_len):
    print(actor_name, n_agents, n_cooperatives)
    processes = []
    success_episode = np.zeros((n_episodes,), dtype=np.float32)
    time_episode = np.zeros((n_episodes,n_cooperatives), dtype=np.float32)
    roughness_episode = np.zeros((n_episodes,n_cooperatives), dtype=np.float32)
    path_len_episode = np.zeros((n_episodes,n_cooperatives), dtype=np.float32)
    shm1 = shared_memory.SharedMemory(create=True, size=success_episode.nbytes)
    shm2 = shared_memory.SharedMemory(create=True, size=time_episode.nbytes)
    shm3 = shared_memory.SharedMemory(create=True, size=roughness_episode.nbytes)
    shm4 = shared_memory.SharedMemory(create=True, size=path_len_episode.nbytes)
    success_episode = np.ndarray((n_episodes,), dtype=np.float32, buffer=shm1.buf)
    time_episode = np.ndarray((n_episodes,n_cooperatives), dtype=np.float32, buffer=shm2.buf)
    roughness_episode = np.ndarray((n_episodes,n_cooperatives), dtype=np.float32, buffer=shm3.buf)
    path_len_episode = np.ndarray((n_episodes,n_cooperatives), dtype=np.float32, buffer=shm4.buf)
    for idx in range(8):
        processes.append(Process(target=simulate_square, 
                                args=(idx, n_cooperatives, n_agents-n_cooperatives, actor_name, agent_radius, room_size, 
                                        success_episode, time_episode, roughness_episode, path_len_episode)))
        processes[idx].start()
    for idx in range(8):
        processes[idx].join()

    shm1.unlink()
    shm2.unlink()
    shm3.unlink()
    shm4.unlink()
    time_episode = np.array(time_episode)[~np.isnan(time_episode)]
    roughness_episode = np.array(roughness_episode)[~np.isnan(roughness_episode)]
    path_len_episode = np.array(path_len_episode)[~np.isnan(path_len_episode)]

    successes[n_agents_idx, n_cooperatives_idx] = np.array(success_episode).mean()
    if len(time_episode) > 0:
        times[n_agents_idx, n_cooperatives_idx] = np.array(time_episode).mean()
        std_times[n_agents_idx, n_cooperatives_idx] = np.array(time_episode).std()
        roughness[n_agents_idx, n_cooperatives_idx] = np.array(roughness_episode).mean()
        std_roughness[n_agents_idx, n_cooperatives_idx] = np.array(roughness_episode).std()
        path_len[n_agents_idx, n_cooperatives_idx] = np.array(path_len_episode).mean()
        std_path_len[n_agents_idx, n_cooperatives_idx] = np.array(path_len_episode).std()
    Path("data").mkdir(exist_ok=True)
    np.savetxt('data/' + actor_name + '-success-square.out', successes, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-times-square.out', times, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-std-times-square.out', std_times, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-roughness-square.out', roughness, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-std-roughness-square.out', std_roughness, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-path_len-square.out', path_len, fmt='%1.4e')
    np.savetxt('data/' + actor_name + '-std-path_len-square.out', std_path_len, fmt='%1.4e')
    return successes, times, roughness, path_len, std_times, std_roughness, std_path_len

### Circle tests
def circle_tests():
    n_agents_arr = [10, 12, 15, 17]
    agent_radius = 0.2
    # for actor_name in ["orca", "avocado1", "avocado2", "avocado3", "avocado4", "mpc", "rl-rvo", "drl"]:
    for actor_name in ["avocado1", "avocado2", "avocado3", "avocado4"]:
        print("-------------------------------------------------------------------")
        print("------------------------", actor_name, "-------------------------------")
        print("-------------------------------------------------------------------")
        # successes = np.zeros((len(n_agents_arr), np.max(np.array(n_agents_arr))))
        successes = np.zeros((len(n_agents_arr), 5))
        times = np.full_like(successes, np.nan)
        roughness = np.full_like(successes, np.nan)
        path_len = np.full_like(successes, np.nan)
        std_times = np.full_like(successes, np.nan)
        std_roughness = np.full_like(successes, np.nan)
        std_path_len = np.full_like(successes, np.nan)
        for n_agents_idx in range(len(n_agents_arr)):
            n_agents = n_agents_arr[n_agents_idx]
            room_size = max(2.5, (2.3*2*agent_radius*n_agents)/(2*np.pi))
            # room_size = (2.3*2*agent_radius*n_agents)/(2*np.pi)
            n_cooperatives_array = [1, int(np.ceil(n_agents/4.)), int(np.ceil(n_agents/2)), int(np.ceil(3*(n_agents/4))), n_agents]
            for n_cooperatives_idx in range(5):
                successes, times, roughness, path_len, std_times, std_roughness, std_path_len = multithread_simulation_circle(actor_name, n_agents, n_agents_idx, n_cooperatives_array[n_cooperatives_idx], n_cooperatives_idx, agent_radius, room_size,
                                                                    successes, times, roughness, path_len, std_times, std_roughness, std_path_len)

### Circle tests
def square_tests():
    n_agents_arr = [10, 12, 15, 17, 20, 22, 25]
    agent_radius = 0.2
    # for actor_name in ["orca", "avocado1", "avocado2", "avocado3", "avocado4", "mpc", "rl-rvo", "drl"]:
    for actor_name in ["avocado1", "avocado2", "avocado3", "avocado4"]:
        print("-------------------------------------------------------------------")
        print("------------------------", actor_name, "-------------------------------")
        print("-------------------------------------------------------------------")
        # successes = np.zeros((len(n_agents_arr), np.max(np.array(n_agents_arr))))
        successes = np.zeros((len(n_agents_arr), 5))
        times = np.full_like(successes, np.nan)
        roughness = np.full_like(successes, np.nan)
        path_len = np.full_like(successes, np.nan)
        std_times = np.full_like(successes, np.nan)
        std_roughness = np.full_like(successes, np.nan)
        std_path_len = np.full_like(successes, np.nan)
        for n_agents_idx in range(len(n_agents_arr)):
            n_agents = n_agents_arr[n_agents_idx]
            room_size = agent_radius*n_agents*1.5
            n_cooperatives_array = [1, int(np.ceil(n_agents/4.)), int(np.ceil(n_agents/2)), int(np.ceil(3*(n_agents/4))), n_agents]
            for n_cooperatives_idx in range(5):
                successes, times, roughness, path_len, std_times, std_roughness, std_path_len = multithread_simulation_square(actor_name, n_agents, n_agents_idx, n_cooperatives_array[n_cooperatives_idx], n_cooperatives_idx, agent_radius, room_size,
                                                                    successes, times, roughness, path_len, std_times, std_roughness, std_path_len)

circle_tests()
# square_tests()