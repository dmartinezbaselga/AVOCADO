#!/usr/bin/env python

#
# bayesian_optimization.py
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
from multiprocessing import Process
import optuna
from optuna.samplers import TPESampler
from multiprocessing import shared_memory
from simple_simulator import CircleSimulator
from actors import AVOCADO_Actor

n_agents = 25
agent_radius = 0.2

def run_simulation(process_id, n_agents, n_cooperatives_vec, agent_radius, room_size, a, c, d, kappa, epsilon, delta, results):
    for seed in range(round((128/8)*process_id), round((128/8)*(process_id+1))):
        np.random.seed(seed)
        idx_non_cooperatives = np.random.choice(np.arange(n_agents), size=(n_agents-n_cooperatives_vec[seed%len(n_cooperatives_vec)]), replace=False)
        actor = AVOCADO_Actor(agent_radius, 0.1, alpha=[100.], a=a, c=c, d=d, kappa=kappa, epsilon=epsilon, delta=delta, bias=[0.])
        sim = CircleSimulator(n_agents=n_agents, circle_radius=room_size, idx_non_cooperative=idx_non_cooperatives, actor=actor)
        metrics = sim.run_simulation(required_metrics=["collision_rate", "mean_agent_time"])
        results[seed, :] = np.array([metrics["collision_rate"], metrics["mean_agent_time"]])

def multi_objective(trial):
    room_size = max(2.5, (2.3*2*agent_radius*n_agents)/(2*np.pi))
    a=trial.suggest_float("a",0.0,1.0)
    c=trial.suggest_float("c",0.0,1.0)
    d=trial.suggest_float("d",1,15)
    kappa=trial.suggest_float("kappa",1.0,20)
    epsilon=trial.suggest_float("epsilon",1,4)
    delta=trial.suggest_float("delta",0.5,0.99)
    # a=trial.suggest_float("a",0.1,0.1)
    # c=trial.suggest_float("c",0.9,0.9)
    # d=trial.suggest_float("d",10,10)
    # kappa=trial.suggest_float("kappa",4,4)
    # epsilon=trial.suggest_float("epsilon",2,2)
    # delta=trial.suggest_float("delta",0.9,0.9)
    processes = []
    results = np.zeros((128, 2))
    shm = shared_memory.SharedMemory(create=True, size=results.nbytes)
    results = np.ndarray((128, 2), dtype=np.float32, buffer=shm.buf)
    n_cooperatives_vec = [1, int(np.ceil(n_agents/4.)), int(np.ceil(n_agents/2)), int(np.ceil(3*(n_agents/4)))]
    for idx in range(8):
        processes.append(Process(target=run_simulation, 
                                args=(idx, n_agents, n_cooperatives_vec, agent_radius, room_size, a, c, d, kappa, epsilon, delta, results)))
        processes[idx].start()
    for idx in range(8):
        processes[idx].join()
    # with concurrent.futures.ProcessPoolExecutor(max_workers=8) as executor:
    #         for idx in range(n_agents - 1):
    #             executor.submit(run_simulation, n_agents, idx_villians[idx], a, c, d, kappa, epsilon, delta, results, idx)
    results_ = results
    shm.unlink()
    results_ = np.array(results_)
    return np.mean(results_[:,0][~np.isnan(results_[:,0])]), np.mean(results_[:,1][~np.isnan(results_[:,1])])

directions = ["minimize", "minimize"]
# optuna.logging.get_logger("optuna").addHandler(logging.StreamHandler(sys.stdout))
study_name = "study-circle"
storage_name = "sqlite:///{}.db".format(study_name)
study = optuna.create_study(study_name=study_name + str(n_agents), sampler=TPESampler(multivariate=True, group=True), directions=directions,
                            storage=storage_name, load_if_exists=True)
study.optimize(multi_objective, n_trials=100000)