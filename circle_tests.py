#!/usr/bin/env python

#
# circle_tests.py
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

def run_simulation(n_agents, circle_radius, idx_non_cooperative, a, c, d, kappa, epsilon, delta, results, process_idx):
    sim = CircleSimulator(n_agents=n_agents, circle_radius=circle_radius, idx_non_cooperative=idx_non_cooperative, 
                          a=a, c=c, d=d, kappa=kappa, epsilon=epsilon, delta=delta)
    collision, time = sim.run_simulation()
    results[process_idx, :] = np.array([collision, time])

def multi_objective(trial):
    n_agents = 22
    circle_radius = 2.1
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
    results = np.zeros((n_agents - 2, 2))
    shm = shared_memory.SharedMemory(create=True, size=results.nbytes)
    results = np.ndarray((n_agents - 2, 2), dtype=np.float32, buffer=shm.buf)
    for idx in range(n_agents - 2):
        idx_non_cooperative = np.arange(0, n_agents, idx+3)
        processes.append(Process(target=run_simulation, 
                                args=(n_agents, circle_radius, idx_non_cooperative, a, c, d, kappa, epsilon, delta, results, idx)))
        processes[idx].start()
    for idx in range(n_agents - 2):
        processes[idx].join()
    # with concurrent.futures.ProcessPoolExecutor(max_workers=8) as executor:
    #         for idx in range(n_agents - 1):
    #             executor.submit(run_simulation, n_agents, idx_villians[idx], a, c, d, kappa, epsilon, delta, results, idx)
    results_ = results
    shm.unlink()
    return (*(results_.mean(axis=0)),)

directions = ["minimize", "minimize"]
# optuna.logging.get_logger("optuna").addHandler(logging.StreamHandler(sys.stdout))
study_name = "study-circle"
storage_name = "sqlite:///{}.db".format(study_name)
study = optuna.create_study(study_name=study_name, sampler=TPESampler(multivariate=True, group=True), directions=directions,
                            storage=storage_name, load_if_exists=True)
study.optimize(multi_objective, n_trials=100000)