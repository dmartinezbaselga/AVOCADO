#
# plot_data_results.py
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
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import matplotlib

matplotlib.rcParams['mathtext.fontset'] = 'cm'
matplotlib.rcParams['font.family'] = 'STIXGeneral'
matplotlib.rcParams['font.size'] = 22

y_labels = np.array([10, 12, 15, 17, 20, 22, 25])
x_labels = np.array([0.01,0.25,0.5,0.75,1.0])
# metrics = ["success-circle", "success-square"]
# metrics = ["success-circle", "times-circle", "roughness-circle", "path_len-circle"]
# metrics.extend(["success-square", "times-square", "roughness-square", "path_len-square"])
metrics = ["times-circle", "times-square"]
actor_names = ["orca", "mpc", "drl", "rl-rvo", "avocado1", "avocado2", "avocado3", "avocado4"]
plotting_names = [r'ORCA', r'T-MPC', r'SARL', r'RL-RVO', r'AVOCADO_1 (ours)', r'AVOCADO_2 (ours)', r'AVOCADO_3 (ours)', r'AVOCADO_4 (ours)']

# for metric in metrics:
#     values = []
#     std = []

#     # Find global min and max values across all data
#     for actor_name in actor_names:
#         filename = f"data/{actor_name}-{metric}.out"
#         # Use np.loadtxt with appropriate parameters
#         values.append(np.loadtxt(filename, dtype=float))

#         if metric != "success-circle" and metric != "success-square":
#             filename = f"data/{actor_name}-std-{metric}.out"
#             std.append(np.loadtxt(filename, dtype=float))

#     values = np.array(values)
#     std = np.array(std)
#     # fig.suptitle(metric)
#     for i in range(len(y_labels)):
#         plt.figure(figsize=(7, 7), layout='constrained')
#         plt.xlabel(r'$\mathsf{P}$')
#         plt.ylabel(metric)
#         for j in range(len(actor_names)):
#             plt.plot(x_labels, values[j,i,:], label=plotting_names[j], marker='o', linestyle='-')
#             if metric != "success-circle" and metric != "success-square":
#                 plt.fill_between(x_labels, values[j,i,:] - std[j,i,:], values[j,i,:] + std[j,i,:], alpha=0.2)
#         plt.title(r'$\mathsf{{N}} = {}$'.format(y_labels[i]))
#         plt.legend(loc="lower left")
#         plt.grid(True, which='both', linestyle='--', linewidth=1.0)
#         plt.savefig("images/" + metric + "-N-fixed-" + str(i) + ".png", format='png', bbox_inches='tight')

for metric in metrics:
    values = []
    std = []
    # Find global min and max values across all data
    for actor_name in actor_names:
        filename = f"data/{actor_name}-{metric}.out"
        # Use np.loadtxt with appropriate parameters
        values.append(np.loadtxt(filename, dtype=float))
        if metric != "success-circle" and metric != "success-square":
            filename = f"data/{actor_name}-std-{metric}.out"
            std.append(np.loadtxt(filename, dtype=float))
    std = np.array(std)
    values = np.array(values)
    for i in range(len(x_labels)):
        plt.figure(figsize=(10, 6), layout='constrained')
        plt.xlabel(r'$\mathsf{N}$')
        if metric == "times-square":
            plt.ylabel(r'Mean navigation time (s)')
        else:
            plt.ylabel(metric)
        for j in range(len(actor_names)):
            plt.plot(y_labels, values[j,:,i], label=plotting_names[j], marker='o', linestyle='-')
            if metric != "success-circle" and metric != "success-square":
                plt.fill_between(y_labels, values[j,:,i] - std[j,:,i], values[j,:,i] + std[j,:,i], alpha=0.2)
            
        # plt.title(r'$\mathsf{{P}} = {}$'.format(x_labels[i]))
        plt.legend(loc="upper left")
        plt.grid(True, which='both', linestyle='--', linewidth=1.0)
 
        plt.savefig("images/" + metric + "-P-fixed-" + str(i) + ".png", format='png', bbox_inches='tight')

metrics = ["success-circle", "success-square"]

matplotlib.rcParams['font.size'] = 15

for metric in metrics:
    values = []

    # Find global min and max values across all data
    max_v = -np.inf
    min_v = np.inf
    for actor_name in actor_names:
        filename = f"data/{actor_name}-{metric}.out"
        # Use np.loadtxt with appropriate parameters
        values.append(np.loadtxt(filename, dtype=float))
        v = values[-1]
        max_v = np.max([max_v, np.max(v[~np.isnan(v)])])  # Update max_v considering all values
        min_v = np.min([min_v, np.min(v[~np.isnan(v)])])  # Update min_v considering all values

    values = np.array(values)
    if "success" in metric:
        max_indexes = np.argmax(np.array(values), axis=0)
    else:
        max_indexes = np.argmin(np.array(values), axis=0)
    fig, axs = plt.subplots(4, 2, sharex=True, sharey=True, figsize=(8, 10), layout='constrained')
    fig.supxlabel(r'$\mathsf{P}$')
    fig.supylabel(r'$\mathsf{N}$')
    # fig.suptitle(metric)
    axs = axs.flatten()
    
    plt.yticks(np.arange(len(y_labels)), y_labels)  # Set y-axis ticks and labels
    plt.xticks(np.arange(len(x_labels)), x_labels)  # Set y-axis ticks and labels
    # cbar = fig.colorbar(plt.cm.ScalarMappable(cmap="viridis"), fraction=fraction)
    for i in range(len(axs)):
        print(f"Actor: {actor_names[i]}, metric: {metric}")
        print(values[i])
        axs[i].imshow(values[i], cmap="viridis", vmin=min_v, vmax=max_v, aspect='auto', interpolation='nearest')
        axs[i].set_title(plotting_names[i])
    for x in range(max_indexes.shape[0]):
        for y in range(max_indexes.shape[1]):
            print(x,y)
            for z in range(values.shape[0]):
                if values[max_indexes[x,y], x, y] == values[z,x,y]:
                    axs[z].plot(y, x, 'rx', markersize=12, mew=5)
    normalizer = Normalize(min_v, max_v)
    colorbar = fig.colorbar(plt.cm.ScalarMappable(cmap="viridis", norm=normalizer), ax=axs.ravel().tolist())
    plt.savefig("images/" + metric + ".png", format='png', bbox_inches='tight')

# metrics = ["success-circle", "success-square"]

# # exit()
# # -------------------------- Times circle 2 ---------------------------------------------------------
# x_labels = np.array([0.01,0.25,0.5,0.75])
# metrics = ["times-circle"]
# for metric in metrics:
#     values = []

#     # Find global min and max values across all data
#     max_v = -np.inf
#     min_v = np.inf
#     for actor_name in actor_names:
#         filename = f"data/{actor_name}-{metric}.out"
#         # Use np.loadtxt with appropriate parameters
#         values.append(np.loadtxt(filename, dtype=float))
#         v = values[-1][:,:-1]
#         max_v = np.max([max_v, np.max(v[~np.isnan(v)])])  # Update max_v considering all values
#         min_v = np.min([min_v, np.min(v[~np.isnan(v)])])  # Update min_v considering all values

#     values = np.array(values)
#     if "success" in metric:
#         max_indexes = np.argmax(np.array(values), axis=0)
#     else:
#         max_indexes = np.argmin(np.array(values), axis=0)
#     fig, axs = plt.subplots(4, 2, sharex=True, sharey=True, figsize=(8, 10), layout='constrained')
#     fig.supxlabel(r'$\mathsf{P}$')
#     fig.supylabel(r'$\mathsf{N}$')
#     # fig.suptitle(metric)
#     axs = axs.flatten()
    
#     plt.yticks(np.arange(len(y_labels)), y_labels)  # Set y-axis ticks and labels
#     plt.xticks(np.arange(len(x_labels)), x_labels)  # Set y-axis ticks and labels
#     # cbar = fig.colorbar(plt.cm.ScalarMappable(cmap="viridis"), fraction=fraction)
#     for i in range(len(axs)):
#         print(f"Actor: {plotting_names[i]}, metric: {metric}")
#         print(values[i])
#         axs[i].imshow(values[i][:,:-1], cmap="viridis", vmin=min_v, vmax=max_v, aspect='auto')
#         axs[i].set_title(plotting_names[i])
#     for x in range(max_indexes.shape[0]):
#         for y in range(max_indexes.shape[1]):
#             print(x,y)
#             for z in range(values.shape[0]):
#                 if values[max_indexes[x,y], x, y] == values[z,x,y]:
#                     axs[z].plot(y, x, 'rx')
#     normalizer = Normalize(min_v, max_v)
#     colorbar = fig.colorbar(plt.cm.ScalarMappable(cmap="viridis", norm=normalizer), ax=axs.ravel().tolist())
#     plt.savefig("images/" + metric + "2.png", format='png', bbox_inches='tight')
# exit()
