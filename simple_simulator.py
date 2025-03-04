#!/usr/bin/env python

#
# simple_simulator.py
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

import avocado
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import matplotlib.cm as cm
from actors import MultiActor
import time
from matplotlib.patches import Polygon

matplotlib.rcParams['mathtext.fontset'] = 'cm'
matplotlib.rcParams['font.family'] = 'STIXGeneral'
matplotlib.rcParams['font.size'] = 42
# matplotlib.rcParams.update({'font.size': 82})
# matplotlib.pyplot.title(r'ABC123 vs $\mathrm{ABC123}^{123}$')

class Simulator:
    def __init__(self, pos_agents, goal_agents, idx_non_cooperative, timestep, agent_radius, actor: MultiActor, orca_vel, agent_max_vel) -> None:
        self.actor = actor
        self.max_vel = agent_max_vel
        self.orca_vel = orca_vel
        self.ids_sim_non_cooperatives = []
        self.n_agents = len(pos_agents)
        self.pos_agents = np.array(pos_agents)
        self.vel_agents = np.zeros_like(self.pos_agents)
        self.goal_agents = np.array(goal_agents)
        self.idx_non_cooperative = idx_non_cooperative
        self.agent_radius = agent_radius
        self.timestep = timestep               
        self.init_pos_non_cooperatives = np.array(pos_agents)[idx_non_cooperative] 
        self.sim_non_cooperative = avocado.PyRVOSimulator(timestep, 2.5, 15, 2.5, 2.5, agent_radius*1.1, orca_vel)
        for k in idx_non_cooperative:
            self.ids_sim_non_cooperatives.append(self.sim_non_cooperative.addAgent((pos_agents[k][0], pos_agents[k][1]), const_alpha=100., a=0.0478095298776161, 
                                                                                   c=0.9306259459385456, d=5.785584919565897, kappa=12.014871987687101, 
                                                                                   epsilon=1.433510968298775, delta=0.6410002165042703, max_noise=0.01))

    def get_desired_vel(self, agent_no):
        pos = self.pos_agents[agent_no]
        vel = np.array(self.goal_agents[agent_no]) - np.array(pos)
        return vel / max(1., (np.linalg.norm(vel)/self.orca_vel))        
        # return vel / np.linalg.norm(vel)

    def update_agents(self):
        mask = ~np.isin(np.arange(self.n_agents), self.idx_non_cooperative)  # Create mask
        
        start_time = time.time()
        self.vel_agents[mask] = self.actor.act(self.pos_agents[mask], self.pos_agents[self.idx_non_cooperative],
                                              self.vel_agents[mask], self.vel_agents[self.idx_non_cooperative], 
                                              self.goal_agents[mask], self.max_vel)
        end_time = time.time()
        for k in range(len(self.idx_non_cooperative)):
            vel = self.get_desired_vel(self.idx_non_cooperative[k])
            self.sim_non_cooperative.setAgentPrefVelocity(self.ids_sim_non_cooperatives[k], (vel[0], vel[1]))
        self.sim_non_cooperative.doStep()
        self.pos_agents[mask] = self.pos_agents[mask] + self.vel_agents[mask]*self.timestep
        
        for k in range(len(self.idx_non_cooperative)):
            self.pos_agents[self.idx_non_cooperative[k]] = self.sim_non_cooperative.getAgentPosition(self.ids_sim_non_cooperatives[k])
            self.vel_agents[self.idx_non_cooperative[k]] = self.sim_non_cooperative.getAgentVelocity(self.ids_sim_non_cooperatives[k])

        # print(np.array([self.reached_goal(self.pos_agents[self.idx_non_cooperative[k]], self.goal_agents[self.idx_non_cooperative[k]], 0.2) for k in range(len(self.idx_non_cooperative))]))
        if np.array([self.reached_goal(self.pos_agents[self.idx_non_cooperative[k]], self.goal_agents[self.idx_non_cooperative[k]], 0.2) for k in range(len(self.idx_non_cooperative))]).all():
            self.goal_agents[self.idx_non_cooperative] = self.init_pos_non_cooperatives
            self.init_pos_non_cooperatives = self.pos_agents[self.idx_non_cooperative]
        return end_time - start_time

    def check_collision(self, robot1, robot2):
        distance = np.linalg.norm(np.array(robot1) - np.array(robot2))
        return distance < 2*self.agent_radius
    
    def reached_goal(self, robot, goal, goal_tolerance):
        distance = np.linalg.norm(np.array(robot) - np.array(goal))
        # print(np.array(robot), np.array(goal), distance)
        return distance < goal_tolerance
    
    def get_times(self, num_timesteps):
        finished = False
        frame = 0
        collision = np.zeros((self.n_agents,), dtype=bool)
        success = np.zeros((self.n_agents,), dtype=bool)
        poses = []
        mask = ~np.isin(np.arange(self.n_agents), self.idx_non_cooperative)  # Create mask
        goal_tolerance = 0.1
        comp_times = []
        # First update
        self.update_agents()
        while not finished:
            time = frame*self.timestep
            # print(time)
            
            for agent_i in range(self.n_agents):
                for agent_j in range(agent_i+1, self.n_agents):
                    if (self.check_collision(self.pos_agents[agent_i], self.pos_agents[agent_j])
                        and not (agent_i in self.idx_non_cooperative and agent_j in self.idx_non_cooperative)):
                        collision[agent_i] = not success[agent_i]
                        collision[agent_j] = not success[agent_j]
                        # print("Collision produced")
                if ((not agent_i in self.idx_non_cooperative) and
                    self.reached_goal(self.pos_agents[agent_i], self.goal_agents[agent_i], goal_tolerance)):
                    if not success[agent_i] and not collision[agent_i]:
                        success[agent_i] = True
            if np.sum(collision[mask]) + np.sum(success[mask]) == (self.n_agents - len(self.idx_non_cooperative)):
                finished = True
            elif frame == 200:
                finished = True
            frame += 1
            if not finished:
                comp_times.append(self.update_agents())
            # print(len(comp_times), num_timesteps)
            if len(comp_times) >= num_timesteps:
                finished = True
        return comp_times     

    def run_simulation(self, required_metrics, visualize=False, save_visualization=False, animate=False, animate_with_alphas=False, file_name="scenario"):
        finished = False
        frame = 0
        collision = np.zeros((self.n_agents,), dtype=bool)
        success = np.zeros((self.n_agents,), dtype=bool)
        time_agents = np.ones((self.n_agents,), dtype=np.float32)*np.nan
        path_lengths = np.zeros((self.n_agents,), dtype=np.float32)
        final_path_len = np.ones((self.n_agents,), dtype=np.float32)*np.nan
        roughness = np.zeros((self.n_agents,), dtype=np.float32)
        final_roughness = np.ones((self.n_agents,), dtype=np.float32)*np.nan
        alphas = []
        poses = []
        collision_anim = []
        mask = ~np.isin(np.arange(len(time_agents)), self.idx_non_cooperative)  # Create mask
        if animate:
            goal_tolerance = 0.1
        else:
            goal_tolerance = 0.1
        while not finished:
            time = frame*self.timestep
            # print(time)
            if "roughness" in required_metrics:
                previous_vel = self.vel_agents.copy()
            self.update_agents()
            if "roughness" in required_metrics:
                acc = self.vel_agents - previous_vel
                roughness = roughness + np.linalg.norm(acc, axis=1)
            for agent_i in range(self.n_agents):
                for agent_j in range(agent_i+1, self.n_agents):
                    if (self.check_collision(self.pos_agents[agent_i], self.pos_agents[agent_j])
                        and not (agent_i in self.idx_non_cooperative and agent_j in self.idx_non_cooperative)):
                        collision[agent_i] = not success[agent_i]
                        collision[agent_j] = not success[agent_j]
                        # print("Collision produced")
                if ((not agent_i in self.idx_non_cooperative) and
                    self.reached_goal(self.pos_agents[agent_i], self.goal_agents[agent_i], goal_tolerance)):
                    if np.isnan(time_agents[agent_i]) and not collision[agent_i]:
                        time_agents[agent_i] = time
                        success[agent_i] = True
                        if "path_lens" in required_metrics:
                            final_path_len[agent_i] = path_lengths[agent_i]
                        if "roughness" in required_metrics:
                            final_roughness[agent_i] = roughness[agent_i]/frame
            if "alphas" in required_metrics or animate_with_alphas:
                # alphas.append([[self.actor.sim.getAlpha(self.actor.id_agents[agent_i], self.actor.id_agents[agent_j]) for agent_j in range(self.n_agents)]
                #                for agent_i in range(self.n_agents)])
                alphas.append(self.actor.sim.getAlpha(0, 1))
            if "path_lens" in required_metrics:
                increment = np.linalg.norm(self.vel_agents, axis=1)
                increment[success] = 0.0
                path_lengths = path_lengths + np.linalg.norm(self.vel_agents, axis=1)*self.timestep
            if visualize or save_visualization or animate or animate_with_alphas:
                poses.append(self.pos_agents.copy())
                collision_anim.append(collision.copy())
            if np.sum(collision[mask]) + np.sum(success[mask]) == (self.n_agents - len(self.idx_non_cooperative)):
                finished = True
            elif frame == 1000:
                finished = True
            frame += 1
        metrics = {}
        filtered_time_agents_all = np.array(time_agents[mask])
        filtered_collision = np.array(collision[mask])
        filtered_success = np.array(success[mask])
        time_out = ~filtered_success & ~filtered_collision
        filtered_time_agents = filtered_time_agents_all[filtered_success]
        if "collision_rate" in required_metrics:
            metrics["collision_rate"] = filtered_collision.sum()/len(filtered_collision)
        if "time_out_rate" in required_metrics:
            metrics["time_out_rate"] = time_out.sum()/len(time_out)
        if "success_rate" in required_metrics:
            metrics["success_rate"] = filtered_success.sum()/len(filtered_success)
        if "sim_time" in required_metrics:
            if len(filtered_time_agents) > 0:
                metrics["sim_time"] = np.max(filtered_time_agents)
            else:
                metrics["sim_time"] = np.nan
        if "mean_agent_time" in required_metrics:
            if len(filtered_time_agents) > 0:
                metrics["mean_agent_time"] = np.mean(filtered_time_agents)
            else:
                metrics["mean_agent_time"] = np.nan
        if "times" in required_metrics:
            metrics["times"] = filtered_time_agents_all
        if "alphas" in required_metrics:
            metrics["alphas"] = np.array(alphas)
        if "path_lens" in required_metrics:
            metrics["path_lens"] = final_path_len[mask]
        if "roughness" in required_metrics:
            metrics["roughness"] = final_roughness[mask]
        if visualize or save_visualization:
            self.plot_simulation(np.array(poses), visualize, save_visualization, file_name, collision_anim)
        if animate:
            self.animate(np.array(poses), self.goal_agents, save_visualization, file_name, collision_anim)
        if animate_with_alphas:
            self.animate_with_alphas(np.array(poses), self.goal_agents, save_visualization, file_name, collision_anim, alphas)
        return metrics
    
    def plot_one_traj(self, poses, color, coll_vec=None):
        plt.plot(poses[:,0], poses[:,1], marker='.', color=color)
        for i in range(len(poses)):
            # circle = plt.Circle((poses[i,0], poses[i,1]), self.agent_radius, color=color, alpha=0.95**(len(poses)-i)*0.9)
            if coll_vec is not None:
                circle = plt.Circle((poses[i,0], poses[i,1]), self.agent_radius, color=color, alpha=((i + 1) / len(poses))*0.2, fill=(~coll_vec[i]))
            else:
                circle = plt.Circle((poses[i,0], poses[i,1]), self.agent_radius, color=color, alpha=((i + 1) / len(poses))*0.2)
            plt.gca().add_patch(circle)

    def plot_simulation(self, poses, visualize, save_visualization, file_name, collision_anim):
        matplotlib.rcParams['font.size'] = 21
        # plt.figure(figsize=(10/2, 7/2))
        plt.figure(figsize=(10, 10))
        plt.grid()
        n_non_cooperatives_filled = 0
        if self.n_agents == 2:
            for i in range(self.n_agents):
                if i in self.idx_non_cooperative:
                    self.plot_one_traj(poses[:, i, :], "black")
                    n_non_cooperatives_filled = n_non_cooperatives_filled+1
                else:
                    self.plot_one_traj(poses[:, i, :], cm.brg((1-i)/(self.n_agents)),
                                    np.array(collision_anim)[:, i])
        else:
            for i in range(self.n_agents):
                if i in self.idx_non_cooperative:
                    self.plot_one_traj(poses[:, i, :], "black")
                    n_non_cooperatives_filled = n_non_cooperatives_filled+1
                else:
                    self.plot_one_traj(poses[:, i, :], cm.rainbow((i-n_non_cooperatives_filled)/(self.n_agents - len(self.idx_non_cooperative))),
                                    np.array(collision_anim)[:, i])
        plt.axis('equal')
        # plt.axis([-1,1,-2,2])
        # plt.xlim((-1., 1.))
        # plt.ylim((-0.7, 0.7)) 
        plt.xlim((-3., 3.))
        plt.ylim((-3., 3.)) 
        plt.xlabel(r'$\mathrm{x (m)}$', fontweight ='bold')
        plt.ylabel(r'$\mathrm{y (m)}$', fontweight ='bold')
        if save_visualization:
            plt.savefig("images/" + file_name + ".png", format='png', bbox_inches='tight')
        if visualize:
            plt.show()
    
    def animate(self, poses, goals, save_visualization, file_name, collision_anim):
        matplotlib.rcParams['font.size'] = 32
        plt.close("all")
        fig, ax = plt.subplots(1,1, figsize=(9,9))
        plt.grid()
        ax.set_xlabel(r'$\mathrm{x (m)}$')
        ax.set_ylabel(r'$\mathrm{y (m)}$')
        ax.axis("equal")
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        # ax.set_xlim(-8, 8)
        # ax.set_ylim(-8, 8)
        agents_artist = []
        lines_artist = []
        past_artist = []
        # past_len = len(poses)
        past_len = 25
        
        n_non_cooperatives_filled = 0
        for agent_no in range(self.n_agents):
            past_one_agent = []
            if agent_no in self.idx_non_cooperative:
                agent = plt.Circle((poses[0][agent_no][0], poses[0][agent_no][1]), self.agent_radius, color="black", fill=True)
                ax.add_artist(plt.Circle(goals[agent_no], 0.05, color="black", fill=True))
                line, = plt.plot(poses[0, agent_no, 0], poses[0, agent_no, 1], marker='.', color="black")
                n_non_cooperatives_filled = n_non_cooperatives_filled+1
                for i in range(past_len):
                    # circle = plt.Circle((poses[i,0], poses[i,1]), self.agent_radius, color=color, alpha=0.95**(len(poses)-i)*0.9)
                    circle = plt.Circle((poses[0, agent_no, 0], poses[0, agent_no, 1]), self.agent_radius, color="black", alpha=((past_len-i) / past_len)*0.2)
                    past_one_agent.append(circle)
            else:
                color = cm.rainbow((agent_no-n_non_cooperatives_filled)/(self.n_agents - len(self.idx_non_cooperative)))
                agent = plt.Circle((poses[0][agent_no][0], poses[0][agent_no][1]), self.agent_radius, color=color, fill=True)
                ax.add_artist(plt.Circle(goals[agent_no], 0.05, color=color, fill=True))
                line, = plt.plot(poses[0, agent_no, 0], poses[0, agent_no, 1], marker='.', color=color)
                for i in range(past_len):
                    # circle = plt.Circle((poses[i,0], poses[i,1]), self.agent_radius, color=color, alpha=0.95**(len(poses)-i)*0.9)
                    circle = plt.Circle((poses[0, agent_no, 0], poses[0, agent_no, 1]), self.agent_radius, color=color, alpha=((past_len-i) / past_len)*0.2)
                    past_one_agent.append(circle)
            ax.add_artist(agent)
            agents_artist.append(agent)
            ax.add_artist(line)
            lines_artist.append(line)
            for c in past_one_agent:
                ax.add_artist(c)
            past_artist.append(past_one_agent)
        collision_text = plt.Text(0.5, 0.9, '', ha='center', va='center', transform=ax.transAxes, fontsize=20)
        ax.add_artist(collision_text)
        agents_artist.append(collision_text)
        self.finished = False
        # alpha_list = []
        time_text = plt.text(0., 3.5, '', ha='center', va='top', fontsize=32)
        # time_text = plt.text(0., 9.0, '', ha='center', va='top', fontsize=32)

        def update(frame):
            if not (frame >= len(poses)):
                time = frame*self.timestep
                # time_text.set_text(f"Time: {time:.2f}")
                time_text.set_text(f"Time: {time:.2f}")
                                
                for agent_i in range(self.n_agents):
                    if collision_anim[frame][agent_i] and agent_i not in self.idx_non_cooperative:
                        agents_artist[agent_i].fill = False
                    agents_artist[agent_i].center = (poses[frame][agent_i][0], poses[frame][agent_i][1])
                    for c in range(len(past_artist[agent_i])):
                        if frame - c < 0:
                            past_artist[agent_i][c].center = (poses[frame][agent_i][0], poses[frame][agent_i][1])
                        else:
                            if collision_anim[frame - c][agent_i] and agent_i not in self.idx_non_cooperative:
                                past_artist[agent_i][c].fill = False
                            past_artist[agent_i][c].center = (poses[frame - c][agent_i][0], poses[frame - c][agent_i][1])
                    lines_artist[agent_i].set_data(poses[:frame+1, agent_i, 0], poses[:frame+1, agent_i, 1])
                    

            return *agents_artist, time_text
        anim = animation.FuncAnimation(fig, update, frames=len(poses)+10, interval=self.timestep*1000, blit=False)
        anim.running = True
        if save_visualization:
            anim.save("videos/" + file_name + ".mp4", writer='ffmpeg')
        else:
            plt.show()
        # Fake return
        return False, 10000            
    
    def animate_with_alphas(self, poses, goals, save_visualization, file_name, collision_anim, alphas):
        matplotlib.rcParams['font.size'] = 22
        plt.close("all")
        fig, (ax, ax_alphas) = plt.subplots(2,1, figsize=(9,18))
        ax.grid()
        ax.set_xlabel(r'$\mathrm{x (m)}$')
        ax.set_ylabel(r'$\mathrm{y (m)}$')
        ax_alphas.set_xlabel(r'$\mathrm{t (s)}$')
        ax_alphas.set_ylabel(r'$\mathrm{o_i}$')
        ax_alphas.set_xlim(0, len(alphas)*0.1)
        ax_alphas.set_ylim(-0.7, 0.7)        
        ax.axis("equal")
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        agents_artist = []
        lines_artist = []
        past_artist = []
        # past_len = len(poses)
        past_len = 25
        
        n_non_cooperatives_filled = 0
        for agent_no in range(self.n_agents):
            past_one_agent = []
            if agent_no in self.idx_non_cooperative:
                agent = plt.Circle((poses[0][agent_no][0], poses[0][agent_no][1]), self.agent_radius, color="black", fill=True)
                ax.add_artist(plt.Circle(goals[agent_no], 0.05, color="black", fill=True))
                line, = ax.plot(poses[0, agent_no, 0], poses[0, agent_no, 1], marker='.', color="black")
                n_non_cooperatives_filled = n_non_cooperatives_filled+1
                for i in range(past_len):
                    # circle = plt.Circle((poses[i,0], poses[i,1]), self.agent_radius, color=color, alpha=0.95**(len(poses)-i)*0.9)
                    circle = plt.Circle((poses[0, agent_no, 0], poses[0, agent_no, 1]), self.agent_radius, color="black", alpha=((past_len-i) / past_len)*0.2)
                    past_one_agent.append(circle)
            else:
                color = cm.rainbow((agent_no-n_non_cooperatives_filled)/(self.n_agents - len(self.idx_non_cooperative)))
                if agent_no == 1:
                    color = "black"
                agent = plt.Circle((poses[0][agent_no][0], poses[0][agent_no][1]), self.agent_radius, color=color, fill=True)
                ax.add_artist(plt.Circle(goals[agent_no], 0.05, color=color, fill=True))
                line, = ax.plot(poses[0, agent_no, 0], poses[0, agent_no, 1], marker='.', color=color)
                for i in range(past_len):
                    # circle = plt.Circle((poses[i,0], poses[i,1]), self.agent_radius, color=color, alpha=0.95**(len(poses)-i)*0.9)
                    circle = plt.Circle((poses[0, agent_no, 0], poses[0, agent_no, 1]), self.agent_radius, color=color, alpha=((past_len-i) / past_len)*0.2)
                    past_one_agent.append(circle)
            ax.add_artist(agent)
            agents_artist.append(agent)
            ax.add_artist(line)
            lines_artist.append(line)
            for c in past_one_agent:
                ax.add_artist(c)
            past_artist.append(past_one_agent)
        collision_text = plt.Text(0.5, 0.9, '', ha='center', va='center', transform=ax.transAxes, fontsize=20)
        ax.add_artist(collision_text)
        agents_artist.append(collision_text)
        self.finished = False
        # alpha_list = []
        # time_text = plt.text(0., 3.5, '', ha='center', va='top', fontsize=14)
        # time_text = ax.text(0., 4.5, '', ha='center', va='top', fontsize=32)

        def update(frame):
            if not (frame >= len(poses)):
                time = frame*self.timestep
                ax_alphas.plot(np.arange(frame)*0.1, alphas[:frame], "b")
                # time_text.set_text(f"Time: {time:.2f}")
                # time_text.set_text(f"Time: {time:.2f}")
                                
                for agent_i in range(self.n_agents):
                    if collision_anim[frame][agent_i] and agent_i not in self.idx_non_cooperative:
                        agents_artist[agent_i].fill = False
                    agents_artist[agent_i].center = (poses[frame][agent_i][0], poses[frame][agent_i][1])
                    for c in range(len(past_artist[agent_i])):
                        if frame - c < 0:
                            past_artist[agent_i][c].center = (poses[frame][agent_i][0], poses[frame][agent_i][1])
                        else:
                            if collision_anim[frame - c][agent_i] and agent_i not in self.idx_non_cooperative:
                                past_artist[agent_i][c].fill = False
                            past_artist[agent_i][c].center = (poses[frame - c][agent_i][0], poses[frame - c][agent_i][1])
                    lines_artist[agent_i].set_data(poses[:frame+1, agent_i, 0], poses[:frame+1, agent_i, 1])
                    

            return *agents_artist, #time_text
        anim = animation.FuncAnimation(fig, update, frames=len(poses)+10, interval=self.timestep*1000, blit=False)
        anim.running = True
        if save_visualization:
            anim.save("videos/" + file_name + ".mp4", writer='ffmpeg')
        else:
            plt.show()
        # Fake return
        return False, 10000          

class CircleSimulator(Simulator):
    def __init__(self, n_agents, circle_radius, idx_non_cooperative, actor: MultiActor, timestep=0.1, agent_radius=0.2, orca_vel=0.75, agent_vel=1.0, seed=None) -> None:
        if seed is not None:
            np.random.seed(seed)
        pos_agents = []
        goal_agents = []
        for ag in range(int(n_agents)):
            # pos_agents.append((circle_radius*np.cos(2*np.pi*(ag+1)/n_agents) + random.random()/2 - 0.25, circle_radius*np.sin(2*np.pi*(ag+1)/n_agents) + random.random()/2 - 0.25))
            pos_agents.append((circle_radius*np.cos(2*np.pi*(ag+1)/n_agents), circle_radius*np.sin(2*np.pi*(ag+1)/n_agents)))
            goal_agents.append((circle_radius*np.cos(2*np.pi*(ag+1)/n_agents + np.pi), circle_radius*np.sin(2*np.pi*(ag+1)/n_agents + np.pi)))
        
        super().__init__(pos_agents, goal_agents, idx_non_cooperative, timestep, agent_radius, actor, orca_vel, agent_vel)

class SquareSimulator(Simulator):
    def __init__(self, n_cooperative, n_non_cooperative, square_width, actor: MultiActor, timestep=0.1, agent_radius=0.2, seed=None, orca_vel=0.75, agent_vel=1.0) -> None:
        if seed is not None:
            np.random.seed(seed)
        pos_agents = []
        goal_agents = []
        for _ in range(n_cooperative):
            while True:
                if np.random.random() > 0.5:
                    sign = -1
                else:
                    sign = 1
                px = 1.5*(np.random.random()-0.5) * square_width 
                py = sign*square_width
                collide = False
                for agent in pos_agents:
                    if np.linalg.norm((px - agent[0], py - agent[1])) < agent_radius*2.5:
                        collide = True
                        break
                if not collide:
                    break
            while True:
                gx = 1.5*(np.random.random()-0.5) * square_width 
                gy = -sign*square_width
                collide = False
                for agent in goal_agents:
                    if np.linalg.norm((gx - agent[0], gy - agent[1])) < agent_radius*2.5:
                        collide = True
                        break
                if not collide:
                    break
            pos_agents.append([px, py])
            goal_agents.append([gx, gy])
        for _ in range(n_non_cooperative):
            while True:
                if np.random.random() > 0.5:
                    sign = -1
                else:
                    sign = 1
                px = (np.random.random()+0.5) * square_width * sign
                py = (np.random.random()-0.5) * square_width
                collide = False
                for agent in pos_agents:
                    if np.linalg.norm((px - agent[0], py - agent[1])) < agent_radius*2.5:
                        collide = True
                        break
                if not collide:
                    break
            while True:
                gx = (np.random.random()+0.5) * square_width * (- sign)
                gy = (np.random.random()-0.5) * square_width
                collide = False
                for agent in goal_agents:
                    if np.linalg.norm((gx - agent[0], gy - agent[1])) < agent_radius*2.5:
                        collide = True
                        break
                if not collide:
                    break
            pos_agents.append([px, py])
            goal_agents.append([gx, gy])
        super().__init__(np.array(pos_agents), np.array(goal_agents), np.arange(n_cooperative, n_cooperative+n_non_cooperative), timestep, agent_radius, actor, orca_vel, agent_vel)


class StaticObsSimulator(Simulator):
    # List of the vertices of the polygonal obstacle are in counterclockwise order. To add a "negative" obstacle, 
    # e.g. a bounding polygon around the environment, the vertices should be listed in clockwise order.
    # Currently, we only support polygonal obstalce visualization.
    def __init__(self, obstacles_static, pos_agents, goal_agents, idx_non_cooperative, actor: MultiActor, timestep=0.1, agent_radius=0.2, orca_vel=0.75, agent_vel=1.0) -> None:
        super().__init__(pos_agents, goal_agents, idx_non_cooperative, timestep, agent_radius, actor, orca_vel, agent_vel)
        for o in obstacles_static:
            self.sim_non_cooperative.addObstacle(o)
        self.sim_non_cooperative.processObstacles()
        self.obstacles_static = obstacles_static
    
    def plot_simulation(self, poses, visualize, save_visualization, file_name, collision_anim):
        # matplotlib.rcParams['font.size'] = 21
        # plt.figure(figsize=(10/2, 7/2))
        plt.figure(figsize=(10, 10))
        plt.grid()
        n_non_cooperatives_filled = 0
        if self.n_agents == 2:
            for i in range(self.n_agents):
                if i in self.idx_non_cooperative:
                    self.plot_one_traj(poses[:, i, :], "black")
                    n_non_cooperatives_filled = n_non_cooperatives_filled+1
                else:
                    self.plot_one_traj(poses[:, i, :], cm.brg((1-i)/(self.n_agents)),
                                    np.array(collision_anim)[:, i])
        else:
            for i in range(self.n_agents):
                if i in self.idx_non_cooperative:
                    self.plot_one_traj(poses[:, i, :], "black")
                    n_non_cooperatives_filled = n_non_cooperatives_filled+1
                else:
                    self.plot_one_traj(poses[:, i, :], cm.rainbow((i-n_non_cooperatives_filled)/(self.n_agents - len(self.idx_non_cooperative))),
                                    np.array(collision_anim)[:, i])
        for o in self.obstacles_static:
            polygon = Polygon(o, closed=True, color='black')
            plt.gca().add_patch(polygon)
        plt.axis('equal')
        # plt.axis([-1,1,-2,2])
        # plt.xlim((-1., 1.))
        # plt.ylim((-0.7, 0.7)) 
        plt.xlim((-3., 3.))
        plt.ylim((-3., 3.)) 
        plt.xlabel(r'$\mathrm{x (m)}$', fontweight ='bold')
        plt.ylabel(r'$\mathrm{y (m)}$', fontweight ='bold')
        if save_visualization:
            plt.savefig("images/" + file_name + ".png", format='png', bbox_inches='tight')
        if visualize:
            plt.show()
    
    def animate(self, poses, goals, save_visualization, file_name, collision_anim):
        matplotlib.rcParams['font.size'] = 32
        plt.close("all")
        fig, ax = plt.subplots(1,1, figsize=(9,9))
        plt.grid()
        ax.set_xlabel(r'$\mathrm{x (m)}$')
        ax.set_ylabel(r'$\mathrm{y (m)}$')
        ax.axis("equal")
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        # ax.set_xlim(-8, 8)
        # ax.set_ylim(-8, 8)
        agents_artist = []
        lines_artist = []
        past_artist = []
        # past_len = len(poses)
        past_len = 25
        
        n_non_cooperatives_filled = 0
        for agent_no in range(self.n_agents):
            past_one_agent = []
            if agent_no in self.idx_non_cooperative:
                agent = plt.Circle((poses[0][agent_no][0], poses[0][agent_no][1]), self.agent_radius, color="black", fill=True)
                ax.add_artist(plt.Circle(goals[agent_no], 0.05, color="black", fill=True))
                line, = plt.plot(poses[0, agent_no, 0], poses[0, agent_no, 1], marker='.', color="black")
                n_non_cooperatives_filled = n_non_cooperatives_filled+1
                for i in range(past_len):
                    # circle = plt.Circle((poses[i,0], poses[i,1]), self.agent_radius, color=color, alpha=0.95**(len(poses)-i)*0.9)
                    circle = plt.Circle((poses[0, agent_no, 0], poses[0, agent_no, 1]), self.agent_radius, color="black", alpha=((past_len-i) / past_len)*0.2)
                    past_one_agent.append(circle)
            else:
                color = cm.rainbow((agent_no-n_non_cooperatives_filled)/(self.n_agents - len(self.idx_non_cooperative)))
                agent = plt.Circle((poses[0][agent_no][0], poses[0][agent_no][1]), self.agent_radius, color=color, fill=True)
                ax.add_artist(plt.Circle(goals[agent_no], 0.05, color=color, fill=True))
                line, = plt.plot(poses[0, agent_no, 0], poses[0, agent_no, 1], marker='.', color=color)
                for i in range(past_len):
                    # circle = plt.Circle((poses[i,0], poses[i,1]), self.agent_radius, color=color, alpha=0.95**(len(poses)-i)*0.9)
                    circle = plt.Circle((poses[0, agent_no, 0], poses[0, agent_no, 1]), self.agent_radius, color=color, alpha=((past_len-i) / past_len)*0.2)
                    past_one_agent.append(circle)
            ax.add_artist(agent)
            agents_artist.append(agent)
            ax.add_artist(line)
            lines_artist.append(line)
            for c in past_one_agent:
                ax.add_artist(c)
            past_artist.append(past_one_agent)
        collision_text = plt.Text(0.5, 0.9, '', ha='center', va='center', transform=ax.transAxes, fontsize=20)
        ax.add_artist(collision_text)
        agents_artist.append(collision_text)
        self.finished = False
        # alpha_list = []
        time_text = plt.text(0., 3.5, '', ha='center', va='top', fontsize=32)
        # time_text = plt.text(0., 9.0, '', ha='center', va='top', fontsize=32)
        for o in self.obstacles_static:
            polygon = Polygon(o, closed=True, color='black')
            plt.gca().add_patch(polygon)

        def update(frame):
            if not (frame >= len(poses)):
                time = frame*self.timestep
                # time_text.set_text(f"Time: {time:.2f}")
                time_text.set_text(f"Time: {time:.2f}")
                                
                for agent_i in range(self.n_agents):
                    if collision_anim[frame][agent_i] and agent_i not in self.idx_non_cooperative:
                        agents_artist[agent_i].fill = False
                    agents_artist[agent_i].center = (poses[frame][agent_i][0], poses[frame][agent_i][1])
                    for c in range(len(past_artist[agent_i])):
                        if frame - c < 0:
                            past_artist[agent_i][c].center = (poses[frame][agent_i][0], poses[frame][agent_i][1])
                        else:
                            if collision_anim[frame - c][agent_i] and agent_i not in self.idx_non_cooperative:
                                past_artist[agent_i][c].fill = False
                            past_artist[agent_i][c].center = (poses[frame - c][agent_i][0], poses[frame - c][agent_i][1])
                    lines_artist[agent_i].set_data(poses[:frame+1, agent_i, 0], poses[:frame+1, agent_i, 1])
                    

            return *agents_artist, time_text
        anim = animation.FuncAnimation(fig, update, frames=len(poses)+10, interval=self.timestep*1000, blit=False)
        anim.running = True
        if save_visualization:
            anim.save("videos/" + file_name + ".mp4", writer='ffmpeg')
        else:
            plt.show()
        # Fake return
        return False, 10000            

class StaticObsCircleSimulator(StaticObsSimulator):
    def __init__(self, static_obs, n_agents, circle_radius, idx_non_cooperative, actor: MultiActor, timestep=0.1, agent_radius=0.2, orca_vel=0.75, agent_vel=1.0, seed = None) -> None:
        if seed is not None:
            np.random.seed(seed)
        pos_agents = []
        goal_agents = []
        for ag in range(int(n_agents)):
            # pos_agents.append((circle_radius*np.cos(2*np.pi*(ag+1)/n_agents) + random.random()/2 - 0.25, circle_radius*np.sin(2*np.pi*(ag+1)/n_agents) + random.random()/2 - 0.25))
            pos_agents.append((circle_radius*np.cos(2*np.pi*(ag+1)/n_agents), circle_radius*np.sin(2*np.pi*(ag+1)/n_agents)))
            goal_agents.append((circle_radius*np.cos(2*np.pi*(ag+1)/n_agents + np.pi), circle_radius*np.sin(2*np.pi*(ag+1)/n_agents + np.pi)))
        
        super().__init__(static_obs, pos_agents, goal_agents, idx_non_cooperative, actor, timestep, agent_radius, orca_vel, agent_vel)
