/*
 * Agent.h
 * AVOCADO library
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPDX-FileCopyrightText: 2024 University of Zaragoza
 * SPDX-License-Identifier: AGPL-3.0-or-later
 *
 * This file is part of AVOCADO, a derivative work of the RVO2 Library.
 * Portions of this file are licensed under the Apache License, Version 2.0,
 * and modifications are licensed under the GNU Affero General Public License,
 * version 3 or later.
 *
 * If you use AVOCADO in academic work, please cite:
 * Martinez-Baselga, D., Sebastián, E., Montijano, E., Riazuelo, L., Sagüés, C., & Montano, L. (2024). AVOCADO: Adaptive Optimal Collision Avoidance driven by Opinion. arXiv preprint arXiv:2407.00507.
 * 
 * For details, see the LICENSE file at the root of the repository.
 * 
 * Contact: diegomartinez@unizar.es
 * 			esebastian@unizar.es
 * 
 */

#ifndef RVO_AGENT_H_
#define RVO_AGENT_H_

/**
 * \file       Agent.h
 * \brief      Contains the Agent class.
 */

#include "Definitions.h"
#include "RVOSimulator.h"
#include <unordered_map>
#include <random>

namespace RVO {
	/**
	 * \brief      Defines an agent in the simulation.
	 */
	class Agent {
	private:
		/**
		 * \brief      Constructs an agent instance.
		 * \param      sim             The simulator instance.
		 */
		explicit Agent(RVOSimulator *sim, float max_noise=0.0);

		/**
		 * \brief      Computes the neighbors of this agent.
		 */
		void computeNeighbors();

		/**
		 * \brief      Computes the alphas of this agent.
		 */
		void computeAlphas();

		/**
		 * \brief      Computes the new velocity of this agent.
		 */
		void computeNewVelocity();

		/**
		 * \brief      Inserts an agent neighbor into the set of neighbors of
		 *             this agent.
		 * \param      agent           A pointer to the agent to be inserted.
		 * \param      rangeSq         The squared range around this agent.
		 */
		void insertAgentNeighbor(const Agent *agent, float &rangeSq);

		/**
		 * \brief      Inserts a static obstacle neighbor into the set of neighbors
		 *             of this agent.
		 * \param      obstacle        The number of the static obstacle to be
		 *                             inserted.
		 * \param      rangeSq         The squared range around this agent.
		 */
		void insertObstacleNeighbor(const Obstacle *obstacle, float rangeSq);

		/**
		 * \brief      Updates the two-dimensional position and two-dimensional
		 *             velocity of this agent.
		 */
		void update();

		struct AlphaU
		{
			float alpha, attention;
			Vector2 u;
		};
		

		std::vector<std::pair<float, const Agent *> > agentNeighbors_;
		std::unordered_map<size_t, AlphaU> alphas_;
		size_t maxNeighbors_;
		float maxSpeed_;
		float neighborDist_;
		Vector2 newVelocity_;
		std::vector<std::pair<float, const Obstacle *> > obstacleNeighbors_;
		std::vector<Line> orcaLines_;
		Vector2 position_;
		Vector2 prefVelocity_;
		float radius_;
		RVOSimulator *sim_;
		float timeHorizon_;
		float timeHorizonObst_;
		Vector2 velocity_;
		Vector2 prev_velocity_;
		float const_alpha_;
		float d_, a_, b_, c_;
		float kappa_, epsilon_, delta_;
		std::default_random_engine generator;
  		std::uniform_real_distribution<double> distribution;
		size_t id_;

		friend class KdTree;
		friend class RVOSimulator;
	};

	/**
	 * \relates    Agent
	 * \brief      Solves a one-dimensional linear program on a specified line
	 *             subject to linear constraints defined by lines and a circular
	 *             constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      lineNo        The specified line constraint.
	 * \param      radius        The radius of the circular constraint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     True if successful.
	 */
	bool linearProgram1(const std::vector<Line> &lines, size_t lineNo,
						float radius, const Vector2 &optVelocity,
						bool directionOpt, Vector2 &result);

	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             constraints defined by lines and a circular constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      radius        The radius of the circular constraint.
	 * \param      optVelocity   The optimization velocity.
	 * \param      directionOpt  True if the direction should be optimized.
	 * \param      result        A reference to the result of the linear program.
	 * \return     The number of the line it fails on, and the number of lines if successful.
	 */
	size_t linearProgram2(const std::vector<Line> &lines, float radius,
						  const Vector2 &optVelocity, bool directionOpt,
						  Vector2 &result);

	/**
	 * \relates    Agent
	 * \brief      Solves a two-dimensional linear program subject to linear
	 *             constraints defined by lines and a circular constraint.
	 * \param      lines         Lines defining the linear constraints.
	 * \param      numObstLines  Count of obstacle lines.
	 * \param      beginLine     The line on which the 2-d linear program failed.
	 * \param      radius        The radius of the circular constraint.
	 * \param      result        A reference to the result of the linear program.
	 */
	void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, size_t beginLine,
						float radius, Vector2 &result);
}

#endif /* RVO_AGENT_H_ */
