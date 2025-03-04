/*
 * Obstacle.h
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

#ifndef RVO_OBSTACLE_H_
#define RVO_OBSTACLE_H_

/**
 * \file       Obstacle.h
 * \brief      Contains the Obstacle class.
 */

#include "Definitions.h"

namespace RVO {
	/**
	 * \brief      Defines static obstacles in the simulation.
	 */
	class Obstacle {
	private:
		/**
		 * \brief      Constructs a static obstacle instance.
		 */
		Obstacle();

		bool isConvex_;
		Obstacle *nextObstacle_;
		Vector2 point_;
		Obstacle *prevObstacle_;
		Vector2 unitDir_;

		size_t id_;

		friend class Agent;
		friend class KdTree;
		friend class RVOSimulator;
	};
}

#endif /* RVO_OBSTACLE_H_ */
