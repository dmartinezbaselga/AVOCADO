#
# src/CMakeLists.txt
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

set(RVO_HEADERS
	RVO.h
	RVOSimulator.h
	Vector2.h)

set(RVO_SOURCES
	Agent.cpp
	Agent.h
	Definitions.h
	KdTree.cpp
	KdTree.h
	Obstacle.cpp
	Obstacle.h
	RVOSimulator.cpp)

add_library(AVOCADO ${RVO_HEADERS} ${RVO_SOURCES})

if(WIN32)
    set_target_properties(AVOCADO PROPERTIES COMPILE_DEFINITIONS NOMINMAX)
endif()

install(FILES ${RVO_HEADERS} DESTINATION include)
install(TARGETS AVOCADO DESTINATION lib)
