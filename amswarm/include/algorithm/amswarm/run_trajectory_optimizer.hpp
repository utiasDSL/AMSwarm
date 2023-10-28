#pragma once
#include "algorithm/amswarm/trajectory_utils.hpp"
#include "algorithm/amswarm/obstacles_utils.hpp" 
#include "algorithm/amswarm/solve_polar_var.hpp"
#include "algorithm/amswarm/solve_position_var.hpp"

void checkResiduals(probData &prob_data, int VERBOSE);
void initializeOptimizer(probData &prob_data, int VERBOSE);
void deployAgent(probData &prob_data, int VERBOSE);