#pragma once
#include <fstream>
#include <chrono>
#include "algorithm/optim_scp_swarm.hpp"

class Simulator{
    public:
        bool success;
        Simulator(int cf_num, bool read_cf, int num_drones, bool use_model, std::vector<float> noise);
        void runSimulation();
        void saveMetrics();
    private:
        std :: ofstream save_data, save_data_2;

        std :: string path;
        YAML :: Node params;

        int config_num;
        int VERBOSE;
        int num;
        int num_drone;
        int sim_iter;
        bool read_config; 

        bool free_space, out_space;

        float a_drone;
        float b_drone;
        float c_drone;
        
        float max_time;
        float t_plan;
        float dist_stop;
        float dt;

        float mission_time;
        bool collision_agent, collision_obstacle;
        std :: chrono :: duration<double, std::milli> total_time;

        probData *prob_data;

        Eigen :: ArrayXXf agents_x, 
                        agents_y, 
                        agents_z;

        Eigen :: ArrayXf smoothness,
                        arc_length,
                        dist_to_goal;

        
        std :: vector<std :: vector<float>> _init_drone;
        std :: vector<std :: vector<float>> _goal_drone;

        std :: vector<std :: vector<float>> _pos_static_obs;
        std :: vector<std :: vector<float>> _dim_static_obs;

        std :: vector <float> smoothness_agent, traj_length_agent, comp_time_agent, inter_agent_dist, agent_obs_dist;
        int num_obs, num_obs_2;    

        void shareInformation();
        void runAlgorithm();
        void checkCollision();
        void checkAtGoal();
        void checkViolation();
        void calculateDistances();
};