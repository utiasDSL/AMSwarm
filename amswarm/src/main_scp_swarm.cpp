#define _USE_MATH_DEFINES
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>

#include "simulator/sim_scp_swarm.hpp"

int main()
{
    std :: string path = ros :: package::getPath("amswarm");
    YAML :: Node params = YAML :: LoadFile(path+"/params/config_sim_swarm.yaml");

    int start_config = params["start_config"].as<int>(); 
    int end_config = params["end_config"].as<int>(); 
    bool read_config = params["read_config"].as<bool>();
    bool use_model = params["use_model"].as<bool>();
    std :: vector<float> noise = params["noise"].as<std::vector<float>>();
    std :: vector<float> num_drones = params["num_drones"].as<std::vector<float>>();

    for(int j = 0; j < num_drones.size(); j++){
        if(read_config)
            ROS_INFO_STREAM("Agent size = " << num_drones[j] << " Configuration numbers = " << end_config - start_config + 1);
        int success_trials = 0;
        for(int i = start_config; i < end_config; i++){
            Simulator sim = Simulator(i, read_config, num_drones[j], use_model, noise);
            sim.runSimulation();
            sim.saveMetrics();

            if(sim.success)
                success_trials += 1;
            if(!read_config)
                break;
            // ROS_INFO_STREAM("Success Trials = " << success_trials << " out of " << i+1);
        }
        ROS_INFO_STREAM("Success Trials = " << success_trials << " out of " << end_config);
        if(!read_config)
            break;
    }
    return 0;
}