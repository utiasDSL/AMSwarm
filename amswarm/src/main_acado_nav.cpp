#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <vector>
#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <ros/package.h>
#include <ros/ros.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */


/* Global variables used by the solver. */
extern "C"{
__thread ACADOvariables acadoVariables;
__thread ACADOworkspace acadoWorkspace;
}

struct probData{
	float x_init, y_init, z_init;
    float vx_init, vy_init, vz_init;
    float ax_init, ay_init, az_init;
    float gamma, dist_to_goal;

    float a_drone, b_drone, c_drone, buffer, slack_norm;
    float x_goal, y_goal, z_goal;
    int max_iter, mpc_step, id_badge, num_drone, max_drone, state;

    bool run;
    Eigen :: ArrayXXf agents_x, agents_y, agents_z; 

    std :: vector<float> smoothness, arc_length, inter_agent_dist, inter_agent_dist_min;

    YAML :: Node params;
};

void distToAgents(probData &prob_data){
    prob_data.inter_agent_dist.clear();

    Eigen :: ArrayXXf agents_x, agents_y, agents_z;

    agents_x = prob_data.agents_x;
    agents_y = prob_data.agents_y;
    agents_z = prob_data.agents_z;

    Eigen :: ArrayXf dist = (sqrt(pow((agents_x(prob_data.id_badge, 0) - agents_x.col(0)),2) 
                                + pow((agents_y(prob_data.id_badge, 0) - agents_y.col(0)),2)
                                + pow((agents_z(prob_data.id_badge, 0) - agents_z.col(0)),2))).max(0.0);

    
    for(int i = 0; i < agents_x.rows(); i++)
    {
        if(i == prob_data.id_badge)
            continue;
        prob_data.inter_agent_dist.push_back(dist(i));
    }
}

void initOtherAgents(probData &prob_data){
    int k = 0;
    for (int i = 0; i < (N + 1); ++i){
        //xo
        k = 0;
        for(int j = 0; j < (prob_data.max_drone-1); j++){
            if(k == prob_data.id_badge)
                k++;
            acadoVariables.od[i * NOD + j + (prob_data.max_drone-1)*0] = (1 + abs(prob_data.num_drone - 0.00001 - j)/(prob_data.num_drone - 0.00001 - j))/2 * prob_data.agents_x(k, i);
            k++;
        }
        //yo
        k = 0;
        for(int j = 0; j < (prob_data.max_drone-1); j++){
            if(k == prob_data.id_badge)
                k++;
            acadoVariables.od[i * NOD + j + (prob_data.max_drone-1)*1] = (1 + abs(prob_data.num_drone - 0.00001 - j)/(prob_data.num_drone - 0.00001 - j))/2 * prob_data.agents_y(k, i);
            k++;
        }

        //zo
        k = 0;
        for(int j = 0; j < (prob_data.max_drone-1); j++){
            if(k == prob_data.id_badge)
                k++;
            acadoVariables.od[i * NOD + j + (prob_data.max_drone-1)*2] = (1 + abs(prob_data.num_drone - 0.00001 - j)/(prob_data.num_drone - 0.00001 - j))/2 * prob_data.agents_z(k, i);
            k++;
        }
        
        //ao
        k = 0;
        for(int j = 0; j < (prob_data.max_drone-1); j++){
            acadoVariables.od[i * NOD + j + (prob_data.max_drone-1)*3] = (2 * prob_data.a_drone + prob_data.buffer);
        }
    
        //bo
        for(int j = 0; j < (prob_data.max_drone-1); j++){
            acadoVariables.od[i * NOD + j + (prob_data.max_drone-1)*4] = (2 * prob_data.b_drone + prob_data.buffer);
        }

        //co
        k = 0;
        for(int j = 0; j < (prob_data.max_drone-1); j++){
            acadoVariables.od[i * NOD + j + (prob_data.max_drone-1)*5] = (2 * prob_data.c_drone + prob_data.buffer);
        }

        acadoVariables.od[i * NOD + (prob_data.max_drone-1)*6] = prob_data.gamma;
    }
}
int deployAgent(probData &prob_data){
    
    YAML :: Node params = prob_data.params;

    prob_data.vx_init = 0.0;
    prob_data.vy_init = 0.0;
    prob_data.vz_init = 0.0;

    prob_data.ax_init = 0.0;
    prob_data.ay_init = 0.0;
    prob_data.az_init = 0.0;

    prob_data.dist_to_goal = 10000.0;
    
    prob_data.max_iter = params["max_iter"].as<int>();

    prob_data.a_drone = params["a_drone"].as<float>();
    prob_data.b_drone = params["b_drone"].as<float>();
    prob_data.c_drone = params["c_drone"].as<float>();
    prob_data.buffer = params["buffer"].as<float>();

    prob_data.gamma = params["gamma"].as<float>();
    prob_data.run = true;

    acado_timer t;    
    acado_initializeSolver();
    
    for (int i = 0; i < (N + 1); ++i){  
        acadoVariables.x[ i*NX + 0] = prob_data.x_init;			// x
        acadoVariables.x[ i*NX + 1] = prob_data.y_init;			// y
        acadoVariables.x[ i*NX + 2] = prob_data.z_init;			// y
        acadoVariables.x[ i*NX + 3] = prob_data.vx_init;			// vx
        acadoVariables.x[ i*NX + 4] = prob_data.vy_init;			// vy
        acadoVariables.x[ i*NX + 5] = prob_data.vz_init;			// vz
        acadoVariables.x[ i*NX + 6] = prob_data.ax_init;			// vx
        acadoVariables.x[ i*NX + 7] = prob_data.ay_init;			// vy
        acadoVariables.x[ i*NX + 8] = prob_data.az_init;			// vz
        acadoVariables.x[ i*NX + 9] = 0.0;
    }
    for (int i = 0; i < N; ++i){
        for(int j = 0; j < NU; j++){
            acadoVariables.u[ i*NU + j ] = 0.0;                       
        }
    }

    for (int i = 0; i < N; ++i) {
        for(int j = 0; j < NY; j++){
            acadoVariables.y[ i*NY + j ] = 0.0;                                   
        }
    }
    acadoVariables.yN[ 0 ] = prob_data.x_goal;                    
    acadoVariables.yN[ 1 ] = prob_data.y_goal;                    
    acadoVariables.yN[ 2 ] = prob_data.z_goal;                    

    
    for (int i = 0; i < N ; i++){
        acadoVariables.W[NY*NY*i + (NY+1)*0] = prob_data.params["weight_smoothness"].as<float>();
        acadoVariables.W[NY*NY*i + (NY+1)*1] = prob_data.params["weight_smoothness"].as<float>();
        acadoVariables.W[NY*NY*i + (NY+1)*2] = prob_data.params["weight_smoothness"].as<float>();
        for(int j = 3; j < NY; j++){	    
                acadoVariables.W[NY*NY*i + (NY+1)*j] = prob_data.params["weight_quad_slack"].as<float>();	    
        }   
    }
    acadoVariables.WN[(NYN+1)*0] = prob_data.params["weight_goal"].as<float>();		        
    acadoVariables.WN[(NYN+1)*1] = prob_data.params["weight_goal"].as<float>();	            
    acadoVariables.WN[(NYN+1)*2] = prob_data.params["weight_goal"].as<float>();                
    
    
    
    while(prob_data.run){
        if(prob_data.state == 1){
            
            initOtherAgents(prob_data);
            distToAgents(prob_data);
            
            prob_data.x_init = acadoVariables.x[ NX + 0];
            prob_data.y_init = acadoVariables.x[ NX + 1];
            prob_data.z_init = acadoVariables.x[ NX + 2];
            prob_data.vx_init = acadoVariables.x[ NX + 3];
            prob_data.vy_init = acadoVariables.x[ NX + 4];
            prob_data.vz_init = acadoVariables.x[ NX + 5];
            prob_data.ax_init = acadoVariables.x[ NX + 6];
            prob_data.ay_init = acadoVariables.x[ NX + 7];
            prob_data.az_init = acadoVariables.x[ NX + 8];

            prob_data.dist_to_goal = sqrt(pow(prob_data.x_init - prob_data.x_goal, 2) + pow(prob_data.y_init - prob_data.y_goal, 2) + pow(prob_data.z_init - prob_data.z_goal, 2));
            prob_data.smoothness.push_back(sqrt(pow(prob_data.ax_init, 2) + pow(prob_data.ay_init, 2) + pow(prob_data.az_init, 2)));
            prob_data.arc_length.push_back(sqrt(pow(prob_data.x_init - acadoVariables.x0[ 0 ], 2) + pow(prob_data.y_init - acadoVariables.x0[ 1 ], 2) + pow(prob_data.z_init - acadoVariables.x0[ 2 ], 2)));

            acadoVariables.x0[ 0 ] = prob_data.x_init;
            acadoVariables.x0[ 1 ] = prob_data.y_init;
            acadoVariables.x0[ 2 ] = prob_data.z_init;
            acadoVariables.x0[ 3 ] = prob_data.vx_init;
            acadoVariables.x0[ 4 ] = prob_data.vy_init;
            acadoVariables.x0[ 5 ] = prob_data.vz_init;
            acadoVariables.x0[ 6 ] = prob_data.ax_init;
            acadoVariables.x0[ 7 ] = prob_data.ay_init;
            acadoVariables.x0[ 8 ] = prob_data.az_init;
            acadoVariables.x0[ 9 ] = acadoVariables.x[ NX + 9];
            
            
            if(prob_data.inter_agent_dist.size() != 0)
                prob_data.inter_agent_dist_min.push_back(*std::min_element(prob_data.inter_agent_dist.begin(), prob_data.inter_agent_dist.end()));
                
            for(int iter = 0; iter < prob_data.max_iter; ++iter)
            {
                acado_preparationStep();
                acado_feedbackStep( );
            }
            
            float slack_temp = 0.0;
            for(int i = 0; i < N; i++){
                for(int j = 0; j < prob_data.num_drone; j++){
                    slack_temp += acadoVariables.u[i*NU + j + 3]*acadoVariables.u[i*NU + j + 3];
                }
            }
            prob_data.slack_norm = sqrt(slack_temp);
            
            prob_data.state = 2;
            prob_data.max_iter = 1;
            prob_data.mpc_step++;
                        
        }
        else if(prob_data.state == 2){
            
            for(int i = 1; i < N+1; i++){
                prob_data.agents_x(prob_data.id_badge, i-1) = acadoVariables.x[i*NX + 0];
                prob_data.agents_y(prob_data.id_badge, i-1) = acadoVariables.x[i*NX + 1];
                prob_data.agents_z(prob_data.id_badge, i-1) = acadoVariables.x[i*NX + 2];
            }
            prob_data.agents_x(prob_data.id_badge, N) =  acadoVariables.x[N*NX + 0] - acadoVariables.x[(N-1)*NX + 0];
            prob_data.agents_y(prob_data.id_badge, N) =  acadoVariables.x[N*NX + 1] - acadoVariables.x[(N-1)*NX + 1];
            prob_data.agents_z(prob_data.id_badge, N) =  acadoVariables.x[N*NX + 2] - acadoVariables.x[(N-1)*NX + 2];
            prob_data.state = 3;
        }
        else{
            sleep(0.0001);
        }
    }
}

int main( )
{
    std :: string path = ros :: package::getPath("amswarm");
    std :: ofstream save_data;

    
    YAML :: Node params = YAML :: LoadFile(path+"/params/config_acado_swarm.yaml");

    int max_drone = NU - 2;//params["num_drone"].as<int>();

    Eigen :: ArrayXXf agents_x = Eigen :: ArrayXXf :: Zero(max_drone, N+1);
    Eigen :: ArrayXXf agents_y = Eigen :: ArrayXXf :: Zero(max_drone, N+1);
    Eigen :: ArrayXXf agents_z = Eigen :: ArrayXXf :: Zero(max_drone, N+1);
    Eigen :: ArrayXf dist_to_goal = Eigen :: ArrayXf :: Ones(max_drone);
    Eigen :: ArrayXf smoothness = Eigen :: ArrayXf :: Zero(max_drone);
    Eigen :: ArrayXf arc_length = Eigen :: ArrayXf :: Zero(max_drone);
                        

    int num_drone = NU - 2;//params["num_drone"].as<int>();
    float max_time = params["max_time"].as<float>();
    float dt = params["t_plan"].as<float>()/params["num"].as<float>();
    float dist_stop = params["dist_stop"].as<float>();
    float a_drone = params["a_drone"].as<float>();
    float b_drone = params["b_drone"].as<float>();
    float c_drone = params["c_drone"].as<float>();
    
    std :: vector<std::vector<float>> _init_drone = params["init_drone"].as< std :: vector<std::vector<float>>>();
    std :: vector<std::vector<float>> _goal_drone = params["goal_drone"].as< std :: vector<std::vector<float>>>();

    if(num_drone > _init_drone.size()){
        num_drone = _init_drone.size();
        ROS_ERROR_STREAM("num_drone does not match size with given start-goal configurations");
        return 1;
    }

    ROS_INFO_STREAM("Number of drones = " << num_drone);

    save_data.open(path+"/data/sim_info_acado.txt");
    save_data << num_drone << " " << 0 << " " << dt << "\n"; 
    save_data << a_drone << " " << b_drone << " " << c_drone << "\n"; 
    
    for(int i = 0; i < num_drone; i++) save_data << _init_drone[i][0] << " " << _init_drone[i][1] << " " << _init_drone[i][2] << "\n";
    for(int i = 0; i < num_drone; i++) save_data << _goal_drone[i][0] << " " << _goal_drone[i][1] << " " << _goal_drone[i][2] << "\n";

    save_data.close();



    probData prob_data[num_drone];
    std :: thread agent_thread[num_drone];

    for(int i = 0; i < num_drone; i++){
        agents_x.row(i) = _init_drone[i][0];
        agents_y.row(i) = _init_drone[i][1];
        agents_z.row(i) = _init_drone[i][2];
    }
    for(int i = 0; i < num_drone; i++){
        prob_data[i].x_init = _init_drone[i][0];
        prob_data[i].y_init = _init_drone[i][1];
        prob_data[i].z_init = _init_drone[i][2];

        prob_data[i].x_goal = _goal_drone[i][0];
        prob_data[i].y_goal = _goal_drone[i][1];
        prob_data[i].z_goal = _goal_drone[i][2];

        prob_data[i].mpc_step = 0.0;
        prob_data[i].id_badge = i;
        prob_data[i].num_drone = num_drone - 1;

        prob_data[i].agents_x = agents_x;
        prob_data[i].agents_y = agents_y;
        prob_data[i].agents_z = agents_z;

        prob_data[i].params = params;
        prob_data[i].max_drone = max_drone;

        prob_data[i].state = 1;        
    }
    
    for(int i = 0; i < num_drone; i++)
        agent_thread[i] = std :: thread(deployAgent, std::ref(prob_data[i]));
        
    for(int i = 0; i < num_drone; i++)
        agent_thread[i].detach();

    ROS_INFO_STREAM("Threads started");
    
    int sim_iter;
    float mission_time = 0;

    std::chrono::high_resolution_clock::time_point t1, t2,  start, end;
    t1 = std :: chrono :: high_resolution_clock::now();
    start = t1;

    std :: vector<float> comp_time_agent, slack_norm_agent, inter_agent_dist;
    

    save_data.open(path+"/data/sim_data_acado.txt");
    for(sim_iter = 0; sim_iter < max_time/dt;){

        int state_sum  = 0;
        for(int i = 0; i < num_drone; i++){
            if(prob_data[i].state == 3)
                state_sum += 1;
            else
                state_sum = 0;
        }

        if(state_sum == num_drone){
            
            mission_time = (sim_iter+1)*dt; 
            sim_iter++;
            
            t2 = std :: chrono :: high_resolution_clock::now();
            std :: chrono :: duration<double, std::milli> time = t2 - t1;
            
            comp_time_agent.push_back(time.count()/1000.0/num_drone);

            for(int i = 0; i < num_drone; i++){
                agents_x.row(i) = prob_data[i].agents_x.row(i);
                agents_y.row(i) = prob_data[i].agents_y.row(i);
                agents_z.row(i) = prob_data[i].agents_z.row(i);
                dist_to_goal(i) = prob_data[i].dist_to_goal;
                slack_norm_agent.push_back(prob_data[i].slack_norm);
            }
            for(int i = 0; i < num_drone; i++){
                prob_data[i].agents_x = agents_x;
                prob_data[i].agents_y = agents_y;
                prob_data[i].agents_z = agents_z;
            }
            
            std :: vector <float> temp_inter_agent;        
            for(int i = 0; i < num_drone; i++){
                for(int j = i+1; j < num_drone; j++){
                    temp_inter_agent.push_back(sqrt(pow(prob_data[i].x_init - prob_data[j].x_init, 2) 
                                                    + pow(prob_data[i].y_init - prob_data[j].y_init, 2)
                                                    + pow(prob_data[i].z_init - prob_data[j].z_init, 2)));
                }
            }
            if(temp_inter_agent.size() != 0)
                inter_agent_dist.push_back(*std::min_element(temp_inter_agent.begin(), temp_inter_agent.end()));
                
            if((dist_to_goal.topRows(num_drone) < dist_stop).all())
                break;

            for(int i = 0; i < num_drone; i++)
                prob_data[i].state = 1;

            save_data << (agents_x.leftCols(N)).topRows(num_drone) << "\n" << (agents_y.leftCols(N)).topRows(num_drone) << "\n" << (agents_z.leftCols(N)).topRows(num_drone) << "\n";
            t1 = std :: chrono :: high_resolution_clock::now();
               
        }
         sleep(0.00001);
    }
    end = std :: chrono :: high_resolution_clock::now();
    std :: chrono :: duration<double, std::milli> total_time = end - start;
    save_data.close();


    for(int i = 0; i < num_drone; i++){
        prob_data[i].run = false;
    }

    save_data.open(path+"/data/acado_cbf/sim_results_acado"+ std::to_string(num_drone)+".txt");
    std :: vector <float> smoothness_agent, traj_length_agent;

    for(int i = 0; i < num_drone; i++){
        float smoothness = 0, arc_length = 0;
        for(int j = 0; j < prob_data[i].smoothness.size(); j++){
            smoothness += pow(prob_data[i].smoothness[j], 2);
            arc_length += prob_data[i].arc_length[j];
        }
        smoothness = sqrt(smoothness);

        smoothness_agent.push_back(smoothness);
        traj_length_agent.push_back(arc_length);
    }

    save_data << sim_iter << "\n";          // 1. SIM ITERATIONS

    float avg_smoothness=0.0, avg_traj_length=0.0, 
            min_inter_agent_dist=10000.0, avg_inter_agent_dist=0.0, avg_slack_norm=0.0;

    for(int i = 0; i < slack_norm_agent.size(); i++){
        avg_slack_norm += slack_norm_agent[i];
    }
    for(int i = 0; i < comp_time_agent.size(); i++){
        save_data << comp_time_agent[i] << "\n";        // 2. COMPUTE TIME
    }
    for(int i = 0; i < smoothness_agent.size(); i++){ 
        save_data << smoothness_agent[i] << "\n";       // 3. SMOOTHNESS
        avg_smoothness += smoothness_agent[i];
    };
    for(int i = 0; i < traj_length_agent.size(); i++){
        save_data << traj_length_agent[i] << "\n";      // 4. TRAJECTORY LENGTH
        avg_traj_length += traj_length_agent[i];
    }
    for(int i = 0; i < inter_agent_dist.size(); i++){
        save_data << inter_agent_dist[i] << "\n";      // 5. INTER-AGENT DIST
        avg_inter_agent_dist += inter_agent_dist[i];

        if(min_inter_agent_dist > inter_agent_dist[i])
            min_inter_agent_dist = inter_agent_dist[i];
    }
    

    // for(int i = 0; i < num_drone; i++){
    //     for(int j = 0; j < prob_data[i].inter_agent_dist_min.size(); j++){
    //         avg_inter_agent_dist += prob_data[i].inter_agent_dist_min[j]; 
            
    //         save_data << prob_data[i].inter_agent_dist_min[j] << "\n";          // 5. INTER-AGENT DIST
    //         if(prob_data[i].inter_agent_dist_min[j] < min_inter_agent_dist)
    //             min_inter_agent_dist = prob_data[i].inter_agent_dist_min[j];
    //     }
    // }
    
    if(smoothness.size()!=0) avg_smoothness = avg_smoothness / (smoothness.size());
    if(traj_length_agent.size()!=0) avg_traj_length = avg_traj_length / (traj_length_agent.size());
    if(inter_agent_dist.size()!=0) avg_inter_agent_dist = avg_inter_agent_dist / (inter_agent_dist.size());
    if(slack_norm_agent.size()!=0) avg_slack_norm = avg_slack_norm / (slack_norm_agent.size());

    save_data << mission_time << "\n";                          // 7. MISSION TIME
    save_data << total_time.count()/1000.0 << "\n";
    save_data << total_time.count()/1000.0/prob_data[0].mpc_step << "\n";
    save_data << total_time.count()/1000.0/prob_data[0].mpc_step/num_drone << "\n";
    save_data << avg_smoothness << "\n";
    save_data << avg_traj_length << "\n";
    save_data << avg_inter_agent_dist << "\n";
    save_data << min_inter_agent_dist << "\n";
    save_data.close();    

    ROS_INFO_STREAM("________ SUCCESS! _________");
    ROS_INFO_STREAM("Mission completion time = " << mission_time << " s");
    ROS_INFO_STREAM("Total time to compute = " << total_time.count()/1000.0 << " s");
    ROS_INFO_STREAM("Average time to compute = " << total_time.count()/1000.0/prob_data[0].mpc_step << " s");
    ROS_INFO_STREAM("Average run time per agent = " << total_time.count()/1000.0/prob_data[0].mpc_step/num_drone << " s");
    ROS_INFO_STREAM("Average smoothness = " << avg_smoothness << " ms^-2");
    ROS_INFO_STREAM("Average trajectory length = " << avg_traj_length << " m");
    ROS_INFO_STREAM("Average inter-agent dist = " << avg_inter_agent_dist << " m");
    ROS_INFO_STREAM("Smallest inter-agent dist = " << min_inter_agent_dist << " m");
    ROS_INFO_STREAM("Average slack norm = " << avg_slack_norm);

    return 0;
}
