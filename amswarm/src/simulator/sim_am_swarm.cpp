#include "simulator/sim_am_swarm.hpp"
#include <thread>
#include <ros/package.h>
#include <ros/ros.h>


Simulator :: Simulator(int cf_num, bool read_cf, int num_drones, bool use_model, std::vector<float> noise){

    path = ros :: package::getPath("amswarm");
    params = YAML :: LoadFile(path+"/params/config_am_swarm.yaml");

    read_config = read_cf;
    config_num = cf_num;    

    VERBOSE = params["verbose"].as<int>();
    
    num = params["num"].as<int>();
    num_up = params["num_up"].as<int>();
    if(read_config)
        num_drone = num_drones;
    else
        num_drone = params["num_drone"].as<int>();
    
    free_space = params["free_space"].as<bool>();

    a_drone = params["a_drone"].as<float>();
    b_drone = params["b_drone"].as<float>();
    c_drone = params["c_drone"].as<float>();
    
    max_time = params["max_time"].as<float>();
    t_plan = params["t_plan"].as<float>();
    dist_stop = params["dist_stop"].as<float>();
    dt = t_plan/num;

    success = false;
    collision_agent = false;
    collision_obstacle = false;

    prob_data = new probData[num_drone];
    

    agents_x = Eigen :: ArrayXXf(num_drone, num); 
    agents_y = Eigen :: ArrayXXf(num_drone, num); 
    agents_z = Eigen :: ArrayXXf(num_drone, num);

    smoothness = Eigen :: ArrayXf(num_drone);
    arc_length = Eigen :: ArrayXf(num_drone);
    dist_to_goal = Eigen :: ArrayXf(num_drone);

    if(!read_config){
        _init_drone = params["init_drone"].as< std :: vector<std::vector<float>>>();
        _goal_drone = params["goal_drone"].as< std :: vector<std::vector<float>>>();

        _pos_static_obs = params["pos_static_obs"].as< std :: vector<std::vector<float>>>();
        _dim_static_obs = params["dim_static_obs"].as< std :: vector<std::vector<float>>>();

        num_obs = _pos_static_obs.size();
        if(free_space) num_obs = 0;
        
        if(_pos_static_obs.size() != _dim_static_obs.size())
            ROS_ERROR_STREAM("pos_obs and dim_obs sizes do not match");

        if(num_drone > _init_drone.size()){
            num_drone = _init_drone.size();
            ROS_WARN_STREAM("num_drone does not match size with given start-goal configurations");
        }
        ROS_INFO_STREAM("Number of drones " << num_drone);
    }
    else{
        Eigen :: ArrayXXf config_data(1000, 1);
        std :: string file_path;
                
        num_obs_2 = 16;
        num_obs = params["num_obs"].as<int>();
        
        file_path = path+"/data/point_to_point/config_data/varying_agents/obs_" + std::to_string(num_obs_2) +"/drone_" +std :: to_string(num_drone)+"_config_"+std :: to_string(config_num)+".txt";
            
        std :: ifstream read_data(file_path);
        if (!read_data.is_open()) {
            ROS_ERROR_STREAM("There was a problem opening the input file!\n");
        }

        int k = 0;
        while (read_data >> config_data(k) >> config_data(k+1) >> config_data(k+2)) 
            k+=3;

        config_data.conservativeResize(k, 1);
        config_data = (reshape(config_data, 3, num_drone*2 + num_obs*2)).transpose();
        
        Eigen :: ArrayXXf obs_data = config_data.bottomRows(2*num_obs);
        
        for(int i = 0; i < num_obs; i++){
            _pos_static_obs.push_back({obs_data(i, 0), obs_data(i, 1), obs_data(i, 2)});
            _dim_static_obs.push_back({obs_data(i+num_obs, 0), obs_data(i+num_obs, 1), obs_data(i+num_obs, 2)});
        }
        
        for(int i = 0; i < num_drone; i++){            
            _init_drone.push_back({config_data(i, 0), config_data(i, 1), config_data(i, 2)});
            _goal_drone.push_back({config_data(i+num_drone, 0), config_data(i+num_drone, 1), config_data(i+num_drone, 2)});
        }
        
    }

    if(read_config){
        if(params["gamma"].as<float>() == 1.0) 
            folder_name << "";
        else 
            folder_name << "_gamma_"<< std::setprecision(2) << params["gamma"].as<float>();
    
        // ROS_INFO_STREAM(folder_name.str());

        if(params["axis_wise"].as<bool>())
            save_data.open(path+"/data/point_to_point/config_data/varying_agents/obs_" + std::to_string(num_obs) +"/results_am_ax"+folder_name.str()+"/sim_info_drone_" +std :: to_string(num_drone)+"_config_"+std :: to_string(config_num)+".txt");
        else
            save_data.open(path+"/data/point_to_point/config_data/varying_agents/obs_" + std::to_string(num_obs) +"/results_am_qd"+folder_name.str()+"/sim_info_drone_" +std :: to_string(num_drone)+"_config_"+std :: to_string(config_num)+".txt");
            
    }
    else
        save_data.open(path+"/data/sim_info.txt");
    save_data << num_drone << " " << num_obs << " " << dt << "\n"; 
    save_data << a_drone << " " << b_drone << " " << c_drone << "\n"; 
    
    for(int i = 0; i < num_drone; i++) save_data << _init_drone[i][0] << " " << _init_drone[i][1] << " " << _init_drone[i][2] << "\n";
    for(int i = 0; i < num_drone; i++) save_data << _goal_drone[i][0] << " " << _goal_drone[i][1] << " " << _goal_drone[i][2] << "\n";
    
    for(int i = 0; i < num_obs; i++) save_data << _pos_static_obs[i][0] << " " << _pos_static_obs[i][1] << " " << _pos_static_obs[i][2] << "\n"; 
    for(int i = 0; i < num_obs; i++) save_data << _dim_static_obs[i][0] << " " << _dim_static_obs[i][1] << " " << _dim_static_obs[i][2] << "\n";

    save_data.close();
    
    
    for(int i = 0; i < num_drone; i++){
        prob_data[i].id_badge = i;
        prob_data[i].num_drone = num_drone - 1;
        prob_data[i].x_init = _init_drone[i][0];
        prob_data[i].y_init = _init_drone[i][1];
        prob_data[i].z_init = _init_drone[i][2];

        prob_data[i].x_goal = _goal_drone[i][0];
        prob_data[i].y_goal = _goal_drone[i][1];
        prob_data[i].z_goal = _goal_drone[i][2];

        prob_data[i].pos_static_obs = _pos_static_obs;
        prob_data[i].dim_static_obs = _dim_static_obs;
        
        prob_data[i].params = params;
        prob_data[i].mpc_step = 0;

        prob_data[i].use_model = use_model;
        prob_data[i].mean = noise[0];
        prob_data[i].stdev = noise[1];
    }
    
}

void Simulator :: shareInformation(){
    // Update common variable
    for(int i = 0; i < num_drone; i++){
        if(sim_iter == 0){
            agents_x.row(i) = Eigen :: ArrayXXf :: Ones(1, num) * _init_drone[i][0];
            agents_y.row(i) = Eigen :: ArrayXXf :: Ones(1, num) * _init_drone[i][1];
            agents_z.row(i) = Eigen :: ArrayXXf :: Ones(1, num) * _init_drone[i][2];
        }
        else{
            agents_x.row(i) = prob_data[i].x.transpose();
            agents_y.row(i) = prob_data[i].y.transpose();
            agents_z.row(i) = prob_data[i].z.transpose();
        }
    }

    // Share information
    for(int i = 0; i < num_drone; i++){
        prob_data[i].agents_x = Eigen :: ArrayXXf(num_drone, num);
        prob_data[i].agents_y = Eigen :: ArrayXXf(num_drone, num);
        prob_data[i].agents_z = Eigen :: ArrayXXf(num_drone, num);

        prob_data[i].agents_x = agents_x;
        prob_data[i].agents_y = agents_y;
        prob_data[i].agents_z = agents_z;
    }
}

void Simulator :: checkCollision(){
    // Collision check
    for(int i = 0; i < num_drone; i++){
        Eigen :: ArrayXf coll;
        coll = pow((agents_x(i, 0) - agents_x.col(0))/(2*a_drone), 2) 
            + pow((agents_y(i, 0) - agents_y.col(0))/(2*b_drone), 2)
            + pow((agents_z(i, 0) - agents_z.col(0))/(2*c_drone), 2); 
        for(int j = 0; j < num_drone; j++){
            if(i!=j){
                float val = pow(agents_x(i, 0) - agents_x(j, 0), 2)/pow(2*a_drone, 2)
                        + pow(agents_y(i, 0) - agents_y(j, 0), 2)/pow(2*b_drone, 2)
                        + pow(agents_z(i, 0) - agents_z(j, 0), 2)/pow(2*c_drone, 2);
                if(val < 1.0){
                    if(VERBOSE == 3)
                        ROS_WARN_STREAM("Collision between agent " << i << " and agent " << j << " at MPC step " << prob_data[i].mpc_step);   
                    collision_agent = true;
                }
            }
        }
        
        for(int m = 0; m < num_obs; m++){
            float val = pow(agents_x(i, 0) - _pos_static_obs[m][0], 2)/pow(_dim_static_obs[m][0] + a_drone, 2)
                        + pow(agents_y(i, 0) - _pos_static_obs[m][1], 2)/pow(_dim_static_obs[m][1] + b_drone, 2)
                        + pow(agents_z(i, 0) - _pos_static_obs[m][2], 2)/pow(_dim_static_obs[m][2] + c_drone, 2);
            if(val < 1.0){
                if(VERBOSE == 3)
                    ROS_WARN_STREAM("Collision between agent " << i << " and obstacle " << m << " at MPC step " << prob_data[i].mpc_step);   
                collision_obstacle = true;
            }
        }
    }
}

void Simulator :: checkAtGoal(){
    // Dist-to-goal check
    for(int i = 0; i < num_drone; i++){
        prob_data[i].mpc_step++;
        dist_to_goal(i) = prob_data[i].dist_to_goal;
    }
    if((dist_to_goal < dist_stop).all()){
        success = true; 
    }
}

void Simulator :: runAlgorithm(){

    std :: thread agent_thread[num_drone];

    // Start Threads
    auto start = std :: chrono :: high_resolution_clock::now(); 
    for(int i = 0; i < num_drone; i++){
        agent_thread[i] = std :: thread(deployAgent, std::ref(prob_data[i]), VERBOSE);
    }

    // Waiting for threads to terminate
    for(int i = 0; i < num_drone; i++)
        agent_thread[i].join();
    auto end = std :: chrono :: high_resolution_clock::now();
    std :: chrono :: duration<double, std::milli> total_time = end - start;
    
    comp_time_agent.push_back(total_time.count()/1000.0/num_drone);

    if(VERBOSE == 4)
        ROS_INFO_STREAM("Time to compute = " << total_time.count()/1000.0 << " s, Planning Frequency = " << 1000.0/total_time.count());
}

void Simulator :: checkViolation(){
    // check for any violations in pos, vel, acc bounds
    for(int i = 0; i < num_drone; i++){
        // check for any violations in pos, vel, acc bounds
		
        if(std::floor(prob_data[i].x_init/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].x_max || std::ceil(prob_data[i].x_init/prob_data[i].thresold)*prob_data[i].thresold < prob_data[i].x_min){
			out_space = true;
            ROS_WARN_STREAM("Positional bounds not satisfied in x " << prob_data[i].x_init << " " << prob_data[i].x_min << " " << prob_data[i].x_max);
        }
		if(std::floor(prob_data[i].y_init/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].y_max  || std::ceil(prob_data[i].y_init/prob_data[i].thresold)*prob_data[i].thresold < prob_data[i].y_min){
			out_space = true;
            ROS_WARN_STREAM("Positional bounds not satisfied in y " << prob_data[i].y_init << " " << prob_data[i].y_min << " " << prob_data[i].y_max);
        }
		if(std::floor(prob_data[i].z_init/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].z_max  || std::ceil(prob_data[i].z_init/prob_data[i].thresold)*prob_data[i].thresold < prob_data[i].z_min){
			out_space = true;
            ROS_WARN_STREAM("Positional bounds not satisfied in z " << prob_data[i].z_init << " " << prob_data[i].z_min << " " << prob_data[i].z_max);
        }
		if(prob_data[i].axis_wise){
			if(std::floor(abs(prob_data[i].vx_init)/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].vel_max + 0.01){
				ROS_WARN_STREAM("Velocity bounds not satisfied in x " << abs(prob_data[i].vx_init) << " " << prob_data[i].vel_max);
                out_space = true;
            }
			if(std::floor(abs(prob_data[i].vy_init)/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].vel_max + 0.01){
				ROS_WARN_STREAM("Velocity bounds not satisfied in y " << abs(prob_data[i].vy_init) << " " << prob_data[i].vel_max);
                out_space = true;
            }
			if(std::floor(abs(prob_data[i].vz_init)/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].vel_max + 0.01){
				ROS_WARN_STREAM("Velocity bounds not satisfied in z " << abs(prob_data[i].vz_init) << " " << prob_data[i].vel_max);
                out_space = true;    
            }
			float acc_control = sqrt(pow(prob_data[i].ax_init,2) + pow(prob_data[i].ay_init,2) + pow(prob_data[i].gravity+prob_data[i].az_init,2));
			if(std::floor(acc_control/prob_data[i].thresold/10)*prob_data[i].thresold*10 > prob_data[i].f_max + 0.01 || std::ceil(acc_control/prob_data[i].thresold/10)*prob_data[i].thresold*10 < prob_data[i].f_min - 0.01){
                ROS_WARN_STREAM("Acceleration bounds not satisfied " << acc_control << " " << prob_data[i].f_min << " " << prob_data[i].f_max);
                out_space = true;
            }

		}
		else{
			float acc_control = sqrt(pow(prob_data[i].ax_init,2) + pow(prob_data[i].ay_init,2) + pow(prob_data[i].gravity+prob_data[i].az_init,2));
			float vel_control = sqrt(pow(prob_data[i].vx_init,2) + pow(prob_data[i].vy_init,2) + pow(prob_data[i].vz_init,2));
			if(std::floor(vel_control/prob_data[i].thresold)*prob_data[i].thresold > prob_data[i].vel_max + 0.01 ){
                out_space = true;
				ROS_WARN_STREAM("Velocity bounds not satisfied " << vel_control << " " << prob_data[i].vel_max);
			}
			if(std::floor(acc_control/prob_data[i].thresold/10)*prob_data[i].thresold*10 > prob_data[i].f_max + 0.01  || std::ceil(acc_control/prob_data[i].thresold/10)*prob_data[i].thresold*10 < prob_data[i].f_min - 0.01){
                out_space = true;
				ROS_WARN_STREAM("Acceleration bounds not satisfied " << acc_control << " " << prob_data[i].f_min << " " << prob_data[i].f_max);
            }
		}
    }
}

void Simulator :: runSimulation(){
    
    if(read_config){
        if(params["axis_wise"].as<bool>()){
            save_data.open(path+"/data/point_to_point/config_data/varying_agents/obs_" + std::to_string(num_obs) +"/results_am_ax"+folder_name.str()+"/sim_data_drone_" +std :: to_string(num_drone)+"_config_"+std :: to_string(config_num)+".txt");
            save_data_2.open(path+"/data/point_to_point/config_data/varying_agents/obs_" + std::to_string(num_obs) +"/results_am_ax"+folder_name.str()+"/sim_residue_drone_" +std :: to_string(num_drone)+"_config_"+std :: to_string(config_num)+".txt");       
            }
        else{
            save_data.open(path+"/data/point_to_point/config_data/varying_agents/obs_" + std::to_string(num_obs) +"/results_am_qd"+folder_name.str()+"/sim_data_drone_" +std :: to_string(num_drone)+"_config_"+std :: to_string(config_num)+".txt");
            save_data_2.open(path+"/data/point_to_point/config_data/varying_agents/obs_" + std::to_string(num_obs) +"/results_am_qd"+folder_name.str()+"/sim_residue_drone_" +std :: to_string(num_drone)+"_config_"+std :: to_string(config_num)+".txt");
        }
    }
    else{
        save_data.open(path+"/data/sim_data.txt");
        save_data_2.open(path+"/data/sim_data_upsampled_x.txt");
        save_data_3.open(path+"/data/sim_data_upsampled_y.txt");
        save_data_4.open(path+"/data/sim_data_upsampled_z.txt");
    }
    auto start = std :: chrono :: high_resolution_clock::now();            
    
    for(sim_iter = 0; sim_iter < max_time/dt; sim_iter++){
        
        out_space = false;

        Simulator :: shareInformation();
        Simulator :: runAlgorithm();
        Simulator :: checkCollision();
        Simulator :: checkViolation();
        Simulator :: checkAtGoal();
        Simulator :: calculateDistances();
        mission_time = (sim_iter+1)*dt; 
        save_data << agents_x << "\n" << agents_y << "\n" << agents_z << "\n";
        
        Eigen :: ArrayXXf temp_x_upsampled((int)(num_up/num), num_drone), 
                          temp_y_upsampled((int)(num_up/num), num_drone), 
                          temp_z_upsampled((int)(num_up/num), num_drone),
                          temp_vx_upsampled((int)(num_up/num), num_drone), 
                          temp_vy_upsampled((int)(num_up/num), num_drone), 
                          temp_vz_upsampled((int)(num_up/num), num_drone),
                          temp_ax_upsampled((int)(num_up/num), num_drone), 
                          temp_ay_upsampled((int)(num_up/num), num_drone), 
                          temp_az_upsampled((int)(num_up/num), num_drone);
        for(int i = 0; i < num_drone; i++){
            temp_x_upsampled.col(i) = prob_data[i].x_up.topRows((int)(num_up/num));
            temp_y_upsampled.col(i) = prob_data[i].y_up.topRows((int)(num_up/num));
            temp_z_upsampled.col(i) = prob_data[i].z_up.topRows((int)(num_up/num));

            temp_vx_upsampled.col(i) = prob_data[i].xdot_up.topRows((int)(num_up/num));
            temp_vy_upsampled.col(i) = prob_data[i].ydot_up.topRows((int)(num_up/num));
            temp_vz_upsampled.col(i) = prob_data[i].zdot_up.topRows((int)(num_up/num));

            temp_ax_upsampled.col(i) = prob_data[i].xddot_up.topRows((int)(num_up/num));
            temp_ay_upsampled.col(i) = prob_data[i].yddot_up.topRows((int)(num_up/num));
            temp_az_upsampled.col(i) = prob_data[i].zddot_up.topRows((int)(num_up/num));
        }
        save_data_2 << temp_x_upsampled.transpose() << "\n" << temp_vx_upsampled.transpose() << "\n" << temp_ax_upsampled.transpose() << "\n"; 
        save_data_3 << temp_y_upsampled.transpose() << "\n" << temp_vy_upsampled.transpose() << "\n" << temp_ay_upsampled.transpose() << "\n";
        save_data_4 << temp_z_upsampled.transpose() << "\n" << temp_vz_upsampled.transpose() << "\n" << temp_az_upsampled.transpose() << "\n";

        // for(int i = 0; i < num_drone; i++){
        //     save_data_2 << prob_data[i].res_x_static_obs_norm << " " << prob_data[i].res_y_static_obs_norm 
        //                 << " " << prob_data[i].res_x_drone_norm << " " << prob_data[i].res_y_drone_norm << " " << prob_data[i].res_z_drone_norm
        //                 << " " << prob_data[i].res_x_vel_norm << " " << prob_data[i].res_y_vel_norm << " " << prob_data[i].res_z_vel_norm
        //                 << " " << prob_data[i].res_x_acc_norm << " " << prob_data[i].res_y_acc_norm << " " << prob_data[i].res_z_acc_norm
        //                 << " " << prob_data[i].res_x_ineq_norm << " " << prob_data[i].res_y_ineq_norm << " " << prob_data[i].res_z_ineq_norm << "\n";
        // }


        if(VERBOSE == 2){
            ROS_INFO_STREAM("Simulation time = " << sim_iter);
            ROS_INFO_STREAM("Distance to goals = " << dist_to_goal.transpose());
        }

        if(success)
            break;
        if(collision_agent || collision_obstacle)
            break;
        // if(out_space)
        //     break;
    }
    auto end = std :: chrono :: high_resolution_clock::now();
    total_time = end - start;
    save_data.close();
    save_data_2.close();
    save_data_3.close();
    save_data_4.close();
}
void Simulator :: calculateDistances(){
    std :: vector <float> temp_inter_agent, temp_agent_obs;
    for(int i = 0; i < num_drone; i++){
        for(int j = i+1; j < num_drone; j++){
            temp_inter_agent.push_back(sqrt(pow(prob_data[i].x_init - prob_data[j].x_init, 2) 
                                            + pow(prob_data[i].y_init - prob_data[j].y_init, 2)
                                            + pow(prob_data[i].z_init - prob_data[j].z_init, 2)));
        }

        for(int j = 0; j < prob_data[0].x_static_obs_og.rows(); j++){
            temp_agent_obs.push_back(sqrt(pow(prob_data[i].x_init - prob_data[i].x_static_obs_og(j, 0), 2) 
                                            + pow(prob_data[i].y_init - prob_data[i].y_static_obs_og(j, 0), 2)) 
                                            - prob_data[i].a_static_obs_og(j, 0) 
                                            + prob_data[i].lx_drone + prob_data[i].buffer);
        }
    }
    if(temp_inter_agent.size() != 0)
        inter_agent_dist.push_back(*std::min_element(temp_inter_agent.begin(), temp_inter_agent.end()));
    if(temp_agent_obs.size() != 0)
        agent_obs_dist.push_back(*std::min_element(temp_agent_obs.begin(), temp_agent_obs.end()));
    
}
void Simulator :: saveMetrics(){
    
    if(success && !collision_agent && !collision_obstacle){
        

        if(read_config){
            if(prob_data[0].axis_wise)
                save_data.open(path+"/data/point_to_point/config_data/varying_agents/obs_" + std::to_string(num_obs) +"/results_am_ax"+folder_name.str()+"/sim_results_drone_" +std :: to_string(num_drone)+"_config_"+std :: to_string(config_num)+".txt");
            else
                save_data.open(path+"/data/point_to_point/config_data/varying_agents/obs_" + std::to_string(num_obs) +"/results_am_qd"+folder_name.str()+"/sim_results_drone_" +std :: to_string(num_drone)+"_config_"+std :: to_string(config_num)+".txt");
        }
        else
            save_data.open(path+"/data/sim_results.txt");
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
              min_inter_agent_dist=10000.0, avg_inter_agent_dist=0.0,
              min_agent_obs_dist=10000.0, avg_agent_obs_dist=0.0;

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
        for(int i = 0; i < agent_obs_dist.size(); i++){
            save_data << agent_obs_dist[i] << "\n";      // 6. AGENT-OBS DIST
            avg_agent_obs_dist += agent_obs_dist[i];

            if(min_agent_obs_dist > agent_obs_dist[i])
                min_agent_obs_dist = agent_obs_dist[i];
        }
        
        if(smoothness.size()!=0) avg_smoothness = avg_smoothness / (smoothness.size());
        if(traj_length_agent.size()!=0) avg_traj_length = avg_traj_length / (traj_length_agent.size());
        if(inter_agent_dist.size()!=0) avg_inter_agent_dist = avg_inter_agent_dist / (inter_agent_dist.size());
        if(agent_obs_dist.size()!=0) avg_agent_obs_dist = avg_agent_obs_dist / (agent_obs_dist.size());
        


        save_data << mission_time << "\n";                          // 7. MISSION TIME
        save_data << total_time.count()/1000.0 << "\n";
        save_data << total_time.count()/1000.0/prob_data[0].mpc_step << "\n";
        save_data << total_time.count()/1000.0/prob_data[0].mpc_step/num_drone << "\n";
        save_data << avg_smoothness << "\n";
        save_data << avg_traj_length << "\n";
        save_data << avg_inter_agent_dist << "\n";
        save_data << min_inter_agent_dist << "\n";
        save_data << avg_agent_obs_dist << "\n";
        save_data << min_agent_obs_dist << "\n";
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
        ROS_INFO_STREAM("Average obs-agent dist = " << avg_agent_obs_dist << " m");
        ROS_INFO_STREAM("Smallest obs-agent dist = " << min_agent_obs_dist << " m");
    }
    else{
        success = false;
        ROS_INFO_STREAM("________ FAILURE! _________");
        if(collision_agent)
            ROS_WARN_STREAM("Inter-agent collision");
        else if(collision_obstacle)
            ROS_WARN_STREAM("Obstacle-agent collision");
        else
            ROS_WARN_STREAM("Goal not Reached");
    }
    
}