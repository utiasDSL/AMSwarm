#include "algorithm/amswarm/run_trajectory_optimizer.hpp"

void initializeOptimizer(probData &prob_data, int VERBOSE){
	// @ Load parameters from config file
	YAML :: Node params = prob_data.params;

	std :: vector<std :: vector<float>> _pos_static_obs = prob_data.pos_static_obs;
	std :: vector<std :: vector<float>> _dim_static_obs = prob_data.dim_static_obs;
	
	prob_data.num_static_obs = _pos_static_obs.size();
	
	Eigen :: ArrayXf x_static_obs(prob_data.num_static_obs); 
	Eigen :: ArrayXf y_static_obs(prob_data.num_static_obs);
	Eigen :: ArrayXf z_static_obs(prob_data.num_static_obs);

	Eigen :: ArrayXf a_static_obs(prob_data.num_static_obs); 
	Eigen :: ArrayXf b_static_obs(prob_data.num_static_obs);
	Eigen :: ArrayXf c_static_obs(prob_data.num_static_obs);
	
	for(int i = 0; i < prob_data.num_static_obs; i++){
		x_static_obs(i) = _pos_static_obs[i][0];
		y_static_obs(i) = _pos_static_obs[i][1];
		z_static_obs(i) = _pos_static_obs[i][2];
	
		a_static_obs(i) = _dim_static_obs[i][0];
		b_static_obs(i) = _dim_static_obs[i][1];
		c_static_obs(i) = _dim_static_obs[i][2];
	}

	
	prob_data.free_space = params["free_space"].as<bool>();
	
	if(prob_data.free_space){
		prob_data.num_static_obs = 0;
	}
	
	std :: vector<float> _x_lim = params["x_lim"].as<std::vector<float>>();
	std :: vector<float> _y_lim = params["y_lim"].as<std::vector<float>>();
	std :: vector<float> _z_lim = params["z_lim"].as<std::vector<float>>();

	
	
	prob_data.world = params["world"].as<int>();
	prob_data.num = params["num"].as<int>();
	
	prob_data.t_plan = params["t_plan"].as<int>();
	prob_data.num_up = params["num_up"].as<int>();
	prob_data.dist_stop = params["dist_stop"].as<float>();
	prob_data.kappa = params["kappa"].as<int>();

	prob_data.max_iter = params["max_iter"].as<int>();
	prob_data.max_sim_time = params["max_time"].as<int>();
	prob_data.weight_goal = params["weight_goal"].as<float>();
	prob_data.weight_smoothness = params["weight_smoothness"].as<float>();
	prob_data.weight_goal_og = params["weight_goal"].as<float>();
	prob_data.weight_smoothness_og = params["weight_smoothness"].as<float>();
	
	prob_data.delta_aggressive = params["delta_aggressive"].as<float>();
	prob_data.delta_static_obs = params["delta_static_obs"].as<float>();
	prob_data.delta_drone = params["delta_drone"].as<float>();
	prob_data.delta_vel = params["delta_vel"].as<float>();
	prob_data.delta_acc = params["delta_acc"].as<float>();
	prob_data.delta_jerk = params["delta_jerk"].as<float>();
	prob_data.delta_snap = params["delta_snap"].as<float>();
	prob_data.delta_ineq = params["delta_ineq"].as<float>();

	prob_data.rho_static_obs_max = params["rho_static_obs_max"].as<float>();
	prob_data.rho_drone_max = params["rho_drone_max"].as<float>();
	prob_data.rho_vel_max = params["rho_vel_max"].as<float>();
	prob_data.rho_acc_max = params["rho_acc_max"].as<float>();
	prob_data.rho_jerk_max = params["rho_jerk_max"].as<float>();
	prob_data.rho_snap_max = params["rho_snap_max"].as<float>();
	prob_data.rho_ineq_max = params["rho_ineq_max"].as<float>();

	prob_data.axis_wise = params["axis_wise"].as<bool>();
	prob_data.jerk_snap_constraints = params["jerk_snap_constraints"].as<bool>();
	if(!prob_data.axis_wise){
		prob_data.vel_max = params["vel_max"].as<float>();
		prob_data.acc_max = params["acc_max"].as<float>();
		prob_data.jerk_max = params["jerk_max"].as<float>();
		prob_data.snap_max = params["snap_max"].as<float>();
	}
	else{
		prob_data.vel_max = params["vel_max"].as<float>()/sqrt(3);
		prob_data.acc_max = params["acc_max"].as<float>()/sqrt(3);
		prob_data.jerk_max = params["jerk_max"].as<float>()/sqrt(3);
		prob_data.snap_max = params["snap_max"].as<float>()/sqrt(3);
	}
	prob_data.x_min = _x_lim[0] + params["a_drone"].as<float>();
	prob_data.y_min = _y_lim[0] + params["b_drone"].as<float>();
	prob_data.z_min = _z_lim[0] + params["a_drone"].as<float>();
	
	prob_data.x_max = _x_lim[1] - params["a_drone"].as<float>();
	prob_data.y_max = _y_lim[1] - params["b_drone"].as<float>();
	prob_data.z_max = _z_lim[1] - params["a_drone"].as<float>();

	prob_data.thresold = params["thresold"].as<float>();

	prob_data.prox_agent = params["prox_agent"].as<float>();
	prob_data.prox_obs = params["prox_obs"].as<float>();
	prob_data.buffer = params["buffer"].as<float>();
	
	prob_data.gamma = params["gamma"].as<float>();
	prob_data.gravity = params["gravity"].as<float>();	
	prob_data.use_thrust_values = params["use_thrust_values"].as<bool>();
	prob_data.f_min = params["f_min"].as<float>() * prob_data.gravity;
	prob_data.f_max = params["f_max"].as<float>() * prob_data.gravity;

	prob_data.dt = prob_data.t_plan / prob_data.num;
	prob_data.nvar = 11;
	prob_data.mpc_step = 0;
	
	// @ Initial State
	prob_data.vx_init = 0.0;
	prob_data.vy_init = 0.0;
	prob_data.vz_init = 0.0;

	prob_data.az_init = 0.0;
	prob_data.ay_init = 0.0;
	prob_data.ax_init = 0.0;

	prob_data.x_ref = Eigen :: ArrayXXf :: Ones(prob_data.kappa, 1) * prob_data.x_goal;
	prob_data.y_ref = Eigen :: ArrayXXf :: Ones(prob_data.kappa, 1) * prob_data.y_goal;
	prob_data.z_ref = Eigen :: ArrayXXf :: Ones(prob_data.kappa, 1) * prob_data.z_goal;
	
	// @ Compute Bernstein P, Pdot, Pddot,... matrix
	Eigen :: ArrayXf tot_time = Eigen :: ArrayXf(prob_data.num);
	Eigen :: ArrayXf tot_time_up = Eigen :: ArrayXf(prob_data.num_up);
	
	tot_time.setLinSpaced(prob_data.num, 0.0, prob_data.t_plan);
	tot_time_up.setLinSpaced(prob_data.num_up, 0.0, prob_data.t_plan);

	five_var PPP = computeBernstein(tot_time, prob_data.t_plan, prob_data.num);
	prob_data.nvar = PPP.a.cols();
	prob_data.P = PPP.a;
	prob_data.Pdot = PPP.b;
	prob_data.Pddot = PPP.c;
	prob_data.Pdddot = PPP.d;
	prob_data.Pddddot = PPP.e;

	five_var PPP_up = computeBernstein(tot_time_up, prob_data.t_plan, prob_data.num_up);
	prob_data.P_up = PPP_up.a;
	prob_data.Pdot_up = PPP_up.b;
	prob_data.Pddot_up = PPP_up.c;
	prob_data.Pdddot_up = PPP_up.d;
	prob_data.Pddddot_up = PPP_up.e;

	
	// @ Position Constraints
	prob_data.A_ineq = stack(prob_data.P, -prob_data.P, 'v');

	prob_data.b_x_ineq = stack(prob_data.x_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -prob_data.x_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	prob_data.b_y_ineq = stack(prob_data.y_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -prob_data.y_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	prob_data.b_z_ineq = stack(prob_data.z_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -prob_data.z_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');

	
	// @ Initial Conditions
	prob_data.A_eq = stack(prob_data.P.row(0), stack(prob_data.Pdot.row(0), prob_data.Pddot.row(0), 'v'), 'v');

	prob_data.b_x_eq = Eigen :: ArrayXXf(3, 1);
	prob_data.b_y_eq = Eigen :: ArrayXXf(3, 1);
	prob_data.b_z_eq = Eigen :: ArrayXXf(3, 1);

	prob_data.b_x_eq << prob_data.x_init, prob_data.vx_init, prob_data.ax_init;
	prob_data.b_y_eq << prob_data.y_init, prob_data.vy_init, prob_data.ay_init;
	prob_data.b_z_eq << prob_data.z_init, prob_data.vz_init, prob_data.az_init;

	
	// @ Static Obstacle Avoidance Constraints
	if(prob_data.num_static_obs!=0){
		prob_data.A_static_obs = prob_data.P;
		for(int i = 0; i < prob_data.num_static_obs - 1; i++) prob_data.A_static_obs = stack(prob_data.A_static_obs, prob_data.P, 'v');
		
		prob_data.alpha_static_obs = Eigen :: ArrayXXf :: Zero(prob_data.num_static_obs, prob_data.num);
		prob_data.d_static_obs = Eigen :: ArrayXXf :: Ones(prob_data.num_static_obs, prob_data.num);
		prob_data.d_static_obs_old = prob_data.d_static_obs;

		prob_data.x_static_obs = (Eigen :: ArrayXXf :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * x_static_obs; 
		prob_data.y_static_obs = (Eigen :: ArrayXXf :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * y_static_obs;
		prob_data.z_static_obs = (Eigen :: ArrayXXf :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * z_static_obs;

		prob_data.a_static_obs = (Eigen :: ArrayXXf :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * a_static_obs + prob_data.buffer + params["a_drone"].as<float>();
		prob_data.b_static_obs = (Eigen :: ArrayXXf :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * b_static_obs + prob_data.buffer + params["b_drone"].as<float>();
		prob_data.c_static_obs = (Eigen :: ArrayXXf :: Ones(prob_data.num_static_obs, prob_data.num)).colwise() * c_static_obs + prob_data.buffer + params["c_drone"].as<float>();

		prob_data.x_static_obs_og = prob_data.x_static_obs;
		prob_data.y_static_obs_og = prob_data.y_static_obs;
		prob_data.z_static_obs_og = prob_data.z_static_obs;

		prob_data.a_static_obs_og = prob_data.a_static_obs;
		prob_data.b_static_obs_og = prob_data.b_static_obs;
		prob_data.c_static_obs_og = prob_data.c_static_obs;
	}
	
	prob_data.lx_drone = params["a_drone"].as<float>();
	prob_data.ly_drone = params["b_drone"].as<float>();
	prob_data.lz_drone = params["c_drone"].as<float>();
	prob_data.rmin = prob_data.lx_drone * 2;
	
	// @ Inter-agent Collision Avoidance Constraints
	if(prob_data.num_drone!=0){

		prob_data.a_drone = Eigen :: ArrayXXf :: Ones(prob_data.num_drone, prob_data.num) * (2*prob_data.lx_drone + prob_data.buffer);
		prob_data.b_drone = Eigen :: ArrayXXf :: Ones(prob_data.num_drone, prob_data.num) * (2*prob_data.ly_drone + prob_data.buffer);
		prob_data.c_drone = Eigen :: ArrayXXf :: Ones(prob_data.num_drone, prob_data.num) * (2*prob_data.lz_drone + prob_data.buffer);
	
		prob_data.alpha_drone = Eigen :: ArrayXXf :: Zero(prob_data.num_drone, prob_data.num);
		prob_data.beta_drone = Eigen :: ArrayXXf :: Ones(prob_data.num_drone, prob_data.num) * M_PI_2;
		prob_data.d_drone = Eigen :: ArrayXXf :: Ones(prob_data.num_drone, prob_data.num);
		prob_data.d_drone_old = prob_data.d_drone;

		prob_data.A_drone = prob_data.P;    
		for(int i = 0; i < prob_data.num_drone - 1; i++) prob_data.A_drone = stack(prob_data.A_drone, prob_data.P, 'v');
	}
	else{
		ROS_WARN_STREAM("Only one drone in environment");
	}

	if(!prob_data.axis_wise){
		// @ Velocity Constraints
		prob_data.alpha_vel = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
		prob_data.beta_vel = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * M_PI_2;
		prob_data.d_vel = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * prob_data.vel_max * 0;
		
		// @ Acceleration Constraints
		prob_data.alpha_acc = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
		prob_data.beta_acc = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * M_PI_2;
		prob_data.d_acc = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * prob_data.acc_max * 0;
		
		
		// @ Jerk Constraints
		prob_data.alpha_jerk = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
		prob_data.beta_jerk = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * M_PI_2;
		prob_data.d_jerk = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * prob_data.jerk_max * 0;
		
		
		// @ Snap Constraints
		prob_data.alpha_snap = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
		prob_data.beta_snap = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * M_PI_2;
		prob_data.d_snap = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * prob_data.snap_max * 0;
	}
	else{
		// @ Velocity Constraints
		prob_data.A_v_ineq = stack(prob_data.Pdot, -prob_data.Pdot, 'v');

		prob_data.b_vx_ineq = stack(prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		prob_data.b_vy_ineq = stack(prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		prob_data.b_vz_ineq = stack(prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');

		// @ Acceleration Constraints
		prob_data.A_a_ineq = stack(prob_data.Pddot, -prob_data.Pddot, 'v');

		if(!prob_data.use_thrust_values && prob_data.world == 2){
			prob_data.b_ax_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			prob_data.b_ay_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			prob_data.b_az_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		}
		else{

			/** 
			 * 2x^2 + (x+g)^2 = (1.5g)^2
			 * 2x^2 + x^2 + 2xg + g^2-(1.5g)^2=0
			 * 3x^2 + 2xg + (g^2-(1.5g)^2)
			 * **/

			prob_data.acc_max = (-(2*prob_data.gravity) + sqrt(pow(2*prob_data.gravity,2) - 4*3*(pow(prob_data.gravity,2) - pow(1.5*prob_data.gravity,2))))/6.0; 
			float z_acc_min = prob_data.f_min - prob_data.gravity;
			
			prob_data.b_ax_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			prob_data.b_ay_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			prob_data.b_az_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -z_acc_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			
		}
		// @ Jerk Constraints
		prob_data.A_j_ineq = stack(prob_data.Pdddot, -prob_data.Pdddot, 'v');

		prob_data.b_jx_ineq = stack(prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		prob_data.b_jy_ineq = stack(prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		prob_data.b_jz_ineq = stack(prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');

		// @ Snap Constraints
		prob_data.A_s_ineq = stack(prob_data.Pddddot, -prob_data.Pddddot, 'v');

		prob_data.b_sx_ineq = stack(prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		prob_data.b_sy_ineq = stack(prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		prob_data.b_sz_ineq = stack(prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	}
	
	// @ Constant Costs
	prob_data.cost_ineq = prob_data.A_ineq.transpose().matrix() * prob_data.A_ineq.matrix();
	if(prob_data.params["order_smoothness"].as<int>() == 4)
		prob_data.cost_smoothness = prob_data.Pddddot.transpose().matrix() * prob_data.Pddddot.matrix();
	else if(prob_data.params["order_smoothness"].as<int>() == 3)
		prob_data.cost_smoothness = prob_data.Pdddot.transpose().matrix() * prob_data.Pdddot.matrix();
	else if(prob_data.params["order_smoothness"].as<int>() == 2)
		prob_data.cost_smoothness = prob_data.Pddot.transpose().matrix() * prob_data.Pddot.matrix();
	else	
		ROS_ERROR_STREAM("Invalid order_smoothness value");

	
	prob_data.cost_goal = prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.P.bottomRows(prob_data.kappa).matrix();    
	prob_data.I = Eigen :: ArrayXXf(prob_data.nvar + prob_data.A_eq.rows(), prob_data.nvar + prob_data.A_eq.rows());
	prob_data.I.matrix().setIdentity();

	if(!prob_data.axis_wise){
		prob_data.cost_vel = prob_data.Pdot.transpose().matrix() * prob_data.Pdot.matrix();
		prob_data.cost_acc = prob_data.Pddot.transpose().matrix() * prob_data.Pddot.matrix();
		prob_data.cost_jerk = prob_data.Pdddot.transpose().matrix() * prob_data.Pdddot.matrix();
		prob_data.cost_snap = prob_data.Pddddot.transpose().matrix() * prob_data.Pddddot.matrix();
	}
	else{
		prob_data.cost_vel = prob_data.A_v_ineq.transpose().matrix() * prob_data.A_v_ineq.matrix();
		prob_data.cost_acc = prob_data.A_a_ineq.transpose().matrix() * prob_data.A_a_ineq.matrix();
		prob_data.cost_jerk = prob_data.A_j_ineq.transpose().matrix() * prob_data.A_j_ineq.matrix();
		prob_data.cost_snap = prob_data.A_s_ineq.transpose().matrix() * prob_data.A_s_ineq.matrix();
	}
	
	prob_data.x = prob_data.x_init * Eigen :: ArrayXXf :: Ones(prob_data.num, 1);
	prob_data.xdot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
	prob_data.xddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
	prob_data.xdddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
	prob_data.xddddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);

	prob_data.y = prob_data.y_init * Eigen :: ArrayXXf :: Ones(prob_data.num, 1);
	prob_data.ydot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
	prob_data.yddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
	prob_data.ydddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
	prob_data.yddddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);

	prob_data.z = prob_data.z_init * Eigen :: ArrayXXf :: Ones(prob_data.num, 1);
	prob_data.zdot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
	prob_data.zddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
	prob_data.zdddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
	prob_data.zddddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);

	prob_data.num_drone = 0;
	prob_data.num_static_obs = 0;

	
}

void deployAgent(probData &prob_data, int VERBOSE){
	
	// Initialize paramters and build matrices
	if(prob_data.mpc_step == 0){
		// ROS_INFO_STREAM("Initializing Drone " << prob_data.id_badge);
		initializeOptimizer(prob_data, VERBOSE);
	}
	// @ Initialize alpha betas ds
	if(prob_data.mpc_step > 0){
		// @ Neighboring agents and obstacles -- don't change the order 
		if(!prob_data.free_space)
			initObstacles(prob_data, VERBOSE);
		neigbhoringAgents(prob_data, VERBOSE);		

		if(prob_data.world == 2)
			initAlpha(prob_data, VERBOSE);
		else if(prob_data.world == 3)
			initAlphaBeta(prob_data, VERBOSE);
		else
			ROS_ERROR_STREAM("Cannot identify world");
	}
	

	// @ Solve xyz
	if(prob_data.axis_wise){
		if(prob_data.world == 2)
			computeXYAxis(prob_data, VERBOSE);
		else if(prob_data.world == 3)
			computeXYZAxis(prob_data, VERBOSE);
	}
	else{
		if(prob_data.world == 2)
			computeXY(prob_data, VERBOSE);
		else if(prob_data.world == 3)
			computeXYZ(prob_data, VERBOSE);
	}

	prob_data.smoothness.push_back(sqrt(pow(prob_data.ax_init, 2) + pow(prob_data.ay_init, 2) + pow(prob_data.az_init, 2)));
	prob_data.arc_length.push_back(sqrt(pow(prob_data.x_init - prob_data.x(1), 2) + pow(prob_data.y_init - prob_data.y(1), 2) + pow(prob_data.z_init - prob_data.z(1), 2)));
	
	// if(prob_data.inter_agent_dist.size() != 0)
	// 	prob_data.inter_agent_dist_min.push_back(*std::min_element(prob_data.inter_agent_dist.begin(), prob_data.inter_agent_dist.end()));
	// if(prob_data.agent_obs_dist.size() != 0)
	// 	prob_data.agent_obs_dist_min.push_back(*std::min_element(prob_data.agent_obs_dist.begin(), prob_data.agent_obs_dist.end()));


	// @ Apply control input				
	if(!prob_data.use_model){
		prob_data.x_init = prob_data.x(1);
		prob_data.y_init = prob_data.y(1);
		prob_data.z_init = prob_data.z(1);

		prob_data.vx_init = prob_data.xdot(1);
		prob_data.vy_init = prob_data.ydot(1);
		prob_data.vz_init = prob_data.zdot(1);

		prob_data.ax_init = prob_data.xddot(1);
		prob_data.ay_init = prob_data.yddot(1);
		prob_data.az_init = prob_data.zddot(1);
	}
	else{
		float del = (abs(prob_data.world - 2.0001)/(prob_data.world - 2.0001) + 1)/2;

		std::default_random_engine generator;
		generator.seed(std::chrono::system_clock::now().time_since_epoch().count());

		std::normal_distribution<double> dist(prob_data.mean, prob_data.stdev);
		
		prob_data.ax_init = prob_data.xddot(1) + dist(generator);
		prob_data.ay_init = prob_data.yddot(1) + dist(generator);
		prob_data.az_init = prob_data.zddot(1) + dist(generator)*del;
	
		prob_data.vx_init += prob_data.ax_init * prob_data.dt + dist(generator);
		prob_data.vy_init += prob_data.ay_init * prob_data.dt + dist(generator);
		prob_data.vz_init += prob_data.az_init * prob_data.dt + dist(generator)*del;

		prob_data.x_init += prob_data.vx_init * prob_data.dt + 0.5 * prob_data.ax_init * prob_data.dt * prob_data.dt + dist(generator);
		prob_data.y_init += prob_data.vy_init * prob_data.dt + 0.5 * prob_data.ay_init * prob_data.dt * prob_data.dt + dist(generator);
		prob_data.z_init += prob_data.vz_init * prob_data.dt + 0.5 * prob_data.az_init * prob_data.dt * prob_data.dt + dist(generator)*del;
	}

	
	prob_data.dist_to_goal = sqrt(pow(prob_data.x_init - prob_data.x_goal, 2) + pow(prob_data.y_init - prob_data.y_goal, 2) + pow(prob_data.z_init - prob_data.z_goal, 2));
	

	if(VERBOSE == 1)
		checkResiduals(prob_data, VERBOSE);
}

