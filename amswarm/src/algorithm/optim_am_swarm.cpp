#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <chrono>
#include <ros/package.h>
#include <ros/ros.h>

#include "yaml-cpp/yaml.h"

#include "amswarm/algorithm/optim_am_swarm.hpp"
#include <eigen-quadprog/QuadProg.h>
#include <eigen-quadprog/eigen_quadprog_api.h>


namespace Optim
{
    float binomialCoeff(float n, float k)
	{
		if (k == 0 || k == n)
			return 1;

		return binomialCoeff(n - 1, k - 1) +
			binomialCoeff(n - 1, k);
	}
	Eigen :: ArrayXXf diff(Eigen :: ArrayXXf arr)
	{
	    Eigen :: ArrayXXf temp(arr.rows() - 1, 1);
	    for (int i = 0; i < arr.rows() - 1; i++)
	    {
		temp(i) = arr(i + 1) - arr(i);
	    }
	    return temp;
	}
	Eigen :: ArrayXXf maximum(float val, Eigen :: ArrayXXf arr2)
	{
		Eigen :: ArrayXXf temp(arr2.rows(), arr2.cols());
		temp = val;

		int k = 0;
		for (int i = 0; i < arr2.cols(); i++)
		{
			for (int j = 0; j < arr2.rows(); j++)
			{
				if (arr2(k) > val)
					temp(k) = arr2(k);
				k++;
			}
		}
		return temp;
	}
	Eigen :: ArrayXXf arctan2(Eigen :: ArrayXXf arr1, Eigen :: ArrayXXf arr2)
	{
		Eigen :: ArrayXXf temp(arr1.rows(), arr1.cols());

		int k = 0;
		for (int i = 0; i < arr1.cols(); i++)
		{
			for (int j = 0; j < arr1.rows(); j++)
			{
				temp(k) = atan2(arr1(k), arr2(k));
				k++;
			}
		}
		return temp;
	}
	Eigen :: ArrayXXf reshape(Eigen :: ArrayXXf x, uint32_t r, uint32_t c)
	{
		// starts with columns 
		Eigen :: Map<Eigen :: ArrayXXf> rx(x.data(), r, c);
		return rx;
	}
    Eigen :: ArrayXXf stack(Eigen :: ArrayXXf arr1, Eigen :: ArrayXXf arr2, char ch)
	{
		if (ch == 'v')
		{
			Eigen :: ArrayXXf temp(arr1.rows() + arr2.rows(), arr1.cols());
			temp << arr1, arr2;
			return temp;
		}
		else
		{
			Eigen :: ArrayXXf temp(arr1.rows(), arr1.cols() + arr2.cols());
			temp << arr1, arr2;
			return temp;
		}

	}
	Eigen :: ArrayXXf delete_values(float val, Eigen :: ArrayXXf arr)
	{
		Eigen :: ArrayXXf temp(arr.rows(), 1);

		int k = 0;
		for(int i = 0; i < arr.rows(); i++)
		{
			if(arr(i) != val){
				temp(k) = arr(i);
				k++;
			}
		}
		temp.conservativeResize(k, 1);
		return temp;
	}
    five_var bernsteinCoeffOrder10(float n, float tmin, float tmax, Eigen :: ArrayXXf t_actual, int num)
	{
		five_var s;
		float l = tmax - tmin;
		Eigen :: ArrayXXf t = (t_actual - tmin) / l;

		Eigen :: ArrayXXf P(num, (int)n + 1), Pdot(num, (int)n + 1), Pddot(num, (int)n + 1), Pdddot(num, (int)n + 1), Pddddot(num, (int)n + 1);

		P.col(0) = binomialCoeff(n, 0) * pow(1 - t, n - 0) * pow(t, 0);
		P.col(1) = binomialCoeff(n, 1) * pow(1 - t, n - 1) * pow(t, 1);
		P.col(2) = binomialCoeff(n, 2) * pow(1 - t, n - 2) * pow(t, 2);
		P.col(3) = binomialCoeff(n, 3) * pow(1 - t, n - 3) * pow(t, 3);
		P.col(4) = binomialCoeff(n, 4) * pow(1 - t, n - 4) * pow(t, 4);
		P.col(5) = binomialCoeff(n, 5) * pow(1 - t, n - 5) * pow(t, 5);
		P.col(6) = binomialCoeff(n, 6) * pow(1 - t, n - 6) * pow(t, 6);
		P.col(7) = binomialCoeff(n, 7) * pow(1 - t, n - 7) * pow(t, 7);
		P.col(8) = binomialCoeff(n, 8) * pow(1 - t, n - 8) * pow(t, 8);
		P.col(9) = binomialCoeff(n, 9) * pow(1 - t, n - 9) * pow(t, 9);
		P.col(10) = binomialCoeff(n, 10) * pow(1 - t, n - 10) * pow(t, 10);

		Pdot.col(0) = -10.0 * pow(-t + 1, 9);
		Pdot.col(1) = -90.0 * t * pow(-t + 1, 8) + 10.0 * pow(-t + 1, 9);
		Pdot.col(2) = -360.0 * pow(t, 2) * pow(-t + 1, 7) + 90.0 * t * pow(-t + 1, 8);
		Pdot.col(3) = -840.0 * pow(t, 3) * pow(-t + 1, 6) + 360.0 * pow(t, 2) * pow(-t + 1, 7);
		Pdot.col(4) = -1260.0 * pow(t, 4) * pow(-t + 1, 5) + 840.0 * pow(t, 3) * pow(-t + 1, 6);
		Pdot.col(5) = -1260.0 * pow(t, 5) * pow(-t + 1, 4) + 1260.0 * pow(t, 4) * pow(-t + 1, 5);
		Pdot.col(6) = -840.0 * pow(t, 6) * pow(-t + 1, 3) + 1260.0 * pow(t, 5) * pow(-t + 1, 4);
		Pdot.col(7) = -360.0 * pow(t, 7) * pow(-t + 1, 2) + 840.0 * pow(t, 6) * pow(-t + 1, 3);
		Pdot.col(8) = 45.0 * pow(t, 8) * (2 * t - 2) + 360.0 * pow(t, 7) * pow(-t + 1, 2);
		Pdot.col(9) = -10.0 * pow(t, 9) + 9 * pow(t, 8) * (-10.0 * t + 10.0);
		Pdot.col(10) = 10.0 * pow(t, 9);

		Pddot.col(0) = 90.0 * pow(-t + 1, 8.0);
		Pddot.col(1) = 720.0 * t * pow(-t + 1, 7) - 180.0 * pow(-t + 1, 8);
		Pddot.col(2) = 2520.0 * pow(t, 2) * pow(-t + 1, 6) - 1440.0 * t * pow(-t + 1, 7) + 90.0 * pow(-t + 1, 8);
		Pddot.col(3) = 5040.0 * pow(t, 3) * pow(-t + 1, 5) - 5040.0 * pow(t, 2) * pow(-t + 1, 6) + 720.0 * t * pow(-t + 1, 7);
		Pddot.col(4) = 6300.0 * pow(t, 4) * pow(-t + 1, 4) - 10080.0 * pow(t, 3) * pow(-t + 1, 5) + 2520.0 * pow(t, 2) * pow(-t + 1, 6);
		Pddot.col(5) = 5040.0 * pow(t, 5) * pow(-t + 1, 3) - 12600.0 * pow(t, 4) * pow(-t + 1, 4) + 5040.0 * pow(t, 3) * pow(-t + 1, 5);
		Pddot.col(6) = 2520.0 * pow(t, 6) * pow(-t + 1, 2) - 10080.0 * pow(t, 5) * pow(-t + 1, 3) + 6300.0 * pow(t, 4) * pow(-t + 1, 4);
		Pddot.col(7) = -360.0 * pow(t, 7) * (2 * t - 2) - 5040.0 * pow(t, 6) * pow(-t + 1, 2) + 5040.0 * pow(t, 5) * pow(-t + 1, 3);
		Pddot.col(8) = 90.0 * pow(t, 8) + 720.0 * pow(t, 7) * (2 * t - 2) + 2520.0 * pow(t, 6) * pow(-t + 1, 2);
		Pddot.col(9) = -180.0 * pow(t, 8) + 72 * pow(t, 7) * (-10.0 * t + 10.0);
		Pddot.col(10) = 90.0 * pow(t, 8);

		Pdddot.col(0) = -720.0 * pow(-t + 1, 7);
		Pdddot.col(1) = 2160 * pow(1 - t, 7) - 5040 * pow(1 - t, 6) * t;
		Pdddot.col(2) = -15120 * pow(1-t, 5) * pow(t, 2) + 15120 * pow(1 - t, 6) * t - 2160 * pow(1 - t, 7);
		Pdddot.col(3) = -720 * pow(t-1, 4) * (120 * pow(t, 3) - 108 * pow(t, 2) + 24 * t - 1);
		Pdddot.col(4) = 5040 * pow(t-1, 3) * t * (30 * pow(t, 3) - 36*pow(t, 2) + 12 * t - 1);
		Pdddot.col(5) = -15120 * pow(t-1, 2) * pow(t, 2) * (12 * pow(t, 3) - 18*pow(t, 2) + 8 * t - 1);
		Pdddot.col(6) = 5040 * (t - 1) * pow(t, 3) * (30 * pow(t, 3)-54 * pow(t, 2) + 30 * t - 5);
		Pdddot.col(7) = -720 * pow(t, 7) + 10080 * (1-t) * pow(t, 6) - 45360 * pow(1 - t, 2) * pow(t, 5) + 25200 * pow(1-t, 3) * pow(t, 4) - 2520 * pow(t, 6) * (2 * t - 2);
		Pdddot.col(8) = 2160 * pow(t, 7) - 5040 * (1 - t) * pow(t, 6) + 15120 * pow(1-t, 2) * pow(t, 5) + 5040 * pow(t, 6) * (2 * t - 2);
		Pdddot.col(9) = 504 * (10 - 10 * t) * pow(t, 6) - 2160 * pow(t, 7);
		Pdddot.col(10) = 720.0 * pow(t, 7);

		Pddddot.col(0) = 5040.0 * pow(-t +1, 6);
		Pddddot.col(1) = -4320 * pow(t - 1, 5) * (10 * t - 3)-7200 * pow(t - 1, 6);
		Pddddot.col(2) = 10800 * pow(t - 1, 4) * (15 * pow(t, 2) - 9 * t + 1) + 2160 * pow(t-1, 5) * (30 * t - 9);
		Pddddot.col(3) = -20160 * pow(t-1, 3) * (30 * pow(t, 3) - 36 * pow(t, 2) + 12 * t - 1);
		Pddddot.col(4) = 5040 * pow(t-1, 2) * (210 * pow(t, 4) - 336 * pow(t, 3) + 168 * pow(t, 2) - 28 * t + 1);
		Pddddot.col(5) = -30240 * (t - 1) * t * (42 * pow(t, 4) - 84 * pow(t, 3) + 56 * pow(t, 2) - 14 * t + 1);
		Pddddot.col(6) = 1058400 * pow(t, 6) - 2540160 * pow(t, 5) + 2116800 * pow(t, 4) - 705600 * pow(t, 3) + 75600 * pow(t, 2);
		Pddddot.col(7) = -604800 * pow(t, 6) + 1088640 * pow(t, 5) - 604800 * pow(t, 4) + 100800 * pow(t, 3);
		Pddddot.col(8) = 226800 * pow(t, 6) - 272160 * pow(t, 5) + 75600 * pow(t, 4);
		Pddddot.col(9) = 30240 * pow(t, 5) - 50400 * pow(t, 6);
		Pddddot.col(10) = 5040.0 * pow(t, 6);

		s.a = P;
		s.b = Pdot / l;
		s.c = Pddot / (l * l);
		s.d = Pdddot / (l * l * l);
		s.e = Pddddot / (l * l * l * l);

		return s;
	}
    five_var computeBernstein(Eigen :: ArrayXXf tot_time, float t_fin, int num)
	{
		five_var PPP;
		PPP = bernsteinCoeffOrder10(10.0, tot_time(0), t_fin, tot_time, num);
		return PPP;
	}

	void initAlpha(probData &prob_data, int VERBOSE){
		Eigen :: ArrayXXf ws_alpha_static_obs, wc_alpha_static_obs,
			 			  ws_alpha_drone, wc_alpha_drone,
						  ws_alpha_vel, wc_alpha_vel,
						  ws_alpha_acc, wc_alpha_acc,
						  ws_alpha_jerk, wc_alpha_jerk,
						  ws_alpha_snap, wc_alpha_snap;
		Eigen :: ArrayXXf c1_d, c2_d;

		// @ Static Obstacles
		if(prob_data.num_static_obs!=0){
			ws_alpha_static_obs = (-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0);
			wc_alpha_static_obs = (-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0);
			prob_data.alpha_static_obs = arctan2(ws_alpha_static_obs * prob_data.a_static_obs, wc_alpha_static_obs * prob_data.b_static_obs);

			c1_d = pow(prob_data.a_static_obs, 2) * pow(cos(prob_data.alpha_static_obs), 2) + pow(prob_data.b_static_obs, 2) * pow(sin(prob_data.alpha_static_obs), 2);
			c2_d = prob_data.a_static_obs * wc_alpha_static_obs * cos(prob_data.alpha_static_obs) + prob_data.b_static_obs * ws_alpha_static_obs * sin(prob_data.alpha_static_obs);

			// prob_data.d_static_obs = (c2_d/c1_d).max(1.0);
			if(prob_data.d_static_obs_old.rows() != prob_data.num_static_obs){
				prob_data.d_static_obs_old = Eigen :: ArrayXXf :: Ones(prob_data.num_static_obs, prob_data.num);
			}
			else{
				prob_data.d_static_obs_old.rightCols(prob_data.num - 1) = prob_data.d_static_obs_old.leftCols(prob_data.num - 1);
				prob_data.d_static_obs_old.col(0) = 1.0; 
			}

			prob_data.d_static_obs = (c2_d/c1_d).max(1.0 + (1.0 - prob_data.gamma) * (prob_data.d_static_obs_old - 1.0));
			prob_data.d_static_obs_old = prob_data.d_static_obs;
			
		}

		// @ Inter-Agent Collision Avoidance
		if(prob_data.num_drone!=0){
			ws_alpha_drone = (-prob_data.y_drone).rowwise() + prob_data.y.transpose().row(0);
			wc_alpha_drone = (-prob_data.x_drone).rowwise() + prob_data.x.transpose().row(0);
			prob_data.alpha_drone = arctan2(ws_alpha_drone * prob_data.a_drone, wc_alpha_drone * prob_data.b_drone);

			c1_d = pow(prob_data.a_drone, 2) * pow(cos(prob_data.alpha_drone), 2) + pow(prob_data.b_drone, 2) * pow(sin(prob_data.alpha_drone), 2);
			c2_d = prob_data.a_drone * wc_alpha_drone * cos(prob_data.alpha_drone) + prob_data.b_drone * ws_alpha_drone * sin(prob_data.alpha_drone);
		
			// prob_data.d_drone = (c2_d/c1_d).max(1.0);
			if(prob_data.d_drone_old.rows() != prob_data.num_drone){
				prob_data.d_drone_old = Eigen :: ArrayXXf :: Ones(prob_data.num_drone, prob_data.num);
			}
			else{
				prob_data.d_drone_old.rightCols(prob_data.num - 1) = prob_data.d_drone_old.leftCols(prob_data.num - 1);
				prob_data.d_drone_old.col(0) = 1.0; 
			}

			prob_data.d_drone = (c2_d/c1_d).max(1.0 + (1.0 - prob_data.gamma) * (prob_data.d_drone_old - 1.0));	
			prob_data.d_drone_old = prob_data.d_drone;
		}
		
		if(!prob_data.axis_wise){
			// @ Velocity Constraints
			wc_alpha_vel = prob_data.xdot;				
			ws_alpha_vel = prob_data.ydot;
			prob_data.alpha_vel = arctan2(ws_alpha_vel, wc_alpha_vel);
			c1_d = pow(cos(prob_data.alpha_vel), 2) + pow(sin(prob_data.alpha_vel), 2);
			c2_d = wc_alpha_vel*cos(prob_data.alpha_vel) + ws_alpha_vel*sin(prob_data.alpha_vel);
			
			prob_data.d_vel = (c2_d/c1_d).min(prob_data.vel_max);
		
			// @ Acceleration Constraints
			wc_alpha_acc = prob_data.xddot;				
			ws_alpha_acc = prob_data.yddot;
			prob_data.alpha_acc = arctan2(ws_alpha_acc, wc_alpha_acc);
			c1_d = pow(cos(prob_data.alpha_acc), 2) + pow(sin(prob_data.alpha_acc), 2);
			c2_d = wc_alpha_acc*cos(prob_data.alpha_acc) + ws_alpha_acc*sin(prob_data.alpha_acc);

			prob_data.d_acc = (c2_d/c1_d).min(prob_data.acc_max);
			
			if(prob_data.jerk_snap_constraints){
				// @ Jerk Constraints
				wc_alpha_jerk = prob_data.xdddot;				
				ws_alpha_jerk = prob_data.ydddot;
				prob_data.alpha_jerk = arctan2(ws_alpha_jerk, wc_alpha_jerk);
				c1_d = pow(cos(prob_data.alpha_jerk), 2) + pow(sin(prob_data.alpha_jerk), 2);
				c2_d = wc_alpha_jerk*cos(prob_data.alpha_jerk) + ws_alpha_jerk*sin(prob_data.alpha_jerk);

				prob_data.d_jerk = (c2_d/c1_d).min(prob_data.jerk_max);

				// @ Snap Constraints
				wc_alpha_snap = prob_data.xddddot;				
				ws_alpha_snap = prob_data.yddddot;
				prob_data.alpha_snap = arctan2(ws_alpha_snap, wc_alpha_snap);
				c1_d = pow(cos(prob_data.alpha_snap), 2) + pow(sin(prob_data.alpha_snap), 2);
				c2_d = wc_alpha_snap*cos(prob_data.alpha_snap) + ws_alpha_snap*sin(prob_data.alpha_snap);
				
				prob_data.d_snap = (c2_d/c1_d).min(prob_data.snap_max);
			}
		}
	}

	void computeXYAxis(probData &prob_data, int VERBOSE)
	{
							  
		Eigen :: ArrayXXf cost_xy, cost_xy_inv;
		Eigen :: ArrayXXf objective_xy, lincost_x, lincost_y;
		Eigen :: ArrayXXf cost_drone, cost_static_obs;
		Eigen :: ArrayXXf temp_x_static_obs, temp_y_static_obs;
		Eigen :: ArrayXXf temp_x_drone, temp_y_drone;
		Eigen :: ArrayXXf sol_x, sol_y, primal_sol_x, primal_sol_y;
		Eigen :: ArrayXXf res_x_static_obs, res_y_static_obs,
						  res_x_drone, res_y_drone,
						  res_vx_ineq, res_vy_ineq,
						  res_ax_ineq, res_ay_ineq,
						  res_jx_ineq, res_jy_ineq,
						  res_sx_ineq, res_sy_ineq,
						  res_x_ineq, res_y_ineq;
		
		// @ Set Initial Conidtions for next MPC step
		prob_data.b_x_eq << prob_data.x_init, prob_data.vx_init, prob_data.ax_init;
		prob_data.b_y_eq << prob_data.y_init, prob_data.vy_init, prob_data.ay_init;

		float thresold = prob_data.thresold;

		prob_data.rho_static_obs = 1.0;
		prob_data.rho_drone = 1.0;
		prob_data.rho_vel = 1.0;
		prob_data.rho_acc = 1.0;
		prob_data.rho_jerk = 1.0;
		prob_data.rho_snap = 1.0;
		prob_data.rho_ineq = 1.0;
		
		
		 // @ Lagrange Multiplier
		prob_data.lamda_x = Eigen :: ArrayXXf :: Ones(prob_data.nvar, 1) *0;
		prob_data.lamda_y = Eigen :: ArrayXXf :: Ones(prob_data.nvar, 1) *0;
		
		// @ Position Constraints
		prob_data.s_x_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_y_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
	
		// @ Velocity Constraints
		prob_data.s_vx_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_vy_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		
		// @ Acceleration Constraints
		prob_data.s_ax_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_ay_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
	
		// @ Jerk Constraints
		prob_data.s_jx_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_jy_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		
		// @ Snap Constraints
		prob_data.s_sx_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_sy_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		

		if(prob_data.num_drone!=0)cost_drone = prob_data.A_drone.transpose().matrix() * prob_data.A_drone.matrix();
		if(prob_data.num_static_obs!=0)cost_static_obs = prob_data.A_static_obs.transpose().matrix() * prob_data.A_static_obs.matrix();
		
		int break_flag;
		for(int i = 0; i < prob_data.max_iter; i++){

			break_flag = 0;
			
			prob_data.B_x_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_x_ineq - prob_data.s_x_ineq).matrix();
			prob_data.B_y_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_y_ineq - prob_data.s_y_ineq).matrix();

			prob_data.B_vx_ineq = prob_data.A_v_ineq.transpose().matrix() * (prob_data.b_vx_ineq - prob_data.s_vx_ineq).matrix();
			prob_data.B_vy_ineq = prob_data.A_v_ineq.transpose().matrix() * (prob_data.b_vy_ineq - prob_data.s_vy_ineq).matrix();
		
			prob_data.B_ax_ineq = prob_data.A_a_ineq.transpose().matrix() * (prob_data.b_ax_ineq - prob_data.s_ax_ineq).matrix();
			prob_data.B_ay_ineq = prob_data.A_a_ineq.transpose().matrix() * (prob_data.b_ay_ineq - prob_data.s_ay_ineq).matrix();
			
			objective_xy = prob_data.weight_goal * prob_data.cost_goal 
					+ prob_data.weight_smoothness * prob_data.cost_smoothness 
					+ prob_data.rho_vel * prob_data.cost_vel 
					+ prob_data.rho_acc * prob_data.cost_acc 
					+ prob_data.rho_ineq * prob_data.cost_ineq;


			lincost_x = -prob_data.lamda_x
						-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.x_ref.matrix()).array()
						-prob_data.rho_vel * prob_data.B_vx_ineq
						-prob_data.rho_acc * prob_data.B_ax_ineq
						-prob_data.rho_ineq * prob_data.B_x_ineq;

			lincost_y = -prob_data.lamda_y
						-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.y_ref.matrix()).array()
						-prob_data.rho_vel * prob_data.B_vy_ineq
						-prob_data.rho_acc * prob_data.B_ay_ineq
						-prob_data.rho_ineq * prob_data.B_y_ineq;
			
		
			if(prob_data.jerk_snap_constraints){
				prob_data.B_jx_ineq = prob_data.A_j_ineq.transpose().matrix() * (prob_data.b_jx_ineq - prob_data.s_jx_ineq).matrix();
				prob_data.B_jy_ineq = prob_data.A_j_ineq.transpose().matrix() * (prob_data.b_jy_ineq - prob_data.s_jy_ineq).matrix();

				prob_data.B_sx_ineq = prob_data.A_s_ineq.transpose().matrix() * (prob_data.b_sx_ineq - prob_data.s_sx_ineq).matrix();
				prob_data.B_sy_ineq = prob_data.A_s_ineq.transpose().matrix() * (prob_data.b_sy_ineq - prob_data.s_sy_ineq).matrix();
			
				objective_xy += prob_data.rho_jerk * prob_data.cost_jerk
					+ prob_data.rho_snap * prob_data.cost_snap;
				

				lincost_x += -prob_data.rho_jerk * prob_data.B_jx_ineq 
						-prob_data.rho_snap * prob_data.B_sx_ineq;
				lincost_y += -prob_data.rho_jerk * prob_data.B_jy_ineq
						-prob_data.rho_snap * prob_data.B_sy_ineq;
			}
			
			// @ Check for obstacles
			if(prob_data.num_static_obs!=0){
				temp_x_static_obs =  prob_data.x_static_obs + prob_data.d_static_obs * cos(prob_data.alpha_static_obs) * prob_data.a_static_obs;
				temp_y_static_obs =  prob_data.y_static_obs + prob_data.d_static_obs * sin(prob_data.alpha_static_obs) * prob_data.b_static_obs;

				prob_data.b_x_static_obs = Optim :: reshape(temp_x_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
				prob_data.b_y_static_obs = Optim :: reshape(temp_y_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
			
				objective_xy += prob_data.rho_static_obs * cost_static_obs;
				lincost_x += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_x_static_obs.matrix()).array();
				lincost_y += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_y_static_obs.matrix()).array();

			}
			
			if(prob_data.num_drone!=0){
				
				temp_x_drone =  prob_data.x_drone + prob_data.d_drone * cos(prob_data.alpha_drone) * prob_data.a_drone;
				temp_y_drone =  prob_data.y_drone + prob_data.d_drone * sin(prob_data.alpha_drone) * prob_data.b_drone;

				prob_data.b_x_drone = Optim :: reshape(temp_x_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
				prob_data.b_y_drone = Optim :: reshape(temp_y_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
			
				objective_xy += prob_data.rho_drone * cost_drone;
				
				lincost_x += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_x_drone.matrix()).array();
				lincost_y += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_y_drone.matrix()).array();			
			}

			// @ Solve set of linear equations
			cost_xy = Optim :: stack(stack(objective_xy, prob_data.A_eq.transpose(), 'h'), 
							   stack(prob_data.A_eq, Eigen::ArrayXXf::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
			cost_xy_inv = (cost_xy.matrix()).householderQr().solve(prob_data.I.matrix());

			
			sol_x = cost_xy_inv.matrix() * stack(-lincost_x, prob_data.b_x_eq, 'v').matrix();
			sol_y = cost_xy_inv.matrix() * stack(-lincost_y, prob_data.b_y_eq, 'v').matrix(); 

			primal_sol_x = sol_x.topRows(prob_data.nvar);
			primal_sol_y = sol_y.topRows(prob_data.nvar);

			prob_data.x = prob_data.P.matrix() * primal_sol_x.matrix();
			prob_data.xdot = prob_data.Pdot.matrix() * primal_sol_x.matrix();
			prob_data.xddot = prob_data.Pddot.matrix() * primal_sol_x.matrix();
			prob_data.xdddot = prob_data.Pdddot.matrix() * primal_sol_x.matrix();
			prob_data.xddddot = prob_data.Pddddot.matrix() * primal_sol_x.matrix();

			prob_data.y = prob_data.P.matrix() * primal_sol_y.matrix();
			prob_data.ydot = prob_data.Pdot.matrix() * primal_sol_y.matrix();
			prob_data.yddot = prob_data.Pddot.matrix() * primal_sol_y.matrix();
			prob_data.ydddot = prob_data.Pdddot.matrix() * primal_sol_y.matrix();
			prob_data.yddddot = prob_data.Pddddot.matrix() * primal_sol_y.matrix();	

			prob_data.z = prob_data.z_init * Eigen :: ArrayXXf :: Ones(prob_data.num, 1);
			prob_data.zdot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
			prob_data.zddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
			prob_data.zdddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
			prob_data.zddddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);

			//////
			prob_data.x_up = prob_data.P_up.matrix() * primal_sol_x.matrix();
			prob_data.y_up = prob_data.P_up.matrix() * primal_sol_y.matrix();
			prob_data.z_up = prob_data.z_init * Eigen :: ArrayXXf :: Ones(prob_data.num, 1);

			prob_data.xdot_up = prob_data.Pdot_up.matrix() * primal_sol_x.matrix();
			prob_data.ydot_up = prob_data.Pdot_up.matrix() * primal_sol_y.matrix();
			prob_data.zdot_up = Eigen :: ArrayXXf :: Zero(prob_data.num_up, 1);

			prob_data.xddot_up = prob_data.Pddot_up.matrix() * primal_sol_x.matrix();
			prob_data.yddot_up = prob_data.Pddot_up.matrix() * primal_sol_y.matrix();
			prob_data.zddot_up = Eigen :: ArrayXXf :: Zero(prob_data.num_up, 1);
			/////
			
			
			// @ Residual and Lagrange Update
			initAlpha(prob_data, VERBOSE);

			// Position
			prob_data.s_x_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_x_ineq).max(0.0);
			prob_data.s_y_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_y_ineq).max(0.0);

			res_x_ineq = (prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_x_ineq + prob_data.s_x_ineq;
			res_y_ineq = (prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_y_ineq + prob_data.s_y_ineq;


			// Velocity
			prob_data.s_vx_ineq = ((-prob_data.A_v_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_vx_ineq).max(0.0);
			prob_data.s_vy_ineq = ((-prob_data.A_v_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_vy_ineq).max(0.0);
			
			res_vx_ineq = (prob_data.A_v_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_vx_ineq + prob_data.s_vx_ineq;
			res_vy_ineq = (prob_data.A_v_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_vy_ineq + prob_data.s_vy_ineq;
			

			// Acceleration
			prob_data.s_ax_ineq = ((-prob_data.A_a_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_ax_ineq).max(0.0);
			prob_data.s_ay_ineq = ((-prob_data.A_a_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_ay_ineq).max(0.0);
			
			
			res_ax_ineq = (prob_data.A_a_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_ax_ineq + prob_data.s_ax_ineq;
			res_ay_ineq = (prob_data.A_a_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_ay_ineq + prob_data.s_ay_ineq;
			

			prob_data.lamda_x = prob_data.lamda_x 
					-prob_data.rho_vel * (prob_data.A_v_ineq.transpose().matrix() * res_vx_ineq.matrix()).array()
					-prob_data.rho_acc * (prob_data.A_a_ineq.transpose().matrix() * res_ax_ineq.matrix()).array()
					-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_x_ineq.matrix()).array();


			prob_data.lamda_y = prob_data.lamda_y 
					-prob_data.rho_vel * (prob_data.A_v_ineq.transpose().matrix() * res_vy_ineq.matrix()).array()
					-prob_data.rho_acc * (prob_data.A_a_ineq.transpose().matrix() * res_ay_ineq.matrix()).array()
					-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_y_ineq.matrix()).array();
			
			

			if(prob_data.jerk_snap_constraints){
				// Jerk
				prob_data.s_jx_ineq = ((-prob_data.A_j_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_jx_ineq).max(0.0);
				prob_data.s_jy_ineq = ((-prob_data.A_j_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_jy_ineq).max(0.0);
				
				res_jx_ineq = (prob_data.A_j_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_jx_ineq + prob_data.s_jx_ineq;
				res_jy_ineq = (prob_data.A_j_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_jy_ineq + prob_data.s_jy_ineq;

				// Snap
				prob_data.s_sx_ineq = ((-prob_data.A_s_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_sx_ineq).max(0.0);
				prob_data.s_sy_ineq = ((-prob_data.A_s_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_sy_ineq).max(0.0);
				

				res_sx_ineq = (prob_data.A_s_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_sx_ineq + prob_data.s_sx_ineq;
				res_sy_ineq = (prob_data.A_s_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_sy_ineq + prob_data.s_sy_ineq;
				

				prob_data.lamda_x = prob_data.lamda_x 
								-prob_data.rho_jerk * (prob_data.A_j_ineq.transpose().matrix() * res_jx_ineq.matrix()).array()
								-prob_data.rho_snap * (prob_data.A_s_ineq.transpose().matrix() * res_sx_ineq.matrix()).array();
				
				prob_data.lamda_y = prob_data.lamda_y 
								-prob_data.rho_jerk * (prob_data.A_j_ineq.transpose().matrix() * res_jy_ineq.matrix()).array()
								-prob_data.rho_snap * (prob_data.A_s_ineq.transpose().matrix() * res_sy_ineq.matrix()).array();


				prob_data.res_x_jerk_norm = res_jx_ineq.matrix().norm();
				prob_data.res_y_jerk_norm = res_jy_ineq.matrix().norm();
				

				prob_data.res_x_snap_norm = res_sx_ineq.matrix().norm();
				prob_data.res_y_snap_norm = res_sy_ineq.matrix().norm();
				
				if(prob_data.res_x_jerk_norm > thresold || prob_data.res_y_jerk_norm > thresold){;
					prob_data.rho_jerk *= prob_data.delta_jerk;
					if(prob_data.rho_jerk > prob_data.rho_jerk_max) prob_data.rho_jerk = prob_data.rho_jerk_max;
				}
				else break_flag++;
				if(prob_data.res_x_snap_norm > thresold || prob_data.res_y_snap_norm > thresold){
						prob_data.rho_snap *= prob_data.delta_snap;
						if(prob_data.rho_snap > prob_data.rho_snap_max) prob_data.rho_snap = prob_data.rho_snap_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_jerk_norm = 0.0;
				prob_data.res_y_jerk_norm = 0.0;
				prob_data.res_z_jerk_norm = 0.0;

				prob_data.res_x_snap_norm = 0.0;
				prob_data.res_y_snap_norm = 0.0;
				prob_data.res_z_snap_norm = 0.0;

				break_flag += 2;
			}

			if(prob_data.num_static_obs!=0){
				
				res_x_static_obs = reshape(((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_static_obs * prob_data.d_static_obs * cos(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
				res_y_static_obs = reshape(((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_static_obs * prob_data.d_static_obs * sin(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
			
				prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_x_static_obs.matrix()).array();
				prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_y_static_obs.matrix()).array();
			
				prob_data.res_x_static_obs_norm = res_x_static_obs.matrix().norm();
				prob_data.res_y_static_obs_norm = res_y_static_obs.matrix().norm();
			
				if(prob_data.res_x_static_obs_norm > thresold || prob_data.res_y_static_obs_norm > thresold){
					prob_data.rho_static_obs *= prob_data.delta_static_obs;
					if(prob_data.rho_static_obs > prob_data.rho_static_obs_max) prob_data.rho_static_obs = prob_data.rho_static_obs_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_static_obs_norm = 0;
				prob_data.res_y_static_obs_norm = 0;
				break_flag++;
			}

			if(prob_data.num_drone!=0){
				
				res_x_drone = reshape(((-prob_data.x_drone).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_drone * prob_data.d_drone * cos(prob_data.alpha_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
				res_y_drone = reshape(((-prob_data.y_drone).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_drone * prob_data.d_drone * sin(prob_data.alpha_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
				
			
				prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_x_drone.matrix()).array();
				prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_y_drone.matrix()).array();
			
				prob_data.res_x_drone_norm = res_x_drone.matrix().norm();
				prob_data.res_y_drone_norm = res_y_drone.matrix().norm();

				if(prob_data.res_x_drone_norm > thresold || prob_data.res_y_drone_norm > thresold){
					prob_data.rho_drone *= prob_data.delta_drone;
					if(prob_data.rho_drone > prob_data.rho_drone_max) prob_data.rho_drone = prob_data.rho_drone_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_drone_norm = 0;
				prob_data.res_y_drone_norm = 0;
				prob_data.res_z_drone_norm = 0;
				break_flag++;
			}
			
			prob_data.res_x_ineq_norm = res_x_ineq.matrix().norm();
			prob_data.res_y_ineq_norm = res_y_ineq.matrix().norm();

			prob_data.res_x_vel_norm = res_vx_ineq.matrix().norm();
			prob_data.res_y_vel_norm = res_vy_ineq.matrix().norm();
			

			prob_data.res_x_acc_norm = res_ax_ineq.matrix().norm();
			prob_data.res_y_acc_norm = res_ay_ineq.matrix().norm();
			
			
			
			if(prob_data.res_x_ineq_norm > thresold || prob_data.res_y_ineq_norm > thresold){;
					prob_data.rho_ineq *= prob_data.delta_ineq;
					if(prob_data.rho_ineq > prob_data.rho_ineq_max) prob_data.rho_ineq = prob_data.rho_ineq_max;
			}
			else break_flag++;
			if(prob_data.res_x_vel_norm > thresold || prob_data.res_y_vel_norm > thresold){;
					prob_data.rho_vel *= prob_data.delta_vel;
					if(prob_data.rho_vel > prob_data.rho_vel_max) prob_data.rho_vel = prob_data.rho_vel_max;
			}
			else break_flag++;
			if(prob_data.res_x_acc_norm > thresold || prob_data.res_y_acc_norm > thresold){;
					prob_data.rho_acc *= prob_data.delta_acc;
					if(prob_data.rho_acc > prob_data.rho_acc_max) prob_data.rho_acc = prob_data.rho_acc_max;
			}
			else break_flag++;
			

			if(break_flag == 7)
				break;	
		}
		
		if(break_flag != 7){
				prob_data.weight_goal *= prob_data.delta_aggressive;
				prob_data.weight_smoothness *= prob_data.delta_aggressive;
			}
		else{
			prob_data.weight_goal = prob_data.weight_goal_og;
			prob_data.weight_smoothness = prob_data.weight_smoothness_og;

			if(prob_data.weight_goal > prob_data.weight_goal_og)
				prob_data.weight_goal = prob_data.weight_goal_og;

			if(prob_data.weight_goal > prob_data.weight_smoothness_og)
				prob_data.weight_smoothness = prob_data.weight_smoothness_og;
		}
	}

	void computeXY(probData &prob_data, int VERBOSE)
	{
							  
		Eigen :: ArrayXXf cost_xy, cost_xy_inv;
		Eigen :: ArrayXXf objective_xy, lincost_x, lincost_y;
		Eigen :: ArrayXXf cost_drone, cost_static_obs;
		Eigen :: ArrayXXf temp_x_static_obs, temp_y_static_obs;
		Eigen :: ArrayXXf temp_x_drone, temp_y_drone;
		Eigen :: ArrayXXf sol_x, sol_y, primal_sol_x, primal_sol_y;
		Eigen :: ArrayXXf res_x_static_obs, res_y_static_obs,
						  res_x_drone, res_y_drone,
						  res_x_vel, res_y_vel,
						  res_x_acc, res_y_acc,
						  res_x_jerk, res_y_jerk,
						  res_x_snap, res_y_snap,
						  res_x_ineq, res_y_ineq;
		
		// @ Set Initial Conidtions for next MPC step
		prob_data.b_x_eq << prob_data.x_init, prob_data.vx_init, prob_data.ax_init;
		prob_data.b_y_eq << prob_data.y_init, prob_data.vy_init, prob_data.ay_init;

		float thresold = prob_data.thresold;

		prob_data.rho_static_obs = 1.0;
		prob_data.rho_drone = 1.0;
		prob_data.rho_vel = 1.0;
		prob_data.rho_acc = 1.0;
		prob_data.rho_jerk = 1.0;
		prob_data.rho_snap = 1.0;
		prob_data.rho_ineq = 1.0;
		
		
		// @ Lagrange Multiplier
		prob_data.lamda_x = Eigen :: ArrayXXf :: Ones(prob_data.nvar, 1) *0;
		prob_data.lamda_y = Eigen :: ArrayXXf :: Ones(prob_data.nvar, 1) *0;

		// @ Position Constraints
		prob_data.s_x_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_y_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
	
		
		if(prob_data.num_drone!=0)cost_drone = prob_data.A_drone.transpose().matrix() * prob_data.A_drone.matrix();
		if(prob_data.num_static_obs!=0)cost_static_obs = prob_data.A_static_obs.transpose().matrix() * prob_data.A_static_obs.matrix();
		
		
		int break_flag;
		for(int i = 0; i < prob_data.max_iter; i++){

			break_flag = 0;

			prob_data.b_x_vel = prob_data.d_vel * cos(prob_data.alpha_vel);
			prob_data.b_y_vel = prob_data.d_vel * sin(prob_data.alpha_vel);

			prob_data.b_x_acc = prob_data.d_acc * cos(prob_data.alpha_acc);
			prob_data.b_y_acc = prob_data.d_acc * sin(prob_data.alpha_acc);
			
			prob_data.B_x_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_x_ineq - prob_data.s_x_ineq).matrix();
			prob_data.B_y_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_y_ineq - prob_data.s_y_ineq).matrix();
			
			
			objective_xy = prob_data.weight_goal * prob_data.cost_goal 
					+ prob_data.weight_smoothness * prob_data.cost_smoothness 
					+ prob_data.rho_vel * prob_data.cost_vel 
					+ prob_data.rho_acc * prob_data.cost_acc 
					+ prob_data.rho_ineq * prob_data.cost_ineq;
			
			lincost_x = -prob_data.lamda_x
						-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.x_ref.matrix()).array()
						-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * prob_data.b_x_vel.matrix()).array()
						-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * prob_data.b_x_acc.matrix()).array()
						-prob_data.rho_ineq * prob_data.B_x_ineq;

			lincost_y = -prob_data.lamda_y
						-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.y_ref.matrix()).array()
						-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * prob_data.b_y_vel.matrix()).array()
						-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * prob_data.b_y_acc.matrix()).array()
						-prob_data.rho_ineq * prob_data.B_y_ineq;
			

			// jerk-snap
			if(prob_data.jerk_snap_constraints){
				prob_data.b_x_jerk = prob_data.d_jerk * cos(prob_data.alpha_jerk);
				prob_data.b_y_jerk = prob_data.d_jerk * sin(prob_data.alpha_jerk);

				prob_data.b_x_snap = prob_data.d_snap * cos(prob_data.alpha_snap);
				prob_data.b_y_snap = prob_data.d_snap * sin(prob_data.alpha_snap);
				
				objective_xy += prob_data.rho_jerk * prob_data.cost_jerk
					+ prob_data.rho_snap * prob_data.cost_snap;
				
				lincost_x += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * prob_data.b_x_jerk.matrix()).array()
						-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * prob_data.b_x_snap.matrix()).array();

				lincost_y += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * prob_data.b_y_jerk.matrix()).array()
						-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * prob_data.b_y_snap.matrix()).array();
			}			

			
			// @ Check for obstacles
			if(prob_data.num_static_obs!=0){
				temp_x_static_obs =  prob_data.x_static_obs + prob_data.d_static_obs * cos(prob_data.alpha_static_obs) * prob_data.a_static_obs;
				temp_y_static_obs =  prob_data.y_static_obs + prob_data.d_static_obs * sin(prob_data.alpha_static_obs) * prob_data.b_static_obs;

				prob_data.b_x_static_obs = Optim :: reshape(temp_x_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
				prob_data.b_y_static_obs = Optim :: reshape(temp_y_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
			
				objective_xy += prob_data.rho_static_obs * cost_static_obs;
				lincost_x += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_x_static_obs.matrix()).array();
				lincost_y += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_y_static_obs.matrix()).array();

			}
			
			// drones
			if(prob_data.num_drone!=0){
				
				temp_x_drone =  prob_data.x_drone + prob_data.d_drone * cos(prob_data.alpha_drone) * prob_data.a_drone;
				temp_y_drone =  prob_data.y_drone + prob_data.d_drone * sin(prob_data.alpha_drone) * prob_data.b_drone;

				prob_data.b_x_drone = Optim :: reshape(temp_x_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
				prob_data.b_y_drone = Optim :: reshape(temp_y_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
				
				objective_xy += prob_data.rho_drone * cost_drone;
				
				
				lincost_x += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_x_drone.matrix()).array();
				lincost_y += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_y_drone.matrix()).array();			
			}

			// @ Solve set of linear equations
			cost_xy = Optim :: stack(stack(objective_xy, prob_data.A_eq.transpose(), 'h'), 
							   stack(prob_data.A_eq, Eigen::ArrayXXf::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
			cost_xy_inv = (cost_xy.matrix()).householderQr().solve(prob_data.I.matrix());

			
			sol_x = cost_xy_inv.matrix() * stack(-lincost_x, prob_data.b_x_eq, 'v').matrix();
			sol_y = cost_xy_inv.matrix() * stack(-lincost_y, prob_data.b_y_eq, 'v').matrix(); 

			

			prob_data.z = prob_data.z_init * Eigen :: ArrayXXf :: Ones(prob_data.num, 1);
			prob_data.zdot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
			prob_data.zddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
			prob_data.zdddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
			prob_data.zddddot = Eigen :: ArrayXXf :: Zero(prob_data.num, 1);
						
			primal_sol_x = sol_x.topRows(prob_data.nvar);
			primal_sol_y = sol_y.topRows(prob_data.nvar);

			prob_data.x = prob_data.P.matrix() * primal_sol_x.matrix();
			prob_data.y = prob_data.P.matrix() * primal_sol_y.matrix();
			
			prob_data.xdot = prob_data.Pdot.matrix() * primal_sol_x.matrix();
			prob_data.ydot = prob_data.Pdot.matrix() * primal_sol_y.matrix();
			
			prob_data.xddot = prob_data.Pddot.matrix() * primal_sol_x.matrix();
			prob_data.yddot = prob_data.Pddot.matrix() * primal_sol_y.matrix();

			prob_data.xdddot = prob_data.Pdddot.matrix() * primal_sol_x.matrix();
			prob_data.ydddot = prob_data.Pdddot.matrix() * primal_sol_y.matrix();

			prob_data.xddddot = prob_data.Pddddot.matrix() * primal_sol_x.matrix();
			prob_data.yddddot = prob_data.Pddddot.matrix() * primal_sol_y.matrix();	
			
			prob_data.x_up = prob_data.P_up.matrix() * primal_sol_x.matrix();
			prob_data.y_up = prob_data.P_up.matrix() * primal_sol_y.matrix();
			prob_data.z_up = prob_data.z_init * Eigen :: ArrayXXf :: Ones(prob_data.num_up, 1);

			prob_data.xdot_up = prob_data.Pdot_up.matrix() * primal_sol_x.matrix();
			prob_data.ydot_up = prob_data.Pdot_up.matrix() * primal_sol_y.matrix();
			prob_data.zdot_up = Eigen :: ArrayXXf :: Zero(prob_data.num_up, 1);

			prob_data.xddot_up = prob_data.Pddot_up.matrix() * primal_sol_x.matrix();
			prob_data.yddot_up = prob_data.Pddot_up.matrix() * primal_sol_y.matrix();
			prob_data.zddot_up = Eigen :: ArrayXXf :: Zero(prob_data.num_up, 1);
			
			// @ Residual and Lagrange Update
			initAlpha(prob_data, VERBOSE);
			

			prob_data.s_x_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_x_ineq).max(0.0);
			prob_data.s_y_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_y_ineq).max(0.0);

			res_x_ineq = (prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_x_ineq + prob_data.s_x_ineq;
			res_y_ineq = (prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_y_ineq + prob_data.s_y_ineq;
			
			res_x_vel = prob_data.xdot - prob_data.d_vel * cos(prob_data.alpha_vel);
			res_y_vel = prob_data.ydot - prob_data.d_vel * sin(prob_data.alpha_vel);
			
			res_x_acc = prob_data.xddot - prob_data.d_acc * cos(prob_data.alpha_acc);
			res_y_acc = prob_data.yddot - prob_data.d_acc * sin(prob_data.alpha_acc);
			
			
		
			prob_data.lamda_x = prob_data.lamda_x 
						-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * res_x_vel.matrix()).array()
						-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * res_x_acc.matrix()).array()
						-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_x_ineq.matrix()).array();

			prob_data.lamda_y = prob_data.lamda_y 
						-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * res_y_vel.matrix()).array()
						-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * res_y_acc.matrix()).array()
						-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_y_ineq.matrix()).array();

			if(prob_data.jerk_snap_constraints){
				res_x_jerk = prob_data.xdddot - prob_data.d_jerk * cos(prob_data.alpha_jerk);
				res_y_jerk = prob_data.ydddot - prob_data.d_jerk * sin(prob_data.alpha_jerk);
				
				res_x_snap = prob_data.xddddot - prob_data.d_snap * cos(prob_data.alpha_snap);
				res_y_snap = prob_data.yddddot - prob_data.d_snap * sin(prob_data.alpha_snap);
				

				prob_data.lamda_x += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * res_x_jerk.matrix()).array()
						-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * res_x_snap.matrix()).array();

				prob_data.lamda_y += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * res_y_jerk.matrix()).array()
						-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * res_y_snap.matrix()).array();


				prob_data.res_x_jerk_norm = res_x_jerk.matrix().norm();
				prob_data.res_y_jerk_norm = res_y_jerk.matrix().norm();

				prob_data.res_x_snap_norm = res_x_snap.matrix().norm();
				prob_data.res_y_snap_norm = res_y_snap.matrix().norm();

				if(prob_data.res_x_jerk_norm > thresold || prob_data.res_y_jerk_norm > thresold){;
					prob_data.rho_jerk *= prob_data.delta_jerk;
					if(prob_data.rho_jerk > prob_data.rho_jerk_max) prob_data.rho_jerk = prob_data.rho_jerk_max;
				}
				else break_flag++;

				if(prob_data.res_x_snap_norm > thresold || prob_data.res_y_snap_norm > thresold){;
						prob_data.rho_snap *= prob_data.delta_snap;
						if(prob_data.rho_snap > prob_data.rho_snap_max) prob_data.rho_snap = prob_data.rho_snap_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_jerk_norm = 0.0;
				prob_data.res_y_jerk_norm = 0.0;
				prob_data.res_z_jerk_norm = 0.0;

				prob_data.res_x_snap_norm = 0.0;
				prob_data.res_y_snap_norm = 0.0;
				prob_data.res_z_snap_norm = 0.0;

				break_flag += 2;
			}


			if(prob_data.num_static_obs!=0){
				
				res_x_static_obs = reshape(((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_static_obs * prob_data.d_static_obs * cos(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
				res_y_static_obs = reshape(((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_static_obs * prob_data.d_static_obs * sin(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
			
				prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_x_static_obs.matrix()).array();
				prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_y_static_obs.matrix()).array();
			
				prob_data.res_x_static_obs_norm = res_x_static_obs.matrix().norm();
				prob_data.res_y_static_obs_norm = res_y_static_obs.matrix().norm();
			
				if(prob_data.res_x_static_obs_norm > thresold || prob_data.res_y_static_obs_norm > thresold){
					prob_data.rho_static_obs *= prob_data.delta_static_obs;
					if(prob_data.rho_static_obs > prob_data.rho_static_obs_max) prob_data.rho_static_obs = prob_data.rho_static_obs_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_static_obs_norm = 0;
				prob_data.res_y_static_obs_norm = 0;
				break_flag++;
			}

			if(prob_data.num_drone!=0){
				
				res_x_drone = reshape(((-prob_data.x_drone).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_drone * prob_data.d_drone * cos(prob_data.alpha_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
				res_y_drone = reshape(((-prob_data.y_drone).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_drone * prob_data.d_drone * sin(prob_data.alpha_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
							
				prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_x_drone.matrix()).array();
				prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_y_drone.matrix()).array();
			
				prob_data.res_x_drone_norm = res_x_drone.matrix().norm();
				prob_data.res_y_drone_norm = res_y_drone.matrix().norm();

				if(prob_data.res_x_drone_norm > thresold || prob_data.res_y_drone_norm > thresold){
					prob_data.rho_drone *= prob_data.delta_drone;
					if(prob_data.rho_drone > prob_data.rho_drone_max) prob_data.rho_drone = prob_data.rho_drone_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_drone_norm = 0;
				prob_data.res_y_drone_norm = 0;
				prob_data.res_z_drone_norm = 0;
				break_flag++;
			}
			
			prob_data.res_x_ineq_norm = res_x_ineq.matrix().norm();
			prob_data.res_y_ineq_norm = res_y_ineq.matrix().norm();

			prob_data.res_x_vel_norm = res_x_vel.matrix().norm();
			prob_data.res_y_vel_norm = res_y_vel.matrix().norm();

			prob_data.res_x_acc_norm = res_x_acc.matrix().norm();
			prob_data.res_y_acc_norm = res_y_acc.matrix().norm();			
			
			if((prob_data.res_x_ineq_norm > thresold || prob_data.res_y_ineq_norm > thresold) && prob_data.rho_ineq){;
					prob_data.rho_ineq *= prob_data.delta_ineq;
					if(prob_data.rho_ineq > prob_data.rho_ineq_max) prob_data.rho_ineq = prob_data.rho_ineq_max;
			}
			else break_flag++;
			if(prob_data.res_x_vel_norm > thresold || prob_data.res_y_vel_norm > thresold){;
					prob_data.rho_vel *= prob_data.delta_vel;
					if(prob_data.rho_vel > prob_data.rho_vel_max) prob_data.rho_vel = prob_data.rho_vel_max;
			}
			else break_flag++;
			if(prob_data.res_x_acc_norm > thresold || prob_data.res_y_acc_norm > thresold){;
					prob_data.rho_acc *= prob_data.delta_acc;
					if(prob_data.rho_acc > prob_data.rho_acc_max) prob_data.rho_acc = prob_data.rho_acc_max;
			}
			else break_flag++;
			
			if(break_flag == 7)
				break;	
		}
		
		if(break_flag != 7){
				prob_data.weight_goal *= prob_data.delta_aggressive;
				prob_data.weight_smoothness *= prob_data.delta_aggressive;
			}
		else{
			prob_data.weight_goal = prob_data.weight_goal_og;
			prob_data.weight_smoothness = prob_data.weight_smoothness_og;

			if(prob_data.weight_goal > prob_data.weight_goal_og)
				prob_data.weight_goal = prob_data.weight_goal_og;

			if(prob_data.weight_goal > prob_data.weight_smoothness_og)
				prob_data.weight_smoothness = prob_data.weight_smoothness_og;
		}					
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void initAlphaBeta(probData &prob_data, int VERBOSE)
	{
		Eigen :: ArrayXXf ws_alpha_static_obs, wc_alpha_static_obs,
			 			  ws_alpha_drone, wc_alpha_drone,
						  ws_alpha_vel, wc_alpha_vel,
						  ws_alpha_acc, wc_alpha_acc,
						  ws_alpha_jerk, wc_alpha_jerk,
						  ws_alpha_snap, wc_alpha_snap,
						  ws_beta_drone, wc_beta_drone,
						  ws_beta_vel, wc_beta_vel,
						  ws_beta_acc, wc_beta_acc,
						  ws_beta_jerk, wc_beta_jerk,
						  ws_beta_snap, wc_beta_snap;
		Eigen :: ArrayXXf c1_d, c2_d;

		
		// @ Static Obstacles
		if(prob_data.num_static_obs!=0){
			ws_alpha_static_obs = (-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0);
			wc_alpha_static_obs = (-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0);
			prob_data.alpha_static_obs = arctan2(ws_alpha_static_obs * prob_data.a_static_obs, wc_alpha_static_obs * prob_data.b_static_obs);

			c1_d = pow(prob_data.a_static_obs, 2) * pow(cos(prob_data.alpha_static_obs), 2) + pow(prob_data.b_static_obs, 2) * pow(sin(prob_data.alpha_static_obs), 2);
			c2_d = prob_data.a_static_obs * wc_alpha_static_obs * cos(prob_data.alpha_static_obs) + prob_data.b_static_obs * ws_alpha_static_obs * sin(prob_data.alpha_static_obs);

			if(prob_data.d_static_obs_old.rows() != prob_data.num_static_obs){
				prob_data.d_static_obs_old = Eigen :: ArrayXXf :: Ones(prob_data.num_static_obs, prob_data.num);
			}
			else{
				prob_data.d_static_obs_old.rightCols(prob_data.num - 1) = prob_data.d_static_obs_old.leftCols(prob_data.num - 1);
				prob_data.d_static_obs_old.col(0) = 1.0; 
			}

			prob_data.d_static_obs = (c2_d/c1_d).max(1.0 + (1.0 - prob_data.gamma) * (prob_data.d_static_obs_old - 1.0));
			prob_data.d_static_obs_old = prob_data.d_static_obs;
		}

		
		// @ Inter-Agent Collision Avoidance
		if(prob_data.num_drone!=0){
			
			ws_alpha_drone = (-prob_data.y_drone).rowwise() + prob_data.y.transpose().row(0);
			wc_alpha_drone = (-prob_data.x_drone).rowwise() + prob_data.x.transpose().row(0);
			prob_data.alpha_drone = arctan2(ws_alpha_drone * prob_data.a_drone, wc_alpha_drone * prob_data.b_drone);

			wc_beta_drone = (-prob_data.z_drone).rowwise() + prob_data.z.transpose().row(0);			

			prob_data.beta_drone = Eigen :: ArrayXXf :: Zero(prob_data.num_drone, prob_data.num);
			for(int i = 0; i < wc_beta_drone.rows()*wc_beta_drone.cols(); i++){
				prob_data.beta_drone(i) = atan2(wc_alpha_drone(i) / prob_data.a_drone(i) / cos(prob_data.alpha_drone(i)), wc_beta_drone(i) / prob_data.c_drone(i));
			}
			
			c1_d = (pow(sin(prob_data.beta_drone), 2) * pow(cos(prob_data.alpha_drone), 2)) * pow(prob_data.a_drone, 2) + (pow(sin(prob_data.alpha_drone), 2) * pow(sin(prob_data.beta_drone), 2)) * pow(prob_data.b_drone, 2) + pow(cos(prob_data.beta_drone), 2) * pow(prob_data.c_drone, 2);
			c2_d = (wc_alpha_drone * sin(prob_data.beta_drone) * cos(prob_data.alpha_drone)) * prob_data.a_drone + (ws_alpha_drone * sin(prob_data.alpha_drone) * sin(prob_data.beta_drone)) * prob_data.b_drone + (wc_beta_drone * cos(prob_data.beta_drone)) * prob_data.c_drone;
		
			if(prob_data.d_drone_old.rows() != prob_data.num_drone){
				prob_data.d_drone_old = Eigen :: ArrayXXf :: Ones(prob_data.num_drone, prob_data.num);
			}
			else{
				prob_data.d_drone_old.rightCols(prob_data.num - 1) = prob_data.d_drone_old.leftCols(prob_data.num - 1);
				prob_data.d_drone_old.col(0) = 1.0; 
			}

			prob_data.d_drone = (c2_d/c1_d).max(1.0 + (1.0 - prob_data.gamma) * (prob_data.d_drone_old - 1.0));	
			prob_data.d_drone_old = prob_data.d_drone;
		}
		
		if(!prob_data.axis_wise){
			// @ Velocity Constraints
			wc_alpha_vel = prob_data.xdot;				
			ws_alpha_vel = prob_data.ydot;
			prob_data.alpha_vel = arctan2(ws_alpha_vel, wc_alpha_vel);

			wc_beta_vel = prob_data.zdot;
			for(int i = 0; i < prob_data.beta_vel.rows(); i++){
				prob_data.beta_vel(i) = atan2(wc_alpha_vel(i)/cos(prob_data.alpha_vel(i)), wc_beta_vel(i));
			}
			c1_d = pow(sin(prob_data.beta_vel), 2) * pow(cos(prob_data.alpha_vel), 2) + pow(sin(prob_data.beta_vel), 2) * pow(sin(prob_data.alpha_vel), 2) + pow(cos(prob_data.beta_vel), 2);
			c2_d = wc_alpha_vel * sin(prob_data.beta_vel) * cos(prob_data.alpha_vel) + ws_alpha_vel * sin(prob_data.beta_vel) * sin(prob_data.alpha_vel) + wc_beta_vel * cos(prob_data.beta_vel);

			prob_data.d_vel =(c2_d/c1_d).min(prob_data.vel_max);
			

			// @ Acceleration Constraints
			if(!prob_data.use_thrust_values){
				wc_alpha_acc = prob_data.xddot;				
				ws_alpha_acc = prob_data.yddot;
				prob_data.alpha_acc = arctan2(ws_alpha_acc, wc_alpha_acc);

				wc_beta_acc = prob_data.zddot;
				for(int i = 0; i < prob_data.beta_acc.rows(); i++){
					prob_data.beta_acc(i) = atan2(wc_alpha_acc(i)/cos(prob_data.alpha_acc(i)), wc_beta_acc(i));
				}
				c1_d = pow(sin(prob_data.beta_acc), 2) * pow(cos(prob_data.alpha_acc), 2) + pow(sin(prob_data.beta_acc), 2) * pow(sin(prob_data.alpha_acc), 2) + pow(cos(prob_data.beta_acc), 2);
				c2_d = wc_alpha_acc * sin(prob_data.beta_acc) * cos(prob_data.alpha_acc) + ws_alpha_acc * sin(prob_data.beta_acc) * sin(prob_data.alpha_acc) + wc_beta_acc * cos(prob_data.beta_acc);

				prob_data.d_acc =(c2_d/c1_d).min(prob_data.acc_max);
			}
			else{
				wc_alpha_acc = prob_data.xddot;				
				ws_alpha_acc = prob_data.yddot;
				prob_data.alpha_acc = arctan2(ws_alpha_acc, wc_alpha_acc);

				wc_beta_acc = prob_data.zddot + prob_data.gravity;
				for(int i = 0; i < prob_data.beta_acc.rows(); i++){
					prob_data.beta_acc(i) = atan2(wc_alpha_acc(i)/cos(prob_data.alpha_acc(i)), wc_beta_acc(i));
				}
				c1_d = pow(sin(prob_data.beta_acc), 2) * pow(cos(prob_data.alpha_acc), 2) + pow(sin(prob_data.beta_acc), 2) * pow(sin(prob_data.alpha_acc), 2) + pow(cos(prob_data.beta_acc), 2);
				c2_d = wc_alpha_acc * sin(prob_data.beta_acc) * cos(prob_data.alpha_acc) + ws_alpha_acc * sin(prob_data.beta_acc) * sin(prob_data.alpha_acc) + wc_beta_acc * cos(prob_data.beta_acc);

				prob_data.d_acc = ((c2_d/c1_d).min(prob_data.f_max)).max(prob_data.f_min);
			}

			if(prob_data.jerk_snap_constraints){
				// @ Jerk Constraints
				wc_alpha_jerk = prob_data.xdddot;				
				ws_alpha_jerk = prob_data.ydddot;
				prob_data.alpha_jerk = arctan2(ws_alpha_jerk, wc_alpha_jerk);

				wc_beta_jerk = prob_data.zdddot;
				for(int i = 0; i < prob_data.beta_jerk.rows(); i++){
					prob_data.beta_jerk(i) = atan2(wc_alpha_jerk(i)/cos(prob_data.alpha_jerk(i)), wc_beta_jerk(i));
				}
				c1_d = pow(sin(prob_data.beta_jerk), 2) * pow(cos(prob_data.alpha_jerk), 2) + pow(sin(prob_data.beta_jerk), 2) * pow(sin(prob_data.alpha_jerk), 2) + pow(cos(prob_data.beta_jerk), 2);
				c2_d = wc_alpha_jerk * sin(prob_data.beta_jerk) * cos(prob_data.alpha_jerk) + ws_alpha_jerk * sin(prob_data.beta_jerk) * sin(prob_data.alpha_jerk) + wc_beta_jerk * cos(prob_data.beta_jerk);

				prob_data.d_jerk =(c2_d/c1_d).min(prob_data.jerk_max);


				// @ Snap Constraints
				wc_alpha_snap = prob_data.xddddot;				
				ws_alpha_snap = prob_data.yddddot;
				prob_data.alpha_snap = arctan2(ws_alpha_snap, wc_alpha_snap);

				wc_beta_snap = prob_data.zddddot;
				for(int i = 0; i < prob_data.beta_snap.rows(); i++){
					prob_data.beta_snap(i) = atan2(wc_alpha_snap(i)/cos(prob_data.alpha_snap(i)), wc_beta_snap(i));
				}
				c1_d = pow(sin(prob_data.beta_snap), 2) * pow(cos(prob_data.alpha_snap), 2) + pow(sin(prob_data.beta_snap), 2) * pow(sin(prob_data.alpha_snap), 2) + pow(cos(prob_data.beta_snap), 2);
				c2_d = wc_alpha_snap * sin(prob_data.beta_snap) * cos(prob_data.alpha_snap) + ws_alpha_snap * sin(prob_data.beta_snap) * sin(prob_data.alpha_snap) + wc_beta_snap * cos(prob_data.beta_snap);

				prob_data.d_snap = (c2_d/c1_d).min(prob_data.snap_max);  
			}
		}
		
	}
	void computeXYZAxis(probData &prob_data, int VERBOSE)
	{
							  
		Eigen :: ArrayXXf cost_xy, cost_z, cost_xy_inv, cost_z_inv;
		Eigen :: ArrayXXf objective_xy, objective_z, lincost_x, lincost_y, lincost_z;
		Eigen :: ArrayXXf cost_drone, cost_static_obs;
		Eigen :: ArrayXXf temp_x_static_obs, temp_y_static_obs;
		Eigen :: ArrayXXf temp_x_drone, temp_y_drone, temp_z_drone;
		Eigen :: ArrayXXf sol_x, sol_y, sol_z, primal_sol_x, primal_sol_y, primal_sol_z;
		Eigen :: ArrayXXf res_x_static_obs, res_y_static_obs,
						  res_x_drone, res_y_drone, res_z_drone,
						  res_vx_ineq, res_vy_ineq, res_vz_ineq,
						  res_ax_ineq, res_ay_ineq, res_az_ineq,
						  res_jx_ineq, res_jy_ineq, res_jz_ineq,
						  res_sx_ineq, res_sy_ineq, res_sz_ineq,
						  res_x_ineq, res_y_ineq, res_z_ineq;
		
		// @ Set Initial Conidtions for next MPC step
		prob_data.b_x_eq << prob_data.x_init, prob_data.vx_init, prob_data.ax_init;
		prob_data.b_y_eq << prob_data.y_init, prob_data.vy_init, prob_data.ay_init;
		prob_data.b_z_eq << prob_data.z_init, prob_data.vz_init, prob_data.az_init;

		float thresold = prob_data.thresold;

		prob_data.rho_static_obs = 1.0;
		prob_data.rho_drone = 1.0;
		prob_data.rho_vel = 1.0;
		prob_data.rho_acc = 1.0;
		prob_data.rho_jerk = 1.0;
		prob_data.rho_snap = 1.0;
		prob_data.rho_ineq = 1.0;
		
		
		 // @ Lagrange Multiplier
		prob_data.lamda_x = Eigen :: ArrayXXf :: Ones(prob_data.nvar, 1) *0;
		prob_data.lamda_y = Eigen :: ArrayXXf :: Ones(prob_data.nvar, 1) *0;
		prob_data.lamda_z = Eigen :: ArrayXXf :: Ones(prob_data.nvar, 1) *0;

		// @ Position Constraints
		prob_data.s_x_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_y_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_z_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		
		// @ Velocity Constraints
		prob_data.s_vx_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_vy_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_vz_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;

		// @ Acceleration Constraints
		prob_data.s_ax_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_ay_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_az_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;

		// @ Jerk Constraints
		prob_data.s_jx_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_jy_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_jz_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;

		// @ Snap Constraints
		prob_data.s_sx_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_sy_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_sz_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;


		if(prob_data.num_drone!=0)cost_drone = prob_data.A_drone.transpose().matrix() * prob_data.A_drone.matrix();
		if(prob_data.num_static_obs!=0)cost_static_obs = prob_data.A_static_obs.transpose().matrix() * prob_data.A_static_obs.matrix();
		
		int break_flag;
		for(int i = 0; i < prob_data.max_iter; i++){

			break_flag = 0;
			
			prob_data.B_x_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_x_ineq - prob_data.s_x_ineq).matrix();
			prob_data.B_y_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_y_ineq - prob_data.s_y_ineq).matrix();
			prob_data.B_z_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_z_ineq - prob_data.s_z_ineq).matrix();

			prob_data.B_vx_ineq = prob_data.A_v_ineq.transpose().matrix() * (prob_data.b_vx_ineq - prob_data.s_vx_ineq).matrix();
			prob_data.B_vy_ineq = prob_data.A_v_ineq.transpose().matrix() * (prob_data.b_vy_ineq - prob_data.s_vy_ineq).matrix();
			prob_data.B_vz_ineq = prob_data.A_v_ineq.transpose().matrix() * (prob_data.b_vz_ineq - prob_data.s_vz_ineq).matrix();

			prob_data.B_ax_ineq = prob_data.A_a_ineq.transpose().matrix() * (prob_data.b_ax_ineq - prob_data.s_ax_ineq).matrix();
			prob_data.B_ay_ineq = prob_data.A_a_ineq.transpose().matrix() * (prob_data.b_ay_ineq - prob_data.s_ay_ineq).matrix();
			prob_data.B_az_ineq = prob_data.A_a_ineq.transpose().matrix() * (prob_data.b_az_ineq - prob_data.s_az_ineq).matrix();

			objective_xy = prob_data.weight_goal * prob_data.cost_goal 
					+ prob_data.weight_smoothness * prob_data.cost_smoothness 
					+ prob_data.rho_vel * prob_data.cost_vel 
					+ prob_data.rho_acc * prob_data.cost_acc 
					+ prob_data.rho_ineq * prob_data.cost_ineq;
			
			objective_z = prob_data.weight_goal * prob_data.cost_goal 
				+ prob_data.weight_smoothness * prob_data.cost_smoothness 
				+ prob_data.rho_vel * prob_data.cost_vel 
				+ prob_data.rho_acc * prob_data.cost_acc 
				+ prob_data.rho_ineq * prob_data.cost_ineq;


			lincost_x = -prob_data.lamda_x
						-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.x_ref.matrix()).array()
						-prob_data.rho_vel * prob_data.B_vx_ineq
						-prob_data.rho_acc * prob_data.B_ax_ineq
						-prob_data.rho_ineq * prob_data.B_x_ineq;

			lincost_y = -prob_data.lamda_y
						-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.y_ref.matrix()).array()
						-prob_data.rho_vel * prob_data.B_vy_ineq
						-prob_data.rho_acc * prob_data.B_ay_ineq
						-prob_data.rho_ineq * prob_data.B_y_ineq;
			
		
			lincost_z = -prob_data.lamda_z
						-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.z_ref.matrix()).array()
						-prob_data.rho_vel * prob_data.B_vz_ineq
						-prob_data.rho_acc * prob_data.B_az_ineq
						-prob_data.rho_ineq * prob_data.B_z_ineq;
		
			if(prob_data.jerk_snap_constraints){
				prob_data.B_jx_ineq = prob_data.A_j_ineq.transpose().matrix() * (prob_data.b_jx_ineq - prob_data.s_jx_ineq).matrix();
				prob_data.B_jy_ineq = prob_data.A_j_ineq.transpose().matrix() * (prob_data.b_jy_ineq - prob_data.s_jy_ineq).matrix();
				prob_data.B_jz_ineq = prob_data.A_j_ineq.transpose().matrix() * (prob_data.b_jz_ineq - prob_data.s_jz_ineq).matrix();

				prob_data.B_sx_ineq = prob_data.A_s_ineq.transpose().matrix() * (prob_data.b_sx_ineq - prob_data.s_sx_ineq).matrix();
				prob_data.B_sy_ineq = prob_data.A_s_ineq.transpose().matrix() * (prob_data.b_sy_ineq - prob_data.s_sy_ineq).matrix();
				prob_data.B_sz_ineq = prob_data.A_s_ineq.transpose().matrix() * (prob_data.b_sz_ineq - prob_data.s_sz_ineq).matrix();
			
				objective_xy += prob_data.rho_jerk * prob_data.cost_jerk
					+ prob_data.rho_snap * prob_data.cost_snap;
				objective_z += prob_data.rho_jerk * prob_data.cost_jerk
					+ prob_data.rho_snap * prob_data.cost_snap;

				lincost_x += -prob_data.rho_jerk * prob_data.B_jx_ineq 
						-prob_data.rho_snap * prob_data.B_sx_ineq;
				lincost_y += -prob_data.rho_jerk * prob_data.B_jy_ineq
						-prob_data.rho_snap * prob_data.B_sy_ineq;
				lincost_z += -prob_data.rho_jerk * prob_data.B_jz_ineq
						-prob_data.rho_snap * prob_data.B_sz_ineq;
			}
			
			// @ Check for obstacles
			if(prob_data.num_static_obs!=0){
				temp_x_static_obs =  prob_data.x_static_obs + prob_data.d_static_obs * cos(prob_data.alpha_static_obs) * prob_data.a_static_obs;
				temp_y_static_obs =  prob_data.y_static_obs + prob_data.d_static_obs * sin(prob_data.alpha_static_obs) * prob_data.b_static_obs;

				prob_data.b_x_static_obs = Optim :: reshape(temp_x_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
				prob_data.b_y_static_obs = Optim :: reshape(temp_y_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
			
				objective_xy += prob_data.rho_static_obs * cost_static_obs;
				lincost_x += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_x_static_obs.matrix()).array();
				lincost_y += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_y_static_obs.matrix()).array();

			}
			
			if(prob_data.num_drone!=0){
				
				temp_x_drone =  prob_data.x_drone + prob_data.d_drone * cos(prob_data.alpha_drone) * sin(prob_data.beta_drone) * prob_data.a_drone;
				temp_y_drone =  prob_data.y_drone + prob_data.d_drone * sin(prob_data.alpha_drone) * sin(prob_data.beta_drone) * prob_data.b_drone;
				temp_z_drone =  prob_data.z_drone + prob_data.d_drone * cos(prob_data.beta_drone) * prob_data.c_drone;

				prob_data.b_x_drone = Optim :: reshape(temp_x_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
				prob_data.b_y_drone = Optim :: reshape(temp_y_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
				prob_data.b_z_drone = Optim :: reshape(temp_z_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
			
				objective_xy += prob_data.rho_drone * cost_drone;
				objective_z += prob_data.rho_drone * cost_drone;
				
				lincost_x += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_x_drone.matrix()).array();
				lincost_y += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_y_drone.matrix()).array();
				lincost_z += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_z_drone.matrix()).array();
			
			}

			// @ Solve set of linear equations
			cost_xy = Optim :: stack(stack(objective_xy, prob_data.A_eq.transpose(), 'h'), 
							   stack(prob_data.A_eq, Eigen::ArrayXXf::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
			cost_xy_inv = (cost_xy.matrix()).householderQr().solve(prob_data.I.matrix());

			cost_z = Optim :: stack(stack(objective_z, prob_data.A_eq.transpose(), 'h'), 
						stack(prob_data.A_eq, Eigen::ArrayXXf::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
			cost_z_inv = (cost_z.matrix()).householderQr().solve(prob_data.I.matrix());
			
			sol_x = cost_xy_inv.matrix() * stack(-lincost_x, prob_data.b_x_eq, 'v').matrix();
			sol_y = cost_xy_inv.matrix() * stack(-lincost_y, prob_data.b_y_eq, 'v').matrix(); 
			sol_z = cost_z_inv.matrix() * stack(-lincost_z, prob_data.b_z_eq, 'v').matrix(); 

			primal_sol_x = sol_x.topRows(prob_data.nvar);
			primal_sol_y = sol_y.topRows(prob_data.nvar);
			primal_sol_z = sol_z.topRows(prob_data.nvar);

			prob_data.x = prob_data.P.matrix() * primal_sol_x.matrix();
			prob_data.xdot = prob_data.Pdot.matrix() * primal_sol_x.matrix();
			prob_data.xddot = prob_data.Pddot.matrix() * primal_sol_x.matrix();
			prob_data.xdddot = prob_data.Pdddot.matrix() * primal_sol_x.matrix();
			prob_data.xddddot = prob_data.Pddddot.matrix() * primal_sol_x.matrix();

			prob_data.y = prob_data.P.matrix() * primal_sol_y.matrix();
			prob_data.ydot = prob_data.Pdot.matrix() * primal_sol_y.matrix();
			prob_data.yddot = prob_data.Pddot.matrix() * primal_sol_y.matrix();
			prob_data.ydddot = prob_data.Pdddot.matrix() * primal_sol_y.matrix();
			prob_data.yddddot = prob_data.Pddddot.matrix() * primal_sol_y.matrix();	

			prob_data.z = prob_data.P.matrix() * primal_sol_z.matrix();
			prob_data.zdot = prob_data.Pdot.matrix() * primal_sol_z.matrix();
			prob_data.zddot = prob_data.Pddot.matrix() * primal_sol_z.matrix();
			prob_data.zdddot = prob_data.Pdddot.matrix() * primal_sol_z.matrix();
			prob_data.zddddot = prob_data.Pddddot.matrix() * primal_sol_z.matrix();

			//////
			prob_data.x_up = prob_data.P_up.matrix() * primal_sol_x.matrix();
			prob_data.y_up = prob_data.P_up.matrix() * primal_sol_y.matrix();
			prob_data.z_up = prob_data.P_up.matrix() * primal_sol_z.matrix();

			prob_data.xdot_up = prob_data.Pdot_up.matrix() * primal_sol_x.matrix();
			prob_data.ydot_up = prob_data.Pdot_up.matrix() * primal_sol_y.matrix();
			prob_data.zdot_up = prob_data.Pdot_up.matrix() * primal_sol_z.matrix();

			prob_data.xddot_up = prob_data.Pddot_up.matrix() * primal_sol_x.matrix();
			prob_data.yddot_up = prob_data.Pddot_up.matrix() * primal_sol_y.matrix();
			prob_data.zddot_up = prob_data.Pddot_up.matrix() * primal_sol_z.matrix();
			/////
			
			
			// @ Residual and Lagrange Update
			initAlphaBeta(prob_data, VERBOSE);

			// Position
			prob_data.s_x_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_x_ineq).max(0.0);
			prob_data.s_y_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_y_ineq).max(0.0);
			prob_data.s_z_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_z.matrix()).array() + prob_data.b_z_ineq).max(0.0);

			res_x_ineq = (prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_x_ineq + prob_data.s_x_ineq;
			res_y_ineq = (prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_y_ineq + prob_data.s_y_ineq;
			res_z_ineq = (prob_data.A_ineq.matrix() * primal_sol_z.matrix()).array() - prob_data.b_z_ineq + prob_data.s_z_ineq;


			// Velocity
			prob_data.s_vx_ineq = ((-prob_data.A_v_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_vx_ineq).max(0.0);
			prob_data.s_vy_ineq = ((-prob_data.A_v_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_vy_ineq).max(0.0);
			prob_data.s_vz_ineq = ((-prob_data.A_v_ineq.matrix() * primal_sol_z.matrix()).array() + prob_data.b_vz_ineq).max(0.0);
			
			res_vx_ineq = (prob_data.A_v_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_vx_ineq + prob_data.s_vx_ineq;
			res_vy_ineq = (prob_data.A_v_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_vy_ineq + prob_data.s_vy_ineq;
			res_vz_ineq = (prob_data.A_v_ineq.matrix() * primal_sol_z.matrix()).array() - prob_data.b_vz_ineq + prob_data.s_vz_ineq;

			// Acceleration
			prob_data.s_ax_ineq = ((-prob_data.A_a_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_ax_ineq).max(0.0);
			prob_data.s_ay_ineq = ((-prob_data.A_a_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_ay_ineq).max(0.0);
			prob_data.s_az_ineq = ((-prob_data.A_a_ineq.matrix() * primal_sol_z.matrix()).array() + prob_data.b_az_ineq).max(0.0);
			
			res_ax_ineq = (prob_data.A_a_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_ax_ineq + prob_data.s_ax_ineq;
			res_ay_ineq = (prob_data.A_a_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_ay_ineq + prob_data.s_ay_ineq;
			res_az_ineq = (prob_data.A_a_ineq.matrix() * primal_sol_z.matrix()).array() - prob_data.b_az_ineq + prob_data.s_az_ineq;

			prob_data.lamda_x = prob_data.lamda_x 
					-prob_data.rho_vel * (prob_data.A_v_ineq.transpose().matrix() * res_vx_ineq.matrix()).array()
					-prob_data.rho_acc * (prob_data.A_a_ineq.transpose().matrix() * res_ax_ineq.matrix()).array()
					-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_x_ineq.matrix()).array();

			prob_data.lamda_z = prob_data.lamda_z 
					-prob_data.rho_vel * (prob_data.A_v_ineq.transpose().matrix() * res_vz_ineq.matrix()).array()
					-prob_data.rho_acc * (prob_data.A_a_ineq.transpose().matrix() * res_az_ineq.matrix()).array()
					-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_z_ineq.matrix()).array();

			prob_data.lamda_y = prob_data.lamda_y 
					-prob_data.rho_vel * (prob_data.A_v_ineq.transpose().matrix() * res_vy_ineq.matrix()).array()
					-prob_data.rho_acc * (prob_data.A_a_ineq.transpose().matrix() * res_ay_ineq.matrix()).array()
					-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_y_ineq.matrix()).array();
			
			

			if(prob_data.jerk_snap_constraints){
				// Jerk
				prob_data.s_jx_ineq = ((-prob_data.A_j_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_jx_ineq).max(0.0);
				prob_data.s_jy_ineq = ((-prob_data.A_j_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_jy_ineq).max(0.0);
				prob_data.s_jz_ineq = ((-prob_data.A_j_ineq.matrix() * primal_sol_z.matrix()).array() + prob_data.b_jz_ineq).max(0.0);
				
				res_jx_ineq = (prob_data.A_j_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_jx_ineq + prob_data.s_jx_ineq;
				res_jy_ineq = (prob_data.A_j_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_jy_ineq + prob_data.s_jy_ineq;
				res_jz_ineq = (prob_data.A_j_ineq.matrix() * primal_sol_z.matrix()).array() - prob_data.b_jz_ineq + prob_data.s_jz_ineq;


				// Snap
				prob_data.s_sx_ineq = ((-prob_data.A_s_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_sx_ineq).max(0.0);
				prob_data.s_sy_ineq = ((-prob_data.A_s_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_sy_ineq).max(0.0);
				prob_data.s_sz_ineq = ((-prob_data.A_s_ineq.matrix() * primal_sol_z.matrix()).array() + prob_data.b_sz_ineq).max(0.0);

				res_sx_ineq = (prob_data.A_s_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_sx_ineq + prob_data.s_sx_ineq;
				res_sy_ineq = (prob_data.A_s_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_sy_ineq + prob_data.s_sy_ineq;
				res_sz_ineq = (prob_data.A_s_ineq.matrix() * primal_sol_z.matrix()).array() - prob_data.b_sz_ineq + prob_data.s_sz_ineq;

				prob_data.lamda_x = prob_data.lamda_x 
								-prob_data.rho_jerk * (prob_data.A_j_ineq.transpose().matrix() * res_jx_ineq.matrix()).array()
								-prob_data.rho_snap * (prob_data.A_s_ineq.transpose().matrix() * res_sx_ineq.matrix()).array();
				
				prob_data.lamda_y = prob_data.lamda_y 
								-prob_data.rho_jerk * (prob_data.A_j_ineq.transpose().matrix() * res_jy_ineq.matrix()).array()
								-prob_data.rho_snap * (prob_data.A_s_ineq.transpose().matrix() * res_sy_ineq.matrix()).array();

				prob_data.lamda_z = prob_data.lamda_z
								-prob_data.rho_jerk * (prob_data.A_j_ineq.transpose().matrix() * res_jz_ineq.matrix()).array()
								-prob_data.rho_snap * (prob_data.A_s_ineq.transpose().matrix() * res_sz_ineq.matrix()).array();

				prob_data.res_x_jerk_norm = res_jx_ineq.matrix().norm();
				prob_data.res_y_jerk_norm = res_jy_ineq.matrix().norm();
				prob_data.res_z_jerk_norm = res_jz_ineq.matrix().norm();

				prob_data.res_x_snap_norm = res_sx_ineq.matrix().norm();
				prob_data.res_y_snap_norm = res_sy_ineq.matrix().norm();
				prob_data.res_z_snap_norm = res_sz_ineq.matrix().norm();

				if(prob_data.res_x_jerk_norm > thresold || prob_data.res_y_jerk_norm > thresold || prob_data.res_z_jerk_norm > thresold){;
					prob_data.rho_jerk *= prob_data.delta_jerk;
					if(prob_data.rho_jerk > prob_data.rho_jerk_max) prob_data.rho_jerk = prob_data.rho_jerk_max;
				}
				else break_flag++;
				if(prob_data.res_x_snap_norm > thresold || prob_data.res_y_snap_norm > thresold || prob_data.res_z_snap_norm > thresold){
						prob_data.rho_snap *= prob_data.delta_snap;
						if(prob_data.rho_snap > prob_data.rho_snap_max) prob_data.rho_snap = prob_data.rho_snap_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_jerk_norm = 0.0;
				prob_data.res_y_jerk_norm = 0.0;
				prob_data.res_z_jerk_norm = 0.0;

				prob_data.res_x_snap_norm = 0.0;
				prob_data.res_y_snap_norm = 0.0;
				prob_data.res_z_snap_norm = 0.0;

				break_flag += 2;
			}

			if(prob_data.num_static_obs!=0){
				
				res_x_static_obs = reshape(((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_static_obs * prob_data.d_static_obs * cos(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
				res_y_static_obs = reshape(((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_static_obs * prob_data.d_static_obs * sin(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
			
				prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_x_static_obs.matrix()).array();
				prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_y_static_obs.matrix()).array();
			
				prob_data.res_x_static_obs_norm = res_x_static_obs.matrix().norm();
				prob_data.res_y_static_obs_norm = res_y_static_obs.matrix().norm();
			
				if(prob_data.res_x_static_obs_norm > thresold || prob_data.res_y_static_obs_norm > thresold){
					prob_data.rho_static_obs *= prob_data.delta_static_obs;
					if(prob_data.rho_static_obs > prob_data.rho_static_obs_max) prob_data.rho_static_obs = prob_data.rho_static_obs_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_static_obs_norm = 0;
				prob_data.res_y_static_obs_norm = 0;
				break_flag++;
			}

			if(prob_data.num_drone!=0){
				
				res_x_drone = reshape(((-prob_data.x_drone).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_drone * prob_data.d_drone * cos(prob_data.alpha_drone) * sin(prob_data.beta_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
				res_y_drone = reshape(((-prob_data.y_drone).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_drone * prob_data.d_drone * sin(prob_data.alpha_drone) * sin(prob_data.beta_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
				res_z_drone = reshape(((-prob_data.z_drone).rowwise() + prob_data.z.transpose().row(0) - prob_data.c_drone * prob_data.d_drone * cos(prob_data.beta_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);

				prob_data.lamda_z = prob_data.lamda_z -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_z_drone.matrix()).array();
				prob_data.res_z_drone_norm = res_z_drone.matrix().norm();
			
				prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_x_drone.matrix()).array();
				prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_y_drone.matrix()).array();
			
				prob_data.res_x_drone_norm = res_x_drone.matrix().norm();
				prob_data.res_y_drone_norm = res_y_drone.matrix().norm();

				if(prob_data.res_x_drone_norm > thresold || prob_data.res_y_drone_norm > thresold || prob_data.res_z_drone_norm > thresold){
					prob_data.rho_drone *= prob_data.delta_drone;
					if(prob_data.rho_drone > prob_data.rho_drone_max) prob_data.rho_drone = prob_data.rho_drone_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_drone_norm = 0;
				prob_data.res_y_drone_norm = 0;
				prob_data.res_z_drone_norm = 0;
				break_flag++;
			}
			
			prob_data.res_x_ineq_norm = res_x_ineq.matrix().norm();
			prob_data.res_y_ineq_norm = res_y_ineq.matrix().norm();
			prob_data.res_z_ineq_norm = res_z_ineq.matrix().norm();

			prob_data.res_x_vel_norm = res_vx_ineq.matrix().norm();
			prob_data.res_y_vel_norm = res_vy_ineq.matrix().norm();
			prob_data.res_z_vel_norm = res_vz_ineq.matrix().norm();

			prob_data.res_x_acc_norm = res_ax_ineq.matrix().norm();
			prob_data.res_y_acc_norm = res_ay_ineq.matrix().norm();
			prob_data.res_z_acc_norm = res_az_ineq.matrix().norm();
			
			
			if(prob_data.res_x_ineq_norm > thresold || prob_data.res_y_ineq_norm > thresold || prob_data.res_z_ineq_norm > thresold){;
					prob_data.rho_ineq *= prob_data.delta_ineq;
					if(prob_data.rho_ineq > prob_data.rho_ineq_max) prob_data.rho_ineq = prob_data.rho_ineq_max;
			}
			else break_flag++;
			if(prob_data.res_x_vel_norm > thresold || prob_data.res_y_vel_norm > thresold || prob_data.res_z_vel_norm > thresold){;
					prob_data.rho_vel *= prob_data.delta_vel;
					if(prob_data.rho_vel > prob_data.rho_vel_max) prob_data.rho_vel = prob_data.rho_vel_max;
			}
			else break_flag++;
			if(prob_data.res_x_acc_norm > thresold || prob_data.res_y_acc_norm > thresold || prob_data.res_z_acc_norm > thresold){;
					prob_data.rho_acc *= prob_data.delta_acc;
					if(prob_data.rho_acc > prob_data.rho_acc_max) prob_data.rho_acc = prob_data.rho_acc_max;
			}
			else break_flag++;
			

			if(break_flag == 7)
				break;	
		}
		
		if(break_flag != 7){
				prob_data.weight_goal *= prob_data.delta_aggressive;
				prob_data.weight_smoothness *= prob_data.delta_aggressive;
			}
		else{
			prob_data.weight_goal = prob_data.weight_goal_og;
			prob_data.weight_smoothness = prob_data.weight_smoothness_og;

			if(prob_data.weight_goal > prob_data.weight_goal_og)
				prob_data.weight_goal = prob_data.weight_goal_og;

			if(prob_data.weight_goal > prob_data.weight_smoothness_og)
				prob_data.weight_smoothness = prob_data.weight_smoothness_og;
		}
	}

	void computeXYZ(probData &prob_data, int VERBOSE)
	{
							  
		Eigen :: ArrayXXf cost_xy, cost_z, cost_xy_inv, cost_z_inv;
		Eigen :: ArrayXXf objective_xy, objective_z, lincost_x, lincost_y, lincost_z;
		Eigen :: ArrayXXf cost_drone, cost_static_obs;
		Eigen :: ArrayXXf temp_x_static_obs, temp_y_static_obs;
		Eigen :: ArrayXXf temp_x_drone, temp_y_drone, temp_z_drone;
		Eigen :: ArrayXXf sol_x, sol_y, sol_z, primal_sol_x, primal_sol_y, primal_sol_z;
		Eigen :: ArrayXXf res_x_static_obs, res_y_static_obs,
						  res_x_drone, res_y_drone, res_z_drone,
						  res_x_vel, res_y_vel, res_z_vel,
						  res_x_acc, res_y_acc, res_z_acc,
						  res_x_jerk, res_y_jerk, res_z_jerk,
						  res_x_snap, res_y_snap, res_z_snap,
						  res_x_ineq, res_y_ineq, res_z_ineq;
		
		// @ Set Initial Conidtions for next MPC step
		prob_data.b_x_eq << prob_data.x_init, prob_data.vx_init, prob_data.ax_init;
		prob_data.b_y_eq << prob_data.y_init, prob_data.vy_init, prob_data.ay_init;
		prob_data.b_z_eq << prob_data.z_init, prob_data.vz_init, prob_data.az_init;

		float thresold = prob_data.thresold;

		prob_data.rho_static_obs = 1.0;
		prob_data.rho_drone = 1.0;
		prob_data.rho_vel = 1.0;
		prob_data.rho_acc = 1.0;
		prob_data.rho_jerk = 1.0;
		prob_data.rho_snap = 1.0;
		prob_data.rho_ineq = 1.0;
		
		
		// @ Lagrange Multiplier
		prob_data.lamda_x = Eigen :: ArrayXXf :: Ones(prob_data.nvar, 1) *0;
		prob_data.lamda_y = Eigen :: ArrayXXf :: Ones(prob_data.nvar, 1) *0;
		prob_data.lamda_z = Eigen :: ArrayXXf :: Ones(prob_data.nvar, 1) *0;

		// @ Position Constraints
		prob_data.s_x_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_y_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
		prob_data.s_z_ineq = Eigen :: ArrayXXf :: Ones(2*prob_data.num, 1) *0;
	
		
		if(prob_data.num_drone!=0)cost_drone = prob_data.A_drone.transpose().matrix() * prob_data.A_drone.matrix();
		if(prob_data.num_static_obs!=0)cost_static_obs = prob_data.A_static_obs.transpose().matrix() * prob_data.A_static_obs.matrix();
		
		
		int break_flag;
		for(int i = 0; i < prob_data.max_iter; i++){

			break_flag = 0;

			prob_data.b_x_vel = prob_data.d_vel * cos(prob_data.alpha_vel) * sin(prob_data.beta_vel);
			prob_data.b_y_vel = prob_data.d_vel * sin(prob_data.alpha_vel) * sin(prob_data.beta_vel);
			prob_data.b_z_vel = prob_data.d_vel * cos(prob_data.beta_vel);

			prob_data.b_x_acc = prob_data.d_acc * cos(prob_data.alpha_acc) * sin(prob_data.beta_acc);
			prob_data.b_y_acc = prob_data.d_acc * sin(prob_data.alpha_acc) * sin(prob_data.beta_acc);
			prob_data.b_z_acc = prob_data.d_acc * cos(prob_data.beta_acc);
			
			prob_data.B_x_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_x_ineq - prob_data.s_x_ineq).matrix();
			prob_data.B_y_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_y_ineq - prob_data.s_y_ineq).matrix();
			
			
			objective_xy = prob_data.weight_goal * prob_data.cost_goal 
					+ prob_data.weight_smoothness * prob_data.cost_smoothness 
					+ prob_data.rho_vel * prob_data.cost_vel 
					+ prob_data.rho_acc * prob_data.cost_acc 
					+ prob_data.rho_ineq * prob_data.cost_ineq;
			
			lincost_x = -prob_data.lamda_x
						-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.x_ref.matrix()).array()
						-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * prob_data.b_x_vel.matrix()).array()
						-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * prob_data.b_x_acc.matrix()).array()
						-prob_data.rho_ineq * prob_data.B_x_ineq;

			lincost_y = -prob_data.lamda_y
						-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.y_ref.matrix()).array()
						-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * prob_data.b_y_vel.matrix()).array()
						-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * prob_data.b_y_acc.matrix()).array()
						-prob_data.rho_ineq * prob_data.B_y_ineq;
			
			
			prob_data.B_z_ineq = prob_data.A_ineq.transpose().matrix() * (prob_data.b_z_ineq - prob_data.s_z_ineq).matrix();
			objective_z = prob_data.weight_goal * prob_data.cost_goal 
				+ prob_data.weight_smoothness * prob_data.cost_smoothness 
				+ prob_data.rho_vel * prob_data.cost_vel 
				+ prob_data.rho_acc * prob_data.cost_acc 
				+ prob_data.rho_ineq * prob_data.cost_ineq;
		
			lincost_z = -prob_data.lamda_z
						-prob_data.weight_goal * (prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.z_ref.matrix()).array()
						-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * prob_data.b_z_vel.matrix()).array()
						-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * prob_data.b_z_acc.matrix()).array()
						-prob_data.rho_ineq * prob_data.B_z_ineq;

			// jerk-snap
			if(prob_data.jerk_snap_constraints){
				prob_data.b_x_jerk = prob_data.d_jerk * cos(prob_data.alpha_jerk) * sin(prob_data.beta_jerk);
				prob_data.b_y_jerk = prob_data.d_jerk * sin(prob_data.alpha_jerk) * sin(prob_data.beta_jerk);
				prob_data.b_z_jerk = prob_data.d_jerk * cos(prob_data.beta_jerk);

				prob_data.b_x_snap = prob_data.d_snap * cos(prob_data.alpha_snap) * sin(prob_data.beta_snap);
				prob_data.b_y_snap = prob_data.d_snap * sin(prob_data.alpha_snap) * sin(prob_data.beta_snap);
				prob_data.b_z_snap = prob_data.d_snap * cos(prob_data.beta_snap);

				objective_xy += prob_data.rho_jerk * prob_data.cost_jerk
					+ prob_data.rho_snap * prob_data.cost_snap;
				
				objective_z += prob_data.rho_jerk * prob_data.cost_jerk
					+ prob_data.rho_snap * prob_data.cost_snap;

				lincost_x += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * prob_data.b_x_jerk.matrix()).array()
						-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * prob_data.b_x_snap.matrix()).array();

				lincost_y += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * prob_data.b_y_jerk.matrix()).array()
						-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * prob_data.b_y_snap.matrix()).array();

				lincost_z += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * prob_data.b_z_jerk.matrix()).array()
						-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * prob_data.b_z_snap.matrix()).array();
			}			

			
			// @ Check for obstacles
			if(prob_data.num_static_obs!=0){
				temp_x_static_obs =  prob_data.x_static_obs + prob_data.d_static_obs * cos(prob_data.alpha_static_obs) * prob_data.a_static_obs;
				temp_y_static_obs =  prob_data.y_static_obs + prob_data.d_static_obs * sin(prob_data.alpha_static_obs) * prob_data.b_static_obs;

				prob_data.b_x_static_obs = Optim :: reshape(temp_x_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
				prob_data.b_y_static_obs = Optim :: reshape(temp_y_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
			
				objective_xy += prob_data.rho_static_obs * cost_static_obs;
				lincost_x += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_x_static_obs.matrix()).array();
				lincost_y += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_y_static_obs.matrix()).array();

			}
			
			// drones
			if(prob_data.num_drone!=0){
				
				temp_x_drone =  prob_data.x_drone + prob_data.d_drone * cos(prob_data.alpha_drone) * sin(prob_data.beta_drone) * prob_data.a_drone;
				temp_y_drone =  prob_data.y_drone + prob_data.d_drone * sin(prob_data.alpha_drone) * sin(prob_data.beta_drone) * prob_data.b_drone;
				temp_z_drone =  prob_data.z_drone + prob_data.d_drone * cos(prob_data.beta_drone) * prob_data.c_drone;

				prob_data.b_x_drone = Optim :: reshape(temp_x_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
				prob_data.b_y_drone = Optim :: reshape(temp_y_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
				prob_data.b_z_drone = Optim :: reshape(temp_z_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
			
				objective_xy += prob_data.rho_drone * cost_drone;
				objective_z += prob_data.rho_drone * cost_drone;
				
				lincost_x += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_x_drone.matrix()).array();
				lincost_y += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_y_drone.matrix()).array();
				lincost_z += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_z_drone.matrix()).array();
			
			}

			// @ Solve set of linear equations
			cost_xy = Optim :: stack(stack(objective_xy, prob_data.A_eq.transpose(), 'h'), 
							   stack(prob_data.A_eq, Eigen::ArrayXXf::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
			cost_xy_inv = (cost_xy.matrix()).householderQr().solve(prob_data.I.matrix());

			
			sol_x = cost_xy_inv.matrix() * stack(-lincost_x, prob_data.b_x_eq, 'v').matrix();
			sol_y = cost_xy_inv.matrix() * stack(-lincost_y, prob_data.b_y_eq, 'v').matrix(); 

			
				
			cost_z = Optim :: stack(stack(objective_z, prob_data.A_eq.transpose(), 'h'), 
						stack(prob_data.A_eq, Eigen::ArrayXXf::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
			cost_z_inv = (cost_z.matrix()).householderQr().solve(prob_data.I.matrix());
				
				

			sol_z = cost_z_inv.matrix() * stack(-lincost_z, prob_data.b_z_eq, 'v').matrix(); 
			primal_sol_z = sol_z.topRows(prob_data.nvar);


			prob_data.z = prob_data.P.matrix() * primal_sol_z.matrix();
			prob_data.zdot = prob_data.Pdot.matrix() * primal_sol_z.matrix();
			prob_data.zddot = prob_data.Pddot.matrix() * primal_sol_z.matrix();
			prob_data.zdddot = prob_data.Pdddot.matrix() * primal_sol_z.matrix();
			prob_data.zddddot = prob_data.Pddddot.matrix() * primal_sol_z.matrix();
						
			primal_sol_x = sol_x.topRows(prob_data.nvar);
			primal_sol_y = sol_y.topRows(prob_data.nvar);

			prob_data.x = prob_data.P.matrix() * primal_sol_x.matrix();
			prob_data.y = prob_data.P.matrix() * primal_sol_y.matrix();
			
			prob_data.xdot = prob_data.Pdot.matrix() * primal_sol_x.matrix();
			prob_data.ydot = prob_data.Pdot.matrix() * primal_sol_y.matrix();
			
			prob_data.xddot = prob_data.Pddot.matrix() * primal_sol_x.matrix();
			prob_data.yddot = prob_data.Pddot.matrix() * primal_sol_y.matrix();

			prob_data.xdddot = prob_data.Pdddot.matrix() * primal_sol_x.matrix();
			prob_data.ydddot = prob_data.Pdddot.matrix() * primal_sol_y.matrix();

			prob_data.xddddot = prob_data.Pddddot.matrix() * primal_sol_x.matrix();
			prob_data.yddddot = prob_data.Pddddot.matrix() * primal_sol_y.matrix();	
			
			prob_data.x_up = prob_data.P_up.matrix() * primal_sol_x.matrix();
			prob_data.y_up = prob_data.P_up.matrix() * primal_sol_y.matrix();
			prob_data.z_up = prob_data.P_up.matrix() * primal_sol_z.matrix();

			prob_data.xdot_up = prob_data.Pdot_up.matrix() * primal_sol_x.matrix();
			prob_data.ydot_up = prob_data.Pdot_up.matrix() * primal_sol_y.matrix();
			prob_data.zdot_up = prob_data.Pdot_up.matrix() * primal_sol_z.matrix();

			prob_data.xddot_up = prob_data.Pddot_up.matrix() * primal_sol_x.matrix();
			prob_data.yddot_up = prob_data.Pddot_up.matrix() * primal_sol_y.matrix();
			prob_data.zddot_up = prob_data.Pddot_up.matrix() * primal_sol_z.matrix();
			
			// @ Residual and Lagrange Update
			initAlphaBeta(prob_data, VERBOSE);
			

			prob_data.s_x_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() + prob_data.b_x_ineq).max(0.0);
			prob_data.s_y_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() + prob_data.b_y_ineq).max(0.0);
			prob_data.s_z_ineq = ((-prob_data.A_ineq.matrix() * primal_sol_z.matrix()).array() + prob_data.b_z_ineq).max(0.0);

			res_x_ineq = (prob_data.A_ineq.matrix() * primal_sol_x.matrix()).array() - prob_data.b_x_ineq + prob_data.s_x_ineq;
			res_y_ineq = (prob_data.A_ineq.matrix() * primal_sol_y.matrix()).array() - prob_data.b_y_ineq + prob_data.s_y_ineq;
			res_z_ineq = (prob_data.A_ineq.matrix() * primal_sol_z.matrix()).array() - prob_data.b_z_ineq + prob_data.s_z_ineq;

			res_x_vel = prob_data.xdot - prob_data.d_vel * cos(prob_data.alpha_vel) * sin(prob_data.beta_vel);
			res_y_vel = prob_data.ydot - prob_data.d_vel * sin(prob_data.alpha_vel) * sin(prob_data.beta_vel);
			res_z_vel = prob_data.zdot - prob_data.d_vel * cos(prob_data.beta_vel);

			res_x_acc = prob_data.xddot - prob_data.d_acc * cos(prob_data.alpha_acc) * sin(prob_data.beta_acc);
			res_y_acc = prob_data.yddot - prob_data.d_acc * sin(prob_data.alpha_acc) * sin(prob_data.beta_acc);
			if(!prob_data.use_thrust_values)
				res_z_acc = prob_data.zddot - prob_data.d_acc * cos(prob_data.beta_acc);
			else
				res_z_acc = prob_data.zddot + prob_data.gravity - prob_data.d_acc * cos(prob_data.beta_acc);
			
			prob_data.lamda_z = prob_data.lamda_z 
					-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * res_z_vel.matrix()).array()
					-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * res_z_acc.matrix()).array()
					-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_z_ineq.matrix()).array();
		
			prob_data.lamda_x = prob_data.lamda_x 
						-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * res_x_vel.matrix()).array()
						-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * res_x_acc.matrix()).array()
						-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_x_ineq.matrix()).array();

			prob_data.lamda_y = prob_data.lamda_y 
						-prob_data.rho_vel * (prob_data.Pdot.transpose().matrix() * res_y_vel.matrix()).array()
						-prob_data.rho_acc * (prob_data.Pddot.transpose().matrix() * res_y_acc.matrix()).array()
						-prob_data.rho_ineq * (prob_data.A_ineq.transpose().matrix() * res_y_ineq.matrix()).array();

			if(prob_data.jerk_snap_constraints){
				res_x_jerk = prob_data.xdddot - prob_data.d_jerk * cos(prob_data.alpha_jerk) * sin(prob_data.beta_jerk);
				res_y_jerk = prob_data.ydddot - prob_data.d_jerk * sin(prob_data.alpha_jerk) * sin(prob_data.beta_jerk);
				res_z_jerk = prob_data.zdddot - prob_data.d_jerk * cos(prob_data.beta_jerk);

				res_x_snap = prob_data.xddddot - prob_data.d_snap * cos(prob_data.alpha_snap) * sin(prob_data.beta_snap);
				res_y_snap = prob_data.yddddot - prob_data.d_snap * sin(prob_data.alpha_snap) * sin(prob_data.beta_snap);
				res_z_snap = prob_data.zddddot - prob_data.d_snap * cos(prob_data.beta_snap);

				prob_data.lamda_x += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * res_x_jerk.matrix()).array()
						-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * res_x_snap.matrix()).array();

				prob_data.lamda_y += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * res_y_jerk.matrix()).array()
						-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * res_y_snap.matrix()).array();

				prob_data.lamda_z += -prob_data.rho_jerk * (prob_data.Pdddot.transpose().matrix() * res_z_jerk.matrix()).array()
					-prob_data.rho_snap * (prob_data.Pddddot.transpose().matrix() * res_z_snap.matrix()).array();


				prob_data.res_x_jerk_norm = res_x_jerk.matrix().norm();
				prob_data.res_y_jerk_norm = res_y_jerk.matrix().norm();
				prob_data.res_z_jerk_norm = res_z_jerk.matrix().norm();

				prob_data.res_x_snap_norm = res_x_snap.matrix().norm();
				prob_data.res_y_snap_norm = res_y_snap.matrix().norm();
				prob_data.res_z_snap_norm = res_z_snap.matrix().norm();

				if(prob_data.res_x_jerk_norm > thresold || prob_data.res_y_jerk_norm > thresold || prob_data.res_z_jerk_norm > thresold){;
					prob_data.rho_jerk *= prob_data.delta_jerk;
					if(prob_data.rho_jerk > prob_data.rho_jerk_max) prob_data.rho_jerk = prob_data.rho_jerk_max;
				}
				else break_flag++;

				if(prob_data.res_x_snap_norm > thresold || prob_data.res_y_snap_norm > thresold || prob_data.res_z_snap_norm > thresold){;
						prob_data.rho_snap *= prob_data.delta_snap;
						if(prob_data.rho_snap > prob_data.rho_snap_max) prob_data.rho_snap = prob_data.rho_snap_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_jerk_norm = 0.0;
				prob_data.res_y_jerk_norm = 0.0;
				prob_data.res_z_jerk_norm = 0.0;

				prob_data.res_x_snap_norm = 0.0;
				prob_data.res_y_snap_norm = 0.0;
				prob_data.res_z_snap_norm = 0.0;

				break_flag += 2;
			}


			if(prob_data.num_static_obs!=0){
				
				res_x_static_obs = reshape(((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_static_obs * prob_data.d_static_obs * cos(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
				res_y_static_obs = reshape(((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_static_obs * prob_data.d_static_obs * sin(prob_data.alpha_static_obs)).transpose(), prob_data.num_static_obs*prob_data.num, 1);
			
				prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_x_static_obs.matrix()).array();
				prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * res_y_static_obs.matrix()).array();
			
				prob_data.res_x_static_obs_norm = res_x_static_obs.matrix().norm();
				prob_data.res_y_static_obs_norm = res_y_static_obs.matrix().norm();
			
				if(prob_data.res_x_static_obs_norm > thresold || prob_data.res_y_static_obs_norm > thresold){
					prob_data.rho_static_obs *= prob_data.delta_static_obs;
					if(prob_data.rho_static_obs > prob_data.rho_static_obs_max) prob_data.rho_static_obs = prob_data.rho_static_obs_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_static_obs_norm = 0;
				prob_data.res_y_static_obs_norm = 0;
				break_flag++;
			}

			if(prob_data.num_drone!=0){
				
				res_x_drone = reshape(((-prob_data.x_drone).rowwise() + prob_data.x.transpose().row(0) - prob_data.a_drone * prob_data.d_drone * cos(prob_data.alpha_drone) * sin(prob_data.beta_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
				res_y_drone = reshape(((-prob_data.y_drone).rowwise() + prob_data.y.transpose().row(0) - prob_data.b_drone * prob_data.d_drone * sin(prob_data.alpha_drone) * sin(prob_data.beta_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);
				res_z_drone = reshape(((-prob_data.z_drone).rowwise() + prob_data.z.transpose().row(0) - prob_data.c_drone * prob_data.d_drone * cos(prob_data.beta_drone)).transpose(), prob_data.num_drone*prob_data.num, 1);

				prob_data.lamda_z = prob_data.lamda_z -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_z_drone.matrix()).array();
				prob_data.res_z_drone_norm = res_z_drone.matrix().norm();
			
				prob_data.lamda_x = prob_data.lamda_x -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_x_drone.matrix()).array();
				prob_data.lamda_y = prob_data.lamda_y -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * res_y_drone.matrix()).array();
			
				prob_data.res_x_drone_norm = res_x_drone.matrix().norm();
				prob_data.res_y_drone_norm = res_y_drone.matrix().norm();

				if(prob_data.res_x_drone_norm > thresold || prob_data.res_y_drone_norm > thresold || prob_data.res_z_drone_norm > thresold){
					prob_data.rho_drone *= prob_data.delta_drone;
					if(prob_data.rho_drone > prob_data.rho_drone_max) prob_data.rho_drone = prob_data.rho_drone_max;
				}
				else break_flag++;
			}
			else{
				prob_data.res_x_drone_norm = 0;
				prob_data.res_y_drone_norm = 0;
				prob_data.res_z_drone_norm = 0;
				break_flag++;
			}
			
			prob_data.res_x_ineq_norm = res_x_ineq.matrix().norm();
			prob_data.res_y_ineq_norm = res_y_ineq.matrix().norm();
			prob_data.res_z_ineq_norm = res_z_ineq.matrix().norm();

			prob_data.res_x_vel_norm = res_x_vel.matrix().norm();
			prob_data.res_y_vel_norm = res_y_vel.matrix().norm();
			prob_data.res_z_vel_norm = res_z_vel.matrix().norm();

			prob_data.res_x_acc_norm = res_x_acc.matrix().norm();
			prob_data.res_y_acc_norm = res_y_acc.matrix().norm();
			prob_data.res_z_acc_norm = res_z_acc.matrix().norm();
			
			
			if((prob_data.res_x_ineq_norm > thresold || prob_data.res_y_ineq_norm > thresold || prob_data.res_z_ineq_norm > thresold) && prob_data.rho_ineq){;
					prob_data.rho_ineq *= prob_data.delta_ineq;
					if(prob_data.rho_ineq > prob_data.rho_ineq_max) prob_data.rho_ineq = prob_data.rho_ineq_max;
			}
			else break_flag++;
			if(prob_data.res_x_vel_norm > thresold || prob_data.res_y_vel_norm > thresold || prob_data.res_z_vel_norm > thresold){;
					prob_data.rho_vel *= prob_data.delta_vel;
					if(prob_data.rho_vel > prob_data.rho_vel_max) prob_data.rho_vel = prob_data.rho_vel_max;
			}
			else break_flag++;
			if(prob_data.res_x_acc_norm > thresold || prob_data.res_y_acc_norm > thresold || prob_data.res_z_acc_norm > thresold){;
					prob_data.rho_acc *= prob_data.delta_acc;
					if(prob_data.rho_acc > prob_data.rho_acc_max) prob_data.rho_acc = prob_data.rho_acc_max;
			}
			else break_flag++;
			
			if(break_flag == 7)
				break;	
		}
		
		if(break_flag != 7){
				prob_data.weight_goal *= prob_data.delta_aggressive;
				prob_data.weight_smoothness *= prob_data.delta_aggressive;
			}
		else{
			prob_data.weight_goal = prob_data.weight_goal_og;
			prob_data.weight_smoothness = prob_data.weight_smoothness_og;

			if(prob_data.weight_goal > prob_data.weight_goal_og)
				prob_data.weight_goal = prob_data.weight_goal_og;

			if(prob_data.weight_goal > prob_data.weight_smoothness_og)
				prob_data.weight_smoothness = prob_data.weight_smoothness_og;
		}					
	}

	void checkResiduals(probData &prob_data, int VERBOSE)
	{
		float thresold = prob_data.thresold;
		
		
		if(prob_data.res_x_static_obs_norm > thresold || prob_data.res_y_static_obs_norm > thresold){
			ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
			ROS_WARN_STREAM("Static Obstacle Constraints = " << prob_data.res_x_static_obs_norm << " " << prob_data.res_y_static_obs_norm);
		}
		if(prob_data.res_x_drone_norm > thresold || prob_data.res_y_drone_norm > thresold || prob_data.res_z_drone_norm > thresold){
			ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
			ROS_WARN_STREAM("Inter-Agent Collision Constraints = " << prob_data.res_x_drone_norm << " " << prob_data.res_y_drone_norm << " " << prob_data.res_z_drone_norm);
		}
		if(prob_data.res_x_vel_norm > thresold || prob_data.res_y_vel_norm > thresold || prob_data.res_z_vel_norm > thresold){
			ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
			ROS_WARN_STREAM("Velocity Constraints = " << prob_data.res_x_vel_norm << " " << prob_data.res_y_vel_norm << " " << prob_data.res_z_vel_norm);
		}
		if(prob_data.res_x_acc_norm > thresold || prob_data.res_y_acc_norm > thresold || prob_data.res_z_acc_norm > thresold){
			ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
			ROS_WARN_STREAM("Acceleration Constraints = " << prob_data.res_x_acc_norm << " " << prob_data.res_y_acc_norm << " " << prob_data.res_z_acc_norm);
		}
		if(prob_data.res_x_jerk_norm > thresold || prob_data.res_y_jerk_norm > thresold || prob_data.res_z_jerk_norm > thresold){
			ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
			ROS_WARN_STREAM("Jerk Constraints = " << prob_data.res_x_jerk_norm << " " << prob_data.res_y_jerk_norm << " " << prob_data.res_z_jerk_norm);
		}
		if(prob_data.res_x_snap_norm > thresold || prob_data.res_y_snap_norm > thresold || prob_data.res_z_snap_norm > thresold){
			ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
			ROS_WARN_STREAM("Snap Constraints = " << prob_data.res_x_snap_norm << " " << prob_data.res_y_snap_norm << " " << prob_data.res_z_snap_norm);
		}
		if(prob_data.res_x_ineq_norm > thresold || prob_data.res_y_ineq_norm > thresold || prob_data.res_z_ineq_norm > thresold){
			ROS_WARN_STREAM("Drone " << prob_data.id_badge << " MPC Step " << prob_data.mpc_step);
			ROS_WARN_STREAM("Workspace Constraints = " << prob_data.res_x_ineq_norm << " " << prob_data.res_y_ineq_norm << " " << prob_data.res_z_ineq_norm);
		}
	}

	void initObstacles(probData &prob_data, int VERBOSE)
	{
		// prob_data.agent_obs_dist.clear();

		Eigen :: ArrayXXf val = pow(((-prob_data.x_static_obs_og).rowwise() + prob_data.x.transpose().row(0))/(prob_data.a_static_obs_og + prob_data.prox_obs), 2) 
								   + pow(((-prob_data.y_static_obs_og).rowwise() + prob_data.y.transpose().row(0))/(prob_data.b_static_obs_og + prob_data.prox_obs), 2);

		prob_data.x_static_obs = Eigen :: ArrayXXf :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num); 
        prob_data.y_static_obs = Eigen :: ArrayXXf :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num);
        prob_data.z_static_obs = Eigen :: ArrayXXf :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num);

        prob_data.a_static_obs = Eigen :: ArrayXXf :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num);
        prob_data.b_static_obs = Eigen :: ArrayXXf :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num);
        prob_data.c_static_obs = Eigen :: ArrayXXf :: Ones(prob_data.x_static_obs_og.rows(), prob_data.num);


		Eigen :: ArrayXf dist = -prob_data.a_static_obs_og.col(0) + prob_data.lx_drone + prob_data.buffer + (sqrt(pow((prob_data.agents_x(prob_data.id_badge, 0) - prob_data.x_static_obs_og.col(0)),2) 
									+ pow((prob_data.agents_y(prob_data.id_badge, 0) - prob_data.y_static_obs_og.col(0)),2)));
		

		int k = 0;
		prob_data.unify_obs = 0;
		for(int i = 0; i < prob_data.x_static_obs_og.rows(); i++){	
			if((val.row(i) <= 1.0).any()){
				prob_data.x_static_obs.row(k) = prob_data.x_static_obs_og.row(i);
				prob_data.y_static_obs.row(k) = prob_data.y_static_obs_og.row(i);
				prob_data.z_static_obs.row(k) = prob_data.z_static_obs_og.row(i);

				prob_data.a_static_obs.row(k) = prob_data.a_static_obs_og.row(i);
				prob_data.b_static_obs.row(k) = prob_data.b_static_obs_og.row(i);
				prob_data.c_static_obs.row(k) = prob_data.c_static_obs_og.row(i);

				
				if(prob_data.c_static_obs(k, 0) < prob_data.z_max) 
					prob_data.unify_obs++;
				
				k++;
			}	

			// prob_data.agent_obs_dist.push_back(dist(i));
		}

		if(k!=0 && prob_data.x_static_obs_og.rows() > 1){
			prob_data.x_static_obs.conservativeResize(k, prob_data.num);
			prob_data.y_static_obs.conservativeResize(k, prob_data.num);
			prob_data.z_static_obs.conservativeResize(k, prob_data.num);

			prob_data.a_static_obs.conservativeResize(k, prob_data.num);
			prob_data.b_static_obs.conservativeResize(k, prob_data.num);
			prob_data.c_static_obs.conservativeResize(k, prob_data.num);
		}
		prob_data.num_static_obs = k;
	
		prob_data.A_static_obs = prob_data.P;
        for(int i = 0; i < prob_data.num_static_obs - 1; i++) prob_data.A_static_obs = Optim :: stack(prob_data.A_static_obs, prob_data.P, 'v');
	}

	void neigbhoringAgents(probData &prob_data, int VERBOSE)
	{
		// prob_data.inter_agent_dist.clear();

		Eigen :: ArrayXXf agents_x, agents_y, agents_z;

		agents_x = prob_data.agents_x;
		agents_y = prob_data.agents_y;
		agents_z = prob_data.agents_z;

		prob_data.x_drone = Eigen :: ArrayXXf :: Ones(agents_x.rows(), prob_data.num);
		prob_data.y_drone = Eigen :: ArrayXXf :: Ones(agents_y.rows(), prob_data.num);
		prob_data.z_drone = Eigen :: ArrayXXf :: Ones(agents_z.rows(), prob_data.num);

		prob_data.a_drone = Eigen :: ArrayXXf :: Ones(agents_x.rows(), prob_data.num) * (2*prob_data.lx_drone + prob_data.buffer);
        prob_data.b_drone = Eigen :: ArrayXXf :: Ones(agents_x.rows(), prob_data.num) * (2*prob_data.ly_drone + prob_data.buffer);
        prob_data.c_drone = Eigen :: ArrayXXf :: Ones(agents_x.rows(), prob_data.num) * (2*prob_data.lz_drone + prob_data.buffer);

		float del = (abs(prob_data.world - 2.0001)/(prob_data.world - 2.0001) + 1)/2;

		int k = 0;
		Eigen :: ArrayXXf val = pow(((-agents_x).rowwise() + agents_x.row(prob_data.id_badge))/(2*prob_data.lx_drone + prob_data.prox_agent), 2) 
								   + pow(((-agents_y).rowwise() + agents_y.row(prob_data.id_badge))/(2*prob_data.ly_drone + prob_data.prox_agent), 2)
								   + del * pow(((-agents_z).rowwise() + agents_z.row(prob_data.id_badge))/(2*prob_data.lz_drone + prob_data.prox_agent), 2);

		Eigen :: ArrayXf dist = (sqrt(pow((agents_x(prob_data.id_badge, 0) - agents_x.col(0)),2) 
									+ pow((agents_y(prob_data.id_badge, 0) - agents_y.col(0)),2)
									+ pow((agents_z(prob_data.id_badge, 0) - agents_z.col(0)),2))).max(0.0);

		
		for(int i = 0; i < agents_x.rows(); i++)
		{
			if(i == prob_data.id_badge)
				continue;
			if((val.row(i) <= 1.0).any()){
				prob_data.x_drone.row(k) = agents_x.row(i).rightCols(prob_data.num);
				prob_data.y_drone.row(k) = agents_y.row(i).rightCols(prob_data.num);
				prob_data.z_drone.row(k) = agents_z.row(i).rightCols(prob_data.num);
				k++; 
			}

			// prob_data.inter_agent_dist.push_back(dist(i));
		}

		
		if(k!=0 && agents_x.rows() > 1){
			prob_data.x_drone.conservativeResize(k, prob_data.num);
			prob_data.y_drone.conservativeResize(k, prob_data.num);
			prob_data.z_drone.conservativeResize(k, prob_data.num);

			prob_data.a_drone.conservativeResize(k, prob_data.num);
			prob_data.b_drone.conservativeResize(k, prob_data.num);
			prob_data.c_drone.conservativeResize(k, prob_data.num);
		}
		prob_data.num_drone = k;

		prob_data.A_drone = prob_data.P;    
        for(int i = 0; i < prob_data.num_drone - 1; i++) prob_data.A_drone = Optim :: stack(prob_data.A_drone, prob_data.P, 'v');

		if(prob_data.unify_obs!=0 && prob_data.num_static_obs!=0){
			
			if(k!=0){
				prob_data.x_drone = stack(prob_data.x_drone, prob_data.x_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v');
				prob_data.y_drone = stack(prob_data.y_drone, prob_data.y_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v');
				prob_data.z_drone = stack(prob_data.z_drone, prob_data.z_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v'); 

				prob_data.a_drone = stack(prob_data.a_drone, prob_data.a_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v');
				prob_data.b_drone = stack(prob_data.b_drone, prob_data.b_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v');
				prob_data.c_drone = stack(prob_data.c_drone, prob_data.c_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs), 'v');
				
				prob_data.A_drone = stack(prob_data.A_drone, prob_data.A_static_obs.bottomRows((prob_data.num_static_obs - prob_data.unify_obs)*prob_data.num), 'v'); 
			}
			else{
				prob_data.x_drone = prob_data.x_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs);
				prob_data.y_drone = prob_data.y_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs);
				prob_data.z_drone = prob_data.z_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs); 

				prob_data.a_drone = prob_data.a_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs);
				prob_data.b_drone = prob_data.b_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs);
				prob_data.c_drone = prob_data.c_static_obs.bottomRows(prob_data.num_static_obs - prob_data.unify_obs);
				
				prob_data.A_drone = prob_data.A_static_obs.bottomRows((prob_data.num_static_obs - prob_data.unify_obs)*prob_data.num); 
			}
			prob_data.num_drone = prob_data.a_drone.rows();
			prob_data.num_static_obs = prob_data.unify_obs;

			if(prob_data.num_static_obs!=0){
				Eigen :: ArrayXXf temp;
				temp = prob_data.x_static_obs.topRows(prob_data.num_static_obs);
				prob_data.x_static_obs = temp;

				temp = prob_data.y_static_obs.topRows(prob_data.num_static_obs);
				prob_data.y_static_obs = temp;

				temp = prob_data.z_static_obs.topRows(prob_data.num_static_obs);
				prob_data.z_static_obs = temp;

				temp = prob_data.a_static_obs.topRows(prob_data.num_static_obs);
				prob_data.a_static_obs = temp;

				temp = prob_data.b_static_obs.topRows(prob_data.num_static_obs);
				prob_data.b_static_obs = temp;

				temp = prob_data.c_static_obs.topRows(prob_data.num_static_obs);
				prob_data.c_static_obs = temp;

				temp = prob_data.A_static_obs.topRows(prob_data.num_static_obs*prob_data.num);
				prob_data.A_static_obs = temp;
			}

		}
	}

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

		Optim :: five_var PPP = Optim :: computeBernstein(tot_time, prob_data.t_plan, prob_data.num);
		prob_data.nvar = PPP.a.cols();
		prob_data.P = PPP.a;
		prob_data.Pdot = PPP.b;
		prob_data.Pddot = PPP.c;
		prob_data.Pdddot = PPP.d;
		prob_data.Pddddot = PPP.e;

		Optim :: five_var PPP_up = Optim :: computeBernstein(tot_time_up, prob_data.t_plan, prob_data.num_up);
		prob_data.P_up = PPP_up.a;
		prob_data.Pdot_up = PPP_up.b;
		prob_data.Pddot_up = PPP_up.c;
		prob_data.Pdddot_up = PPP_up.d;
		prob_data.Pddddot_up = PPP_up.e;

		
		// @ Position Constraints
		prob_data.A_ineq = Optim :: stack(prob_data.P, -prob_data.P, 'v');

		prob_data.b_x_ineq = Optim :: stack(prob_data.x_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -prob_data.x_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		prob_data.b_y_ineq = Optim :: stack(prob_data.y_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -prob_data.y_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		prob_data.b_z_ineq = Optim :: stack(prob_data.z_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -prob_data.z_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');

		
		// @ Initial Conditions
		prob_data.A_eq = Optim :: stack(prob_data.P.row(0), Optim :: stack(prob_data.Pdot.row(0), prob_data.Pddot.row(0), 'v'), 'v');

		prob_data.b_x_eq = Eigen :: ArrayXXf(3, 1);
		prob_data.b_y_eq = Eigen :: ArrayXXf(3, 1);
		prob_data.b_z_eq = Eigen :: ArrayXXf(3, 1);

		prob_data.b_x_eq << prob_data.x_init, prob_data.vx_init, prob_data.ax_init;
		prob_data.b_y_eq << prob_data.y_init, prob_data.vy_init, prob_data.ay_init;
		prob_data.b_z_eq << prob_data.z_init, prob_data.vz_init, prob_data.az_init;

		
		// @ Static Obstacle Avoidance Constraints
		if(prob_data.num_static_obs!=0){
			prob_data.A_static_obs = prob_data.P;
			for(int i = 0; i < prob_data.num_static_obs - 1; i++) prob_data.A_static_obs = Optim :: stack(prob_data.A_static_obs, prob_data.P, 'v');
			
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
			for(int i = 0; i < prob_data.num_drone - 1; i++) prob_data.A_drone = Optim :: stack(prob_data.A_drone, prob_data.P, 'v');
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
			prob_data.A_v_ineq = Optim :: stack(prob_data.Pdot, -prob_data.Pdot, 'v');

			prob_data.b_vx_ineq = Optim :: stack(prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			prob_data.b_vy_ineq = Optim :: stack(prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			prob_data.b_vz_ineq = Optim :: stack(prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');

			// @ Acceleration Constraints
			prob_data.A_a_ineq = Optim :: stack(prob_data.Pddot, -prob_data.Pddot, 'v');

			if(!prob_data.use_thrust_values && prob_data.world == 2){
				prob_data.b_ax_ineq = Optim :: stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
				prob_data.b_ay_ineq = Optim :: stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
				prob_data.b_az_ineq = Optim :: stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			}
			else{

				/** 
				 * 2x^2 + (x+g)^2 = (1.5g)^2
				 * 2x^2 + x^2 + 2xg + g^2-(1.5g)^2=0
				 * 3x^2 + 2xg + (g^2-(1.5g)^2)
				 * **/

				prob_data.acc_max = (-(2*prob_data.gravity) + sqrt(pow(2*prob_data.gravity,2) - 4*3*(pow(prob_data.gravity,2) - pow(1.5*prob_data.gravity,2))))/6.0; 
				float z_acc_min = prob_data.f_min - prob_data.gravity;
				
				prob_data.b_ax_ineq = Optim :: stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
				prob_data.b_ay_ineq = Optim :: stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
				prob_data.b_az_ineq = Optim :: stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -z_acc_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
				
			}
			// @ Jerk Constraints
			prob_data.A_j_ineq = Optim :: stack(prob_data.Pdddot, -prob_data.Pdddot, 'v');

			prob_data.b_jx_ineq = Optim :: stack(prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			prob_data.b_jy_ineq = Optim :: stack(prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			prob_data.b_jz_ineq = Optim :: stack(prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.jerk_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');

			// @ Snap Constraints
			prob_data.A_s_ineq = Optim :: stack(prob_data.Pddddot, -prob_data.Pddddot, 'v');

			prob_data.b_sx_ineq = Optim :: stack(prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			prob_data.b_sy_ineq = Optim :: stack(prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
			prob_data.b_sz_ineq = Optim :: stack(prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.snap_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
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
			Optim :: initializeOptimizer(prob_data, VERBOSE);
		}
		// @ Initialize alpha betas ds
		if(prob_data.mpc_step > 0){
			// @ Neighboring agents and obstacles -- don't change the order 
			if(!prob_data.free_space)
				Optim :: initObstacles(prob_data, VERBOSE);
			Optim :: neigbhoringAgents(prob_data, VERBOSE);		

			if(prob_data.world == 2)
				Optim :: initAlpha(prob_data, VERBOSE);
			else if(prob_data.world == 3)
				Optim :: initAlphaBeta(prob_data, VERBOSE);
			else
				ROS_ERROR_STREAM("Cannot identify world");
		}
		

		// @ Solve xyz
		if(prob_data.axis_wise){
			if(prob_data.world == 2)
				Optim :: computeXYAxis(prob_data, VERBOSE);
			else if(prob_data.world == 3)
				Optim :: computeXYZAxis(prob_data, VERBOSE);
		}
		else{
			if(prob_data.world == 2)
				Optim :: computeXY(prob_data, VERBOSE);
			else if(prob_data.world == 3)
				Optim :: computeXYZ(prob_data, VERBOSE);
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
			Optim :: checkResiduals(prob_data, VERBOSE);
	}
}

