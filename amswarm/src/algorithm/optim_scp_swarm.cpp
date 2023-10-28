#include <iostream>
#include <fstream>
#include <vector>
#include <random>

#include <ros/package.h>
#include <ros/ros.h>

#include "yaml-cpp/yaml.h"

#include "algorithm/optim_scp_swarm.hpp"
#include <eigen-quadprog/QuadProg.h>
#include <eigen-quadprog/eigen_quadprog_api.h>

typedef Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,
        Eigen::Dynamic, Eigen::ColMajor>> MapMatrixCol;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXdRow;

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
Eigen :: ArrayXXf block_diag(Eigen :: ArrayXXf arr1, Eigen :: ArrayXXf arr2)
{
	Eigen :: ArrayXXf temp(arr1.rows() + arr2.rows(), arr1.cols() + arr2.cols());

	temp = 0.0;
	temp.topRows(arr1.rows()).leftCols(arr1.cols()) = arr1;
	temp.bottomRows(arr2.rows()).rightCols(arr2.cols()) = arr2;

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

int computeXYZ(probData &prob_data, int VERBOSE){
	
	int tries = 0;
	prob_data.qp_fail = 1;
	
	// @ INEQUALITY MATRICES
	Eigen :: ArrayXXf A_pos_ineq = block_diag(block_diag(stack(prob_data.P, -prob_data.P, 'v'), stack(prob_data.P, -prob_data.P, 'v')), stack(prob_data.P, -prob_data.P, 'v'));
	Eigen :: ArrayXXf A_vel_ineq = block_diag(block_diag(stack(prob_data.Pdot, -prob_data.Pdot, 'v'), stack(prob_data.Pdot, -prob_data.Pdot, 'v')), stack(prob_data.Pdot, -prob_data.Pdot, 'v'));
	Eigen :: ArrayXXf A_acc_ineq = block_diag(block_diag(stack(prob_data.Pddot, -prob_data.Pddot, 'v'), stack(prob_data.Pddot, -prob_data.Pddot, 'v')), stack(prob_data.Pddot, -prob_data.Pddot, 'v'));

	Eigen :: ArrayXXf b_x_ineq = stack(prob_data.x_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -prob_data.x_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	Eigen :: ArrayXXf b_y_ineq = stack(prob_data.y_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -prob_data.y_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	Eigen :: ArrayXXf b_z_ineq = stack(prob_data.z_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -prob_data.z_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	Eigen :: ArrayXXf b_pos_ineq = stack(stack(b_x_ineq, b_y_ineq, 'v'), b_z_ineq,'v'); 

	Eigen :: ArrayXXf b_vx_ineq = stack(prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	Eigen :: ArrayXXf b_vy_ineq = stack(prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	Eigen :: ArrayXXf b_vz_ineq = stack(prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.vel_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	Eigen :: ArrayXXf b_vel_ineq = stack(stack(b_vx_ineq, b_vy_ineq, 'v'), b_vz_ineq, 'v');

	Eigen :: ArrayXXf b_ax_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	Eigen :: ArrayXXf b_ay_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	Eigen :: ArrayXXf b_az_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
	
	
	if(prob_data.use_thrust_values){
		prob_data.acc_max = (-(2*prob_data.gravity) + sqrt(pow(2*prob_data.gravity,2) - 4*3*(pow(prob_data.gravity,2) - pow(1.5*prob_data.gravity,2))))/6.0; 
		float z_acc_min = prob_data.f_min - prob_data.gravity;
		
		b_ax_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		b_ay_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		b_az_ineq = stack(prob_data.acc_max * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), -z_acc_min * Eigen :: ArrayXXf :: Ones(prob_data.num, 1), 'v');
		
	}

	Eigen :: ArrayXXf b_acc_ineq = stack(stack(b_ax_ineq, b_ay_ineq, 'v'), b_az_ineq, 'v');

	Eigen :: ArrayXXf A_slack_ineq, b_slack_ineq;
	// @ EQUALITY MATRICES
	prob_data.A_eq = block_diag(block_diag(stack(prob_data.P.row(0), stack(prob_data.Pdot.row(0), prob_data.Pddot.row(0), 'v'), 'v'), stack(prob_data.P.row(0), stack(prob_data.Pdot.row(0), prob_data.Pddot.row(0), 'v'), 'v')), stack(prob_data.P.row(0), stack(prob_data.Pdot.row(0), prob_data.Pddot.row(0), 'v'), 'v'));
	prob_data.b_eq = Eigen :: ArrayXXf(9, 1);
	prob_data.b_eq << prob_data.x_init, prob_data.vx_init, prob_data.ax_init,
					prob_data.y_init, prob_data.vy_init, prob_data.ay_init,
					prob_data.z_init, prob_data.vz_init, prob_data.az_init;

	if(prob_data.num_drone!=0){
		if(prob_data.num_static_obs!=0){
			prob_data.num_static_obs += prob_data.num_drone;

			prob_data.x_static_obs = stack(prob_data.x_static_obs, prob_data.x_drone, 'v');
			prob_data.y_static_obs = stack(prob_data.y_static_obs, prob_data.y_drone, 'v');
			prob_data.z_static_obs = stack(prob_data.z_static_obs, prob_data.z_drone, 'v');

			prob_data.a_static_obs = stack(prob_data.a_static_obs, prob_data.a_drone, 'v');;
			prob_data.b_static_obs = stack(prob_data.b_static_obs, prob_data.b_drone, 'v');
			prob_data.c_static_obs = stack(prob_data.c_static_obs, prob_data.c_drone, 'v');
		}
		else{
			
			prob_data.num_static_obs += prob_data.num_drone;

			prob_data.x_static_obs = prob_data.x_drone;
			prob_data.y_static_obs = prob_data.y_drone;
			prob_data.z_static_obs = prob_data.z_drone;

			prob_data.a_static_obs = prob_data.a_drone;
			prob_data.b_static_obs = prob_data.b_drone;
			prob_data.c_static_obs = prob_data.c_drone;
		
		}
	}
	prob_data.colliding_step = 1000;
	prob_data.weight_lin_slack = prob_data.weight_lin_slack_og;
	prob_data.weight_quad_slack = prob_data.weight_quad_slack_og;
	
	while(tries < 1 && prob_data.qp_fail){		
		
		Eigen :: ArrayXXf temp_cost, temp_lincost;

		if(prob_data.num_static_obs != 0){
			
			// STACK ALL INEQUALITIES
			if(tries == 0){

				if(!prob_data.on_demand)
					continuousCA(prob_data, VERBOSE);
				else{
					prob_data.colliding_step = collidingStep(prob_data, VERBOSE);
					
					if(prob_data.colliding_step == 1000){
						prob_data.num_static_obs = 0;
						prob_data.num_drone = 0;
						continue;
					}
					else {
						ondemandCA(prob_data, VERBOSE);
					}
				}
				
				prob_data.slack = Eigen :: ArrayXXf :: Zero(prob_data.num*prob_data.num_static_obs, 1);
				prob_data.A_eq = block_diag(prob_data.A_eq, Eigen::ArrayXXf::Zero(prob_data.num*prob_data.num_static_obs, prob_data.num*prob_data.num_static_obs));
				A_pos_ineq = block_diag(A_pos_ineq, Eigen::ArrayXXf::Zero(prob_data.num*prob_data.num_static_obs, prob_data.num*prob_data.num_static_obs));
				A_vel_ineq = block_diag(A_vel_ineq, Eigen::ArrayXXf::Zero(prob_data.num*prob_data.num_static_obs, prob_data.num*prob_data.num_static_obs));
				A_acc_ineq = block_diag(A_acc_ineq, Eigen::ArrayXXf::Zero(prob_data.num*prob_data.num_static_obs, prob_data.num*prob_data.num_static_obs));
				A_slack_ineq = stack(Eigen::ArrayXXf::Zero(prob_data.num*prob_data.num_static_obs, 3*prob_data.nvar), -Eigen::MatrixXf::Identity(prob_data.num*prob_data.num_static_obs, prob_data.num*prob_data.num_static_obs), 'h');

				b_pos_ineq = stack(b_pos_ineq, Eigen::ArrayXXf::Zero(prob_data.num*prob_data.num_static_obs, 1),'v');
				b_vel_ineq = stack(b_vel_ineq, Eigen::ArrayXXf::Zero(prob_data.num*prob_data.num_static_obs, 1),'v');
				b_acc_ineq = stack(b_acc_ineq, Eigen::ArrayXXf::Zero(prob_data.num*prob_data.num_static_obs, 1),'v');
				b_slack_ineq = Eigen :: ArrayXXf :: Zero(A_slack_ineq.rows(), 1); 

				prob_data.b_eq = stack(prob_data.b_eq, Eigen::ArrayXXf::Zero(prob_data.num*prob_data.num_static_obs, 1),'v');
			}

			prob_data.A_ineq = stack(stack(stack(A_pos_ineq, A_vel_ineq, 'v'), stack(A_acc_ineq, prob_data.A_coll, 'v'), 'v'), A_slack_ineq, 'v');
			prob_data.b_ineq = stack(stack(stack(b_pos_ineq, b_vel_ineq, 'v'), stack(b_acc_ineq, prob_data.b_coll, 'v'), 'v'), b_slack_ineq, 'v');

			
			temp_cost = block_diag(prob_data.weight_goal*prob_data.cost_goal + prob_data.weight_smoothness*prob_data.cost_smoothness, prob_data.weight_quad_slack*prob_data.cost_slack);
			temp_lincost = stack(-prob_data.weight_goal * prob_data.lincost_goal, prob_data.weight_lin_slack * Eigen::ArrayXXf::Ones(prob_data.num*prob_data.num_static_obs, 1), 'v');
		}
		else{
			prob_data.A_ineq = stack(stack(A_pos_ineq, A_vel_ineq, 'v'), A_acc_ineq, 'v');
			prob_data.b_ineq = stack(stack(b_pos_ineq, b_vel_ineq, 'v'), b_acc_ineq, 'v');

			temp_cost = prob_data.weight_goal*prob_data.cost_goal + prob_data.weight_smoothness*prob_data.cost_smoothness;
			temp_lincost = -prob_data.weight_goal * prob_data.lincost_goal;
		}
		
		// SOLVE QP
		Eigen::QuadProgDense solver_xyz(temp_cost.rows(), prob_data.A_eq.rows(), prob_data.A_ineq.rows());

		solver_xyz.solve((temp_cost).cast<double>().matrix(), temp_lincost.cast<double>().matrix(), 
						prob_data.A_eq.cast<double>().matrix(), prob_data.b_eq.cast<double>().matrix(),
						prob_data.A_ineq.cast<double>().matrix(), prob_data.b_ineq.cast<double>().matrix());

		
		Eigen :: ArrayXXf sol = solver_xyz.result().cast<float>();
		Eigen :: ArrayXXf sol_xy = sol.topRows(2*prob_data.nvar);
		Eigen :: ArrayXXf sol_xyz = sol.topRows(3*prob_data.nvar);

		Eigen :: ArrayXXf sol_x = sol_xy.topRows(prob_data.nvar);
		Eigen :: ArrayXXf sol_y = sol_xy.bottomRows(prob_data.nvar);
		Eigen :: ArrayXXf sol_z = sol_xyz.bottomRows(prob_data.nvar);

		if(prob_data.num_static_obs!=0)
			prob_data.slack = sol.bottomRows(prob_data.num*prob_data.num_static_obs);

		prob_data.x = prob_data.P.matrix() * sol_x.matrix();
		prob_data.y = prob_data.P.matrix() * sol_y.matrix();
		prob_data.z = prob_data.P.matrix() * sol_z.matrix();

		prob_data.xdot = prob_data.Pdot.matrix() * sol_x.matrix();
		prob_data.ydot = prob_data.Pdot.matrix() * sol_y.matrix();
		prob_data.zdot = prob_data.Pdot.matrix() * sol_z.matrix();

		prob_data.xddot = prob_data.Pddot.matrix() * sol_x.matrix();
		prob_data.yddot = prob_data.Pddot.matrix() * sol_y.matrix();
		prob_data.zddot = prob_data.Pddot.matrix() * sol_z.matrix();

		prob_data.xdddot = prob_data.Pdddot.matrix() * sol_x.matrix();
		prob_data.ydddot = prob_data.Pdddot.matrix() * sol_y.matrix();
		prob_data.zdddot = prob_data.Pdddot.matrix() * sol_z.matrix();

		prob_data.xddddot = prob_data.Pddddot.matrix() * sol_x.matrix();
		prob_data.yddddot = prob_data.Pddddot.matrix() * sol_y.matrix();
		prob_data.zddddot = prob_data.Pddddot.matrix() * sol_z.matrix();
		
		prob_data.x_up = prob_data.P_up.matrix() * sol_x.matrix();
		prob_data.y_up = prob_data.P_up.matrix() * sol_y.matrix();
		prob_data.z_up = prob_data.P_up.matrix() * sol_z.matrix();

		prob_data.xdot_up = prob_data.Pdot_up.matrix() * sol_x.matrix();
		prob_data.ydot_up = prob_data.Pdot_up.matrix() * sol_y.matrix();
		prob_data.zdot_up = prob_data.Pdot_up.matrix() * sol_z.matrix();

		prob_data.xddot_up = prob_data.Pddot_up.matrix() * sol_x.matrix();
		prob_data.yddot_up = prob_data.Pddot_up.matrix() * sol_y.matrix();
		prob_data.zddot_up = prob_data.Pddot_up.matrix() * sol_z.matrix();

		prob_data.qp_fail = solver_xyz.fail();
		tries++;

		// prob_data.weight_lin_slack *= 0.01;
		// prob_data.weight_quad_slack *= 0.01;

		// if(prob_data.weight_lin_slack < 1.0)
		// 	prob_data.weight_lin_slack = 1.0;

		// if(prob_data.weight_quad_slack < 1.0)
		// 	prob_data.weight_quad_slack = 1.0;

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
	for(int i = 0; i < prob_data.x_static_obs_og.rows(); i++){	
		if((val.row(i) <= 1.0).any()){
			prob_data.x_static_obs.row(k) = prob_data.x_static_obs_og.row(i);
			prob_data.y_static_obs.row(k) = prob_data.y_static_obs_og.row(i);
			prob_data.z_static_obs.row(k) = prob_data.z_static_obs_og.row(i);

			prob_data.a_static_obs.row(k) = prob_data.a_static_obs_og.row(i);
			prob_data.b_static_obs.row(k) = prob_data.b_static_obs_og.row(i);
			prob_data.c_static_obs.row(k) = prob_data.c_static_obs_og.row(i);
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
}

void neigbhoringAgents(probData &prob_data, int VERBOSE)
{
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
}

void continuousCA(probData &prob_data, int VERBOSE){
	// COLLISION AVOIDANCE CONSTRAINTS
	prob_data.cost_slack = Eigen :: ArrayXXf(prob_data.num*prob_data.num_static_obs, prob_data.num*prob_data.num_static_obs);
	prob_data.cost_slack.matrix().setIdentity();

	if(prob_data.world == 2){
		Eigen :: ArrayXXf A_f, temp_x, temp_y;
		
		Eigen :: ArrayXXf temp_delta_fx = -2.0 *((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0))/pow(prob_data.a_static_obs,2); 
		Eigen :: ArrayXXf temp_A_fx = reshape(temp_delta_fx.transpose(), prob_data.num_static_obs*prob_data.num, 1);

		Eigen :: ArrayXXf temp_delta_fy = -2.0 *((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0))/pow(prob_data.b_static_obs,2); 
		Eigen :: ArrayXXf temp_A_fy = reshape(temp_delta_fy.transpose(), prob_data.num_static_obs*prob_data.num, 1);
		

		for(int i = 0; i < prob_data.num_static_obs; i++){
			if(i == 0){
				prob_data.A_obs = prob_data.P;
				temp_x = prob_data.x;
				temp_y = prob_data.y;
			}
			else{ 
				prob_data.A_obs = stack(prob_data.A_obs, prob_data.P, 'v');
				temp_x = stack(temp_x, prob_data.x, 'v');
				temp_y = stack(temp_y, prob_data.y, 'v');
			}
		}
		
		A_f = stack(prob_data.A_obs.colwise() * temp_A_fx.col(0), prob_data.A_obs.colwise() * temp_A_fy.col(0), 'h');
		
		Eigen :: ArrayXXf A_coll = stack(A_f, -Eigen::MatrixXf::Identity(A_f.rows(), A_f.rows()), 'h');
		
		Eigen :: ArrayXXf temp_b_coll = pow((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0), 2)/pow(prob_data.a_static_obs, 2) + pow((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0), 2)/pow(prob_data.b_static_obs, 2) - 1.0;
		Eigen :: ArrayXXf b_coll = reshape(temp_b_coll.transpose(), prob_data.num_static_obs*prob_data.num, 1) + temp_A_fx*temp_x + temp_A_fy*temp_y;

		prob_data.A_coll = A_coll;
		prob_data.b_coll = b_coll;
	}
	else{
		Eigen :: ArrayXXf A_f, temp_x, temp_y, temp_z;
		
		Eigen :: ArrayXXf temp_delta_fx = -2.0 *((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0))/pow(prob_data.a_static_obs,2); 
		Eigen :: ArrayXXf temp_A_fx = reshape(temp_delta_fx.transpose(), prob_data.num_static_obs*prob_data.num, 1);

		Eigen :: ArrayXXf temp_delta_fy = -2.0 *((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0))/pow(prob_data.b_static_obs,2); 
		Eigen :: ArrayXXf temp_A_fy = reshape(temp_delta_fy.transpose(), prob_data.num_static_obs*prob_data.num, 1);
		
		Eigen :: ArrayXXf temp_delta_fz = -2.0 *((-prob_data.z_static_obs).rowwise() + prob_data.z.transpose().row(0))/pow(prob_data.c_static_obs,2); 
		Eigen :: ArrayXXf temp_A_fz = reshape(temp_delta_fz.transpose(), prob_data.num_static_obs*prob_data.num, 1);

		for(int i = 0; i < prob_data.num_static_obs; i++){
			if(i == 0){
				prob_data.A_obs = prob_data.P;
				temp_x = prob_data.x;
				temp_y = prob_data.y;
				temp_z = prob_data.z;
			}
			else{ 
				prob_data.A_obs = stack(prob_data.A_obs, prob_data.P, 'v');
				temp_x = stack(temp_x, prob_data.x, 'v');
				temp_y = stack(temp_y, prob_data.y, 'v');
				temp_z = stack(temp_z, prob_data.z, 'v');
			}
		}
		
		A_f = stack(stack(prob_data.A_obs.colwise() * temp_A_fx.col(0), prob_data.A_obs.colwise() * temp_A_fy.col(0), 'h'), prob_data.A_obs.colwise() * temp_A_fz.col(0), 'h');
		
		Eigen :: ArrayXXf A_coll = stack(A_f, -Eigen::MatrixXf::Identity(A_f.rows(), A_f.rows()), 'h');
		
		Eigen :: ArrayXXf temp_b_coll = pow((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0), 2)/pow(prob_data.a_static_obs, 2) 
									+ pow((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0), 2)/pow(prob_data.b_static_obs, 2)
									+ pow((-prob_data.z_static_obs).rowwise() + prob_data.z.transpose().row(0), 2)/pow(prob_data.c_static_obs, 2) - 1.0;

		Eigen :: ArrayXXf b_coll = reshape(temp_b_coll.transpose(), prob_data.num_static_obs*prob_data.num, 1) + temp_A_fx*temp_x + temp_A_fy*temp_y + temp_A_fz*temp_z;

		prob_data.A_coll = A_coll;
		prob_data.b_coll = b_coll;
	}
}

int collidingStep(probData &prob_data, int VERBOSE){
	int colliding_step = 1000;
	
	float del = (abs(prob_data.world - 2.0001)/(prob_data.world - 2.0001) + 1)/2;

	for(int i = 0; i < prob_data.num_static_obs; i++){
		for(int j = 0; j < prob_data.num; j++){
			float value = pow(prob_data.x(j) - prob_data.x_static_obs(i, j), 2)/pow(prob_data.a_static_obs(i, j), 2) 
						+  pow(prob_data.y(j) - prob_data.y_static_obs(i, j), 2)/pow(prob_data.b_static_obs(i, j), 2)
						+  del * pow(prob_data.z(j) - prob_data.z_static_obs(i, j), 2)/pow(prob_data.c_static_obs(i, j), 2);

			if(value <= 1){
				if(colliding_step > j){
					colliding_step = j;
				}
			}	 
		}
	}
	prob_data.colliding_step = std:: max(colliding_step - 1, 1);
	if(colliding_step == 1000)
		return colliding_step;

	int k = 0;
	Eigen :: ArrayXXf temp_x_obs(prob_data.num_static_obs, prob_data.num), temp_y_obs(prob_data.num_static_obs, prob_data.num), temp_z_obs(prob_data.num_static_obs, prob_data.num), 
					temp_a_obs(prob_data.num_static_obs, prob_data.num), temp_b_obs(prob_data.num_static_obs, prob_data.num), temp_c_obs(prob_data.num_static_obs, prob_data.num);

	if(prob_data.num_static_obs > 1){
		for(int i = 0; i < prob_data.num_static_obs; i++){
			float dist = pow(prob_data.x(colliding_step) - prob_data.x_static_obs(i, colliding_step), 2)/pow(prob_data.a_static_obs(i, colliding_step) + prob_data.prox_agent,2) 
						+ pow(prob_data.y(colliding_step) - prob_data.y_static_obs(i, colliding_step), 2)/pow(prob_data.b_static_obs(i, colliding_step)+ prob_data.prox_agent,2)
						+ pow(prob_data.z(colliding_step) - prob_data.z_static_obs(i, colliding_step), 2)/pow(prob_data.c_static_obs(i, colliding_step)+ prob_data.prox_agent,2);

			if(dist <= 1.0){
				temp_x_obs.row(k) = prob_data.x_static_obs.row(i);
				temp_y_obs.row(k) = prob_data.y_static_obs.row(i);
				temp_z_obs.row(k) = prob_data.z_static_obs.row(i);
				
				temp_a_obs.row(k) = prob_data.a_static_obs.row(i);
				temp_b_obs.row(k) = prob_data.b_static_obs.row(i);
				temp_c_obs.row(k) = prob_data.c_static_obs.row(i);
				k++;
			}
		}
		temp_x_obs.conservativeResize(k, prob_data.num);
		temp_y_obs.conservativeResize(k, prob_data.num);
		temp_z_obs.conservativeResize(k, prob_data.num);

		temp_a_obs.conservativeResize(k, prob_data.num);
		temp_b_obs.conservativeResize(k, prob_data.num);
		temp_c_obs.conservativeResize(k, prob_data.num);

		prob_data.x_static_obs = temp_x_obs;
		prob_data.y_static_obs = temp_y_obs;
		prob_data.z_static_obs = temp_z_obs;

		prob_data.a_static_obs = temp_a_obs;
		prob_data.b_static_obs = temp_b_obs;
		prob_data.c_static_obs = temp_c_obs;

		prob_data.num_static_obs = prob_data.a_static_obs.rows();
	}
	return colliding_step;
}
void ondemandCA(probData &prob_data, int VERBOSE){		
	
	prob_data.cost_slack = Eigen :: ArrayXXf(prob_data.num*prob_data.num_static_obs, prob_data.num*prob_data.num_static_obs);
	prob_data.cost_slack.matrix().setIdentity();

	if(prob_data.world == 2){
		Eigen :: ArrayXXf A_f, temp_x, temp_y;
		
		for(int i = 0; i < prob_data.num_static_obs; i++){
			if(i == 0){
				prob_data.A_obs = prob_data.P;
				temp_x = prob_data.x;
				temp_y = prob_data.y;
			}
			else{ 
				prob_data.A_obs = stack(prob_data.A_obs, prob_data.P, 'v');
				temp_x = stack(temp_x, prob_data.x, 'v');
				temp_y = stack(temp_y, prob_data.y, 'v');
			}
		}

		Eigen :: ArrayXXf temp_delta_fx = -2.0 *((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0))/pow(prob_data.a_static_obs,2); 
		Eigen :: ArrayXXf temp_A_fx = reshape(temp_delta_fx.transpose(), prob_data.num_static_obs*prob_data.num, 1);

		Eigen :: ArrayXXf temp_delta_fy = -2.0 *((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0))/pow(prob_data.b_static_obs,2); 
		Eigen :: ArrayXXf temp_A_fy = reshape(temp_delta_fy.transpose(), prob_data.num_static_obs*prob_data.num, 1);

		A_f = stack(prob_data.A_obs.colwise() * temp_A_fx.col(0), prob_data.A_obs.colwise() * temp_A_fy.col(0), 'h');
		
		Eigen :: ArrayXXf temp_b_coll = pow((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0), 2)/pow(prob_data.a_static_obs, 2) + pow((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0), 2)/pow(prob_data.b_static_obs, 2) - 1.0;
		Eigen :: ArrayXXf b_coll = reshape(temp_b_coll.transpose(), prob_data.num_static_obs*prob_data.num, 1) + temp_A_fx*temp_x + temp_A_fy*temp_y;

		for(int i = 0; i < prob_data.num_static_obs; i++){
			for(int j = 0; j < prob_data.num; j++){
				if(j != prob_data.colliding_step){
					A_f.row(j + i*prob_data.num) = 0.0;
					b_coll.row(j + i*prob_data.num) = 1000.0;
				}
			}
		}
		
		Eigen :: ArrayXXf A_coll = stack(A_f, -Eigen::MatrixXf::Identity(A_f.rows(), A_f.rows()), 'h');
		
		prob_data.A_coll = A_coll;
		prob_data.b_coll = b_coll;
	}
	else{
		Eigen :: ArrayXXf A_f, temp_x, temp_y, temp_z;
		
		for(int i = 0; i < prob_data.num_static_obs; i++){
			if(i == 0){
				prob_data.A_obs = prob_data.P;
				temp_x = prob_data.x;
				temp_y = prob_data.y;
				temp_z = prob_data.z;
			}
			else{ 
				prob_data.A_obs = stack(prob_data.A_obs, prob_data.P, 'v');
				temp_x = stack(temp_x, prob_data.x, 'v');
				temp_y = stack(temp_y, prob_data.y, 'v');
				temp_z = stack(temp_z, prob_data.z, 'v');
			}
		}

		Eigen :: ArrayXXf temp_delta_fx = -2.0 *((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0))/pow(prob_data.a_static_obs,2); 
		Eigen :: ArrayXXf temp_A_fx = reshape(temp_delta_fx.transpose(), prob_data.num_static_obs*prob_data.num, 1);

		Eigen :: ArrayXXf temp_delta_fy = -2.0 *((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0))/pow(prob_data.b_static_obs,2); 
		Eigen :: ArrayXXf temp_A_fy = reshape(temp_delta_fy.transpose(), prob_data.num_static_obs*prob_data.num, 1);

		Eigen :: ArrayXXf temp_delta_fz = -2.0 *((-prob_data.z_static_obs).rowwise() + prob_data.z.transpose().row(0))/pow(prob_data.c_static_obs,2); 
		Eigen :: ArrayXXf temp_A_fz = reshape(temp_delta_fz.transpose(), prob_data.num_static_obs*prob_data.num, 1);

		A_f = stack(stack(prob_data.A_obs.colwise() * temp_A_fx.col(0), prob_data.A_obs.colwise() * temp_A_fy.col(0), 'h'), prob_data.A_obs.colwise() * temp_A_fz.col(0), 'h');
		
		Eigen :: ArrayXXf temp_b_coll = pow((-prob_data.x_static_obs).rowwise() + prob_data.x.transpose().row(0), 2)/pow(prob_data.a_static_obs, 2) 
									+ pow((-prob_data.y_static_obs).rowwise() + prob_data.y.transpose().row(0), 2)/pow(prob_data.b_static_obs, 2)
									+ pow((-prob_data.z_static_obs).rowwise() + prob_data.z.transpose().row(0), 2)/pow(prob_data.c_static_obs, 2) - 1.0;

		Eigen :: ArrayXXf b_coll = reshape(temp_b_coll.transpose(), prob_data.num_static_obs*prob_data.num, 1) + temp_A_fx*temp_x + temp_A_fy*temp_y + temp_A_fz*temp_z;

		for(int i = 0; i < prob_data.num_static_obs; i++){
			for(int j = 0; j < prob_data.num; j++){
				if(j != prob_data.colliding_step){
					A_f.row(j + i*prob_data.num) = 0.0;
					b_coll.row(j + i*prob_data.num) = 1000.0;
				}
			}
		}
		
		Eigen :: ArrayXXf A_coll = stack(A_f, -Eigen::MatrixXf::Identity(A_f.rows(), A_f.rows()), 'h');
		
		prob_data.A_coll = A_coll;
		prob_data.b_coll = b_coll;
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

	prob_data.world = 3;// params["world"].as<int>();
	prob_data.num = params["num"].as<float>();
	
	prob_data.t_plan = params["t_plan"].as<float>();
	prob_data.num_up = params["num_up"].as<float>();
	prob_data.dist_stop = params["dist_stop"].as<float>();
	prob_data.kappa = params["kappa"].as<float>();

	prob_data.max_time = params["max_time"].as<float>();
	prob_data.on_demand = params["on_demand"].as<bool>();

	prob_data.weight_quad_slack = params["weight_quad_slack"].as<float>();
	prob_data.weight_quad_slack_og = params["weight_quad_slack"].as<float>();
	prob_data.weight_lin_slack = params["weight_lin_slack"].as<float>();
	prob_data.weight_lin_slack_og = params["weight_lin_slack"].as<float>();
	prob_data.weight_goal = params["weight_goal"].as<float>();
	prob_data.weight_smoothness = params["weight_smoothness"].as<float>();
			
	prob_data.x_min = _x_lim[0] + params["a_drone"].as<float>();
	prob_data.y_min = _y_lim[0] + params["b_drone"].as<float>();
	prob_data.z_min = _z_lim[0] + params["a_drone"].as<float>();

	prob_data.x_max = _x_lim[1] - params["a_drone"].as<float>();
	prob_data.y_max = _y_lim[1] - params["b_drone"].as<float>();
	prob_data.z_max = _z_lim[1] - params["a_drone"].as<float>();

	prob_data.vel_max = params["vel_max"].as<float>()/sqrt(3);
	prob_data.acc_max = params["acc_max"].as<float>()/sqrt(3);

	prob_data.prox_obs = params["prox_obs"].as<float>();
	prob_data.prox_agent = params["prox_agent"].as<float>();
	prob_data.buffer = params["buffer"].as<float>();
	prob_data.gravity = params["gravity"].as<float>();	
	prob_data.use_thrust_values = params["use_thrust_values"].as<bool>();
	prob_data.f_min = params["f_min"].as<float>() * prob_data.gravity;
	prob_data.f_max = params["f_max"].as<float>() * prob_data.gravity;

	prob_data.solver = params["solver"].as<std::string>();
	prob_data.mpc_step = 0;

	// @ Initial State
	prob_data.vx_init = 0.0;
	prob_data.vy_init = 0.0;
	prob_data.vz_init = 0.0;
	
	prob_data.ax_init = 0.0;
	prob_data.ay_init = 0.0;
	prob_data.az_init = 0.0;

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

	prob_data.dt = prob_data.t_plan / prob_data.num;

	// @ TRAJ INIT
	prob_data.x = prob_data.x_init * Eigen :: ArrayXXf :: Ones(prob_data.num, 1);
	prob_data.y = prob_data.y_init * Eigen :: ArrayXXf :: Ones(prob_data.num, 1);
	prob_data.z = prob_data.z_init * Eigen :: ArrayXXf :: Ones(prob_data.num, 1);

	prob_data.xdot = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * prob_data.vx_init;
	prob_data.ydot = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * prob_data.vy_init;
	prob_data.zdot = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * prob_data.vz_init;

	prob_data.xddot = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * prob_data.ax_init;
	prob_data.yddot = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * prob_data.ay_init;
	prob_data.zddot = Eigen :: ArrayXXf :: Ones(prob_data.num, 1) * prob_data.az_init;
		
	prob_data.lx_drone = params["a_drone"].as<float>();
	prob_data.ly_drone = params["b_drone"].as<float>();
	prob_data.lz_drone = params["c_drone"].as<float>();
	prob_data.rmin = prob_data.lx_drone * 2;
	
	// @ Inter-agent 
	if(prob_data.num_drone != 0){
		
		
		prob_data.a_drone = Eigen :: ArrayXXf :: Ones(prob_data.num_drone, prob_data.num) * (2*prob_data.lx_drone + prob_data.buffer);
		prob_data.b_drone = Eigen :: ArrayXXf :: Ones(prob_data.num_drone, prob_data.num) * (2*prob_data.ly_drone + prob_data.buffer);
		prob_data.c_drone = Eigen :: ArrayXXf :: Ones(prob_data.num_drone, prob_data.num) * (2*prob_data.lz_drone + prob_data.buffer);

	}
	
	// @ Static Obstacles
	if(prob_data.num_static_obs!=0){
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

	// @ COST MATRICES
	// prob_data.cost_smoothness = block_diag(block_diag(prob_data.Pddddot.transpose().matrix() * prob_data.Pddddot.matrix(), prob_data.Pddddot.transpose().matrix() * prob_data.Pddddot.matrix()), prob_data.Pddddot.transpose().matrix() * prob_data.Pddddot.matrix());
	prob_data.cost_goal = block_diag(block_diag(prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.P.bottomRows(prob_data.kappa).matrix(), prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.P.bottomRows(prob_data.kappa).matrix()), prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.P.bottomRows(prob_data.kappa).matrix());
	prob_data.lincost_goal = stack(stack(prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.x_ref.matrix(), prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.y_ref.matrix(), 'v'), prob_data.P.bottomRows(prob_data.kappa).transpose().matrix() * prob_data.z_ref.matrix(),'v');
	if(prob_data.params["order_smoothness"].as<int>() == 4)
		prob_data.cost_smoothness = block_diag(block_diag(prob_data.Pddddot.transpose().matrix() * prob_data.Pddddot.matrix(), prob_data.Pddddot.transpose().matrix() * prob_data.Pddddot.matrix()), prob_data.Pddddot.transpose().matrix() * prob_data.Pddddot.matrix());
	else if(prob_data.params["order_smoothness"].as<int>() == 3)
		prob_data.cost_smoothness = block_diag(block_diag(prob_data.Pdddot.transpose().matrix() * prob_data.Pdddot.matrix(), prob_data.Pdddot.transpose().matrix() * prob_data.Pdddot.matrix()), prob_data.Pdddot.transpose().matrix() * prob_data.Pdddot.matrix());
	else if(prob_data.params["order_smoothness"].as<int>() == 2)
		prob_data.cost_smoothness = block_diag(block_diag(prob_data.Pddot.transpose().matrix() * prob_data.Pddot.matrix(), prob_data.Pddot.transpose().matrix() * prob_data.Pddot.matrix()), prob_data.Pddot.transpose().matrix() * prob_data.Pddot.matrix());
	else	
		ROS_ERROR_STREAM("Invalid order_smoothness value");


	prob_data.num_drone = 0;
	prob_data.num_static_obs = 0;
}

void deployAgent(probData &prob_data, int VERBOSE){

	
	std::default_random_engine generator;
	std::normal_distribution<double> dist(prob_data.mean, prob_data.stdev);

	// Initialize parameters and build matrices
	if(prob_data.mpc_step == 0){
		// ROS_INFO_STREAM("Initializing Drone " << prob_data.id_badge);
		initializeOptimizer(prob_data, VERBOSE);
	}
	
	if(prob_data.mpc_step > 0){
		neigbhoringAgents(prob_data, VERBOSE);
		if(!prob_data.free_space)
			initObstacles(prob_data, VERBOSE);
		else
			prob_data.num_static_obs = 0;
	}
	
	// @ Solve xyz
	// if(prob_data.solver=="quadprog")
		computeXYZ(prob_data, VERBOSE);
	
	// else
	// 	computeXYZ2(prob_data, VERBOSE);

	
	prob_data.smoothness.push_back(sqrt(pow(prob_data.ax_init, 2) + pow(prob_data.ay_init, 2) + pow(prob_data.az_init, 2)));
	prob_data.arc_length.push_back(sqrt(pow(prob_data.x_init - prob_data.x(1), 2) + pow(prob_data.y_init - prob_data.y(1), 2) + pow(prob_data.z_init - prob_data.z(1), 2)));
	
	// if(prob_data.inter_agent_dist.size() != 0)
	// 	prob_data.inter_agent_dist_min.push_back(*std::min_element(prob_data.inter_agent_dist.begin(), prob_data.inter_agent_dist.end()));
	// if(prob_data.agent_obs_dist.size() != 0)
	// 	prob_data.agent_obs_dist_min.push_back(*std::min_element(prob_data.agent_obs_dist.begin(), prob_data.agent_obs_dist.end()));

	if(prob_data.qp_fail){;
		// ROS_WARN_STREAM("Failed QP Run on previous soln");
		
		// if(prob_data.use_model){	
			prob_data.qp_fail = 0;
			prob_data.vx_init += prob_data.ax_init * prob_data.dt;// + dist(generator);
			prob_data.vy_init += prob_data.ay_init * prob_data.dt;// + dist(generator);
			prob_data.vz_init += prob_data.az_init * prob_data.dt;// + dist(generator);

			prob_data.x_init += prob_data.vx_init * prob_data.dt + 0.5 * prob_data.ax_init * prob_data.dt * prob_data.dt;// + dist(generator);
			prob_data.y_init += prob_data.vy_init * prob_data.dt + 0.5 * prob_data.ay_init * prob_data.dt * prob_data.dt;// + dist(generator);
			prob_data.z_init += prob_data.vz_init * prob_data.dt + 0.5 * prob_data.az_init * prob_data.dt * prob_data.dt;// + dist(generator);
		// }
	}
	else{
		
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
			
			
			prob_data.ax_init = prob_data.xddot(1) + dist(generator);
			prob_data.ay_init = prob_data.yddot(1) + dist(generator);
			prob_data.az_init = prob_data.zddot(1) + dist(generator);
		
			prob_data.vx_init += prob_data.ax_init * prob_data.dt + dist(generator);
			prob_data.vy_init += prob_data.ay_init * prob_data.dt + dist(generator);
			prob_data.vz_init += prob_data.az_init * prob_data.dt + dist(generator);

			prob_data.x_init += prob_data.vx_init * prob_data.dt + 0.5 * prob_data.ax_init * prob_data.dt * prob_data.dt + dist(generator);
			prob_data.y_init += prob_data.vy_init * prob_data.dt + 0.5 * prob_data.ay_init * prob_data.dt * prob_data.dt + dist(generator);
			prob_data.z_init += prob_data.vz_init * prob_data.dt + 0.5 * prob_data.az_init * prob_data.dt * prob_data.dt + dist(generator);
		}
		

		prob_data.dist_to_goal = sqrt(pow(prob_data.x_init - prob_data.x_goal, 2) + pow(prob_data.y_init - prob_data.y_goal, 2) + pow(prob_data.z_init - prob_data.z_goal, 2));
	
		
	}

	
}
