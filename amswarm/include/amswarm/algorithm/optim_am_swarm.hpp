#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>
#include "yaml-cpp/yaml.h"


namespace Optim
{  
    struct five_var
    {
        Eigen :: ArrayXXf a, b, c, d, e;
    };    
    struct probData
    {
        bool jerk_snap_constraints, axis_wise, free_space;

        int num, num_up, num_static_obs, num_drone, num_obs, nvar, id_badge, unify_obs,
            max_iter, mpc_step, kappa, world;

        
        float t_plan, vel_max, acc_max, jerk_max, snap_max, max_sim_time, dt, dist_to_goal, total_dist;
        float weight_smoothness, weight_goal, weight_smoothness_og, weight_goal_og;
        float rho_static_obs, rho_drone, rho_vel, rho_acc, rho_jerk, rho_snap, rho_ineq;
        float rho_static_obs_max, rho_drone_max, rho_vel_max, rho_acc_max, rho_jerk_max, rho_snap_max, rho_ineq_max;
        float delta_static_obs, delta_drone, delta_vel, delta_acc, delta_jerk, delta_snap, delta_ineq, delta_aggressive;
        float buffer, prox_obs, prox_agent, dist_stop;
        float lx_drone, ly_drone, lz_drone;

        float x_min, y_min, z_min,
              x_max, y_max, z_max,
              x_init, y_init, z_init,
              x_goal, y_goal, z_goal, 
              vx_init, vy_init, vz_init,
              ax_init, ay_init, az_init;
        
        float gravity, f_min, f_max;
        bool use_thrust_values, use_model;
        float thresold, mean, stdev, gamma, rmin;

        float res_x_static_obs_norm, res_y_static_obs_norm,
              res_x_drone_norm, res_y_drone_norm, res_z_drone_norm,
              res_x_vel_norm, res_y_vel_norm, res_z_vel_norm,
              res_x_acc_norm, res_y_acc_norm, res_z_acc_norm,
              res_x_jerk_norm, res_y_jerk_norm, res_z_jerk_norm,
              res_x_snap_norm, res_y_snap_norm, res_z_snap_norm,
              res_x_ineq_norm, res_y_ineq_norm, res_z_ineq_norm;

        Eigen :: ArrayXXf agents_x, agents_y, agents_z;
        Eigen :: ArrayXXf cost_smoothness, cost_goal, cost_vel, cost_acc, cost_jerk, cost_snap, cost_ineq, I;

        Eigen :: ArrayXXf P, Pdot, Pddot, Pdddot, Pddddot;  
        Eigen :: ArrayXXf P_up, Pdot_up, Pddot_up, Pdddot_up, Pddddot_up;

        Eigen :: ArrayXXf A_ineq, A_v_ineq, A_a_ineq, A_j_ineq, A_s_ineq, A_eq, A_static_obs, A_drone;

        Eigen :: ArrayXXf B_x_ineq, B_y_ineq, B_z_ineq;
        Eigen :: ArrayXXf b_x_ineq, b_y_ineq, b_z_ineq;
        Eigen :: ArrayXXf s_x_ineq, s_y_ineq, s_z_ineq;

        Eigen :: ArrayXXf B_vx_ineq, B_vy_ineq, B_vz_ineq;
        Eigen :: ArrayXXf b_vx_ineq, b_vy_ineq, b_vz_ineq;
        Eigen :: ArrayXXf b_x_vel, b_y_vel, b_z_vel;
        Eigen :: ArrayXXf s_vx_ineq, s_vy_ineq, s_vz_ineq;

        Eigen :: ArrayXXf B_ax_ineq, B_ay_ineq, B_az_ineq;
        Eigen :: ArrayXXf b_ax_ineq, b_ay_ineq, b_az_ineq;
        Eigen :: ArrayXXf b_x_acc, b_y_acc, b_z_acc;
        Eigen :: ArrayXXf s_ax_ineq, s_ay_ineq, s_az_ineq;

        Eigen :: ArrayXXf B_jx_ineq, B_jy_ineq, B_jz_ineq;
        Eigen :: ArrayXXf b_jx_ineq, b_jy_ineq, b_jz_ineq;
        Eigen :: ArrayXXf b_x_jerk, b_y_jerk, b_z_jerk;
        Eigen :: ArrayXXf s_jx_ineq, s_jy_ineq, s_jz_ineq;

        Eigen :: ArrayXXf B_sx_ineq, B_sy_ineq, B_sz_ineq;
        Eigen :: ArrayXXf b_sx_ineq, b_sy_ineq, b_sz_ineq;
        Eigen :: ArrayXXf b_x_snap, b_y_snap, b_z_snap;
        Eigen :: ArrayXXf s_sx_ineq, s_sy_ineq, s_sz_ineq;

        Eigen :: ArrayXXf b_x_eq, b_y_eq, b_z_eq;
        Eigen :: ArrayXXf b_x_static_obs, b_y_static_obs;
        Eigen :: ArrayXXf b_x_drone, b_y_drone, b_z_drone;
        

        Eigen :: ArrayXXf d_static_obs, d_drone, d_vel, d_acc, d_jerk, d_snap, d_drone_old, d_static_obs_old; 
        Eigen :: ArrayXXf alpha_static_obs, alpha_drone, alpha_vel, alpha_acc, alpha_jerk, alpha_snap;
        Eigen :: ArrayXXf beta_drone, beta_vel, beta_acc, beta_jerk, beta_snap; 

        Eigen :: ArrayXXf x_static_obs, y_static_obs, z_static_obs;
        Eigen :: ArrayXXf x_static_obs_og, y_static_obs_og, z_static_obs_og;
        Eigen :: ArrayXXf x_drone, y_drone, z_drone;
        Eigen :: ArrayXXf a_static_obs, b_static_obs, c_static_obs;
        Eigen :: ArrayXXf a_static_obs_og, b_static_obs_og, c_static_obs_og;  
        Eigen :: ArrayXXf a_drone, b_drone, c_drone;

        Eigen :: ArrayXXf x, y, z,
                          xdot, ydot, zdot,
                          xddot, yddot, zddot,
                          xdddot, ydddot, zdddot,
                          xddddot, yddddot, zddddot;
        
        Eigen :: ArrayXXf x_up, y_up, z_up,
                          xdot_up, ydot_up, zdot_up,
                          xddot_up, yddot_up, zddot_up;
        
        Eigen :: ArrayXXf x_ref, y_ref, z_ref;
        Eigen :: ArrayXXf lamda_x, lamda_y, lamda_z;
        Eigen :: ArrayXXf lamda_x_ineq, lamda_y_ineq, lamda_z_ineq;        

        std :: vector<float> smoothness, arc_length, inter_agent_dist, agent_obs_dist, inter_agent_dist_min, agent_obs_dist_min;
        std :: vector<std :: vector<float>> pos_static_obs, dim_static_obs;

        YAML :: Node params;

        
    };
    
    five_var bernsteinCoeffOrder10(float n, float tmin, float tmax, Eigen :: ArrayXXf t_actual, int num);
    five_var computeBernstein(Eigen :: ArrayXXf tot_time, float t_fin, int num);
    Eigen :: ArrayXXf stack(Eigen :: ArrayXXf arr1, Eigen :: ArrayXXf arr2, char ch);
    Eigen :: ArrayXXf reshape(Eigen :: ArrayXXf x, uint32_t r, uint32_t c);
    Eigen :: ArrayXXf arctan2(Eigen :: ArrayXXf arr1, Eigen :: ArrayXXf arr2);
    Eigen :: ArrayXXf maximum(float val, Eigen :: ArrayXXf arr2);
    Eigen :: ArrayXXf delete_values(float val, Eigen :: ArrayXXf arr);
    Eigen :: ArrayXXf diff(Eigen :: ArrayXXf arr);

    void initAlpha(probData &prob_data, int VERBOSE);
    void initAlphaBeta(probData &prob_data, int VERBOSE);

    void computeXYZ(probData &prob_data, int VERBOSE);
    void computeXYZAxis(probData &prob_data, int VERBOSE);
    void computeXY(probData &prob_data, int VERBOSE);
    void computeXYAxis(probData &prob_data, int VERBOSE);

    void checkResiduals(probData &prob_data, int VERBOSE);

    void initObstacles(probData &prob_data, int VERBOSE);
    void neigbhoringAgents(probData &prob_data, int VERBOSE);

    void initializeOptimizer(probData &prob_data, int VERBOSE);
    void deployAgent(probData &prob_data, int VERBOSE);
}