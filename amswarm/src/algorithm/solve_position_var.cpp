#include "algorithm/amswarm/solve_position_var.hpp"
#include "algorithm/amswarm/solve_polar_var.hpp"

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

            prob_data.b_x_static_obs = reshape(temp_x_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
            prob_data.b_y_static_obs = reshape(temp_y_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
        
            objective_xy += prob_data.rho_static_obs * cost_static_obs;
            lincost_x += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_x_static_obs.matrix()).array();
            lincost_y += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_y_static_obs.matrix()).array();

        }
        
        if(prob_data.num_drone!=0){
            
            temp_x_drone =  prob_data.x_drone + prob_data.d_drone * cos(prob_data.alpha_drone) * prob_data.a_drone;
            temp_y_drone =  prob_data.y_drone + prob_data.d_drone * sin(prob_data.alpha_drone) * prob_data.b_drone;

            prob_data.b_x_drone = reshape(temp_x_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
            prob_data.b_y_drone = reshape(temp_y_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
        
            objective_xy += prob_data.rho_drone * cost_drone;
            
            lincost_x += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_x_drone.matrix()).array();
            lincost_y += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_y_drone.matrix()).array();			
        }

        // @ Solve set of linear equations
        cost_xy = stack(stack(objective_xy, prob_data.A_eq.transpose(), 'h'), 
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

            prob_data.b_x_static_obs = reshape(temp_x_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
            prob_data.b_y_static_obs = reshape(temp_y_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
        
            objective_xy += prob_data.rho_static_obs * cost_static_obs;
            lincost_x += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_x_static_obs.matrix()).array();
            lincost_y += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_y_static_obs.matrix()).array();

        }
        
        // drones
        if(prob_data.num_drone!=0){
            
            temp_x_drone =  prob_data.x_drone + prob_data.d_drone * cos(prob_data.alpha_drone) * prob_data.a_drone;
            temp_y_drone =  prob_data.y_drone + prob_data.d_drone * sin(prob_data.alpha_drone) * prob_data.b_drone;

            prob_data.b_x_drone = reshape(temp_x_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
            prob_data.b_y_drone = reshape(temp_y_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
            
            objective_xy += prob_data.rho_drone * cost_drone;
            
            
            lincost_x += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_x_drone.matrix()).array();
            lincost_y += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_y_drone.matrix()).array();			
        }

        // @ Solve set of linear equations
        cost_xy = stack(stack(objective_xy, prob_data.A_eq.transpose(), 'h'), 
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

#include "algorithm/amswarm/solve_position_var.hpp"

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

            prob_data.b_x_static_obs = reshape(temp_x_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
            prob_data.b_y_static_obs = reshape(temp_y_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
        
            objective_xy += prob_data.rho_static_obs * cost_static_obs;
            lincost_x += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_x_static_obs.matrix()).array();
            lincost_y += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_y_static_obs.matrix()).array();

        }
        
        if(prob_data.num_drone!=0){
            
            temp_x_drone =  prob_data.x_drone + prob_data.d_drone * cos(prob_data.alpha_drone) * sin(prob_data.beta_drone) * prob_data.a_drone;
            temp_y_drone =  prob_data.y_drone + prob_data.d_drone * sin(prob_data.alpha_drone) * sin(prob_data.beta_drone) * prob_data.b_drone;
            temp_z_drone =  prob_data.z_drone + prob_data.d_drone * cos(prob_data.beta_drone) * prob_data.c_drone;

            prob_data.b_x_drone = reshape(temp_x_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
            prob_data.b_y_drone = reshape(temp_y_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
            prob_data.b_z_drone = reshape(temp_z_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
        
            objective_xy += prob_data.rho_drone * cost_drone;
            objective_z += prob_data.rho_drone * cost_drone;
            
            lincost_x += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_x_drone.matrix()).array();
            lincost_y += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_y_drone.matrix()).array();
            lincost_z += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_z_drone.matrix()).array();
        
        }

        // @ Solve set of linear equations
        cost_xy = stack(stack(objective_xy, prob_data.A_eq.transpose(), 'h'), 
                            stack(prob_data.A_eq, Eigen::ArrayXXf::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
        cost_xy_inv = (cost_xy.matrix()).householderQr().solve(prob_data.I.matrix());

        cost_z = stack(stack(objective_z, prob_data.A_eq.transpose(), 'h'), 
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

            prob_data.b_x_static_obs = reshape(temp_x_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
            prob_data.b_y_static_obs = reshape(temp_y_static_obs.transpose(), prob_data.num*prob_data.num_static_obs, 1);
        
            objective_xy += prob_data.rho_static_obs * cost_static_obs;
            lincost_x += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_x_static_obs.matrix()).array();
            lincost_y += -prob_data.rho_static_obs * (prob_data.A_static_obs.transpose().matrix() * prob_data.b_y_static_obs.matrix()).array();

        }
        
        // drones
        if(prob_data.num_drone!=0){
            
            temp_x_drone =  prob_data.x_drone + prob_data.d_drone * cos(prob_data.alpha_drone) * sin(prob_data.beta_drone) * prob_data.a_drone;
            temp_y_drone =  prob_data.y_drone + prob_data.d_drone * sin(prob_data.alpha_drone) * sin(prob_data.beta_drone) * prob_data.b_drone;
            temp_z_drone =  prob_data.z_drone + prob_data.d_drone * cos(prob_data.beta_drone) * prob_data.c_drone;

            prob_data.b_x_drone = reshape(temp_x_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
            prob_data.b_y_drone = reshape(temp_y_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
            prob_data.b_z_drone = reshape(temp_z_drone.transpose(), prob_data.num*prob_data.num_drone, 1);
        
            objective_xy += prob_data.rho_drone * cost_drone;
            objective_z += prob_data.rho_drone * cost_drone;
            
            lincost_x += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_x_drone.matrix()).array();
            lincost_y += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_y_drone.matrix()).array();
            lincost_z += -prob_data.rho_drone * (prob_data.A_drone.transpose().matrix() * prob_data.b_z_drone.matrix()).array();
        
        }

        // @ Solve set of linear equations
        cost_xy = stack(stack(objective_xy, prob_data.A_eq.transpose(), 'h'), 
                            stack(prob_data.A_eq, Eigen::ArrayXXf::Zero(prob_data.A_eq.rows(), prob_data.A_eq.rows()), 'h'), 'v');
        cost_xy_inv = (cost_xy.matrix()).householderQr().solve(prob_data.I.matrix());

        
        sol_x = cost_xy_inv.matrix() * stack(-lincost_x, prob_data.b_x_eq, 'v').matrix();
        sol_y = cost_xy_inv.matrix() * stack(-lincost_y, prob_data.b_y_eq, 'v').matrix(); 

        
            
        cost_z = stack(stack(objective_z, prob_data.A_eq.transpose(), 'h'), 
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