#include "algorithm/amswarm/solve_polar_var.hpp"

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