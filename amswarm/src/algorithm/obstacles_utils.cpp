#include "algorithm/amswarm/obstacles_utils.hpp"


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
    for(int i = 0; i < prob_data.num_static_obs - 1; i++) prob_data.A_static_obs = stack(prob_data.A_static_obs, prob_data.P, 'v');
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
    for(int i = 0; i < prob_data.num_drone - 1; i++) prob_data.A_drone = stack(prob_data.A_drone, prob_data.P, 'v');

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