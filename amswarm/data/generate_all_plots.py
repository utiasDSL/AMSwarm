import rospkg
import numpy as np
import seaborn as sns
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from pathlib import Path
from matplotlib.ticker import FormatStrFormatter
from matplotlib.ticker import MaxNLocator
import pandas as pd

rospack = rospkg.RosPack()
rospack.list() 
path = rospack.get_path('amswarm')

res_static_obs = np.array([])
res_drone_obs = np.array([])
res_vel = np.array([])
res_acc = np.array([])
res_pos = np.array([])

########################

success_ours_axis_8 = np.array([0, 0, 0, 0, 0])
success_ours_quad_8 = np.array([0, 0, 0, 0, 0])
success_scp_on_8 = np.array([0, 0, 0, 0, 0])
success_scp_ca_8 = np.array([0, 0, 0, 0, 0])

success_ours_axis_16 = np.array([0, 0, 0, 0, 0])
success_ours_quad_16 = np.array([0, 0, 0, 0, 0])
success_scp_on_16 = np.array([0, 0, 0, 0, 0])
success_scp_ca_16 = np.array([0, 0, 0, 0, 0])

success_ours_axis_g1_16 = np.array([0, 0, 0, 0, 0])
success_ours_quad_g1_16 = np.array([0, 0, 0, 0, 0])
success_ours_axis_g2_16 = np.array([0, 0, 0, 0, 0])
success_ours_quad_g2_16 = np.array([0, 0, 0, 0, 0])

########################
mission_time_ours_axis_10_drone = np.array([])
mission_time_ours_quad_10_drone = np.array([])  
mission_time_scp_on_10_drone = np.array([])
mission_time_scp_ca_10_drone = np.array([])

arc_length_ours_axis_10_drone = np.array([])
arc_length_ours_quad_10_drone = np.array([])
arc_length_scp_on_10_drone = np.array([])
arc_length_scp_ca_10_drone = np.array([]) 

smoothness_ours_axis_10_drone = np.array([])  
smoothness_ours_quad_10_drone = np.array([]) 
smoothness_scp_on_10_drone = np.array([])  
smoothness_scp_ca_10_drone = np.array([]) 

compute_time_ours_axis_10_drone = np.array([]) 
compute_time_ours_quad_10_drone = np.array([]) 
compute_time_scp_on_10_drone = np.array([])
compute_time_scp_ca_10_drone = np.array([])

inter_agent_ours_axis_10_drone = np.array([]) 
inter_agent_ours_quad_10_drone = np.array([]) 
inter_agent_scp_on_10_drone = np.array([])
inter_agent_scp_ca_10_drone = np.array([])

obs_agent_ours_axis_10_drone = np.array([]) 
obs_agent_ours_quad_10_drone = np.array([]) 
obs_agent_scp_on_10_drone = np.array([])
obs_agent_scp_ca_10_drone = np.array([])

################################
mission_time_ours_axis_20_drone = np.array([])
mission_time_ours_quad_20_drone = np.array([])  
mission_time_scp_on_20_drone = np.array([])
mission_time_scp_ca_20_drone = np.array([])

arc_length_ours_axis_20_drone = np.array([])
arc_length_ours_quad_20_drone = np.array([])
arc_length_scp_on_20_drone = np.array([])
arc_length_scp_ca_20_drone = np.array([]) 

smoothness_ours_axis_20_drone = np.array([])  
smoothness_ours_quad_20_drone = np.array([]) 
smoothness_scp_on_20_drone = np.array([])  
smoothness_scp_ca_20_drone = np.array([]) 

compute_time_ours_axis_20_drone = np.array([]) 
compute_time_ours_quad_20_drone = np.array([]) 
compute_time_scp_on_20_drone = np.array([])
compute_time_scp_ca_20_drone = np.array([])

inter_agent_ours_axis_20_drone = np.array([]) 
inter_agent_ours_quad_20_drone = np.array([]) 
inter_agent_scp_on_20_drone = np.array([])
inter_agent_scp_ca_20_drone = np.array([])

obs_agent_ours_axis_20_drone = np.array([]) 
obs_agent_ours_quad_20_drone = np.array([]) 
obs_agent_scp_on_20_drone = np.array([])
obs_agent_scp_ca_20_drone = np.array([])

###########################
mission_time_ours_axis_30_drone = np.array([])
mission_time_ours_quad_30_drone = np.array([])  
mission_time_scp_on_30_drone = np.array([])
mission_time_scp_ca_30_drone = np.array([])

arc_length_ours_axis_30_drone = np.array([])
arc_length_ours_quad_30_drone = np.array([])
arc_length_scp_on_30_drone = np.array([])
arc_length_scp_ca_30_drone = np.array([]) 

smoothness_ours_axis_30_drone = np.array([])  
smoothness_ours_quad_30_drone = np.array([]) 
smoothness_scp_on_30_drone = np.array([])  
smoothness_scp_ca_30_drone = np.array([]) 

compute_time_ours_axis_30_drone = np.array([]) 
compute_time_ours_quad_30_drone = np.array([]) 
compute_time_scp_on_30_drone = np.array([])
compute_time_scp_ca_30_drone = np.array([])

inter_agent_ours_axis_30_drone = np.array([]) 
inter_agent_ours_quad_30_drone = np.array([]) 
inter_agent_scp_on_30_drone = np.array([])
inter_agent_scp_ca_30_drone = np.array([])

obs_agent_ours_axis_30_drone = np.array([]) 
obs_agent_ours_quad_30_drone = np.array([]) 
obs_agent_scp_on_30_drone = np.array([])
obs_agent_scp_ca_30_drone = np.array([])

#########################3
mission_time_ours_axis_40_drone = np.array([])
mission_time_ours_quad_40_drone = np.array([])  
mission_time_scp_on_40_drone = np.array([])
mission_time_scp_ca_40_drone = np.array([])

arc_length_ours_axis_40_drone = np.array([])
arc_length_ours_quad_40_drone = np.array([])
arc_length_scp_on_40_drone = np.array([])
arc_length_scp_ca_40_drone = np.array([]) 

smoothness_ours_axis_40_drone = np.array([])  
smoothness_ours_quad_40_drone = np.array([]) 
smoothness_scp_on_40_drone = np.array([])  
smoothness_scp_ca_40_drone = np.array([]) 

compute_time_ours_axis_40_drone = np.array([]) 
compute_time_ours_quad_40_drone = np.array([]) 
compute_time_scp_on_40_drone = np.array([])
compute_time_scp_ca_40_drone = np.array([])

inter_agent_ours_axis_40_drone = np.array([]) 
inter_agent_ours_quad_40_drone = np.array([]) 
inter_agent_scp_on_40_drone = np.array([])
inter_agent_scp_ca_40_drone = np.array([])

obs_agent_ours_axis_40_drone = np.array([]) 
obs_agent_ours_quad_40_drone = np.array([]) 
obs_agent_scp_on_40_drone = np.array([])
obs_agent_scp_ca_40_drone = np.array([])

###################
mission_time_ours_axis_50_drone = np.array([])
mission_time_ours_quad_50_drone = np.array([])  
mission_time_scp_on_50_drone = np.array([])
mission_time_scp_ca_50_drone = np.array([])

arc_length_ours_axis_50_drone = np.array([])
arc_length_ours_quad_50_drone = np.array([])
arc_length_scp_on_50_drone = np.array([])
arc_length_scp_ca_50_drone = np.array([]) 

smoothness_ours_axis_50_drone = np.array([])  
smoothness_ours_quad_50_drone = np.array([]) 
smoothness_scp_on_50_drone = np.array([])  
smoothness_scp_ca_50_drone = np.array([]) 

compute_time_ours_axis_50_drone = np.array([]) 
compute_time_ours_quad_50_drone = np.array([]) 
compute_time_scp_on_50_drone = np.array([])
compute_time_scp_ca_50_drone = np.array([])

inter_agent_ours_axis_50_drone = np.array([]) 
inter_agent_ours_quad_50_drone = np.array([]) 
inter_agent_scp_on_50_drone = np.array([])
inter_agent_scp_ca_50_drone = np.array([])

obs_agent_ours_axis_50_drone = np.array([]) 
obs_agent_ours_quad_50_drone = np.array([]) 
obs_agent_scp_on_50_drone = np.array([])
obs_agent_scp_ca_50_drone = np.array([])
######################################################

mission_time_ours_axis_g1_10_drone = np.array([])
mission_time_ours_quad_g1_10_drone = np.array([])  
arc_length_ours_axis_g1_10_drone = np.array([])
arc_length_ours_quad_g1_10_drone = np.array([])
smoothness_ours_axis_g1_10_drone = np.array([])  
smoothness_ours_quad_g1_10_drone = np.array([]) 
compute_time_ours_axis_g1_10_drone = np.array([]) 
compute_time_ours_quad_g1_10_drone = np.array([]) 
inter_agent_ours_axis_g1_10_drone = np.array([]) 
inter_agent_ours_quad_g1_10_drone = np.array([]) 
obs_agent_ours_axis_g1_10_drone = np.array([]) 
obs_agent_ours_quad_g1_10_drone = np.array([]) 

mission_time_ours_axis_g2_10_drone = np.array([])
mission_time_ours_quad_g2_10_drone = np.array([])  
arc_length_ours_axis_g2_10_drone = np.array([])
arc_length_ours_quad_g2_10_drone = np.array([])
smoothness_ours_axis_g2_10_drone = np.array([])  
smoothness_ours_quad_g2_10_drone = np.array([]) 
compute_time_ours_axis_g2_10_drone = np.array([]) 
compute_time_ours_quad_g2_10_drone = np.array([]) 
inter_agent_ours_axis_g2_10_drone = np.array([]) 
inter_agent_ours_quad_g2_10_drone = np.array([]) 
obs_agent_ours_axis_g2_10_drone = np.array([]) 
obs_agent_ours_quad_g2_10_drone = np.array([]) 

mission_time_ours_axis_g1_20_drone = np.array([])
mission_time_ours_quad_g1_20_drone = np.array([])  
arc_length_ours_axis_g1_20_drone = np.array([])
arc_length_ours_quad_g1_20_drone = np.array([])
smoothness_ours_axis_g1_20_drone = np.array([])  
smoothness_ours_quad_g1_20_drone = np.array([]) 
compute_time_ours_axis_g1_20_drone = np.array([]) 
compute_time_ours_quad_g1_20_drone = np.array([]) 
inter_agent_ours_axis_g1_20_drone = np.array([]) 
inter_agent_ours_quad_g1_20_drone = np.array([]) 
obs_agent_ours_axis_g1_20_drone = np.array([]) 
obs_agent_ours_quad_g1_20_drone = np.array([]) 

mission_time_ours_axis_g2_30_drone = np.array([])
mission_time_ours_quad_g2_30_drone = np.array([])  
arc_length_ours_axis_g2_30_drone = np.array([])
arc_length_ours_quad_g2_30_drone = np.array([])
smoothness_ours_axis_g2_30_drone = np.array([])  
smoothness_ours_quad_g2_30_drone = np.array([]) 
compute_time_ours_axis_g2_30_drone = np.array([]) 
compute_time_ours_quad_g2_30_drone = np.array([]) 
inter_agent_ours_axis_g2_30_drone = np.array([]) 
inter_agent_ours_quad_g2_30_drone = np.array([]) 
obs_agent_ours_axis_g2_30_drone = np.array([]) 
obs_agent_ours_quad_g2_30_drone = np.array([]) 

mission_time_ours_axis_g1_30_drone = np.array([])
mission_time_ours_quad_g1_30_drone = np.array([])  
arc_length_ours_axis_g1_30_drone = np.array([])
arc_length_ours_quad_g1_30_drone = np.array([])
smoothness_ours_axis_g1_30_drone = np.array([])  
smoothness_ours_quad_g1_30_drone = np.array([]) 
compute_time_ours_axis_g1_30_drone = np.array([]) 
compute_time_ours_quad_g1_30_drone = np.array([]) 
inter_agent_ours_axis_g1_30_drone = np.array([]) 
inter_agent_ours_quad_g1_30_drone = np.array([]) 
obs_agent_ours_axis_g1_30_drone = np.array([]) 
obs_agent_ours_quad_g1_30_drone = np.array([]) 

mission_time_ours_axis_g2_20_drone = np.array([])
mission_time_ours_quad_g2_20_drone = np.array([])  
arc_length_ours_axis_g2_20_drone = np.array([])
arc_length_ours_quad_g2_20_drone = np.array([])
smoothness_ours_axis_g2_20_drone = np.array([])  
smoothness_ours_quad_g2_20_drone = np.array([]) 
compute_time_ours_axis_g2_20_drone = np.array([]) 
compute_time_ours_quad_g2_20_drone = np.array([]) 
inter_agent_ours_axis_g2_20_drone = np.array([]) 
inter_agent_ours_quad_g2_20_drone = np.array([]) 
obs_agent_ours_axis_g2_20_drone = np.array([]) 
obs_agent_ours_quad_g2_20_drone = np.array([]) 

mission_time_ours_axis_g1_40_drone = np.array([])
mission_time_ours_quad_g1_40_drone = np.array([])  
arc_length_ours_axis_g1_40_drone = np.array([])
arc_length_ours_quad_g1_40_drone = np.array([])
smoothness_ours_axis_g1_40_drone = np.array([])  
smoothness_ours_quad_g1_40_drone = np.array([]) 
compute_time_ours_axis_g1_40_drone = np.array([]) 
compute_time_ours_quad_g1_40_drone = np.array([]) 
inter_agent_ours_axis_g1_40_drone = np.array([]) 
inter_agent_ours_quad_g1_40_drone = np.array([]) 
obs_agent_ours_axis_g1_40_drone = np.array([]) 
obs_agent_ours_quad_g1_40_drone = np.array([]) 

mission_time_ours_axis_g2_40_drone = np.array([])
mission_time_ours_quad_g2_40_drone = np.array([])  
arc_length_ours_axis_g2_40_drone = np.array([])
arc_length_ours_quad_g2_40_drone = np.array([])
smoothness_ours_axis_g2_40_drone = np.array([])  
smoothness_ours_quad_g2_40_drone = np.array([]) 
compute_time_ours_axis_g2_40_drone = np.array([]) 
compute_time_ours_quad_g2_40_drone = np.array([]) 
inter_agent_ours_axis_g2_40_drone = np.array([]) 
inter_agent_ours_quad_g2_40_drone = np.array([]) 
obs_agent_ours_axis_g2_40_drone = np.array([]) 
obs_agent_ours_quad_g2_40_drone = np.array([]) 

mission_time_ours_axis_g1_50_drone = np.array([])
mission_time_ours_quad_g1_50_drone = np.array([])  
arc_length_ours_axis_g1_50_drone = np.array([])
arc_length_ours_quad_g1_50_drone = np.array([])
smoothness_ours_axis_g1_50_drone = np.array([])  
smoothness_ours_quad_g1_50_drone = np.array([]) 
compute_time_ours_axis_g1_50_drone = np.array([]) 
compute_time_ours_quad_g1_50_drone = np.array([]) 
inter_agent_ours_axis_g1_50_drone = np.array([]) 
inter_agent_ours_quad_g1_50_drone = np.array([]) 
obs_agent_ours_axis_g1_50_drone = np.array([]) 
obs_agent_ours_quad_g1_50_drone = np.array([]) 

mission_time_ours_axis_g2_50_drone = np.array([])
mission_time_ours_quad_g2_50_drone = np.array([])  
arc_length_ours_axis_g2_50_drone = np.array([])
arc_length_ours_quad_g2_50_drone = np.array([])
smoothness_ours_axis_g2_50_drone = np.array([])  
smoothness_ours_quad_g2_50_drone = np.array([]) 
compute_time_ours_axis_g2_50_drone = np.array([]) 
compute_time_ours_quad_g2_50_drone = np.array([]) 
inter_agent_ours_axis_g2_50_drone = np.array([]) 
inter_agent_ours_quad_g2_50_drone = np.array([]) 
obs_agent_ours_axis_g2_50_drone = np.array([]) 
obs_agent_ours_quad_g2_50_drone = np.array([]) 

##########################################
total_config = 100
for j in range(10, 60, 10):
    num_drone = j
    smoothness_ours_axis_8 = np.array([])
    smoothness_ours_quad_8 = np.array([])
    smoothness_scp_on_8 = np.array([])
    smoothness_scp_ca_8 = np.array([])

    arc_length_ours_axis_8 = np.array([])
    arc_length_ours_quad_8 = np.array([])
    arc_length_scp_on_8 = np.array([])
    arc_length_scp_ca_8 = np.array([])

    mission_time_ours_axis_8 = np.array([])
    mission_time_ours_quad_8 = np.array([])
    mission_time_scp_on_8 = np.array([])
    mission_time_scp_ca_8 = np.array([])

    compute_time_ours_axis_8 = np.array([])
    compute_time_ours_quad_8 = np.array([])
    compute_time_scp_on_8 = np.array([])
    compute_time_scp_ca_8 = np.array([])

    inter_agent_ours_axis_8 = np.array([])
    inter_agent_ours_quad_8 = np.array([])
    inter_agent_scp_on_8 = np.array([])
    inter_agent_scp_ca_8 = np.array([])

    obs_agent_ours_axis_8 = np.array([])
    obs_agent_ours_quad_8 = np.array([])
    obs_agent_scp_on_8 = np.array([])
    obs_agent_scp_ca_8 = np.array([])

    smoothness_ours_axis_16 = np.array([])
    smoothness_ours_quad_16 = np.array([])
    smoothness_scp_on_16 = np.array([])
    smoothness_scp_ca_16 = np.array([])

    arc_length_ours_axis_16 = np.array([])
    arc_length_ours_quad_16 = np.array([])
    arc_length_scp_on_16 = np.array([])
    arc_length_scp_ca_16 = np.array([])

    mission_time_ours_axis_16 = np.array([])
    mission_time_ours_quad_16 = np.array([])
    mission_time_scp_on_16 = np.array([])
    mission_time_scp_ca_16 = np.array([])

    compute_time_ours_axis_16 = np.array([])
    compute_time_ours_quad_16 = np.array([])
    compute_time_scp_on_16 = np.array([])
    compute_time_scp_ca_16 = np.array([])

    inter_agent_ours_axis_16 = np.array([])
    inter_agent_ours_quad_16 = np.array([])
    inter_agent_scp_on_16 = np.array([])
    inter_agent_scp_ca_16 = np.array([])

    obs_agent_ours_axis_16 = np.array([])
    obs_agent_ours_quad_16 = np.array([])
    obs_agent_scp_on_16 = np.array([])
    obs_agent_scp_ca_16 = np.array([])


    ######
    smoothness_ours_axis_g1_16 = np.array([])
    smoothness_ours_quad_g1_16 = np.array([])
    arc_length_ours_axis_g1_16 = np.array([])
    arc_length_ours_quad_g1_16 = np.array([])
    mission_time_ours_axis_g1_16 = np.array([])
    mission_time_ours_quad_g1_16 = np.array([])
    compute_time_ours_axis_g1_16 = np.array([])
    compute_time_ours_quad_g1_16 = np.array([])
    inter_agent_ours_axis_g1_16 = np.array([])
    inter_agent_ours_quad_g1_16 = np.array([])
    obs_agent_ours_axis_g1_16 = np.array([])
    obs_agent_ours_quad_g1_16 = np.array([])

    smoothness_ours_axis_g2_16 = np.array([])
    smoothness_ours_quad_g2_16 = np.array([])
    arc_length_ours_axis_g2_16 = np.array([])
    arc_length_ours_quad_g2_16 = np.array([])
    mission_time_ours_axis_g2_16 = np.array([])
    mission_time_ours_quad_g2_16 = np.array([])
    compute_time_ours_axis_g2_16 = np.array([])
    compute_time_ours_quad_g2_16 = np.array([])
    inter_agent_ours_axis_g2_16 = np.array([])
    inter_agent_ours_quad_g2_16 = np.array([])
    obs_agent_ours_axis_g2_16 = np.array([])
    obs_agent_ours_quad_g2_16 = np.array([])
    #######


    for i in range(total_config):
        config_num = i
        num_obs = 8

        # varying agents 8 obstacles
        file_1 = False
        file_2 = False
        file_3 = False
        file_4 = False
        file_5 = False
        file_6 = False
        file_7 = False
        file_8 = False
        file_9 = False
        file_10 = False
        file_11 = False
        file_12 = False
        
        # AXIS WISE 
        file_name_1 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_am_ax/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_1).is_file():
            success_ours_axis_8[int(j/10) - 1] +=1
            file_1 = True
                       
            
        # QUADRATIC    
        file_name_2 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_am_qd/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_2).is_file():
            success_ours_quad_8[int(j/10) - 1] +=1
            file_2 = True


        # ONDEMAND
        file_name_3 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_on/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_3).is_file():
            success_scp_on_8[int(j/10) - 1] +=1
            file_3 = True
        
        # CONTINUOUS
        file_name_4 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_ca/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_4).is_file():
            success_scp_ca_8[int(j/10) - 1] +=1
            file_4 = True
            
            
        if file_1:
            data = np.loadtxt(file_name_1)
            
            sim_iter = int(data[0])
            compute_time_ours_axis_8 = np.append(compute_time_ours_axis_8, data[1:sim_iter+2])
            smoothness_ours_axis_8 = np.append(smoothness_ours_axis_8, data[sim_iter+2:sim_iter+2+num_drone])
            arc_length_ours_axis_8 = np.append(arc_length_ours_axis_8, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
            inter_agent_ours_axis_8 = np.append(inter_agent_ours_axis_8, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
            obs_agent_ours_axis_8 = np.append(obs_agent_ours_axis_8, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
            mission_time_ours_axis_8 = np.append(mission_time_ours_axis_8, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
            
            
        if file_2:            
            data = np.loadtxt(file_name_2)
            
            sim_iter = int(data[0])
            compute_time_ours_quad_8 = np.append(compute_time_ours_quad_8, data[1:sim_iter+2])
            smoothness_ours_quad_8 = np.append(smoothness_ours_quad_8, data[sim_iter+2:sim_iter+2+num_drone])
            arc_length_ours_quad_8 = np.append(arc_length_ours_quad_8, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
            inter_agent_ours_quad_8 = np.append(inter_agent_ours_quad_8, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
            obs_agent_ours_quad_8 = np.append(obs_agent_ours_quad_8, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
            mission_time_ours_quad_8 = np.append(mission_time_ours_quad_8, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
      

        if file_3:            
            data = np.loadtxt(file_name_3)
            
            sim_iter = int(data[0])
            compute_time_scp_on_8 = np.append(compute_time_scp_on_8, data[1:sim_iter+2])
            smoothness_scp_on_8 = np.append(smoothness_scp_on_8, data[sim_iter+2:sim_iter+2+num_drone])
            arc_length_scp_on_8 = np.append(arc_length_scp_on_8, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
            inter_agent_scp_on_8 = np.append(inter_agent_scp_on_8, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
            obs_agent_scp_on_8 = np.append(obs_agent_scp_on_8, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
            mission_time_scp_on_8 = np.append(mission_time_scp_on_8, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
            
        if file_4:
            data = np.loadtxt(file_name_4)
            
            sim_iter = int(data[0])
            compute_time_scp_ca_8 = np.append(compute_time_scp_ca_8, data[1:sim_iter+2])
            smoothness_scp_ca_8 = np.append(smoothness_scp_ca_8, data[sim_iter+2:sim_iter+2+num_drone])
            arc_length_scp_ca_8 = np.append(arc_length_scp_ca_8, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
            inter_agent_scp_ca_8 = np.append(inter_agent_scp_ca_8, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
            obs_agent_scp_ca_8 = np.append(obs_agent_scp_ca_8, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
            mission_time_scp_ca_8 = np.append(mission_time_scp_ca_8, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
            
        
        # varying agents with 16obs
        num_obs = 16

        # AXIS
        file_name_5 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_am_ax/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_5).is_file():
            success_ours_axis_16[int(j/10) - 1] +=1
            file_5 = True
            
            
        # QUADRATIC    
        file_name_6 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_am_qd/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_6).is_file():
            success_ours_quad_16[int(j/10) - 1] +=1
            file_6 = True

            file_residual = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_am_qd/sim_residue_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
            data_residual = np.loadtxt(file_residual)
            
            temp_res_obs = data_residual[:, 0:2]
            temp_res_drone = data_residual[:, 2:5]
            temp_res_vel = data_residual[:, 5:8]
            temp_res_acc = data_residual[:, 8:11]
            temp_res_pos = data_residual[:, 11:14]

            res_static_obs = np.append(res_static_obs, temp_res_obs.flatten())
            res_drone_obs = np.append(res_drone_obs, temp_res_drone.flatten())
            res_vel = np.append(res_vel, temp_res_vel.flatten())
            res_acc = np.append(res_acc, temp_res_acc.flatten())
            res_pos = np.append(res_pos, temp_res_pos.flatten())

            
        # ONDEMAND    
        file_name_7 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_on/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_7).is_file():
            success_scp_on_16[int(j/10) - 1] +=1
            file_7 = True
            
        # CONTINUOUS
        file_name_8 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_ca/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_8).is_file():
            success_scp_ca_16[int(j/10) - 1] +=1
            file_8 = True
        

        # AXIS gamma1
        file_name_9 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_am_ax_gamma_0.95/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_9).is_file():
            success_ours_axis_g1_16[int(j/10) - 1] +=1
            file_9 = True
            
            
        # QUADRATIC  gamma1  
        file_name_10 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_am_qd_gamma_0.95/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_10).is_file():
            success_ours_quad_g1_16[int(j/10) - 1] +=1
            file_10 = True

            file_residual = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_am_qd/sim_residue_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
            data_residual = np.loadtxt(file_residual)
            
            temp_res_obs = data_residual[:, 0:2]
            temp_res_drone = data_residual[:, 2:5]
            temp_res_vel = data_residual[:, 5:8]
            temp_res_acc = data_residual[:, 8:11]
            temp_res_pos = data_residual[:, 11:14]

            res_static_obs = np.append(res_static_obs, temp_res_obs.flatten())
            res_drone_obs = np.append(res_drone_obs, temp_res_drone.flatten())
            res_vel = np.append(res_vel, temp_res_vel.flatten())
            res_acc = np.append(res_acc, temp_res_acc.flatten())
            res_pos = np.append(res_pos, temp_res_pos.flatten())

        # AXIS gamma2
        file_name_11 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_am_ax_gamma_0.9/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_11).is_file():
            success_ours_axis_g2_16[int(j/10) - 1] +=1
            file_11 = True
            
            
        # QUADRATIC  gamma2  
        file_name_12 = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_am_qd_gamma_0.9/sim_results_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
        if Path(file_name_12).is_file():
            success_ours_quad_g2_16[int(j/10) - 1] +=1
            file_12 = True

            file_residual = path+"/data/point_to_point/config_data/varying_agents/obs_" + str(num_obs) +"/results_am_qd/sim_residue_drone_" +str(num_drone)+"_config_"+str(config_num)+".txt"
            data_residual = np.loadtxt(file_residual)
            
            temp_res_obs = data_residual[:, 0:2]
            temp_res_drone = data_residual[:, 2:5]
            temp_res_vel = data_residual[:, 5:8]
            temp_res_acc = data_residual[:, 8:11]
            temp_res_pos = data_residual[:, 11:14]

            res_static_obs = np.append(res_static_obs, temp_res_obs.flatten())
            res_drone_obs = np.append(res_drone_obs, temp_res_drone.flatten())
            res_vel = np.append(res_vel, temp_res_vel.flatten())
            res_acc = np.append(res_acc, temp_res_acc.flatten())
            res_pos = np.append(res_pos, temp_res_pos.flatten())


        if True: #file_5 and file_6 and file_7 and file_8:
            if file_5:
                data = np.loadtxt(file_name_5)
                
                sim_iter = int(data[0])
                compute_time_ours_axis_16 = np.append(compute_time_ours_axis_16, data[1:sim_iter+2])
                smoothness_ours_axis_16 = np.append(smoothness_ours_axis_16, data[sim_iter+2:sim_iter+2+num_drone])
                arc_length_ours_axis_16 = np.append(arc_length_ours_axis_16, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
                inter_agent_ours_axis_16 = np.append(inter_agent_ours_axis_16, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
                obs_agent_ours_axis_16 = np.append(obs_agent_ours_axis_16, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                mission_time_ours_axis_16 = np.append(mission_time_ours_axis_16, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])

                # print(data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                
            if file_6:            
                data = np.loadtxt(file_name_6)
                
                sim_iter = int(data[0])
                compute_time_ours_quad_16 = np.append(compute_time_ours_quad_16, data[1:sim_iter+2])
                smoothness_ours_quad_16 = np.append(smoothness_ours_quad_16, data[sim_iter+2:sim_iter+2+num_drone])
                arc_length_ours_quad_16 = np.append(arc_length_ours_quad_16, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
                inter_agent_ours_quad_16 = np.append(inter_agent_ours_quad_16, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
                obs_agent_ours_quad_16 = np.append(obs_agent_ours_quad_16, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                mission_time_ours_quad_16 = np.append(mission_time_ours_quad_16, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                
            if file_7:            
                data = np.loadtxt(file_name_7)
                
                sim_iter = int(data[0])
                compute_time_scp_on_16 = np.append(compute_time_scp_on_16, data[1:sim_iter+2])
                smoothness_scp_on_16 = np.append(smoothness_scp_on_16, data[sim_iter+2:sim_iter+2+num_drone])
                arc_length_scp_on_16 = np.append(arc_length_scp_on_16, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
                inter_agent_scp_on_16 = np.append(inter_agent_scp_on_16, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
                obs_agent_scp_on_16 = np.append(obs_agent_scp_on_16, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                mission_time_scp_on_16 = np.append(mission_time_scp_on_16, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                
            if file_8:
                data = np.loadtxt(file_name_8)
                
                sim_iter = int(data[0])
                compute_time_scp_ca_16 = np.append(compute_time_scp_ca_16, data[1:sim_iter+2])
                smoothness_scp_ca_16 = np.append(smoothness_scp_ca_16, data[sim_iter+2:sim_iter+2+num_drone])
                arc_length_scp_ca_16 = np.append(arc_length_scp_ca_16, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
                inter_agent_scp_ca_16 = np.append(inter_agent_scp_ca_16, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
                obs_agent_scp_ca_16 = np.append(obs_agent_scp_ca_16, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                mission_time_scp_ca_16 = np.append(mission_time_scp_ca_16, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                # print(data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1], sim_iter+2+2*num_drone+sim_iter*num_drone+sim_iter*num_drone, config_num, num_drone)

        if file_9 and file_10 and file_11 and file_12:
            if file_9:
                data = np.loadtxt(file_name_9)
                
                sim_iter = int(data[0])
                compute_time_ours_axis_g1_16 = np.append(compute_time_ours_axis_g1_16, data[1:sim_iter+2])
                smoothness_ours_axis_g1_16 = np.append(smoothness_ours_axis_g1_16, data[sim_iter+2:sim_iter+2+num_drone])
                arc_length_ours_axis_g1_16 = np.append(arc_length_ours_axis_g1_16, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
                inter_agent_ours_axis_g1_16 = np.append(inter_agent_ours_axis_g1_16, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
                obs_agent_ours_axis_g1_16 = np.append(obs_agent_ours_axis_g1_16, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                mission_time_ours_axis_g1_16 = np.append(mission_time_ours_axis_g1_16, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                
                
            if file_10:            
                data = np.loadtxt(file_name_10)
                
                sim_iter = int(data[0])
                compute_time_ours_quad_g1_16 = np.append(compute_time_ours_quad_g1_16, data[1:sim_iter+2])
                smoothness_ours_quad_g1_16 = np.append(smoothness_ours_quad_g1_16, data[sim_iter+2:sim_iter+2+num_drone])
                arc_length_ours_quad_g1_16 = np.append(arc_length_ours_quad_g1_16, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
                inter_agent_ours_quad_g1_16 = np.append(inter_agent_ours_quad_g1_16, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
                obs_agent_ours_quad_g1_16 = np.append(obs_agent_ours_quad_g1_16, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                mission_time_ours_quad_g1_16 = np.append(mission_time_ours_quad_g1_16, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])

            if file_11:
                data = np.loadtxt(file_name_11)
                
                sim_iter = int(data[0])
                compute_time_ours_axis_g2_16 = np.append(compute_time_ours_axis_g2_16, data[1:sim_iter+2])
                smoothness_ours_axis_g2_16 = np.append(smoothness_ours_axis_g2_16, data[sim_iter+2:sim_iter+2+num_drone])
                arc_length_ours_axis_g2_16 = np.append(arc_length_ours_axis_g2_16, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
                inter_agent_ours_axis_g2_16 = np.append(inter_agent_ours_axis_g2_16, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
                obs_agent_ours_axis_g2_16 = np.append(obs_agent_ours_axis_g2_16, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                mission_time_ours_axis_g2_16 = np.append(mission_time_ours_axis_g2_16, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                
                
            if file_12:            
                data = np.loadtxt(file_name_12)
                
                sim_iter = int(data[0])
                compute_time_ours_quad_g2_16 = np.append(compute_time_ours_quad_g2_16, data[1:sim_iter+2])
                smoothness_ours_quad_g2_16 = np.append(smoothness_ours_quad_g2_16, data[sim_iter+2:sim_iter+2+num_drone])
                arc_length_ours_quad_g2_16 = np.append(arc_length_ours_quad_g2_16, data[sim_iter+2+num_drone:sim_iter+2+2*num_drone])
                inter_agent_ours_quad_g2_16 = np.append(inter_agent_ours_quad_g2_16, data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1])
                obs_agent_ours_quad_g2_16 = np.append(obs_agent_ours_quad_g2_16, data[sim_iter+2+2*num_drone+sim_iter+1:sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])
                mission_time_ours_quad_g2_16 = np.append(mission_time_ours_quad_g2_16, data[sim_iter+2+2*num_drone+sim_iter+1+sim_iter+1])

    if j == 10:
        print("10")
        mission_time_ours_axis_10_drone = np.concatenate((mission_time_ours_axis_10_drone, mission_time_ours_axis_16 , mission_time_ours_axis_8), axis=None)  
        mission_time_ours_quad_10_drone = np.concatenate((mission_time_ours_quad_10_drone, mission_time_ours_quad_16, mission_time_ours_quad_8), axis=None)  
        mission_time_scp_on_10_drone = np.concatenate((mission_time_scp_on_10_drone, mission_time_scp_on_16, mission_time_scp_on_8), axis=None)  
        mission_time_scp_ca_10_drone = np.concatenate((mission_time_scp_ca_10_drone, mission_time_scp_ca_16, mission_time_scp_ca_8), axis=None)  

        arc_length_ours_axis_10_drone = np.concatenate((arc_length_ours_axis_10_drone, arc_length_ours_axis_16, arc_length_ours_axis_8), axis=None)  
        arc_length_ours_quad_10_drone = np.concatenate((arc_length_ours_quad_10_drone, arc_length_ours_quad_16, arc_length_ours_quad_8), axis=None)  
        arc_length_scp_on_10_drone = np.concatenate((arc_length_scp_on_10_drone, arc_length_scp_on_16, arc_length_scp_on_8), axis=None)  
        arc_length_scp_ca_10_drone = np.concatenate((arc_length_scp_ca_10_drone, arc_length_scp_ca_16, arc_length_scp_ca_8), axis=None)  

        smoothness_ours_axis_10_drone = np.concatenate((smoothness_ours_axis_10_drone, smoothness_ours_axis_16, smoothness_ours_axis_8), axis=None)  
        smoothness_ours_quad_10_drone = np.concatenate((smoothness_ours_quad_10_drone, smoothness_ours_quad_16, smoothness_ours_quad_8), axis=None)  
        smoothness_scp_on_10_drone = np.concatenate((smoothness_scp_on_10_drone, smoothness_scp_on_16, smoothness_scp_on_8), axis=None)  
        smoothness_scp_ca_10_drone = np.concatenate((smoothness_scp_ca_10_drone, smoothness_scp_ca_16, smoothness_scp_ca_8), axis=None)  

        compute_time_ours_axis_10_drone = np.concatenate((compute_time_ours_axis_10_drone, compute_time_ours_axis_16, compute_time_ours_axis_8), axis=None)  
        compute_time_ours_quad_10_drone = np.concatenate((compute_time_ours_quad_10_drone, compute_time_ours_quad_16, compute_time_ours_quad_8), axis=None)  
        compute_time_scp_on_10_drone = np.concatenate((compute_time_scp_on_10_drone, compute_time_scp_on_16, compute_time_scp_on_8), axis=None)  
        compute_time_scp_ca_10_drone = np.concatenate((compute_time_scp_ca_10_drone, compute_time_scp_ca_16, compute_time_scp_ca_8), axis=None)  
        
        inter_agent_ours_axis_10_drone = np.concatenate((inter_agent_ours_axis_10_drone, inter_agent_ours_axis_16, inter_agent_ours_axis_8), axis=None)  
        inter_agent_ours_quad_10_drone = np.concatenate((inter_agent_ours_quad_10_drone, inter_agent_ours_quad_16, inter_agent_ours_quad_8), axis=None)  
        inter_agent_scp_on_10_drone = np.concatenate((inter_agent_scp_on_10_drone, inter_agent_scp_on_16, inter_agent_scp_on_8), axis=None)  
        inter_agent_scp_ca_10_drone = np.concatenate((inter_agent_scp_ca_10_drone, inter_agent_scp_ca_16, inter_agent_scp_ca_8), axis=None)

        obs_agent_ours_axis_10_drone = np.concatenate((obs_agent_ours_axis_10_drone, obs_agent_ours_axis_16, obs_agent_ours_axis_8), axis=None)  
        obs_agent_ours_quad_10_drone = np.concatenate((obs_agent_ours_quad_10_drone, obs_agent_ours_quad_16, obs_agent_ours_quad_8), axis=None)  
        obs_agent_scp_on_10_drone = np.concatenate((obs_agent_scp_on_10_drone, obs_agent_scp_on_16, obs_agent_scp_on_8), axis=None)  
        obs_agent_scp_ca_10_drone = np.concatenate((obs_agent_scp_ca_10_drone, obs_agent_scp_ca_16, obs_agent_scp_ca_8), axis=None)

        ##### 
        mission_time_ours_axis_g1_10_drone = np.concatenate((mission_time_ours_axis_g1_10_drone, mission_time_ours_axis_g1_16), axis=None)  
        mission_time_ours_quad_g1_10_drone = np.concatenate((mission_time_ours_quad_g1_10_drone, mission_time_ours_quad_g1_16), axis=None)  
        arc_length_ours_axis_g1_10_drone = np.concatenate((arc_length_ours_axis_g1_10_drone, arc_length_ours_axis_g1_16), axis=None)  
        arc_length_ours_quad_g1_10_drone = np.concatenate((arc_length_ours_quad_g1_10_drone, arc_length_ours_quad_g1_16), axis=None)  
        smoothness_ours_axis_g1_10_drone = np.concatenate((smoothness_ours_axis_g1_10_drone, smoothness_ours_axis_g1_16), axis=None)  
        smoothness_ours_quad_g1_10_drone = np.concatenate((smoothness_ours_quad_g1_10_drone, smoothness_ours_quad_g1_16), axis=None)  
        compute_time_ours_axis_g1_10_drone = np.concatenate((compute_time_ours_axis_g1_10_drone, compute_time_ours_axis_g1_16), axis=None)  
        compute_time_ours_quad_g1_10_drone = np.concatenate((compute_time_ours_quad_g1_10_drone, compute_time_ours_quad_g1_16), axis=None)  
        inter_agent_ours_axis_g1_10_drone = np.concatenate((inter_agent_ours_axis_g1_10_drone, inter_agent_ours_axis_g1_16), axis=None)  
        inter_agent_ours_quad_g1_10_drone = np.concatenate((inter_agent_ours_quad_g1_10_drone, inter_agent_ours_quad_g1_16), axis=None)  
        obs_agent_ours_axis_g1_10_drone = np.concatenate((obs_agent_ours_axis_g1_10_drone, obs_agent_ours_axis_g1_16), axis=None)  
        obs_agent_ours_quad_g1_10_drone = np.concatenate((obs_agent_ours_quad_g1_10_drone, obs_agent_ours_quad_g1_16), axis=None) 

        mission_time_ours_axis_g2_10_drone = np.concatenate((mission_time_ours_axis_g2_10_drone, mission_time_ours_axis_g2_16), axis=None)  
        mission_time_ours_quad_g2_10_drone = np.concatenate((mission_time_ours_quad_g2_10_drone, mission_time_ours_quad_g2_16), axis=None)  
        arc_length_ours_axis_g2_10_drone = np.concatenate((arc_length_ours_axis_g2_10_drone, arc_length_ours_axis_g2_16), axis=None)  
        arc_length_ours_quad_g2_10_drone = np.concatenate((arc_length_ours_quad_g2_10_drone, arc_length_ours_quad_g2_16), axis=None)  
        smoothness_ours_axis_g2_10_drone = np.concatenate((smoothness_ours_axis_g2_10_drone, smoothness_ours_axis_g2_16), axis=None)  
        smoothness_ours_quad_g2_10_drone = np.concatenate((smoothness_ours_quad_g2_10_drone, smoothness_ours_quad_g2_16), axis=None)  
        compute_time_ours_axis_g2_10_drone = np.concatenate((compute_time_ours_axis_g2_10_drone, compute_time_ours_axis_g2_16), axis=None)  
        compute_time_ours_quad_g2_10_drone = np.concatenate((compute_time_ours_quad_g2_10_drone, compute_time_ours_quad_g2_16), axis=None)  
        inter_agent_ours_axis_g2_10_drone = np.concatenate((inter_agent_ours_axis_g2_10_drone, inter_agent_ours_axis_g2_16), axis=None)  
        inter_agent_ours_quad_g2_10_drone = np.concatenate((inter_agent_ours_quad_g2_10_drone, inter_agent_ours_quad_g2_16), axis=None)  
        obs_agent_ours_axis_g2_10_drone = np.concatenate((obs_agent_ours_axis_g2_10_drone, obs_agent_ours_axis_g2_16), axis=None)  
        obs_agent_ours_quad_g2_10_drone = np.concatenate((obs_agent_ours_quad_g2_10_drone, obs_agent_ours_quad_g2_16), axis=None)  
        #####
    if j == 20:
        print("20")
        mission_time_ours_axis_20_drone = np.concatenate((mission_time_ours_axis_20_drone, mission_time_ours_axis_16 , mission_time_ours_axis_8), axis=None)  
        mission_time_ours_quad_20_drone = np.concatenate((mission_time_ours_quad_20_drone, mission_time_ours_quad_16, mission_time_ours_quad_8), axis=None)  
        mission_time_scp_on_20_drone = np.concatenate((mission_time_scp_on_20_drone, mission_time_scp_on_16, mission_time_scp_on_8), axis=None)  
        mission_time_scp_ca_20_drone = np.concatenate((mission_time_scp_ca_20_drone, mission_time_scp_ca_16, mission_time_scp_ca_8), axis=None)  

        arc_length_ours_axis_20_drone = np.concatenate((arc_length_ours_axis_20_drone, arc_length_ours_axis_16, arc_length_ours_axis_8), axis=None)  
        arc_length_ours_quad_20_drone = np.concatenate((arc_length_ours_quad_20_drone, arc_length_ours_quad_16, arc_length_ours_quad_8), axis=None)  
        arc_length_scp_on_20_drone = np.concatenate((arc_length_scp_on_20_drone, arc_length_scp_on_16, arc_length_scp_on_8), axis=None)  
        arc_length_scp_ca_20_drone = np.concatenate((arc_length_scp_ca_20_drone, arc_length_scp_ca_16, arc_length_scp_ca_8), axis=None)  

        smoothness_ours_axis_20_drone = np.concatenate((smoothness_ours_axis_20_drone, smoothness_ours_axis_16, smoothness_ours_axis_8), axis=None)  
        smoothness_ours_quad_20_drone = np.concatenate((smoothness_ours_quad_20_drone, smoothness_ours_quad_16, smoothness_ours_quad_8), axis=None)  
        smoothness_scp_on_20_drone = np.concatenate((smoothness_scp_on_20_drone, smoothness_scp_on_16, smoothness_scp_on_8), axis=None)  
        smoothness_scp_ca_20_drone = np.concatenate((smoothness_scp_ca_20_drone, smoothness_scp_ca_16, smoothness_scp_ca_8), axis=None)  

        compute_time_ours_axis_20_drone = np.concatenate((compute_time_ours_axis_20_drone, compute_time_ours_axis_16, compute_time_ours_axis_8), axis=None)  
        compute_time_ours_quad_20_drone = np.concatenate((compute_time_ours_quad_20_drone, compute_time_ours_quad_16, compute_time_ours_quad_8), axis=None)  
        compute_time_scp_on_20_drone = np.concatenate((compute_time_scp_on_20_drone, compute_time_scp_on_16, compute_time_scp_on_8), axis=None)  
        compute_time_scp_ca_20_drone = np.concatenate((compute_time_scp_ca_20_drone, compute_time_scp_ca_16, compute_time_scp_ca_8), axis=None)  

        inter_agent_ours_axis_20_drone = np.concatenate((inter_agent_ours_axis_20_drone, inter_agent_ours_axis_16, inter_agent_ours_axis_8), axis=None)  
        inter_agent_ours_quad_20_drone = np.concatenate((inter_agent_ours_quad_20_drone, inter_agent_ours_quad_16, inter_agent_ours_quad_8), axis=None)  
        inter_agent_scp_on_20_drone = np.concatenate((inter_agent_scp_on_20_drone, inter_agent_scp_on_16, inter_agent_scp_on_8), axis=None)  
        inter_agent_scp_ca_20_drone = np.concatenate((inter_agent_scp_ca_20_drone, inter_agent_scp_ca_16, inter_agent_scp_ca_8), axis=None)

        obs_agent_ours_axis_20_drone = np.concatenate((obs_agent_ours_axis_20_drone, obs_agent_ours_axis_16, obs_agent_ours_axis_8), axis=None)  
        obs_agent_ours_quad_20_drone = np.concatenate((obs_agent_ours_quad_20_drone, obs_agent_ours_quad_16, obs_agent_ours_quad_8), axis=None)  
        obs_agent_scp_on_20_drone = np.concatenate((obs_agent_scp_on_20_drone, obs_agent_scp_on_16, obs_agent_scp_on_8), axis=None)  
        obs_agent_scp_ca_20_drone = np.concatenate((obs_agent_scp_ca_20_drone, obs_agent_scp_ca_16, obs_agent_scp_ca_8), axis=None)
        
        ##### 
        mission_time_ours_axis_g1_20_drone = np.concatenate((mission_time_ours_axis_g1_20_drone, mission_time_ours_axis_g1_16), axis=None)  
        mission_time_ours_quad_g1_20_drone = np.concatenate((mission_time_ours_quad_g1_20_drone, mission_time_ours_quad_g1_16), axis=None)  
        arc_length_ours_axis_g1_20_drone = np.concatenate((arc_length_ours_axis_g1_20_drone, arc_length_ours_axis_g1_16), axis=None)  
        arc_length_ours_quad_g1_20_drone = np.concatenate((arc_length_ours_quad_g1_20_drone, arc_length_ours_quad_g1_16), axis=None)  
        smoothness_ours_axis_g1_20_drone = np.concatenate((smoothness_ours_axis_g1_20_drone, smoothness_ours_axis_g1_16), axis=None)  
        smoothness_ours_quad_g1_20_drone = np.concatenate((smoothness_ours_quad_g1_20_drone, smoothness_ours_quad_g1_16), axis=None)  
        compute_time_ours_axis_g1_20_drone = np.concatenate((compute_time_ours_axis_g1_20_drone, compute_time_ours_axis_g1_16), axis=None)  
        compute_time_ours_quad_g1_20_drone = np.concatenate((compute_time_ours_quad_g1_20_drone, compute_time_ours_quad_g1_16), axis=None)  
        inter_agent_ours_axis_g1_20_drone = np.concatenate((inter_agent_ours_axis_g1_20_drone, inter_agent_ours_axis_g1_16), axis=None)  
        inter_agent_ours_quad_g1_20_drone = np.concatenate((inter_agent_ours_quad_g1_20_drone, inter_agent_ours_quad_g1_16), axis=None)  
        obs_agent_ours_axis_g1_20_drone = np.concatenate((obs_agent_ours_axis_g1_20_drone, obs_agent_ours_axis_g1_16), axis=None)  
        obs_agent_ours_quad_g1_20_drone = np.concatenate((obs_agent_ours_quad_g1_20_drone, obs_agent_ours_quad_g1_16), axis=None)  

        mission_time_ours_axis_g2_20_drone = np.concatenate((mission_time_ours_axis_g2_20_drone, mission_time_ours_axis_g2_16), axis=None)  
        mission_time_ours_quad_g2_20_drone = np.concatenate((mission_time_ours_quad_g2_20_drone, mission_time_ours_quad_g2_16), axis=None)  
        arc_length_ours_axis_g2_20_drone = np.concatenate((arc_length_ours_axis_g2_20_drone, arc_length_ours_axis_g2_16), axis=None)  
        arc_length_ours_quad_g2_20_drone = np.concatenate((arc_length_ours_quad_g2_20_drone, arc_length_ours_quad_g2_16), axis=None)  
        smoothness_ours_axis_g2_20_drone = np.concatenate((smoothness_ours_axis_g2_20_drone, smoothness_ours_axis_g2_16), axis=None)  
        smoothness_ours_quad_g2_20_drone = np.concatenate((smoothness_ours_quad_g2_20_drone, smoothness_ours_quad_g2_16), axis=None)  
        compute_time_ours_axis_g2_20_drone = np.concatenate((compute_time_ours_axis_g2_20_drone, compute_time_ours_axis_g2_16), axis=None)  
        compute_time_ours_quad_g2_20_drone = np.concatenate((compute_time_ours_quad_g2_20_drone, compute_time_ours_quad_g2_16), axis=None)  
        inter_agent_ours_axis_g2_20_drone = np.concatenate((inter_agent_ours_axis_g2_20_drone, inter_agent_ours_axis_g2_16), axis=None)  
        inter_agent_ours_quad_g2_20_drone = np.concatenate((inter_agent_ours_quad_g2_20_drone, inter_agent_ours_quad_g2_16), axis=None)  
        obs_agent_ours_axis_g2_20_drone = np.concatenate((obs_agent_ours_axis_g2_20_drone, obs_agent_ours_axis_g2_16), axis=None)  
        obs_agent_ours_quad_g2_20_drone = np.concatenate((obs_agent_ours_quad_g2_20_drone, obs_agent_ours_quad_g2_16), axis=None)  
        #####

    if j == 30:
        print("30")
        mission_time_ours_axis_30_drone = np.concatenate((mission_time_ours_axis_30_drone, mission_time_ours_axis_16 , mission_time_ours_axis_8), axis=None)  
        mission_time_ours_quad_30_drone = np.concatenate((mission_time_ours_quad_30_drone, mission_time_ours_quad_16, mission_time_ours_quad_8), axis=None)  
        mission_time_scp_on_30_drone = np.concatenate((mission_time_scp_on_30_drone, mission_time_scp_on_16, mission_time_scp_on_8), axis=None)  
        mission_time_scp_ca_30_drone = np.concatenate((mission_time_scp_ca_30_drone, mission_time_scp_ca_16, mission_time_scp_ca_8), axis=None)  

        arc_length_ours_axis_30_drone = np.concatenate((arc_length_ours_axis_30_drone, arc_length_ours_axis_16, arc_length_ours_axis_8), axis=None)  
        arc_length_ours_quad_30_drone = np.concatenate((arc_length_ours_quad_30_drone, arc_length_ours_quad_16, arc_length_ours_quad_8), axis=None)  
        arc_length_scp_on_30_drone = np.concatenate((arc_length_scp_on_30_drone, arc_length_scp_on_16, arc_length_scp_on_8), axis=None)  
        arc_length_scp_ca_30_drone = np.concatenate((arc_length_scp_ca_30_drone, arc_length_scp_ca_16, arc_length_scp_ca_8), axis=None)  

        smoothness_ours_axis_30_drone = np.concatenate((smoothness_ours_axis_30_drone, smoothness_ours_axis_16, smoothness_ours_axis_8), axis=None)  
        smoothness_ours_quad_30_drone = np.concatenate((smoothness_ours_quad_30_drone, smoothness_ours_quad_16, smoothness_ours_quad_8), axis=None)  
        smoothness_scp_on_30_drone = np.concatenate((smoothness_scp_on_30_drone, smoothness_scp_on_16, smoothness_scp_on_8), axis=None)  
        smoothness_scp_ca_30_drone = np.concatenate((smoothness_scp_ca_30_drone, smoothness_scp_ca_16, smoothness_scp_ca_8), axis=None)  

        compute_time_ours_axis_30_drone = np.concatenate((compute_time_ours_axis_30_drone, compute_time_ours_axis_16, compute_time_ours_axis_8), axis=None)  
        compute_time_ours_quad_30_drone = np.concatenate((compute_time_ours_quad_30_drone, compute_time_ours_quad_16, compute_time_ours_quad_8), axis=None)  
        compute_time_scp_on_30_drone = np.concatenate((compute_time_scp_on_30_drone, compute_time_scp_on_16, compute_time_scp_on_8), axis=None)  
        compute_time_scp_ca_30_drone = np.concatenate((compute_time_scp_ca_30_drone, compute_time_scp_ca_16, compute_time_scp_ca_8), axis=None)  

        inter_agent_ours_axis_30_drone = np.concatenate((inter_agent_ours_axis_30_drone, inter_agent_ours_axis_16, inter_agent_ours_axis_8), axis=None)  
        inter_agent_ours_quad_30_drone = np.concatenate((inter_agent_ours_quad_30_drone, inter_agent_ours_quad_16, inter_agent_ours_quad_8), axis=None)  
        inter_agent_scp_on_30_drone = np.concatenate((inter_agent_scp_on_30_drone, inter_agent_scp_on_16, inter_agent_scp_on_8), axis=None)  
        inter_agent_scp_ca_30_drone = np.concatenate((inter_agent_scp_ca_30_drone, inter_agent_scp_ca_16, inter_agent_scp_ca_8), axis=None)

        obs_agent_ours_axis_30_drone = np.concatenate((obs_agent_ours_axis_30_drone, obs_agent_ours_axis_16, obs_agent_ours_axis_8), axis=None)  
        obs_agent_ours_quad_30_drone = np.concatenate((obs_agent_ours_quad_30_drone, obs_agent_ours_quad_16, obs_agent_ours_quad_8), axis=None)  
        obs_agent_scp_on_30_drone = np.concatenate((obs_agent_scp_on_30_drone, obs_agent_scp_on_16, obs_agent_scp_on_8), axis=None)  
        obs_agent_scp_ca_30_drone = np.concatenate((obs_agent_scp_ca_30_drone, obs_agent_scp_ca_16, obs_agent_scp_ca_8), axis=None)
        
        ##### 
        mission_time_ours_axis_g1_30_drone = np.concatenate((mission_time_ours_axis_g1_30_drone, mission_time_ours_axis_g1_16), axis=None)  
        mission_time_ours_quad_g1_30_drone = np.concatenate((mission_time_ours_quad_g1_30_drone, mission_time_ours_quad_g1_16), axis=None)  
        arc_length_ours_axis_g1_30_drone = np.concatenate((arc_length_ours_axis_g1_30_drone, arc_length_ours_axis_g1_16), axis=None)  
        arc_length_ours_quad_g1_30_drone = np.concatenate((arc_length_ours_quad_g1_30_drone, arc_length_ours_quad_g1_16), axis=None)  
        smoothness_ours_axis_g1_30_drone = np.concatenate((smoothness_ours_axis_g1_30_drone, smoothness_ours_axis_g1_16), axis=None)  
        smoothness_ours_quad_g1_30_drone = np.concatenate((smoothness_ours_quad_g1_30_drone, smoothness_ours_quad_g1_16), axis=None)  
        compute_time_ours_axis_g1_30_drone = np.concatenate((compute_time_ours_axis_g1_30_drone, compute_time_ours_axis_g1_16), axis=None)  
        compute_time_ours_quad_g1_30_drone = np.concatenate((compute_time_ours_quad_g1_30_drone, compute_time_ours_quad_g1_16), axis=None)  
        inter_agent_ours_axis_g1_30_drone = np.concatenate((inter_agent_ours_axis_g1_30_drone, inter_agent_ours_axis_g1_16), axis=None)  
        inter_agent_ours_quad_g1_30_drone = np.concatenate((inter_agent_ours_quad_g1_30_drone, inter_agent_ours_quad_g1_16), axis=None)  
        obs_agent_ours_axis_g1_30_drone = np.concatenate((obs_agent_ours_axis_g1_30_drone, obs_agent_ours_axis_g1_16), axis=None)  
        obs_agent_ours_quad_g1_30_drone = np.concatenate((obs_agent_ours_quad_g1_30_drone, obs_agent_ours_quad_g1_16), axis=None)  

        mission_time_ours_axis_g2_30_drone = np.concatenate((mission_time_ours_axis_g2_30_drone, mission_time_ours_axis_g2_16), axis=None)  
        mission_time_ours_quad_g2_30_drone = np.concatenate((mission_time_ours_quad_g2_30_drone, mission_time_ours_quad_g2_16), axis=None)  
        arc_length_ours_axis_g2_30_drone = np.concatenate((arc_length_ours_axis_g2_30_drone, arc_length_ours_axis_g2_16), axis=None)  
        arc_length_ours_quad_g2_30_drone = np.concatenate((arc_length_ours_quad_g2_30_drone, arc_length_ours_quad_g2_16), axis=None)  
        smoothness_ours_axis_g2_30_drone = np.concatenate((smoothness_ours_axis_g2_30_drone, smoothness_ours_axis_g2_16), axis=None)  
        smoothness_ours_quad_g2_30_drone = np.concatenate((smoothness_ours_quad_g2_30_drone, smoothness_ours_quad_g2_16), axis=None)  
        compute_time_ours_axis_g2_30_drone = np.concatenate((compute_time_ours_axis_g2_30_drone, compute_time_ours_axis_g2_16), axis=None)  
        compute_time_ours_quad_g2_30_drone = np.concatenate((compute_time_ours_quad_g2_30_drone, compute_time_ours_quad_g2_16), axis=None)  
        inter_agent_ours_axis_g2_30_drone = np.concatenate((inter_agent_ours_axis_g2_30_drone, inter_agent_ours_axis_g2_16), axis=None)  
        inter_agent_ours_quad_g2_30_drone = np.concatenate((inter_agent_ours_quad_g2_30_drone, inter_agent_ours_quad_g2_16), axis=None)  
        obs_agent_ours_axis_g2_30_drone = np.concatenate((obs_agent_ours_axis_g2_30_drone, obs_agent_ours_axis_g2_16), axis=None)  
        obs_agent_ours_quad_g2_30_drone = np.concatenate((obs_agent_ours_quad_g2_30_drone, obs_agent_ours_quad_g2_16), axis=None)  
        #####

    if j == 40:
        print("40")
        mission_time_ours_axis_40_drone = np.concatenate((mission_time_ours_axis_40_drone, mission_time_ours_axis_16 , mission_time_ours_axis_8), axis=None)  
        mission_time_ours_quad_40_drone = np.concatenate((mission_time_ours_quad_40_drone, mission_time_ours_quad_16, mission_time_ours_quad_8), axis=None)  
        mission_time_scp_on_40_drone = np.concatenate((mission_time_scp_on_40_drone, mission_time_scp_on_16, mission_time_scp_on_8), axis=None)  
        mission_time_scp_ca_40_drone = np.concatenate((mission_time_scp_ca_40_drone, mission_time_scp_ca_16, mission_time_scp_ca_8), axis=None)  

        arc_length_ours_axis_40_drone = np.concatenate((arc_length_ours_axis_40_drone, arc_length_ours_axis_16, arc_length_ours_axis_8), axis=None)  
        arc_length_ours_quad_40_drone = np.concatenate((arc_length_ours_quad_40_drone, arc_length_ours_quad_16, arc_length_ours_quad_8), axis=None)  
        arc_length_scp_on_40_drone = np.concatenate((arc_length_scp_on_40_drone, arc_length_scp_on_16, arc_length_scp_on_8), axis=None)  
        arc_length_scp_ca_40_drone = np.concatenate((arc_length_scp_ca_40_drone, arc_length_scp_ca_16, arc_length_scp_ca_8), axis=None)  

        smoothness_ours_axis_40_drone = np.concatenate((smoothness_ours_axis_40_drone, smoothness_ours_axis_16, smoothness_ours_axis_8), axis=None)  
        smoothness_ours_quad_40_drone = np.concatenate((smoothness_ours_quad_40_drone, smoothness_ours_quad_16, smoothness_ours_quad_8), axis=None)  
        smoothness_scp_on_40_drone = np.concatenate((smoothness_scp_on_40_drone, smoothness_scp_on_16, smoothness_scp_on_8), axis=None)  
        smoothness_scp_ca_40_drone = np.concatenate((smoothness_scp_ca_40_drone, smoothness_scp_ca_16, smoothness_scp_ca_8), axis=None)  

        compute_time_ours_axis_40_drone = np.concatenate((compute_time_ours_axis_40_drone, compute_time_ours_axis_16, compute_time_ours_axis_8), axis=None)  
        compute_time_ours_quad_40_drone = np.concatenate((compute_time_ours_quad_40_drone, compute_time_ours_quad_16, compute_time_ours_quad_8), axis=None)  
        compute_time_scp_on_40_drone = np.concatenate((compute_time_scp_on_40_drone, compute_time_scp_on_16, compute_time_scp_on_8), axis=None)  
        compute_time_scp_ca_40_drone = np.concatenate((compute_time_scp_ca_40_drone, compute_time_scp_ca_16, compute_time_scp_ca_8), axis=None)  

        inter_agent_ours_axis_40_drone = np.concatenate((inter_agent_ours_axis_40_drone, inter_agent_ours_axis_16, inter_agent_ours_axis_8), axis=None)  
        inter_agent_ours_quad_40_drone = np.concatenate((inter_agent_ours_quad_40_drone, inter_agent_ours_quad_16, inter_agent_ours_quad_8), axis=None)  
        inter_agent_scp_on_40_drone = np.concatenate((inter_agent_scp_on_40_drone, inter_agent_scp_on_16, inter_agent_scp_on_8), axis=None)  
        inter_agent_scp_ca_40_drone = np.concatenate((inter_agent_scp_ca_40_drone, inter_agent_scp_ca_16, inter_agent_scp_ca_8), axis=None)

        obs_agent_ours_axis_40_drone = np.concatenate((obs_agent_ours_axis_40_drone, obs_agent_ours_axis_16, obs_agent_ours_axis_8), axis=None)  
        obs_agent_ours_quad_40_drone = np.concatenate((obs_agent_ours_quad_40_drone, obs_agent_ours_quad_16, obs_agent_ours_quad_8), axis=None)  
        obs_agent_scp_on_40_drone = np.concatenate((obs_agent_scp_on_40_drone, obs_agent_scp_on_16, obs_agent_scp_on_8), axis=None)  
        obs_agent_scp_ca_40_drone = np.concatenate((obs_agent_scp_ca_40_drone, obs_agent_scp_ca_16, obs_agent_scp_ca_8), axis=None)

        ##### 
        mission_time_ours_axis_g1_40_drone = np.concatenate((mission_time_ours_axis_g1_40_drone, mission_time_ours_axis_g1_16), axis=None)  
        mission_time_ours_quad_g1_40_drone = np.concatenate((mission_time_ours_quad_g1_40_drone, mission_time_ours_quad_g1_16), axis=None)  
        arc_length_ours_axis_g1_40_drone = np.concatenate((arc_length_ours_axis_g1_40_drone, arc_length_ours_axis_g1_16), axis=None)  
        arc_length_ours_quad_g1_40_drone = np.concatenate((arc_length_ours_quad_g1_40_drone, arc_length_ours_quad_g1_16), axis=None)  
        smoothness_ours_axis_g1_40_drone = np.concatenate((smoothness_ours_axis_g1_40_drone, smoothness_ours_axis_g1_16), axis=None)  
        smoothness_ours_quad_g1_40_drone = np.concatenate((smoothness_ours_quad_g1_40_drone, smoothness_ours_quad_g1_16), axis=None)  
        compute_time_ours_axis_g1_40_drone = np.concatenate((compute_time_ours_axis_g1_40_drone, compute_time_ours_axis_g1_16), axis=None)  
        compute_time_ours_quad_g1_40_drone = np.concatenate((compute_time_ours_quad_g1_40_drone, compute_time_ours_quad_g1_16), axis=None)  
        inter_agent_ours_axis_g1_40_drone = np.concatenate((inter_agent_ours_axis_g1_40_drone, inter_agent_ours_axis_g1_16), axis=None)  
        inter_agent_ours_quad_g1_40_drone = np.concatenate((inter_agent_ours_quad_g1_40_drone, inter_agent_ours_quad_g1_16), axis=None)  
        obs_agent_ours_axis_g1_40_drone = np.concatenate((obs_agent_ours_axis_g1_40_drone, obs_agent_ours_axis_g1_16), axis=None)  
        obs_agent_ours_quad_g1_40_drone = np.concatenate((obs_agent_ours_quad_g1_40_drone, obs_agent_ours_quad_g1_16), axis=None)  

        mission_time_ours_axis_g2_40_drone = np.concatenate((mission_time_ours_axis_g2_40_drone, mission_time_ours_axis_g2_16), axis=None)  
        mission_time_ours_quad_g2_40_drone = np.concatenate((mission_time_ours_quad_g2_40_drone, mission_time_ours_quad_g2_16), axis=None)  
        arc_length_ours_axis_g2_40_drone = np.concatenate((arc_length_ours_axis_g2_40_drone, arc_length_ours_axis_g2_16), axis=None)  
        arc_length_ours_quad_g2_40_drone = np.concatenate((arc_length_ours_quad_g2_40_drone, arc_length_ours_quad_g2_16), axis=None)  
        smoothness_ours_axis_g2_40_drone = np.concatenate((smoothness_ours_axis_g2_40_drone, smoothness_ours_axis_g2_16), axis=None)  
        smoothness_ours_quad_g2_40_drone = np.concatenate((smoothness_ours_quad_g2_40_drone, smoothness_ours_quad_g2_16), axis=None)  
        compute_time_ours_axis_g2_40_drone = np.concatenate((compute_time_ours_axis_g2_40_drone, compute_time_ours_axis_g2_16), axis=None)  
        compute_time_ours_quad_g2_40_drone = np.concatenate((compute_time_ours_quad_g2_40_drone, compute_time_ours_quad_g2_16), axis=None)  
        inter_agent_ours_axis_g2_40_drone = np.concatenate((inter_agent_ours_axis_g2_40_drone, inter_agent_ours_axis_g2_16), axis=None)  
        inter_agent_ours_quad_g2_40_drone = np.concatenate((inter_agent_ours_quad_g2_40_drone, inter_agent_ours_quad_g2_16), axis=None)  
        obs_agent_ours_axis_g2_40_drone = np.concatenate((obs_agent_ours_axis_g2_40_drone, obs_agent_ours_axis_g2_16), axis=None)  
        obs_agent_ours_quad_g2_40_drone = np.concatenate((obs_agent_ours_quad_g2_40_drone, obs_agent_ours_quad_g2_16), axis=None)  

        #####
        
    if j == 50:
        print("50")
        mission_time_ours_axis_50_drone = np.concatenate((mission_time_ours_axis_50_drone, mission_time_ours_axis_16 , mission_time_ours_axis_8), axis=None)  
        mission_time_ours_quad_50_drone = np.concatenate((mission_time_ours_quad_50_drone, mission_time_ours_quad_16, mission_time_ours_quad_8), axis=None)  
        mission_time_scp_on_50_drone = np.concatenate((mission_time_scp_on_50_drone, mission_time_scp_on_16, mission_time_scp_on_8), axis=None)  
        mission_time_scp_ca_50_drone = np.concatenate((mission_time_scp_ca_50_drone, mission_time_scp_ca_16, mission_time_scp_ca_8), axis=None)  

        arc_length_ours_axis_50_drone = np.concatenate((arc_length_ours_axis_50_drone, arc_length_ours_axis_16, arc_length_ours_axis_8), axis=None)  
        arc_length_ours_quad_50_drone = np.concatenate((arc_length_ours_quad_50_drone, arc_length_ours_quad_16, arc_length_ours_quad_8), axis=None)  
        arc_length_scp_on_50_drone = np.concatenate((arc_length_scp_on_50_drone, arc_length_scp_on_16, arc_length_scp_on_8), axis=None)  
        arc_length_scp_ca_50_drone = np.concatenate((arc_length_scp_ca_50_drone, arc_length_scp_ca_16, arc_length_scp_ca_8), axis=None)  

        smoothness_ours_axis_50_drone = np.concatenate((smoothness_ours_axis_50_drone, smoothness_ours_axis_16, smoothness_ours_axis_8), axis=None)  
        smoothness_ours_quad_50_drone = np.concatenate((smoothness_ours_quad_50_drone, smoothness_ours_quad_16, smoothness_ours_quad_8), axis=None)  
        smoothness_scp_on_50_drone = np.concatenate((smoothness_scp_on_50_drone, smoothness_scp_on_16, smoothness_scp_on_8), axis=None)  
        smoothness_scp_ca_50_drone = np.concatenate((smoothness_scp_ca_50_drone, smoothness_scp_ca_16, smoothness_scp_ca_8), axis=None)  

        compute_time_ours_axis_50_drone = np.concatenate((compute_time_ours_axis_50_drone, compute_time_ours_axis_16, compute_time_ours_axis_8), axis=None)  
        compute_time_ours_quad_50_drone = np.concatenate((compute_time_ours_quad_50_drone, compute_time_ours_quad_16, compute_time_ours_quad_8), axis=None)  
        compute_time_scp_on_50_drone = np.concatenate((compute_time_scp_on_50_drone, compute_time_scp_on_16, compute_time_scp_on_8), axis=None)  
        compute_time_scp_ca_50_drone = np.concatenate((compute_time_scp_ca_50_drone, compute_time_scp_ca_16, compute_time_scp_ca_8), axis=None)  

        inter_agent_ours_axis_50_drone = np.concatenate((inter_agent_ours_axis_50_drone, inter_agent_ours_axis_16, inter_agent_ours_axis_8), axis=None)  
        inter_agent_ours_quad_50_drone = np.concatenate((inter_agent_ours_quad_50_drone, inter_agent_ours_quad_16, inter_agent_ours_quad_8), axis=None)  
        inter_agent_scp_on_50_drone = np.concatenate((inter_agent_scp_on_50_drone, inter_agent_scp_on_16, inter_agent_scp_on_8), axis=None)  
        inter_agent_scp_ca_50_drone = np.concatenate((inter_agent_scp_ca_50_drone, inter_agent_scp_ca_16, inter_agent_scp_ca_8), axis=None)

        obs_agent_ours_axis_50_drone = np.concatenate((obs_agent_ours_axis_50_drone, obs_agent_ours_axis_16, obs_agent_ours_axis_8), axis=None)  
        obs_agent_ours_quad_50_drone = np.concatenate((obs_agent_ours_quad_50_drone, obs_agent_ours_quad_16, obs_agent_ours_quad_8), axis=None)  
        obs_agent_scp_on_50_drone = np.concatenate((obs_agent_scp_on_50_drone, obs_agent_scp_on_16, obs_agent_scp_on_8), axis=None)  
        obs_agent_scp_ca_50_drone = np.concatenate((obs_agent_scp_ca_50_drone, obs_agent_scp_ca_16, obs_agent_scp_ca_8), axis=None)
            
        ##### 
        mission_time_ours_axis_g1_50_drone = np.concatenate((mission_time_ours_axis_g1_50_drone, mission_time_ours_axis_g1_16), axis=None)  
        mission_time_ours_quad_g1_50_drone = np.concatenate((mission_time_ours_quad_g1_50_drone, mission_time_ours_quad_g1_16), axis=None)  
        arc_length_ours_axis_g1_50_drone = np.concatenate((arc_length_ours_axis_g1_50_drone, arc_length_ours_axis_g1_16), axis=None)  
        arc_length_ours_quad_g1_50_drone = np.concatenate((arc_length_ours_quad_g1_50_drone, arc_length_ours_quad_g1_16), axis=None)  
        smoothness_ours_axis_g1_50_drone = np.concatenate((smoothness_ours_axis_g1_50_drone, smoothness_ours_axis_g1_16), axis=None)  
        smoothness_ours_quad_g1_50_drone = np.concatenate((smoothness_ours_quad_g1_50_drone, smoothness_ours_quad_g1_16), axis=None)  
        compute_time_ours_axis_g1_50_drone = np.concatenate((compute_time_ours_axis_g1_50_drone, compute_time_ours_axis_g1_16), axis=None)  
        compute_time_ours_quad_g1_50_drone = np.concatenate((compute_time_ours_quad_g1_50_drone, compute_time_ours_quad_g1_16), axis=None)  
        inter_agent_ours_axis_g1_50_drone = np.concatenate((inter_agent_ours_axis_g1_50_drone, inter_agent_ours_axis_g1_16), axis=None)  
        inter_agent_ours_quad_g1_50_drone = np.concatenate((inter_agent_ours_quad_g1_50_drone, inter_agent_ours_quad_g1_16), axis=None)  
        obs_agent_ours_axis_g1_50_drone = np.concatenate((obs_agent_ours_axis_g1_50_drone, obs_agent_ours_axis_g1_16), axis=None)  
        obs_agent_ours_quad_g1_50_drone = np.concatenate((obs_agent_ours_quad_g1_50_drone, obs_agent_ours_quad_g1_16), axis=None)  

        mission_time_ours_axis_g2_50_drone = np.concatenate((mission_time_ours_axis_g2_50_drone, mission_time_ours_axis_g2_16), axis=None)  
        mission_time_ours_quad_g2_50_drone = np.concatenate((mission_time_ours_quad_g2_50_drone, mission_time_ours_quad_g2_16), axis=None)  
        arc_length_ours_axis_g2_50_drone = np.concatenate((arc_length_ours_axis_g2_50_drone, arc_length_ours_axis_g2_16), axis=None)  
        arc_length_ours_quad_g2_50_drone = np.concatenate((arc_length_ours_quad_g2_50_drone, arc_length_ours_quad_g2_16), axis=None)  
        smoothness_ours_axis_g2_50_drone = np.concatenate((smoothness_ours_axis_g2_50_drone, smoothness_ours_axis_g2_16), axis=None)  
        smoothness_ours_quad_g2_50_drone = np.concatenate((smoothness_ours_quad_g2_50_drone, smoothness_ours_quad_g2_16), axis=None)  
        compute_time_ours_axis_g2_50_drone = np.concatenate((compute_time_ours_axis_g2_50_drone, compute_time_ours_axis_g2_16), axis=None)  
        compute_time_ours_quad_g2_50_drone = np.concatenate((compute_time_ours_quad_g2_50_drone, compute_time_ours_quad_g2_16), axis=None)  
        inter_agent_ours_axis_g2_50_drone = np.concatenate((inter_agent_ours_axis_g2_50_drone, inter_agent_ours_axis_g2_16), axis=None)  
        inter_agent_ours_quad_g2_50_drone = np.concatenate((inter_agent_ours_quad_g2_50_drone, inter_agent_ours_quad_g2_16), axis=None)  
        obs_agent_ours_axis_g2_50_drone = np.concatenate((obs_agent_ours_axis_g2_50_drone, obs_agent_ours_axis_g2_16), axis=None)  
        obs_agent_ours_quad_g2_50_drone = np.concatenate((obs_agent_ours_quad_g2_50_drone, obs_agent_ours_quad_g2_16), axis=None)  
        #####

            
agents = np.array([10, 20, 30, 40, 50])
        
# plt.figure(1, figsize=(6.5,5))
# ax = plt.gca()
# ax.xaxis.set_major_locator(MaxNLocator(integer=True))

# plt.errorbar(agents, success_ours_axis_8*(100.0/total_config), label='ours_axiswise', '--o')
# plt.errorbar(agents, success_ours_quad_8*(100.0/total_config), label='ours_quadratic', '--o')
# plt.errorbar(agents, success_scp_ca_8*(100.0/total_config), label='scp_ca', '--o')
# plt.errorbar(agents, success_scp_on_8*(100.0/total_config), label='scp_ondemand', '--o')


# plt.scatter(agents, success_ours_axis_8*(100.0/total_config))
# plt.scatter(agents, success_ours_quad_8*(100.0/total_config))
# plt.scatter(agents, success_scp_ca_8*(100.0/total_config))
# plt.scatter(agents, success_scp_on_8*(100.0/total_config))

# plt.ylabel('Success Rate %')
# plt.xlabel('Agents')
# plt.grid()
# plt.title('Success Rate vs Number of Agents')
# plt.legend()
# plt.xticks(agents)

# plt.savefig('success_1.png', dpi=100)

SMALL_SIZE = 8
MEDIUM_SIZE = 10
BIGGER_SIZE = 18

plt.rc('font', size=BIGGER_SIZE)          # controls default text sizes
plt.rc('axes', titlesize=BIGGER_SIZE)     # fontsize of the axes title
plt.rc('axes', labelsize=BIGGER_SIZE)    # fontsize of the x and y labels
plt.rc('xtick', labelsize=BIGGER_SIZE)    # fontsize of the tick labels
plt.rc('ytick', labelsize=BIGGER_SIZE)    # fontsize of the tick labels
plt.rc('legend', fontsize=BIGGER_SIZE)    # legend fontsize
plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title


### MISSION TIME
####
mission_time_ours_axis_g1_10_drone = mission_time_ours_axis_g1_10_drone.reshape(mission_time_ours_axis_g1_10_drone.size, 1)
mission_time_ours_quad_g1_10_drone = mission_time_ours_quad_g1_10_drone.reshape(mission_time_ours_quad_g1_10_drone.size, 1)
mission_time_ours_axis_g2_10_drone = mission_time_ours_axis_g2_10_drone.reshape(mission_time_ours_axis_g2_10_drone.size, 1)
mission_time_ours_quad_g2_10_drone = mission_time_ours_quad_g2_10_drone.reshape(mission_time_ours_quad_g2_10_drone.size, 1)

mission_time_ours_axis_g1_20_drone = mission_time_ours_axis_g1_20_drone.reshape(mission_time_ours_axis_g1_20_drone.size, 1)
mission_time_ours_quad_g1_20_drone = mission_time_ours_quad_g1_20_drone.reshape(mission_time_ours_quad_g1_20_drone.size, 1)
mission_time_ours_axis_g2_20_drone = mission_time_ours_axis_g2_20_drone.reshape(mission_time_ours_axis_g2_20_drone.size, 1)
mission_time_ours_quad_g2_20_drone = mission_time_ours_quad_g2_20_drone.reshape(mission_time_ours_quad_g2_20_drone.size, 1)

mission_time_ours_axis_g1_30_drone = mission_time_ours_axis_g1_30_drone.reshape(mission_time_ours_axis_g1_30_drone.size, 1)
mission_time_ours_quad_g1_30_drone = mission_time_ours_quad_g1_30_drone.reshape(mission_time_ours_quad_g1_30_drone.size, 1)
mission_time_ours_axis_g2_30_drone = mission_time_ours_axis_g2_30_drone.reshape(mission_time_ours_axis_g2_30_drone.size, 1)
mission_time_ours_quad_g2_30_drone = mission_time_ours_quad_g2_30_drone.reshape(mission_time_ours_quad_g2_30_drone.size, 1)

mission_time_ours_axis_g1_40_drone = mission_time_ours_axis_g1_40_drone.reshape(mission_time_ours_axis_g1_40_drone.size, 1)
mission_time_ours_quad_g1_40_drone = mission_time_ours_quad_g1_40_drone.reshape(mission_time_ours_quad_g1_40_drone.size, 1)
mission_time_ours_axis_g2_40_drone = mission_time_ours_axis_g2_40_drone.reshape(mission_time_ours_axis_g2_40_drone.size, 1)
mission_time_ours_quad_g2_40_drone = mission_time_ours_quad_g2_40_drone.reshape(mission_time_ours_quad_g2_40_drone.size, 1)

mission_time_ours_axis_g1_50_drone = mission_time_ours_axis_g1_50_drone.reshape(mission_time_ours_axis_g1_50_drone.size, 1)
mission_time_ours_quad_g1_50_drone = mission_time_ours_quad_g1_50_drone.reshape(mission_time_ours_quad_g1_50_drone.size, 1)
mission_time_ours_axis_g2_50_drone = mission_time_ours_axis_g2_50_drone.reshape(mission_time_ours_axis_g2_50_drone.size, 1)
mission_time_ours_quad_g2_50_drone = mission_time_ours_quad_g2_50_drone.reshape(mission_time_ours_quad_g2_50_drone.size, 1)
#####

mission_time_ours_axis_10_drone = mission_time_ours_axis_10_drone.reshape(mission_time_ours_axis_10_drone.size, 1)
mission_time_ours_quad_10_drone = mission_time_ours_quad_10_drone.reshape(mission_time_ours_quad_10_drone.size, 1)
mission_time_scp_ca_10_drone = mission_time_scp_ca_10_drone.reshape(mission_time_scp_ca_10_drone.size, 1)
mission_time_scp_on_10_drone = mission_time_scp_on_10_drone.reshape(mission_time_scp_on_10_drone.size, 1)

mission_time_ours_axis_20_drone = mission_time_ours_axis_20_drone.reshape(mission_time_ours_axis_20_drone.size, 1)
mission_time_ours_quad_20_drone = mission_time_ours_quad_20_drone.reshape(mission_time_ours_quad_20_drone.size, 1)
mission_time_scp_ca_20_drone = mission_time_scp_ca_20_drone.reshape(mission_time_scp_ca_20_drone.size, 1)
mission_time_scp_on_20_drone = mission_time_scp_on_20_drone.reshape(mission_time_scp_on_20_drone.size, 1)

mission_time_ours_axis_30_drone = mission_time_ours_axis_30_drone.reshape(mission_time_ours_axis_30_drone.size, 1)
mission_time_ours_quad_30_drone = mission_time_ours_quad_30_drone.reshape(mission_time_ours_quad_30_drone.size, 1)
mission_time_scp_ca_30_drone = mission_time_scp_ca_30_drone.reshape(mission_time_scp_ca_30_drone.size, 1)
mission_time_scp_on_30_drone = mission_time_scp_on_30_drone.reshape(mission_time_scp_on_30_drone.size, 1)

mission_time_ours_axis_40_drone = mission_time_ours_axis_40_drone.reshape(mission_time_ours_axis_40_drone.size, 1)
mission_time_ours_quad_40_drone = mission_time_ours_quad_40_drone.reshape(mission_time_ours_quad_40_drone.size, 1)
mission_time_scp_ca_40_drone = mission_time_scp_ca_40_drone.reshape(mission_time_scp_ca_40_drone.size, 1)
mission_time_scp_on_40_drone = mission_time_scp_on_40_drone.reshape(mission_time_scp_on_40_drone.size, 1)

mission_time_ours_axis_50_drone = mission_time_ours_axis_50_drone.reshape(mission_time_ours_axis_50_drone.size, 1)
mission_time_ours_quad_50_drone = mission_time_ours_quad_50_drone.reshape(mission_time_ours_quad_50_drone.size, 1)
mission_time_scp_ca_50_drone = mission_time_scp_ca_50_drone.reshape(mission_time_scp_ca_50_drone.size, 1)
mission_time_scp_on_50_drone = mission_time_scp_on_50_drone.reshape(mission_time_scp_on_50_drone.size, 1)

### COMPUTE
compute_time_ours_axis_g1_10_drone = compute_time_ours_axis_g1_10_drone.reshape(compute_time_ours_axis_g1_10_drone.size, 1)
compute_time_ours_quad_g1_10_drone = compute_time_ours_quad_g1_10_drone.reshape(compute_time_ours_quad_g1_10_drone.size, 1)
compute_time_ours_axis_g2_10_drone = compute_time_ours_axis_g2_10_drone.reshape(compute_time_ours_axis_g2_10_drone.size, 1)
compute_time_ours_quad_g2_10_drone = compute_time_ours_quad_g2_10_drone.reshape(compute_time_ours_quad_g2_10_drone.size, 1)

compute_time_ours_axis_g1_20_drone = compute_time_ours_axis_g1_20_drone.reshape(compute_time_ours_axis_g1_20_drone.size, 1)
compute_time_ours_quad_g1_20_drone = compute_time_ours_quad_g1_20_drone.reshape(compute_time_ours_quad_g1_20_drone.size, 1)
compute_time_ours_axis_g2_20_drone = compute_time_ours_axis_g2_20_drone.reshape(compute_time_ours_axis_g2_20_drone.size, 1)
compute_time_ours_quad_g2_20_drone = compute_time_ours_quad_g2_20_drone.reshape(compute_time_ours_quad_g2_20_drone.size, 1)

compute_time_ours_axis_g1_30_drone = compute_time_ours_axis_g1_30_drone.reshape(compute_time_ours_axis_g1_30_drone.size, 1)
compute_time_ours_quad_g1_30_drone = compute_time_ours_quad_g1_30_drone.reshape(compute_time_ours_quad_g1_30_drone.size, 1)
compute_time_ours_axis_g2_30_drone = compute_time_ours_axis_g2_30_drone.reshape(compute_time_ours_axis_g2_30_drone.size, 1)
compute_time_ours_quad_g2_30_drone = compute_time_ours_quad_g2_30_drone.reshape(compute_time_ours_quad_g2_30_drone.size, 1)

compute_time_ours_axis_g1_40_drone = compute_time_ours_axis_g1_40_drone.reshape(compute_time_ours_axis_g1_40_drone.size, 1)
compute_time_ours_quad_g1_40_drone = compute_time_ours_quad_g1_40_drone.reshape(compute_time_ours_quad_g1_40_drone.size, 1)
compute_time_ours_axis_g2_40_drone = compute_time_ours_axis_g2_40_drone.reshape(compute_time_ours_axis_g2_40_drone.size, 1)
compute_time_ours_quad_g2_40_drone = compute_time_ours_quad_g2_40_drone.reshape(compute_time_ours_quad_g2_40_drone.size, 1)

compute_time_ours_axis_g1_50_drone = compute_time_ours_axis_g1_50_drone.reshape(compute_time_ours_axis_g1_50_drone.size, 1)
compute_time_ours_quad_g1_50_drone = compute_time_ours_quad_g1_50_drone.reshape(compute_time_ours_quad_g1_50_drone.size, 1)
compute_time_ours_axis_g2_50_drone = compute_time_ours_axis_g2_50_drone.reshape(compute_time_ours_axis_g2_50_drone.size, 1)
compute_time_ours_quad_g2_50_drone = compute_time_ours_quad_g2_50_drone.reshape(compute_time_ours_quad_g2_50_drone.size, 1)
####

compute_time_ours_axis_10_drone = compute_time_ours_axis_10_drone.reshape(compute_time_ours_axis_10_drone.size, 1)
compute_time_ours_quad_10_drone = compute_time_ours_quad_10_drone.reshape(compute_time_ours_quad_10_drone.size, 1)
compute_time_scp_ca_10_drone = compute_time_scp_ca_10_drone.reshape(compute_time_scp_ca_10_drone.size, 1)
compute_time_scp_on_10_drone = compute_time_scp_on_10_drone.reshape(compute_time_scp_on_10_drone.size, 1)

compute_time_ours_axis_20_drone = compute_time_ours_axis_20_drone.reshape(compute_time_ours_axis_20_drone.size, 1)
compute_time_ours_quad_20_drone = compute_time_ours_quad_20_drone.reshape(compute_time_ours_quad_20_drone.size, 1)
compute_time_scp_ca_20_drone = compute_time_scp_ca_20_drone.reshape(compute_time_scp_ca_20_drone.size, 1)
compute_time_scp_on_20_drone = compute_time_scp_on_20_drone.reshape(compute_time_scp_on_20_drone.size, 1)

compute_time_ours_axis_30_drone = compute_time_ours_axis_30_drone.reshape(compute_time_ours_axis_30_drone.size, 1)
compute_time_ours_quad_30_drone = compute_time_ours_quad_30_drone.reshape(compute_time_ours_quad_30_drone.size, 1)
compute_time_scp_ca_30_drone = compute_time_scp_ca_30_drone.reshape(compute_time_scp_ca_30_drone.size, 1)
compute_time_scp_on_30_drone = compute_time_scp_on_30_drone.reshape(compute_time_scp_on_30_drone.size, 1)

compute_time_ours_axis_40_drone = compute_time_ours_axis_40_drone.reshape(compute_time_ours_axis_40_drone.size, 1)
compute_time_ours_quad_40_drone = compute_time_ours_quad_40_drone.reshape(compute_time_ours_quad_40_drone.size, 1)
compute_time_scp_ca_40_drone = compute_time_scp_ca_40_drone.reshape(compute_time_scp_ca_40_drone.size, 1)
compute_time_scp_on_40_drone = compute_time_scp_on_40_drone.reshape(compute_time_scp_on_40_drone.size, 1)

compute_time_ours_axis_50_drone = compute_time_ours_axis_50_drone.reshape(compute_time_ours_axis_50_drone.size, 1)
compute_time_ours_quad_50_drone = compute_time_ours_quad_50_drone.reshape(compute_time_ours_quad_50_drone.size, 1)
compute_time_scp_ca_50_drone = compute_time_scp_ca_50_drone.reshape(compute_time_scp_ca_50_drone.size, 1)
compute_time_scp_on_50_drone = compute_time_scp_on_50_drone.reshape(compute_time_scp_on_50_drone.size, 1)

####
inter_agent_ours_axis_g1_10_drone = inter_agent_ours_axis_g1_10_drone.reshape(inter_agent_ours_axis_g1_10_drone.size, 1)
inter_agent_ours_quad_g1_10_drone = inter_agent_ours_quad_g1_10_drone.reshape(inter_agent_ours_quad_g1_10_drone.size, 1)
inter_agent_ours_axis_g2_10_drone = inter_agent_ours_axis_g2_10_drone.reshape(inter_agent_ours_axis_g2_10_drone.size, 1)
inter_agent_ours_quad_g2_10_drone = inter_agent_ours_quad_g2_10_drone.reshape(inter_agent_ours_quad_g2_10_drone.size, 1)

inter_agent_ours_axis_g1_20_drone = inter_agent_ours_axis_g1_20_drone.reshape(inter_agent_ours_axis_g1_20_drone.size, 1)
inter_agent_ours_quad_g1_20_drone = inter_agent_ours_quad_g1_20_drone.reshape(inter_agent_ours_quad_g1_20_drone.size, 1)
inter_agent_ours_axis_g2_20_drone = inter_agent_ours_axis_g2_20_drone.reshape(inter_agent_ours_axis_g2_20_drone.size, 1)
inter_agent_ours_quad_g2_20_drone = inter_agent_ours_quad_g2_20_drone.reshape(inter_agent_ours_quad_g2_20_drone.size, 1)

inter_agent_ours_axis_g1_30_drone = inter_agent_ours_axis_g1_30_drone.reshape(inter_agent_ours_axis_g1_30_drone.size, 1)
inter_agent_ours_quad_g1_30_drone = inter_agent_ours_quad_g1_30_drone.reshape(inter_agent_ours_quad_g1_30_drone.size, 1)
inter_agent_ours_axis_g2_30_drone = inter_agent_ours_axis_g2_30_drone.reshape(inter_agent_ours_axis_g2_30_drone.size, 1)
inter_agent_ours_quad_g2_30_drone = inter_agent_ours_quad_g2_30_drone.reshape(inter_agent_ours_quad_g2_30_drone.size, 1)

inter_agent_ours_axis_g1_40_drone = inter_agent_ours_axis_g1_40_drone.reshape(inter_agent_ours_axis_g1_40_drone.size, 1)
inter_agent_ours_quad_g1_40_drone = inter_agent_ours_quad_g1_40_drone.reshape(inter_agent_ours_quad_g1_40_drone.size, 1)
inter_agent_ours_axis_g2_40_drone = inter_agent_ours_axis_g2_40_drone.reshape(inter_agent_ours_axis_g2_40_drone.size, 1)
inter_agent_ours_quad_g2_40_drone = inter_agent_ours_quad_g2_40_drone.reshape(inter_agent_ours_quad_g2_40_drone.size, 1)

inter_agent_ours_axis_g1_50_drone = inter_agent_ours_axis_g1_50_drone.reshape(inter_agent_ours_axis_g1_50_drone.size, 1)
inter_agent_ours_quad_g1_50_drone = inter_agent_ours_quad_g1_50_drone.reshape(inter_agent_ours_quad_g1_50_drone.size, 1)
inter_agent_ours_axis_g2_50_drone = inter_agent_ours_axis_g2_50_drone.reshape(inter_agent_ours_axis_g2_50_drone.size, 1)
inter_agent_ours_quad_g2_50_drone = inter_agent_ours_quad_g2_50_drone.reshape(inter_agent_ours_quad_g2_50_drone.size, 1)
####

inter_agent_ours_axis_10_drone = inter_agent_ours_axis_10_drone.reshape(inter_agent_ours_axis_10_drone.size, 1)
inter_agent_ours_quad_10_drone = inter_agent_ours_quad_10_drone.reshape(inter_agent_ours_quad_10_drone.size, 1)
inter_agent_scp_ca_10_drone = inter_agent_scp_ca_10_drone.reshape(inter_agent_scp_ca_10_drone.size, 1)
inter_agent_scp_on_10_drone = inter_agent_scp_on_10_drone.reshape(inter_agent_scp_on_10_drone.size, 1)

inter_agent_ours_axis_20_drone = inter_agent_ours_axis_20_drone.reshape(inter_agent_ours_axis_20_drone.size, 1)
inter_agent_ours_quad_20_drone = inter_agent_ours_quad_20_drone.reshape(inter_agent_ours_quad_20_drone.size, 1)
inter_agent_scp_ca_20_drone = inter_agent_scp_ca_20_drone.reshape(inter_agent_scp_ca_20_drone.size, 1)
inter_agent_scp_on_20_drone = inter_agent_scp_on_20_drone.reshape(inter_agent_scp_on_20_drone.size, 1)

inter_agent_ours_axis_30_drone = inter_agent_ours_axis_30_drone.reshape(inter_agent_ours_axis_30_drone.size, 1)
inter_agent_ours_quad_30_drone = inter_agent_ours_quad_30_drone.reshape(inter_agent_ours_quad_30_drone.size, 1)
inter_agent_scp_ca_30_drone = inter_agent_scp_ca_30_drone.reshape(inter_agent_scp_ca_30_drone.size, 1)
inter_agent_scp_on_30_drone = inter_agent_scp_on_30_drone.reshape(inter_agent_scp_on_30_drone.size, 1)

inter_agent_ours_axis_40_drone = inter_agent_ours_axis_40_drone.reshape(inter_agent_ours_axis_40_drone.size, 1)
inter_agent_ours_quad_40_drone = inter_agent_ours_quad_40_drone.reshape(inter_agent_ours_quad_40_drone.size, 1)
inter_agent_scp_ca_40_drone = inter_agent_scp_ca_40_drone.reshape(inter_agent_scp_ca_40_drone.size, 1)
inter_agent_scp_on_40_drone = inter_agent_scp_on_40_drone.reshape(inter_agent_scp_on_40_drone.size, 1)

inter_agent_ours_axis_50_drone = inter_agent_ours_axis_50_drone.reshape(inter_agent_ours_axis_50_drone.size, 1)
inter_agent_ours_quad_50_drone = inter_agent_ours_quad_50_drone.reshape(inter_agent_ours_quad_50_drone.size, 1)
inter_agent_scp_ca_50_drone = inter_agent_scp_ca_50_drone.reshape(inter_agent_scp_ca_50_drone.size, 1)
inter_agent_scp_on_50_drone = inter_agent_scp_on_50_drone.reshape(inter_agent_scp_on_50_drone.size, 1)

####
obs_agent_ours_axis_g1_10_drone = obs_agent_ours_axis_g1_10_drone.reshape(obs_agent_ours_axis_g1_10_drone.size, 1)
obs_agent_ours_quad_g1_10_drone = obs_agent_ours_quad_g1_10_drone.reshape(obs_agent_ours_quad_g1_10_drone.size, 1)
obs_agent_ours_axis_g2_10_drone = obs_agent_ours_axis_g2_10_drone.reshape(obs_agent_ours_axis_g2_10_drone.size, 1)
obs_agent_ours_quad_g2_10_drone = obs_agent_ours_quad_g2_10_drone.reshape(obs_agent_ours_quad_g2_10_drone.size, 1)

obs_agent_ours_axis_g1_20_drone = obs_agent_ours_axis_g1_20_drone.reshape(obs_agent_ours_axis_g1_20_drone.size, 1)
obs_agent_ours_quad_g1_20_drone = obs_agent_ours_quad_g1_20_drone.reshape(obs_agent_ours_quad_g1_20_drone.size, 1)
obs_agent_ours_axis_g2_20_drone = obs_agent_ours_axis_g2_20_drone.reshape(obs_agent_ours_axis_g2_20_drone.size, 1)
obs_agent_ours_quad_g2_20_drone = obs_agent_ours_quad_g2_20_drone.reshape(obs_agent_ours_quad_g2_20_drone.size, 1)

obs_agent_ours_axis_g1_30_drone = obs_agent_ours_axis_g1_30_drone.reshape(obs_agent_ours_axis_g1_30_drone.size, 1)
obs_agent_ours_quad_g1_30_drone = obs_agent_ours_quad_g1_30_drone.reshape(obs_agent_ours_quad_g1_30_drone.size, 1)
obs_agent_ours_axis_g2_30_drone = obs_agent_ours_axis_g2_30_drone.reshape(obs_agent_ours_axis_g2_30_drone.size, 1)
obs_agent_ours_quad_g2_30_drone = obs_agent_ours_quad_g2_30_drone.reshape(obs_agent_ours_quad_g2_30_drone.size, 1)

obs_agent_ours_axis_g1_40_drone = obs_agent_ours_axis_g1_40_drone.reshape(obs_agent_ours_axis_g1_40_drone.size, 1)
obs_agent_ours_quad_g1_40_drone = obs_agent_ours_quad_g1_40_drone.reshape(obs_agent_ours_quad_g1_40_drone.size, 1)
obs_agent_ours_axis_g2_40_drone = obs_agent_ours_axis_g2_40_drone.reshape(obs_agent_ours_axis_g2_40_drone.size, 1)
obs_agent_ours_quad_g2_40_drone = obs_agent_ours_quad_g2_40_drone.reshape(obs_agent_ours_quad_g2_40_drone.size, 1)

obs_agent_ours_axis_g1_50_drone = obs_agent_ours_axis_g1_50_drone.reshape(obs_agent_ours_axis_g1_50_drone.size, 1)
obs_agent_ours_quad_g1_50_drone = obs_agent_ours_quad_g1_50_drone.reshape(obs_agent_ours_quad_g1_50_drone.size, 1)
obs_agent_ours_axis_g2_50_drone = obs_agent_ours_axis_g2_50_drone.reshape(obs_agent_ours_axis_g2_50_drone.size, 1)
obs_agent_ours_quad_g2_50_drone = obs_agent_ours_quad_g2_50_drone.reshape(obs_agent_ours_quad_g2_50_drone.size, 1)
####

obs_agent_ours_axis_10_drone = obs_agent_ours_axis_10_drone.reshape(obs_agent_ours_axis_10_drone.size, 1)
obs_agent_ours_quad_10_drone = obs_agent_ours_quad_10_drone.reshape(obs_agent_ours_quad_10_drone.size, 1)
obs_agent_scp_ca_10_drone = obs_agent_scp_ca_10_drone.reshape(obs_agent_scp_ca_10_drone.size, 1)
obs_agent_scp_on_10_drone = obs_agent_scp_on_10_drone.reshape(obs_agent_scp_on_10_drone.size, 1)

obs_agent_ours_axis_20_drone = obs_agent_ours_axis_20_drone.reshape(obs_agent_ours_axis_20_drone.size, 1)
obs_agent_ours_quad_20_drone = obs_agent_ours_quad_20_drone.reshape(obs_agent_ours_quad_20_drone.size, 1)
obs_agent_scp_ca_20_drone = obs_agent_scp_ca_20_drone.reshape(obs_agent_scp_ca_20_drone.size, 1)
obs_agent_scp_on_20_drone = obs_agent_scp_on_20_drone.reshape(obs_agent_scp_on_20_drone.size, 1)

obs_agent_ours_axis_30_drone = obs_agent_ours_axis_30_drone.reshape(obs_agent_ours_axis_30_drone.size, 1)
obs_agent_ours_quad_30_drone = obs_agent_ours_quad_30_drone.reshape(obs_agent_ours_quad_30_drone.size, 1)
obs_agent_scp_ca_30_drone = obs_agent_scp_ca_30_drone.reshape(obs_agent_scp_ca_30_drone.size, 1)
obs_agent_scp_on_30_drone = obs_agent_scp_on_30_drone.reshape(obs_agent_scp_on_30_drone.size, 1)

obs_agent_ours_axis_40_drone = obs_agent_ours_axis_40_drone.reshape(obs_agent_ours_axis_40_drone.size, 1)
obs_agent_ours_quad_40_drone = obs_agent_ours_quad_40_drone.reshape(obs_agent_ours_quad_40_drone.size, 1)
obs_agent_scp_ca_40_drone = obs_agent_scp_ca_40_drone.reshape(obs_agent_scp_ca_40_drone.size, 1)
obs_agent_scp_on_40_drone = obs_agent_scp_on_40_drone.reshape(obs_agent_scp_on_40_drone.size, 1)

obs_agent_ours_axis_50_drone = obs_agent_ours_axis_50_drone.reshape(obs_agent_ours_axis_50_drone.size, 1)
obs_agent_ours_quad_50_drone = obs_agent_ours_quad_50_drone.reshape(obs_agent_ours_quad_50_drone.size, 1)
obs_agent_scp_ca_50_drone = obs_agent_scp_ca_50_drone.reshape(obs_agent_scp_ca_50_drone.size, 1)
obs_agent_scp_on_50_drone = obs_agent_scp_on_50_drone.reshape(obs_agent_scp_on_50_drone.size, 1)



plt.figure(1, figsize=(4,4))
ax = plt.gca()
ax.xaxis.set_major_locator(MaxNLocator(integer=True))

plt.plot(agents, success_ours_axis_16*(100.0/total_config),  label=r'Ours(Axiswise) $\gamma = 1$', linewidth=1.5, markersize=7.5, linestyle='--', marker='o')
plt.plot(agents, success_ours_quad_16*(100.0/total_config),  label=r'Ours(Quadratic) $\gamma = 1$', linewidth=1.5, markersize=7.5, linestyle='--', marker='o')
plt.plot(agents, success_scp_ca_16*(100.0/total_config),  label=r'SCP(Continuous) $\gamma = 1$', linewidth=1.5, markersize=7.5, linestyle='--', marker='o')
plt.plot(agents, success_scp_on_16*(100.0/total_config),  label=r'SCP(Ondemand) $\gamma = 1$', linewidth=1.5, markersize=7.5, linestyle='--', marker='o')

plt.xticks(agents)
plt.ylabel('Success Rate %')
plt.xlabel('Number Of Agents')
plt.grid()
plt.tight_layout()
plt.savefig('success_1.png',bbox_inches='tight')


plt.figure(2, figsize=(4,4))
ax = plt.gca()
ax.xaxis.set_major_locator(MaxNLocator(integer=True))

# plt.plot(agents, success_ours_axis_16*(100.0/total_config),  label=r'Ours(Axiswise) $\gamma = 1$', linewidth=1.5, markersize=7.5, linestyle='--', marker='o')
plt.plot(agents, success_ours_quad_16*(100.0/total_config),  label=r'Ours(Quadratic) $\gamma = 1$', linewidth=1.5, markersize=7.5, linestyle='--', marker='o', color='purple')
# plt.plot(agents, success_ours_axis_g1_16*(100.0/total_config), label=r'Ours(Quadratic) $\gamma = 0.95$', linewidth=1.5, markersize=7.5, linestyle='--',marker='o', color='blue', alpha=0.65)
plt.plot(agents, success_ours_quad_g1_16*(100.0/total_config), label=r'Ours(Axiswise) $\gamma = 0.95$', linewidth=1.5, markersize=7.5, linestyle='--',marker='o', color='purple', alpha=0.65)
# plt.plot(agents, success_ours_axis_g2_16*(100.0/total_config), label=r'Ours(Quadratic) $\gamma = 0.9$', linewidth=1.5, markersize=7.5, linestyle='--',marker='o', color='blue', alpha=0.4)
plt.plot(agents, success_ours_quad_g2_16*(100.0/total_config), label=r'Ours(Axiswise) $\gamma = 0.9$', linewidth=1.5, markersize=7.5, linestyle='--',marker='o', color='purple', alpha=0.4)

plt.xticks(agents)
plt.ylabel('Success Rate %')
plt.xlabel('Number Of Agents')
plt.grid()
plt.tight_layout()
plt.savefig('success_2.png',bbox_inches='tight')



plt.figure(3, figsize=(4,4))
ax = plt.gca()
ax.xaxis.set_major_locator(MaxNLocator(integer=True))

mean_data_1 = [np.mean(mission_time_ours_axis_10_drone), np.mean(mission_time_ours_axis_20_drone), np.mean(mission_time_ours_axis_30_drone), np.mean(mission_time_ours_axis_40_drone), np.mean(mission_time_ours_axis_50_drone)]
mean_data_2 = [np.mean(mission_time_ours_quad_10_drone), np.mean(mission_time_ours_quad_20_drone), np.mean(mission_time_ours_quad_30_drone), np.mean(mission_time_ours_quad_40_drone), np.mean(mission_time_ours_quad_50_drone)]
mean_data_3 = [np.mean(mission_time_scp_ca_10_drone), np.mean(mission_time_scp_ca_20_drone), np.mean(mission_time_scp_ca_30_drone), np.mean(mission_time_scp_ca_40_drone), np.mean(mission_time_scp_ca_50_drone)]
mean_data_4 = [np.mean(mission_time_scp_on_10_drone), np.mean(mission_time_scp_on_20_drone), np.mean(mission_time_scp_on_30_drone), np.mean(mission_time_scp_on_40_drone), np.mean(mission_time_scp_on_50_drone)]

std_data_1 = [np.std(mission_time_ours_axis_10_drone), np.std(mission_time_ours_axis_20_drone), np.std(mission_time_ours_axis_30_drone), np.std(mission_time_ours_axis_40_drone), np.std(mission_time_ours_axis_50_drone)]
std_data_2 = [np.std(mission_time_ours_quad_10_drone), np.std(mission_time_ours_quad_20_drone), np.std(mission_time_ours_quad_30_drone), np.std(mission_time_ours_quad_40_drone), np.std(mission_time_ours_quad_50_drone)]
std_data_3 = [np.std(mission_time_scp_ca_10_drone), np.std(mission_time_scp_ca_20_drone), np.std(mission_time_scp_ca_30_drone), np.std(mission_time_scp_ca_40_drone), np.std(mission_time_scp_ca_50_drone)]
std_data_4 = [np.std(mission_time_scp_on_10_drone), np.std(mission_time_scp_on_20_drone), np.std(mission_time_scp_on_30_drone), np.std(mission_time_scp_on_40_drone), np.std(mission_time_scp_on_50_drone)]

plt.errorbar(agents, mean_data_1,  label='Ours(Axiswise)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_2,  label='Ours(Quadratic)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_3,  label='SCP(CA)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_4,  label='SCP(OnDemand)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)


plt.xticks(agents)

plt.xlabel("Number of Agents")
plt.ylabel("Mission Time (s)")
plt.grid()
plt.tight_layout()
plt.savefig('mission_time.png', bbox_inches='tight')


plt.figure(4, figsize=(4,4))
ax = plt.gca()
ax.xaxis.set_major_locator(MaxNLocator(integer=True))

mean_data_1 = [np.mean(mission_time_ours_axis_10_drone), np.mean(mission_time_ours_axis_20_drone), np.mean(mission_time_ours_axis_30_drone), np.mean(mission_time_ours_axis_40_drone), np.mean(mission_time_ours_axis_50_drone)]
mean_data_2 = [np.mean(mission_time_ours_quad_10_drone), np.mean(mission_time_ours_quad_20_drone), np.mean(mission_time_ours_quad_30_drone), np.mean(mission_time_ours_quad_40_drone), np.mean(mission_time_ours_quad_50_drone)]
mean_data_3 = [np.mean(mission_time_ours_axis_g1_10_drone), np.mean(mission_time_ours_axis_g1_20_drone), np.mean(mission_time_ours_axis_g1_30_drone), np.mean(mission_time_ours_axis_g1_40_drone), np.mean(mission_time_ours_axis_g1_50_drone)]
mean_data_4 = [np.mean(mission_time_ours_quad_g1_10_drone), np.mean(mission_time_ours_quad_g1_20_drone), np.mean(mission_time_ours_quad_g1_30_drone), np.mean(mission_time_ours_quad_g1_40_drone), np.mean(mission_time_ours_quad_g1_50_drone)]
mean_data_5 = [np.mean(mission_time_ours_axis_g2_10_drone), np.mean(mission_time_ours_axis_g2_20_drone), np.mean(mission_time_ours_axis_g2_30_drone), np.mean(mission_time_ours_axis_g2_40_drone), np.mean(mission_time_ours_axis_g2_50_drone)]
mean_data_6 = [np.mean(mission_time_ours_quad_g2_10_drone), np.mean(mission_time_ours_quad_g2_20_drone), np.mean(mission_time_ours_quad_g2_30_drone), np.mean(mission_time_ours_quad_g2_40_drone), np.mean(mission_time_ours_quad_g2_50_drone)]


std_data_1 = [np.std(mission_time_ours_axis_10_drone), np.std(mission_time_ours_axis_20_drone), np.std(mission_time_ours_axis_30_drone), np.std(mission_time_ours_axis_40_drone), np.std(mission_time_ours_axis_50_drone)]
std_data_2 = [np.std(mission_time_ours_quad_10_drone), np.std(mission_time_ours_quad_20_drone), np.std(mission_time_ours_quad_30_drone), np.std(mission_time_ours_quad_40_drone), np.std(mission_time_ours_quad_50_drone)]
std_data_3 = [np.std(mission_time_ours_axis_g1_10_drone), np.std(mission_time_ours_axis_g1_20_drone), np.std(mission_time_ours_axis_g1_30_drone), np.std(mission_time_ours_axis_g1_40_drone), np.std(mission_time_ours_axis_g1_50_drone)]
std_data_4 = [np.std(mission_time_ours_quad_g1_10_drone), np.std(mission_time_ours_quad_g1_20_drone), np.std(mission_time_ours_quad_g1_30_drone), np.std(mission_time_ours_quad_g1_40_drone), np.std(mission_time_ours_quad_g1_50_drone)]
std_data_5 = [np.std(mission_time_ours_axis_g2_10_drone), np.std(mission_time_ours_axis_g2_20_drone), np.std(mission_time_ours_axis_g2_30_drone), np.std(mission_time_ours_axis_g2_40_drone), np.std(mission_time_ours_axis_g2_50_drone)]
std_data_6 = [np.std(mission_time_ours_quad_g2_10_drone), np.std(mission_time_ours_quad_g2_20_drone), np.std(mission_time_ours_quad_g2_30_drone), np.std(mission_time_ours_quad_g2_40_drone), np.std(mission_time_ours_quad_g2_50_drone)]



# plt.errorbar(agents, mean_data_1,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=2.0)
plt.errorbar(agents, mean_data_2,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=2.0)
# plt.errorbar(agents, mean_data_3,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=0.65)
plt.errorbar(agents, mean_data_4,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=0.65)
# plt.errorbar(agents, mean_data_5,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=0.4)
plt.errorbar(agents, mean_data_6,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=0.4)
print("_____________== Mission time == _____________")
print(np.around(np.array(mean_data_2),2))
print(np.around(np.array(mean_data_4),2))
print(np.around(np.array(mean_data_6),2))

plt.xticks(agents)

plt.xlabel("Number of Agents")
plt.ylabel("Mission Time (s)")
plt.grid()
plt.tight_layout()
plt.savefig('mission_time_2.png', bbox_inches='tight')


plt.figure(5, figsize=(4,4))

mean_data_1 = 1000.0*np.array([np.mean(compute_time_ours_axis_10_drone), np.mean(compute_time_ours_axis_20_drone), np.mean(compute_time_ours_axis_30_drone), np.mean(compute_time_ours_axis_40_drone), np.mean(compute_time_ours_axis_50_drone)])
mean_data_2 = 1000.0*np.array([np.mean(compute_time_ours_quad_10_drone), np.mean(compute_time_ours_quad_20_drone), np.mean(compute_time_ours_quad_30_drone), np.mean(compute_time_ours_quad_40_drone), np.mean(compute_time_ours_quad_50_drone)])
mean_data_3 = 1000.0*np.array([np.mean(compute_time_scp_ca_10_drone), np.mean(compute_time_scp_ca_20_drone), np.mean(compute_time_scp_ca_30_drone), np.mean(compute_time_scp_ca_40_drone), np.mean(compute_time_scp_ca_50_drone)])
mean_data_4 = 1000.0*np.array([np.mean(compute_time_scp_on_10_drone), np.mean(compute_time_scp_on_20_drone), np.mean(compute_time_scp_on_30_drone), np.mean(compute_time_scp_on_40_drone), np.mean(compute_time_scp_on_50_drone)])

std_data_1 = 1000.0*np.array([np.std(compute_time_ours_axis_10_drone), np.std(compute_time_ours_axis_20_drone), np.std(compute_time_ours_axis_30_drone), np.std(compute_time_ours_axis_40_drone), np.std(compute_time_ours_axis_50_drone)])
std_data_2 = 1000.0*np.array([np.std(compute_time_ours_quad_10_drone), np.std(compute_time_ours_quad_20_drone), np.std(compute_time_ours_quad_30_drone), np.std(compute_time_ours_quad_40_drone), np.std(compute_time_ours_quad_50_drone)])
std_data_3 = 1000.0*np.array([np.std(compute_time_scp_ca_10_drone), np.std(compute_time_scp_ca_20_drone), np.std(compute_time_scp_ca_30_drone), np.std(compute_time_scp_ca_40_drone), np.std(compute_time_scp_ca_50_drone)])
std_data_4 = 1000.0*np.array([np.std(compute_time_scp_on_10_drone), np.std(compute_time_scp_on_20_drone), np.std(compute_time_scp_on_30_drone), np.std(compute_time_scp_on_40_drone), np.std(compute_time_scp_on_50_drone)])

# print(std_data_3)
ax = plt.gca()
# ax.set_yscale('log')
plt.errorbar(agents, mean_data_1,  label='Ours(Axiswise)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_2,   label='Ours(Quadratic)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_3,   label='SCP(CA)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_4,   label='SCP(OnDemand)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)

plt.xlabel("Number of Agents")
plt.ylabel("Comp.time per Agent (ms)")
plt.tight_layout()
plt.xticks(agents)
plt.grid()
plt.savefig('average_time.png',bbox_inches='tight')


plt.figure(6, figsize=(4,4))
ax = plt.gca()
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
# ax.set_yscale('log')
mean_data_1 = 1000*np.array([np.mean(compute_time_ours_axis_10_drone), np.mean(compute_time_ours_axis_20_drone), np.mean(compute_time_ours_axis_30_drone), np.mean(compute_time_ours_axis_40_drone), np.mean(compute_time_ours_axis_50_drone)])
mean_data_2 = 1000*np.array([np.mean(compute_time_ours_quad_10_drone), np.mean(compute_time_ours_quad_20_drone), np.mean(compute_time_ours_quad_30_drone), np.mean(compute_time_ours_quad_40_drone), np.mean(compute_time_ours_quad_50_drone)])
mean_data_3 = 1000*np.array([np.mean(compute_time_ours_axis_g1_10_drone), np.mean(compute_time_ours_axis_g1_20_drone), np.mean(compute_time_ours_axis_g1_30_drone), np.mean(compute_time_ours_axis_g1_40_drone), np.mean(compute_time_ours_axis_g1_50_drone)])
mean_data_4 = 1000*np.array([np.mean(compute_time_ours_quad_g1_10_drone), np.mean(compute_time_ours_quad_g1_20_drone), np.mean(compute_time_ours_quad_g1_30_drone), np.mean(compute_time_ours_quad_g1_40_drone), np.mean(compute_time_ours_quad_g1_50_drone)])
mean_data_5 = 1000*np.array([np.mean(compute_time_ours_axis_g2_10_drone), np.mean(compute_time_ours_axis_g2_20_drone), np.mean(compute_time_ours_axis_g2_30_drone), np.mean(compute_time_ours_axis_g2_40_drone), np.mean(compute_time_ours_axis_g2_50_drone)])
mean_data_6 = 1000*np.array([np.mean(compute_time_ours_quad_g2_10_drone), np.mean(compute_time_ours_quad_g2_20_drone), np.mean(compute_time_ours_quad_g2_30_drone), np.mean(compute_time_ours_quad_g2_40_drone), np.mean(compute_time_ours_quad_g2_50_drone)])


std_data_1 = 1000*np.array([np.std(compute_time_ours_axis_10_drone), np.std(compute_time_ours_axis_20_drone), np.std(compute_time_ours_axis_30_drone), np.std(compute_time_ours_axis_40_drone), np.std(compute_time_ours_axis_50_drone)])
std_data_2 = 1000*np.array([np.std(compute_time_ours_quad_10_drone), np.std(compute_time_ours_quad_20_drone), np.std(compute_time_ours_quad_30_drone), np.std(compute_time_ours_quad_40_drone), np.std(compute_time_ours_quad_50_drone)])
std_data_3 = 1000*np.array([np.std(compute_time_ours_axis_g1_10_drone), np.std(compute_time_ours_axis_g1_20_drone), np.std(compute_time_ours_axis_g1_30_drone), np.std(compute_time_ours_axis_g1_40_drone), np.std(compute_time_ours_axis_g1_50_drone)])
std_data_4 = 1000*np.array([np.std(compute_time_ours_quad_g1_10_drone), np.std(compute_time_ours_quad_g1_20_drone), np.std(compute_time_ours_quad_g1_30_drone), np.std(compute_time_ours_quad_g1_40_drone), np.std(compute_time_ours_quad_g1_50_drone)])
std_data_5 = 1000*np.array([np.std(compute_time_ours_axis_g2_10_drone), np.std(compute_time_ours_axis_g2_20_drone), np.std(compute_time_ours_axis_g2_30_drone), np.std(compute_time_ours_axis_g2_40_drone), np.std(compute_time_ours_axis_g2_50_drone)])
std_data_6 = 1000*np.array([np.std(compute_time_ours_quad_g2_10_drone), np.std(compute_time_ours_quad_g2_20_drone), np.std(compute_time_ours_quad_g2_30_drone), np.std(compute_time_ours_quad_g2_40_drone), np.std(compute_time_ours_quad_g2_50_drone)])



# plt.errorbar(agents, mean_data_1,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=2.0)
plt.errorbar(agents, mean_data_2,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=2.0)
# plt.errorbar(agents, mean_data_3,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=0.65)
plt.errorbar(agents, mean_data_4,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=0.65)
# plt.errorbar(agents, mean_data_5,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=0.4)
plt.errorbar(agents, mean_data_6,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=0.4)


plt.xticks(agents)

plt.xlabel("Number of Agents")
plt.ylabel("Comp.time per Agent (ms)")
plt.grid()
plt.tight_layout()
plt.savefig('compute_time_2.png', bbox_inches='tight')
print("_____________== Comp.time per agent == _____________")
print(np.around(np.array(mean_data_2),2))
print(np.around(np.array(mean_data_4),2))
print(np.around(np.array(mean_data_6),2))

plt.figure(7, figsize=(4,4))

mean_data_1 = np.array([np.mean(inter_agent_ours_axis_10_drone), np.mean(inter_agent_ours_axis_20_drone), np.mean(inter_agent_ours_axis_30_drone), np.mean(inter_agent_ours_axis_40_drone), np.mean(inter_agent_ours_axis_50_drone)])
mean_data_2 = np.array([np.mean(inter_agent_ours_quad_10_drone), np.mean(inter_agent_ours_quad_20_drone), np.mean(inter_agent_ours_quad_30_drone), np.mean(inter_agent_ours_quad_40_drone), np.mean(inter_agent_ours_quad_50_drone)])
mean_data_3 = np.array([np.mean(inter_agent_scp_ca_10_drone), np.mean(inter_agent_scp_ca_20_drone), np.mean(inter_agent_scp_ca_30_drone), np.mean(inter_agent_scp_ca_40_drone), np.mean(inter_agent_scp_ca_50_drone)])
mean_data_4 = np.array([np.mean(inter_agent_scp_on_10_drone), np.mean(inter_agent_scp_on_20_drone), np.mean(inter_agent_scp_on_30_drone), np.mean(inter_agent_scp_on_40_drone), np.mean(inter_agent_scp_on_50_drone)])

std_data_1 = np.array([np.std(inter_agent_ours_axis_10_drone), np.std(inter_agent_ours_axis_20_drone), np.std(inter_agent_ours_axis_30_drone), np.std(inter_agent_ours_axis_40_drone), np.std(inter_agent_ours_axis_50_drone)])
std_data_2 = np.array([np.std(inter_agent_ours_quad_10_drone), np.std(inter_agent_ours_quad_20_drone), np.std(inter_agent_ours_quad_30_drone), np.std(inter_agent_ours_quad_40_drone), np.std(inter_agent_ours_quad_50_drone)])
std_data_3 = np.array([np.std(inter_agent_scp_ca_10_drone), np.std(inter_agent_scp_ca_20_drone), np.std(inter_agent_scp_ca_30_drone), np.std(inter_agent_scp_ca_40_drone), np.std(inter_agent_scp_ca_50_drone)])
std_data_4 = np.array([np.std(inter_agent_scp_on_10_drone), np.std(inter_agent_scp_on_20_drone), np.std(inter_agent_scp_on_30_drone), np.std(inter_agent_scp_on_40_drone), np.std(inter_agent_scp_on_50_drone)])

plt.errorbar(agents, mean_data_1,   label='Ours(Axiswise)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_2,   label='Ours(Quadratic)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_3,   label='SCP(CA)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_4,   label='SCP(OnDemand)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)

plt.xticks(agents)
plt.xlabel("Number of Agents")
plt.ylabel("Inter-Agent Distance(m)")
plt.grid()
plt.tight_layout()
plt.savefig('inter_agent.png', bbox_inches='tight')


plt.figure(8, figsize=(4,4))
ax = plt.gca()
ax.xaxis.set_major_locator(MaxNLocator(integer=True))

mean_data_1 = [np.mean(inter_agent_ours_axis_10_drone), np.mean(inter_agent_ours_axis_20_drone), np.mean(inter_agent_ours_axis_30_drone), np.mean(inter_agent_ours_axis_40_drone), np.mean(inter_agent_ours_axis_50_drone)]
mean_data_2 = [np.mean(inter_agent_ours_quad_10_drone), np.mean(inter_agent_ours_quad_20_drone), np.mean(inter_agent_ours_quad_30_drone), np.mean(inter_agent_ours_quad_40_drone), np.mean(inter_agent_ours_quad_50_drone)]
mean_data_3 = [np.mean(inter_agent_ours_axis_g1_10_drone), np.mean(inter_agent_ours_axis_g1_20_drone), np.mean(inter_agent_ours_axis_g1_30_drone), np.mean(inter_agent_ours_axis_g1_40_drone), np.mean(inter_agent_ours_axis_g1_50_drone)]
mean_data_4 = [np.mean(inter_agent_ours_quad_g1_10_drone), np.mean(inter_agent_ours_quad_g1_20_drone), np.mean(inter_agent_ours_quad_g1_30_drone), np.mean(inter_agent_ours_quad_g1_40_drone), np.mean(inter_agent_ours_quad_g1_50_drone)]
mean_data_5 = [np.mean(inter_agent_ours_axis_g2_10_drone), np.mean(inter_agent_ours_axis_g2_20_drone), np.mean(inter_agent_ours_axis_g2_30_drone), np.mean(inter_agent_ours_axis_g2_40_drone), np.mean(inter_agent_ours_axis_g2_50_drone)]
mean_data_6 = [np.mean(inter_agent_ours_quad_g2_10_drone), np.mean(inter_agent_ours_quad_g2_20_drone), np.mean(inter_agent_ours_quad_g2_30_drone), np.mean(inter_agent_ours_quad_g2_40_drone), np.mean(inter_agent_ours_quad_g2_50_drone)]


std_data_1 = [np.std(inter_agent_ours_axis_10_drone), np.std(inter_agent_ours_axis_20_drone), np.std(inter_agent_ours_axis_30_drone), np.std(inter_agent_ours_axis_40_drone), np.std(inter_agent_ours_axis_50_drone)]
std_data_2 = [np.std(inter_agent_ours_quad_10_drone), np.std(inter_agent_ours_quad_20_drone), np.std(inter_agent_ours_quad_30_drone), np.std(inter_agent_ours_quad_40_drone), np.std(inter_agent_ours_quad_50_drone)]
std_data_3 = [np.std(inter_agent_ours_axis_g1_10_drone), np.std(inter_agent_ours_axis_g1_20_drone), np.std(inter_agent_ours_axis_g1_30_drone), np.std(inter_agent_ours_axis_g1_40_drone), np.std(inter_agent_ours_axis_g1_50_drone)]
std_data_4 = [np.std(inter_agent_ours_quad_g1_10_drone), np.std(inter_agent_ours_quad_g1_20_drone), np.std(inter_agent_ours_quad_g1_30_drone), np.std(inter_agent_ours_quad_g1_40_drone), np.std(inter_agent_ours_quad_g1_50_drone)]
std_data_5 = [np.std(inter_agent_ours_axis_g2_10_drone), np.std(inter_agent_ours_axis_g2_20_drone), np.std(inter_agent_ours_axis_g2_30_drone), np.std(inter_agent_ours_axis_g2_40_drone), np.std(inter_agent_ours_axis_g2_50_drone)]
std_data_6 = [np.std(inter_agent_ours_quad_g2_10_drone), np.std(inter_agent_ours_quad_g2_20_drone), np.std(inter_agent_ours_quad_g2_30_drone), np.std(inter_agent_ours_quad_g2_40_drone), np.std(inter_agent_ours_quad_g2_50_drone)]



# plt.errorbar(agents, mean_data_1,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=2.0)
plt.errorbar(agents, mean_data_2,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=2.0)
# plt.errorbar(agents, mean_data_3,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=0.65)
plt.errorbar(agents, mean_data_4,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=0.65)
# plt.errorbar(agents, mean_data_5,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=0.4)
plt.errorbar(agents, mean_data_6,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=0.4)


plt.xticks(agents)

plt.xlabel("Number of Agents")
plt.ylabel("Inter-Agent Distance(m)")
plt.grid()
plt.tight_layout()
plt.savefig('inter_agent_2.png', bbox_inches='tight')
print("_____________== inter-agent distances == _____________")
print(np.around(np.array(mean_data_2),2))
print(np.around((np.array(mean_data_4)-np.array(mean_data_2))/np.array(mean_data_2)*100,2))
print(np.around((np.array(mean_data_6)-np.array(mean_data_2))/np.array(mean_data_4)*100,2))

plt.figure(9, figsize=(4,4))

mean_data_1 = [np.mean(obs_agent_ours_axis_10_drone), np.mean(obs_agent_ours_axis_20_drone), np.mean(obs_agent_ours_axis_30_drone), np.mean(obs_agent_ours_axis_40_drone), np.mean(obs_agent_ours_axis_50_drone)]
mean_data_2 = [np.mean(obs_agent_ours_quad_10_drone), np.mean(obs_agent_ours_quad_20_drone), np.mean(obs_agent_ours_quad_30_drone), np.mean(obs_agent_ours_quad_40_drone), np.mean(obs_agent_ours_quad_50_drone)]
mean_data_3 = [np.mean(obs_agent_scp_ca_10_drone), np.mean(obs_agent_scp_ca_20_drone), np.mean(obs_agent_scp_ca_30_drone), np.mean(obs_agent_scp_ca_40_drone), np.mean(obs_agent_scp_ca_50_drone)]
mean_data_4 = [np.mean(obs_agent_scp_on_10_drone), np.mean(obs_agent_scp_on_20_drone), np.mean(obs_agent_scp_on_30_drone), np.mean(obs_agent_scp_on_40_drone), np.mean(obs_agent_scp_on_50_drone)]

std_data_1 = [np.std(obs_agent_ours_axis_10_drone), np.std(obs_agent_ours_axis_20_drone), np.std(obs_agent_ours_axis_30_drone), np.std(obs_agent_ours_axis_40_drone), np.std(obs_agent_ours_axis_50_drone)]
std_data_2 = [np.std(obs_agent_ours_quad_10_drone), np.std(obs_agent_ours_quad_20_drone), np.std(obs_agent_ours_quad_30_drone), np.std(obs_agent_ours_quad_40_drone), np.std(obs_agent_ours_quad_50_drone)]
std_data_3 = [np.std(obs_agent_scp_ca_10_drone), np.std(obs_agent_scp_ca_20_drone), np.std(obs_agent_scp_ca_30_drone), np.std(obs_agent_scp_ca_40_drone), np.std(obs_agent_scp_ca_50_drone)]
std_data_4 = [np.std(obs_agent_scp_on_10_drone), np.std(obs_agent_scp_on_20_drone), np.std(obs_agent_scp_on_30_drone), np.std(obs_agent_scp_on_40_drone), np.std(obs_agent_scp_on_50_drone)]

plt.errorbar(agents, mean_data_1,   label='Ours(Axiswise)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_2,   label='Ours(Quadratic)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_3,   label='SCP(CA)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.errorbar(agents, mean_data_4,   label='SCP(OnDemand)', linewidth=1.5, markersize=7.5, fmt='--o', capsize=5)
plt.xticks(agents)

plt.xlabel("Number of Agents")
plt.ylabel("Distance-to-Obstacles(m)")
plt.grid()
plt.tight_layout()
plt.savefig('obs_agent.png', bbox_inches='tight')


plt.figure(10, figsize=(4,4))
ax = plt.gca()
ax.xaxis.set_major_locator(MaxNLocator(integer=True))

mean_data_1 = [np.mean(obs_agent_ours_axis_10_drone), np.mean(obs_agent_ours_axis_20_drone), np.mean(obs_agent_ours_axis_30_drone), np.mean(obs_agent_ours_axis_40_drone), np.mean(obs_agent_ours_axis_50_drone)]
mean_data_2 = [np.mean(obs_agent_ours_quad_10_drone), np.mean(obs_agent_ours_quad_20_drone), np.mean(obs_agent_ours_quad_30_drone), np.mean(obs_agent_ours_quad_40_drone), np.mean(obs_agent_ours_quad_50_drone)]
mean_data_3 = [np.mean(obs_agent_ours_axis_g1_10_drone), np.mean(obs_agent_ours_axis_g1_20_drone), np.mean(obs_agent_ours_axis_g1_30_drone), np.mean(obs_agent_ours_axis_g1_40_drone), np.mean(obs_agent_ours_axis_g1_50_drone)]
mean_data_4 = [np.mean(obs_agent_ours_quad_g1_10_drone), np.mean(obs_agent_ours_quad_g1_20_drone), np.mean(obs_agent_ours_quad_g1_30_drone), np.mean(obs_agent_ours_quad_g1_40_drone), np.mean(obs_agent_ours_quad_g1_50_drone)]
mean_data_5 = [np.mean(obs_agent_ours_axis_g2_10_drone), np.mean(obs_agent_ours_axis_g2_20_drone), np.mean(obs_agent_ours_axis_g2_30_drone), np.mean(obs_agent_ours_axis_g2_40_drone), np.mean(obs_agent_ours_axis_g2_50_drone)]
mean_data_6 = [np.mean(obs_agent_ours_quad_g2_10_drone), np.mean(obs_agent_ours_quad_g2_20_drone), np.mean(obs_agent_ours_quad_g2_30_drone), np.mean(obs_agent_ours_quad_g2_40_drone), np.mean(obs_agent_ours_quad_g2_50_drone)]


std_data_1 = [np.std(obs_agent_ours_axis_10_drone), np.std(obs_agent_ours_axis_20_drone), np.std(obs_agent_ours_axis_30_drone), np.std(obs_agent_ours_axis_40_drone), np.std(obs_agent_ours_axis_50_drone)]
std_data_2 = [np.std(obs_agent_ours_quad_10_drone), np.std(obs_agent_ours_quad_20_drone), np.std(obs_agent_ours_quad_30_drone), np.std(obs_agent_ours_quad_40_drone), np.std(obs_agent_ours_quad_50_drone)]
std_data_3 = [np.std(obs_agent_ours_axis_g1_10_drone), np.std(obs_agent_ours_axis_g1_20_drone), np.std(obs_agent_ours_axis_g1_30_drone), np.std(obs_agent_ours_axis_g1_40_drone), np.std(obs_agent_ours_axis_g1_50_drone)]
std_data_4 = [np.std(obs_agent_ours_quad_g1_10_drone), np.std(obs_agent_ours_quad_g1_20_drone), np.std(obs_agent_ours_quad_g1_30_drone), np.std(obs_agent_ours_quad_g1_40_drone), np.std(obs_agent_ours_quad_g1_50_drone)]
std_data_5 = [np.std(obs_agent_ours_axis_g2_10_drone), np.std(obs_agent_ours_axis_g2_20_drone), np.std(obs_agent_ours_axis_g2_30_drone), np.std(obs_agent_ours_axis_g2_40_drone), np.std(obs_agent_ours_axis_g2_50_drone)]
std_data_6 = [np.std(obs_agent_ours_quad_g2_10_drone), np.std(obs_agent_ours_quad_g2_20_drone), np.std(obs_agent_ours_quad_g2_30_drone), np.std(obs_agent_ours_quad_g2_40_drone), np.std(obs_agent_ours_quad_g2_50_drone)]



# plt.errorbar(agents, mean_data_1,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=2.0)
plt.errorbar(agents, mean_data_2,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=2.0)
# plt.errorbar(agents, mean_data_3,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=0.65)
plt.errorbar(agents, mean_data_4,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=0.65)
# plt.errorbar(agents, mean_data_5,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='blue', alpha=0.4)
plt.errorbar(agents, mean_data_6,   linewidth=1.5, markersize=7.5, fmt='--o', capsize=5, color='purple', alpha=0.4)


plt.xticks(agents)

plt.xlabel("Number of Agents")
plt.ylabel("Distance-to-Obstacles(m)")
plt.grid()
plt.tight_layout()
plt.savefig('obs_agent_2.png', bbox_inches='tight')
print("_____________== distance to obstacles == _____________")
print(np.around(np.array(mean_data_2)), 2)
print(np.around((np.array(mean_data_4)-np.array(mean_data_2))/np.array(mean_data_2)*100,2))
print(np.around((np.array(mean_data_6)-np.array(mean_data_2))/np.array(mean_data_4)*100,2))
plt.show()