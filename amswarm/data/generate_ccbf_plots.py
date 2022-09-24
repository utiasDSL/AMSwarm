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

data_ours = []
data_acado = []

drones = [2, 4, 6, 8]
file_acado = [path+"/data/acado_cbf/sim_results_acado"+str(i)+".txt" for i in drones]
file_ours = [path+"/data/ours_cbf/sim_results_ours"+str(i)+".txt" for i in drones]


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

for i in range(0, len(file_ours)):
    data = np.loadtxt(file_ours[i])
    num_drone = drones[i]

    sim_iter = int(data[0])
    data_ours.append({"compute_time":data[1:sim_iter+2], "smoothness":data[sim_iter+2:sim_iter+2+num_drone], "arc_length":data[sim_iter+2+num_drone:sim_iter+2+2*num_drone], "inter_agent":data[sim_iter+2+2*num_drone:sim_iter+2+2*num_drone+sim_iter+1], "mission_time":data[sim_iter+2+2*num_drone+sim_iter+1]})
    print(data[sim_iter+2+2*num_drone+sim_iter+1])

for i in range(0, len(file_acado)):
    data = np.loadtxt(file_acado[i])
    num_drone = drones[i]

    sim_iter = int(data[0])
    data_acado.append({"compute_time":data[1:sim_iter+1], "smoothness":data[sim_iter+1:sim_iter+1+num_drone], "arc_length":data[sim_iter+1+num_drone:sim_iter+1+2*num_drone], "inter_agent":data[sim_iter+1+2*num_drone:sim_iter+1+2*num_drone+sim_iter], "mission_time":data[sim_iter+1+2*num_drone+sim_iter]})
    print(data[sim_iter+1+2*num_drone+sim_iter])

plt.figure(0, figsize=(4,4))
agents = [2, 4, 6, 8]
plt.grid()
plt.plot(agents, 1000*np.array([np.mean(data_ours[0]["compute_time"]), np.mean(data_ours[1]["compute_time"]), np.mean(data_ours[2]["compute_time"]), np.mean(data_ours[3]["compute_time"])]), linestyle='--', marker='o', linewidth=1.5, markersize=7.5, color='orange')
plt.plot(agents, 1000*np.array([np.mean(data_acado[0]["compute_time"]), np.mean(data_acado[1]["compute_time"]), np.mean(data_acado[2]["compute_time"]), np.mean(data_acado[3]["compute_time"])]), linestyle='--', marker='o', linewidth=1.5, markersize=7.5, color='brown')
plt.xlabel('Number Of Agents')
plt.ylabel('Comp.time per Agent (ms)')
plt.xticks(agents)
plt.tight_layout()
plt.savefig('acado_ours_comp_time.png',bbox_inches='tight')
print("Ours compute = ", 1000*np.array([np.mean(data_ours[0]["compute_time"]), np.mean(data_ours[1]["compute_time"]), np.mean(data_ours[2]["compute_time"]), np.mean(data_ours[3]["compute_time"])]))
print("ACADO compute = ", 1000*np.array([np.mean(data_acado[0]["compute_time"]), np.mean(data_acado[1]["compute_time"]), np.mean(data_acado[2]["compute_time"]), np.mean(data_acado[3]["compute_time"])]))


plt.figure(1, figsize=(4,4))
agents = [2, 4, 6, 8]
plt.grid()
plt.plot(agents, np.array([np.mean(data_ours[0]["inter_agent"]), np.mean(data_ours[1]["inter_agent"]), np.mean(data_ours[2]["inter_agent"]), np.mean(data_ours[3]["inter_agent"])]), linestyle='--', marker='o', linewidth=1.5, markersize=7.5, label=r'Ours(Quadratic) $\gamma = 0.9$', color='orange')
plt.plot(agents, np.array([np.mean(data_acado[0]["inter_agent"]), np.mean(data_acado[1]["inter_agent"]), np.mean(data_acado[2]["inter_agent"]), np.mean(data_acado[3]["inter_agent"])]), linestyle='--', marker='o', linewidth=1.5, markersize=7.5, label=r'ACADO $\gamma = 0.9$', color='brown')
plt.xlabel('Number Of Agents')
plt.ylabel('Inter-Agent Distance (m)')
plt.xticks(agents)
plt.tight_layout()
plt.savefig('acado_ours_inter_agent.png', bbox_inches='tight')

# print("Ours inter_agent = ", np.array([np.mean(data_ours[0]["inter_agent"]), np.mean(data_ours[1]["inter_agent"]), np.mean(data_ours[2]["inter_agent"]), np.mean(data_ours[3]["inter_agent"])]))
# print("ACADO inter_agent = ", np.array([np.mean(data_acado[0]["inter_agent"]), np.mean(data_acado[1]["inter_agent"]), np.mean(data_acado[2]["inter_agent"]), np.mean(data_acado[3]["inter_agent"])]))

# plt.figure(2)
# agent_size = ([2]*(data_ours[0]["arc_length"].size+ data_acado[0]["arc_length"].size) + 
#               [4]*(data_ours[1]["arc_length"].size+ data_acado[1]["arc_length"].size) +
#               [6]*(data_ours[2]["arc_length"].size+ data_acado[2]["arc_length"].size) +
#               [8]*(data_ours[3]["arc_length"].size+ data_acado[3]["arc_length"].size))

# approach = (["ours_axiswise"]*data_ours[0]["arc_length"].size + ["acado"]*(data_acado[0]["arc_length"].size) + 
#             ["ours_axiswise"]*data_ours[1]["arc_length"].size + ["acado"]*(data_acado[1]["arc_length"].size) +
#             ["ours_axiswise"]*data_ours[2]["arc_length"].size + ["acado"]*(data_acado[2]["arc_length"].size) +
#             ["ours_axiswise"]*data_ours[3]["arc_length"].size + ["acado"]*(data_acado[3]["arc_length"].size))

# arc_lengths = np.concatenate((data_ours[0]["arc_length"], data_acado[0]["arc_length"], 
#                             data_ours[1]["arc_length"], data_acado[1]["arc_length"],
#                             data_ours[2]["arc_length"], data_acado[2]["arc_length"],
#                             data_ours[3]["arc_length"], data_acado[3]["arc_length"]))

# arc_lengths = [i for i in arc_lengths]
# dict = {"arc_length":arc_lengths, "agent_size":agent_size, "approach":approach}
# ax = sns.boxplot(x="agent_size", y="arc_length", hue="approach", data=dict,showfliers=False)
# plt.xlabel("Agents")
# plt.ylabel("Value in (m)")
# plt.title("Arc Length Distance")
# plt.grid()
# plt.savefig('acado_ours_arc_length.png', dpi=100)

plt.figure(2, figsize=(4,4))
agents = [2, 4, 6, 8]
plt.grid()
plt.plot(agents, np.array([np.mean(data_ours[0]["mission_time"]), np.mean(data_ours[1]["mission_time"]), np.mean(data_ours[2]["mission_time"]), np.mean(data_ours[3]["mission_time"])]), linestyle='--', marker='o', linewidth=1.5, markersize=7.5, label=r'Ours(Quadratic) $\gamma = 0.9$', color='orange')
plt.plot(agents, np.array([np.mean(data_acado[0]["mission_time"]), np.mean(data_acado[1]["mission_time"]), np.mean(data_acado[2]["mission_time"]), np.mean(data_acado[3]["mission_time"])]), linestyle='--', marker='o', linewidth=1.5, markersize=7.5,label=r'ACADO $\gamma = 0.9$', color='brown')
plt.xlabel('Number Of Agents')
plt.ylabel('Mission Time (s)')
plt.xticks(agents)
# plt.legend()
plt.tight_layout()
plt.savefig('acado_ours_mission_time.png', bbox_inches='tight')
# print("Ours mission = ", np.array([np.mean(data_ours[0]["mission_time"]), np.mean(data_ours[1]["mission_time"]), np.mean(data_ours[2]["mission_time"]), np.mean(data_ours[3]["mission_time"])]))
# print("ACADO mission = ", np.array([np.mean(data_acado[0]["mission_time"]), np.mean(data_acado[1]["mission_time"]), np.mean(data_acado[2]["mission_time"]), np.mean(data_acado[3]["mission_time"])]))

plt.show()


    
