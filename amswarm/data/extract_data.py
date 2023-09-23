import rospkg
import numpy as np
import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from mpl_toolkits import mplot3d

rospack = rospkg.RosPack()
rospack.list() 
path = rospack.get_path('amswarm')

sim_data_x = np.loadtxt(path + "/data/sim_data_upsampled_x.txt")
sim_data_y = np.loadtxt(path + "/data/sim_data_upsampled_y.txt")
sim_data_z = np.loadtxt(path + "/data/sim_data_upsampled_z.txt")

sim_info = np.loadtxt(path + "/data/sim_info.txt")
dt = float(sim_info[0][2])
num_obs = int(sim_info[0][1])
num_drone = int(sim_info[0][0])

sim_steps = (len(sim_data_x)/num_drone/3)
### SEND COMMANDS AT PLANNING TIME / NUM_UP CONTROL FREQUENCY
for i in range(sim_steps):
    x_cmd = sim_data_x[i:i+num_drone] 
    y_cmd = sim_data_y[i:i+num_drone]
    z_cmd = sim_data_z[i:i+num_drone]

    xdot_cmd = sim_data_x[i+num_drone:i+2*num_drone]
    ydot_cmd = sim_data_y[i+num_drone:i+2*num_drone]
    zdot_cmd = sim_data_z[i+num_drone:i+2*num_drone]

    xddot_cmd = sim_data_x[i+2*num_drone:i+3*num_drone]
    yddot_cmd = sim_data_y[i+2*num_drone:i+3*num_drone]
    zddot_cmd = sim_data_z[i+2*num_drone:i+3*num_drone]

    """
        structure of x_cmd
        row 0 ---> drone 0 x-commands
        row 1 ---> drone 1 x-commands
        .
        .
        .
        row N-1 ---> drone N-1 x-commads
    """

    for j in range(len(x_cmd[0])):
        
        """
            send x_cmd[0][j] to drone 0
            send x_cmd[1][j] to drone 1
            send x_cmd[2][j] to drone 2
            send x_cmd[N-1][j] to drone N-1
        """
