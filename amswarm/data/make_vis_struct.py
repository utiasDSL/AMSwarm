import yaml
import rospkg
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Ellipse

np.set_printoptions(precision=3)

rospack = rospkg.RosPack()
rospack.list() 
pack_dir = rospack.get_path('amswarm')

with open(pack_dir+"/params/config_generate.yaml", 'r') as stream:
    data_loaded = yaml.safe_load(stream)

x_lim = np.array([-1.8, 1.8])
y_lim = np.array([-1.8, 1.8])

pos_obs = np.array(data_loaded['pos_static_obs'], dtype=float)

x_obs = pos_obs[:, 0] 
y_obs = pos_obs[:, 1] 
z_obs = pos_obs[:, 2] 

dim_obs = np.array(data_loaded['dim_static_obs'], dtype=float)

a_obs = dim_obs[:, 0] 
b_obs = dim_obs[:, 1] 
c_obs = dim_obs[:, 2] 

init_drone = np.array(data_loaded['init_drone'],dtype=float)

x_init = init_drone[:, 0] 
y_init = init_drone[:, 1] 
z_init = init_drone[:, 2] 


plt.rcParams['backend'] = 'TkAgg'
plt.rcParams["figure.figsize"] = [10.0, 10.0]
plt.rcParams["figure.autolayout"] = True

pts_x = []
pts_y = []

# Function to print mouse click event coordinates
def onclick(event):
    pts_x.append(event.xdata)
    pts_y.append(event.ydata)
    

# Create a figure and a set of subplots
fig, ax = plt.subplots()

# Plot a line in the range of 10
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)

ax.grid()
# Bind the button_press_event with the onclick() method
fig.canvas.mpl_connect('button_press_event', onclick)

# ell_obs = [Ellipse(xy=(x_obs[k], y_obs[k]), width=2*(a_obs[k])+0.05, height=2*(b_obs[k])+0.05, 
#                                     zorder = 0, facecolor='gray', edgecolor='gray') for k in range(len(a_obs))]
# [ax.add_artist(items) for items in ell_obs]

# ell_rob = [Ellipse(xy=(x_init[k], y_init[k]), width=2*(0.0675)+0.05, height=2*(0.0675)+0.05, 
#                                     zorder = 0, facecolor='red', edgecolor='red') for k in range(len(x_init))]
# [ax.add_artist(items) for items in ell_rob]

# plt.clf()
ax.plot([x_lim[0], x_lim[0], x_lim[1], x_lim[1], x_lim[0]], [y_lim[0], y_lim[1], y_lim[1], y_lim[0], y_lim[0]], linestyle='--', color='red')

# Display the plot
while(1):
    if len(pts_x) != 0:
        if len(pts_x) >=15:
            break
        pts = plt.scatter(pts_x, pts_y, color='red', marker='*')
    plt.draw()
    plt.pause(0.0001)

pts_x = np.asarray(pts_x)
pts_y = np.asarray(pts_y)
pts_z = np.ones(len(pts_x))*0.5 
temp = np.vstack((pts_x, pts_y, pts_z)).T

print(repr(temp))
print(repr(-temp))

