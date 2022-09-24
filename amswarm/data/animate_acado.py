import rospkg
import numpy as np
import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from mpl_toolkits import mplot3d


rospack = rospkg.RosPack()
rospack.list() 
path = rospack.get_path('amswarm')

sim_data = np.loadtxt(path+"/data/sim_data_acado.txt")
sim_info = np.loadtxt(path +"/data/sim_info_acado.txt")

dt = float(sim_info[0][2])
num_obs = int(sim_info[0][1])
num_drone = int(sim_info[0][0])
dim_drone = sim_info[1]

a_drone, b_drone, c_drone = dim_drone

x_lim = [-2, 2]
y_lim = [-2, 2]
z_lim = [+0, 2]


fig = plt.figure(0)

ax = fig.add_subplot(111, projection='3d')
ax = plt.gca()

mng = plt.get_current_fig_manager()
mng.full_screen_toggle()

ax.set_title('Trajectory')
ax.set_xlabel('x in m')
ax.set_ylabel('y in m')
ax.set_zlabel('z in m')

ax.set_xlim(x_lim[0], x_lim[1])
ax.set_ylim(y_lim[0], y_lim[1])
ax.set_zlim(z_lim[0], z_lim[1])

phi_obs = np.linspace(0,2*np.pi, 10).reshape(10, 1) 
theta_obs = np.linspace(0, np.pi/2, 10).reshape(-1, 10) 

phi_drone = np.linspace(0,2*np.pi, 10).reshape(10, 1) 
theta_drone = np.linspace(0, np.pi, 10).reshape(-1, 10) 

colors=(np.random.choice(range(255),size=[num_drone, 3]))/255.0


        
ax.plot([x_lim[0], x_lim[0], x_lim[1], x_lim[1], x_lim[0] ], [y_lim[0], y_lim[1], y_lim[1], y_lim[0], y_lim[0]], color='red', alpha=0.1)
ax.plot([x_lim[0], x_lim[0], x_lim[1], x_lim[1], x_lim[0] ], [y_lim[0], y_lim[1], y_lim[1], y_lim[0], y_lim[0]], [z_lim[1], z_lim[1], z_lim[1], z_lim[1], z_lim[1]],color='red', alpha=0.1)
ax.plot([x_lim[0], x_lim[0]], [y_lim[0], y_lim[0]], [z_lim[0], z_lim[1]], color='red', alpha=0.1)
ax.plot([x_lim[0], x_lim[0]], [y_lim[1], y_lim[1]], [z_lim[0], z_lim[1]], color='red', alpha=0.1)
ax.plot([x_lim[1], x_lim[1]], [y_lim[1], y_lim[1]], [z_lim[0], z_lim[1]], color='red', alpha=0.1)
ax.plot([x_lim[1], x_lim[1]], [y_lim[0], y_lim[0]], [z_lim[0], z_lim[1]], color='red', alpha=0.1)

collision_count_agent = 0
collision_count_obs = 0
sim_steps = (len(sim_data)/num_drone/3)


for i in range(sim_steps):
    body = []
    predictions = []

    x_data = sim_data[3*i*num_drone + 0: 3*i*num_drone + num_drone]
    y_data = sim_data[3*i*num_drone + num_drone:3*i*num_drone + 2*num_drone]
    z_data = sim_data[3*i*num_drone + 2*num_drone: 3*i*num_drone + 3*num_drone]
    
    for k in range(0, num_drone):
        x_ell_drone = x_data[k, 0] + a_drone * np.sin(theta_drone)*np.cos(phi_drone)
        y_ell_drone = y_data[k, 0] + b_drone * np.sin(theta_drone)*np.sin(phi_drone)
        z_ell_drone = z_data[k, 0] + c_drone * np.cos(theta_drone)
    
        body_temp = ax.plot_surface(x_ell_drone, y_ell_drone, z_ell_drone,  rstride=4, cstride=6, color = colors[k], alpha=0.6)
        body.append(body_temp)

        predictions_temp, = ax.plot(x_data[k], y_data[k], z_data[k], color=colors[k], linestyle='--')
        predictions.append(predictions_temp) 
        if i%3 == 0:
            ax.scatter(x_data[k, 0], y_data[k, 0], z_data[k, 0], color = colors[k], alpha=0.6)

    for m in range(0,num_drone):
        x_check = x_data[m, 0]
        y_check = y_data[m, 0]
        z_check = z_data[m, 0]
        for n in range(0,num_drone):
            if m == n:
                continue
            val = (x_check - x_data[n, 0])**2/(2*a_drone)**2 + (y_check - y_data[n, 0])**2/(2*b_drone)**2 + (z_check - z_data[n, 0])**2/(2*c_drone)**2 
            if val < 1:
                collision_count_agent += 1 
            
    stats = ax.text(0, y_lim[1], z_lim[1],'Obstacle Collision Count = {obs}\nInter-Agent Collision Count = {col}\nAgents = {n}\nSim-Step = {step}'.format(obs=collision_count_obs, col=collision_count_agent, n=num_drone, step=i)) 
    
    plt.draw()
    plt.pause(0.00000001)
    
    if collision_count_agent > 0 or collision_count_obs > 0:
        break
    if i != sim_steps - 1:
        [items.remove() for items in body]
        [items.remove() for items in predictions]
        stats.remove()


plt.show()




