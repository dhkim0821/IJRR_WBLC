import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# file_path = '/Users/donghyunkim/Repository/dynacore/experiment_data_check/'
file_path = '/home/hcrl/Repository/dynacore/experiment_data_check/'
## read files
data_global_pos_offset = \
np.genfromtxt(file_path+'global_pos_local.txt', delimiter=None, dtype=(float))
data_estimated_com = \
np.genfromtxt(file_path+'estimated_com_state.txt', delimiter=None, dtype=(float))
data_com_pos_des = \
np.genfromtxt(file_path+'com_pos_des.txt', delimiter=None, dtype=(float))
data_com_pos = \
np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float))

data_com_vel_des =  \
np.genfromtxt(file_path+'com_vel_des.txt', delimiter=None, dtype=(float))
data_com_vel =  \
np.genfromtxt(file_path+'com_vel.txt', delimiter=None, dtype=(float))

data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

## plot command/torque
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
fig.canvas.set_window_title('com pos')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_com_pos_des[:,i-1] , "r-", \
             data_x, data_com_pos[:,i-1]+ data_global_pos_offset[:, i-1], "b-", \
             data_x, data_estimated_com[:, i-1] + data_global_pos_offset[:, i-1], "k-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x600+480+0")
fig.canvas.set_window_title('com vel')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_com_vel_des[:,i-1], "r-" , \
             data_x, data_com_vel[:,i-1], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

plt.show()

