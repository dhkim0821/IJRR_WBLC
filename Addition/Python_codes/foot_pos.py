import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# file_path = '/Users/donghyunkim/Repository/dynacore/experiment_data_check/'
file_path = '/home/hcrl/Repository/dynacore/experiment_data_check/'
## read files
data_rfoot_pos_des = \
np.genfromtxt(file_path+'rfoot_pos_des.txt', delimiter=None, dtype=(float))
data_rfoot_pos = \
np.genfromtxt(file_path+'rfoot_pos.txt', delimiter=None, dtype=(float))

data_lfoot_pos_des =  \
np.genfromtxt(file_path+'lfoot_pos_des.txt', delimiter=None, dtype=(float))
data_lfoot_pos =  \
np.genfromtxt(file_path+'lfoot_pos.txt', delimiter=None, dtype=(float))

data_rfoot_vel_des = \
np.genfromtxt(file_path+'rfoot_vel_des.txt', delimiter=None, dtype=(float))
data_rfoot_vel = \
np.genfromtxt(file_path+'rfoot_vel.txt', delimiter=None, dtype=(float))

data_lfoot_vel_des =  \
np.genfromtxt(file_path+'lfoot_vel_des.txt', delimiter=None, dtype=(float))
data_lfoot_vel =  \
np.genfromtxt(file_path+'lfoot_vel.txt', delimiter=None, dtype=(float))


data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

## plot command/torque
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
fig.canvas.set_window_title('right_foot')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_rfoot_pos_des[:,i-1], "r-", \
             data_x, data_rfoot_pos[:,i-1], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x600+480+0")
fig.canvas.set_window_title('left_foot')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_lfoot_pos_des[:,i-1], "r-" , \
             data_x, data_lfoot_pos[:,i-1], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

## plot foot vel
fig = plt.figure(3)
plt.get_current_fig_manager().window.wm_geometry("480x600+960+0")
fig.canvas.set_window_title('right_foot (vel)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_rfoot_vel_des[:,i-1], "r-", \
             data_x, data_rfoot_vel[:,i-1], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(4)
plt.get_current_fig_manager().window.wm_geometry("480x600+1440+0")
fig.canvas.set_window_title('left_foot (vel)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_lfoot_vel_des[:,i-1], "r-" , \
             data_x, data_lfoot_vel[:,i-1], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')


plt.show()

