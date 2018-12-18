import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# file_path = '/Users/donghyunkim/Repository/dynacore/experiment_data_check/'
file_path = '/home/hcrl/Repository/dynacore/experiment_data_check/'
## read files
data_jpos_des = \
np.genfromtxt(file_path+'jpos_des.txt', delimiter=None, dtype=(float))
data_config = \
np.genfromtxt(file_path+'config.txt', delimiter=None, dtype=(float))
data_jvel_des = \
np.genfromtxt(file_path+'jvel_des.txt', delimiter=None, dtype=(float))
data_qdot = \
np.genfromtxt(file_path+'qdot.txt', delimiter=None, dtype=(float))

data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

## plot command/jpos
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
fig.canvas.set_window_title('jpos (right_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_jpos_des[:,i-1], "r-", \
             data_x, data_config[:,i-1 + 6], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x600+480+0")
fig.canvas.set_window_title('jpos (left_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_jpos_des[:,i-1 + 3], "r-" , \
             data_x, data_config[:,i-1 + 9], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

## plot jvel
fig = plt.figure(3)
plt.get_current_fig_manager().window.wm_geometry("480x600+960+0")
fig.canvas.set_window_title('jvel (right_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_jvel_des[:,i-1], "r-", \
             data_x, data_qdot[:,i-1 + 6], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(4)
plt.get_current_fig_manager().window.wm_geometry("480x600+1440+0")
fig.canvas.set_window_title('jvel (left_leg)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_jvel_des[:,i-1 + 3], "r-" , \
             data_x, data_qdot[:,i-1 + 9], "b-")
    # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
plt.xlabel('time (sec)')



plt.show()

