import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

file_path = os.getcwd() + "/../../experiment_data_check/"

## read files
data_torq = \
np.genfromtxt(file_path+'torque.txt', delimiter=None, dtype=(float))
data_comm = \
np.genfromtxt(file_path+'command.txt', delimiter=None, dtype=(float))
data_cpos = \
np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float))
data_cdes = \
np.genfromtxt(file_path+'com_pos_des.txt', delimiter=None, dtype=(float))

data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

# read files
# data_torq = np.genfromtxt('torque.txt', delimiter=None, dtype=(float))
# data_comm = np.genfromtxt('command.txt', delimiter=None, dtype=(float))
# data_cpos = np.genfromtxt('com_pos.txt', delimiter=None, dtype=(float))
# data_cdes = np.genfromtxt('com_pos_des.txt', delimiter=None, dtype=(float))
# data_x = np.genfromtxt('time.txt', delimiter='\n', dtype=(float))

## get torque.txt data
for j in range(1,7,1):
    exec('torq%d = []'%j)
for i in data_torq:
    for j in range(1,7,1):
        exec('torq%d.append(i[%d])'%(j,j-1))
## get command.txt data
for j in range(1,7,1):
    exec('comm%d = []'%j)
for i in data_comm:
    for j in range(1,7,1):
        exec('comm%d.append(i[%d])'%(j,j-1))
## get com_pos.txt data
for j in range(1,4,1):
    exec('cpos%d = []'%j)
for i in data_cpos:
    for j in range(1,4,1):
        exec('cpos%d.append(i[%d])'%(j,j-1))
## get com_pos_des.txt data
for j in range(1,4,1):
    exec('cdes%d = []'%j)
for i in data_cdes:
    for j in range(1,4,1):
        exec('cdes%d.append(i[%d])'%(j,j-1))

## plot command/torque
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("400x400+0+0")
fig.canvas.set_window_title('right_torque')
ax1 = plt.subplot(311)
plt.title('l0')
plt.plot(data_x, comm1, "r-" ,data_x, torq1, "b-")
plt.legend(('command', 'torque'), loc='upper left')
plt.ylabel('j0')
plt.grid(True)

ax2 = plt.subplot(312, sharex=ax1)
plt.plot(data_x, comm2, "r-" ,data_x, torq2, "b-")
plt.legend(('command', 'torque'), loc='upper left')
plt.ylabel('j1')
plt.grid(True)

ax3 = plt.subplot(313, sharex=ax1)
plt.plot(data_x, comm3, "r-" ,data_x, torq3, "b-")
plt.legend(('command', 'torque'), loc='upper left')
plt.ylabel('j2')
plt.grid(True)
plt.xlabel('time')

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("400x400+400+0")
fig.canvas.set_window_title('left_torque')
ax1 = plt.subplot(311)
plt.title('l1')
plt.plot(data_x, comm4, "r-" ,data_x, torq4, "b-")
plt.legend(('command', 'torque'), loc='upper left')
plt.ylabel('j0')
plt.grid(True)

ax2 = plt.subplot(312, sharex=ax1)
plt.plot(data_x, comm5, "r-" ,data_x, torq5, "b-")
plt.legend(('command', 'torque'), loc='upper left')
plt.ylabel('j1')
plt.grid(True)

ax3 = plt.subplot(313, sharex=ax1)
plt.plot(data_x, comm6, "r-" ,data_x, torq6, "b-")
plt.legend(('command', 'torque'), loc='upper left')
plt.ylabel('j2')
plt.grid(True)
plt.xlabel('time')

## com_pos/com_pos_des
fig = plt.figure(3)
plt.get_current_fig_manager().window.wm_geometry("400x400+800+0")
fig.canvas.set_window_title('com_pos')
ax1 = plt.subplot(311)
plt.title('com_pos')
plt.plot(data_x, cdes1, "r-" ,data_x, cpos1, "b-")
plt.legend(('com_pos_des', 'com_pos'), loc='upper left')
plt.ylabel('X')
plt.grid(True)

ax2 = plt.subplot(312, sharex=ax1)
plt.plot(data_x, cdes2, "r-" ,data_x, cpos2, "b-")
plt.legend(('com_pos_des', 'com_pos'), loc='upper left')
plt.ylabel('Y')
plt.grid(True)

ax3 = plt.subplot(313, sharex=ax1)
plt.plot(data_x, cdes3, "r-" ,data_x, cpos3, "b-")
plt.legend(('com_pos_des', 'com_pos'), loc='upper left')
plt.ylabel('Z')
plt.grid(True)
plt.xlabel('time')

plt.show()
