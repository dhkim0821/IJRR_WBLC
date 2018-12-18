import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

file_path = '/Users/donghyunkim/Repository/dynacore/experiment_data_check/'
# file_path = '/home/hcrl/Repository/dynacore/experiment_data_check/'
## read files
data_reflect_rf = \
np.genfromtxt(file_path+'refl_react_force.txt', delimiter=None, dtype=(float))

data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

st_idx = 10
end_idx = len(data_x) - 50
data_x = data_x[st_idx:end_idx]

## plot reflected reaction force
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x800+0+200")
fig.canvas.set_window_title('torque (floating)')
for i in range(1,7,1):
    ax1 = plt.subplot(6, 1, i)
    plt.plot(data_x, data_reflect_rf[st_idx:end_idx, i-1], "r-")
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x600+400+200")
fig.canvas.set_window_title('torque (right)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_reflect_rf[st_idx:end_idx, i-1 + 6], "r-")
    plt.grid(True)
plt.xlabel('time (sec)')

fig = plt.figure(3)
plt.get_current_fig_manager().window.wm_geometry("480x600+800+200")
fig.canvas.set_window_title('torque (left)')
for i in range(1,4,1):
    ax1 = plt.subplot(3, 1, i)
    plt.plot(data_x, data_reflect_rf[st_idx:end_idx, i-1 + 9], "r-")
    plt.grid(True)
plt.xlabel('time (sec)')


plt.show()

