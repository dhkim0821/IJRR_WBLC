import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

data_y = np.genfromtxt('com_pos.txt', delimiter=None, dtype=(float))
data_x = np.genfromtxt('time.txt', delimiter='\n', dtype=(float))

y1, y2, y3 = ([] for i in range(3))

for i in data_y:
    y1.append(i[0])
    y2.append(i[1])
    y3.append(i[2])

fig = plt.figure()
plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x400+0+0")
plt.plot(data_x, y1, data_x, y2, data_x, y3)
plt.legend(('X', 'Y', 'Z'), loc='upper left')
fig.canvas.set_window_title('COM_POS/TIME')
plt.xlabel('time')
plt.ylabel('com_pos')
plt.grid(True)

fig = plt.figure()
plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x400+480+0")
plt.plot(data_x, y1)
fig.canvas.set_window_title('COM_POS1/TIME')
plt.xlabel('time')
plt.ylabel('com_pos')
plt.grid(True)

fig = plt.figure()
plt.figure(3)
plt.get_current_fig_manager().window.wm_geometry("480x400+960+0")
plt.plot(data_x, y2)
fig.canvas.set_window_title('COM_POS2/TIME')
plt.xlabel('time')
plt.ylabel('com_pos')
plt.grid(True)

fig = plt.figure()
plt.figure(4)
plt.get_current_fig_manager().window.wm_geometry("480x400+1440+0")
plt.plot(data_x, y3)
fig.canvas.set_window_title('COM_POS3/TIME')
plt.xlabel('time')
plt.ylabel('com_pos')
plt.grid(True)

plt.show()

