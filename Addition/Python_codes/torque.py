import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

data_y = np.genfromtxt('torque.txt', delimiter=None, dtype=(float))
data_x = np.genfromtxt('time.txt', delimiter='\n', dtype=(float))
data_phse = np.genfromtxt('phase.txt', delimiter=None, dtype=(float))
## get phase.txt data
phseChange = []
for i in range(0,len(data_phse)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i+1)
        else:
            pass
print phseChange
# phseChange.insert(0,0)

# phseChange.append(len(data_phse)-1)

# should have different in these x values(pos)
# get indexes
# for i in range(0,10):
    # exec('phse_%d = [index for index, v in enumerate(data_phse) if v == %d]'%(i,i))

#y1, y2, y3, y4, y5, y6 = ([] for i in range(6))
for j in range(1,7,1):
    exec('y%d = []'%j)

for i in data_y:
    for j in range(1,7,1):
        exec('y%d.append(i[%d])'%(j,j-1))
    #y6.append(i[5])

'''
for i in data_y:
    y1.append(i[0])
    y2.append(i[1])
    y3.append(i[2])
    y4.append(i[3])
    y5.append(i[4])
    y6.append(i[5])
'''
fig = plt.figure()
plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("480x400+0+0")
plt.plot(data_x, y1, data_x, y2, data_x, y3, data_x, y4, data_x, y5, data_x, y6)
plt.legend(('1', '2', '3', '4', '5', '6'), loc='upper left')
fig.canvas.set_window_title('TORQUE/TIME')
plt.xlabel('time')
plt.ylabel('torque')
plt.grid(True)

fig = plt.figure()
plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("480x400+480+0")
plt.plot(data_x, y1)
fig.canvas.set_window_title('TORQUE1/TIME')
plt.xlabel('time')
plt.ylabel('torque')
plt.grid(True)

fig = plt.figure()
plt.figure(3)
plt.get_current_fig_manager().window.wm_geometry("480x400+960+0")
plt.plot(data_x, y2)
fig.canvas.set_window_title('TORQUE2/TIME')
plt.xlabel('time')
plt.ylabel('torque')
plt.grid(True)

fig = plt.figure()
plt.figure(4)
plt.get_current_fig_manager().window.wm_geometry("480x400+1470+0")
plt.plot(data_x, y3)
fig.canvas.set_window_title('TORQUE3/TIME')
plt.xlabel('time')
plt.ylabel('torque')
plt.grid(True)

fig = plt.figure()
plt.figure(5)
plt.get_current_fig_manager().window.wm_geometry("480x400+0+444")
plt.plot(data_x, y4)
fig.canvas.set_window_title('TORQUE4/TIME')
plt.xlabel('time')
plt.ylabel('torque')
plt.grid(True)

fig = plt.figure()
plt.figure(6)
plt.get_current_fig_manager().window.wm_geometry("480x400+480+444")
plt.plot(data_x, y5)
fig.canvas.set_window_title('TORQUE5/TIME')
plt.xlabel('time')
plt.ylabel('torque')
plt.grid(True)

fig = plt.figure()
plt.figure(7)
plt.get_current_fig_manager().window.wm_geometry("480x400+960+444")
plt.plot(data_x, y6)
fig.canvas.set_window_title('TORQUE6/TIME')
plt.xlabel('time')
plt.ylabel('torque')
plt.grid(True)

plt.show()

