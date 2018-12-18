import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

## read files    pos posdes vel veldes phase
data_cpos = np.genfromtxt('com_pos.txt', delimiter=None, dtype=(float))
data_dpos = np.genfromtxt('com_pos_des.txt', delimiter=None, dtype=(float))
data_cvel = np.genfromtxt('com_vel.txt', delimiter=None, dtype=(float))
data_dvel = np.genfromtxt('com_vel_des.txt', delimiter=None, dtype=(float))
data_phse = np.genfromtxt('phase.txt', delimiter=None, dtype=(float))

## get com_pos.txt data
for j in range(1,4,1):
    exec('cpos%d = []'%j)
for i in data_cpos:
    for j in range(1,4,1):
        exec('cpos%d.append(i[%d])'%(j,j-1))
## get com_pos_des.txt data
for j in range(1,4,1):
    exec('dpos%d = []'%j)
for i in data_dpos:
    for j in range(1,4,1):
        exec('dpos%d.append(i[%d])'%(j,j-1))
## get com_vel.txt data
for j in range(1,4,1):
    exec('cvel%d = []'%j)
for i in data_cvel:
    for j in range(1,4,1):
        exec('cvel%d.append(i[%d])'%(j,j-1))
## get com_vel_des.txt data
for j in range(1,4,1):
    exec('dvel%d = []'%j)
for i in data_dvel:
    for j in range(1,4,1):
        exec('dvel%d.append(i[%d])'%(j,j-1))
## get phase.txt data
phseChange = []
for i in range(0,len(data_phse)-1):
        if data_phse[i] != data_phse[i+1]:
            phseChange.append(i+1)
        else:
            pass
phseChange.insert(0,0)

phseChange.append(len(data_phse)-1)

## should have different in these x values(pos)
## get indexes
for i in range(0,10):
    exec('phse_%d = [index for index, v in enumerate(data_phse) if v == %d]'%(i,i))

## plot by each phase with a different color
# p%d_index
for i in range(0,10,1):
    exec('p%d_index = []'%i)
    exec('v%d_index = []'%i)
    exec('dp%d_index = []'%i)
    exec('dv%d_index = []'%i)
    exec('py%d_index = []'%i)
    exec('vy%d_index = []'%i)
    exec('dpy%d_index = []'%i)
    exec('dvy%d_index = []'%i)

for i in phse_0:
    p0_index.append(cpos1[i])
for i in phse_1:
    p1_index.append(cpos1[i])
for i in phse_2:
    p2_index.append(cpos1[i])
for i in phse_3:
    p3_index.append(cpos1[i])
for i in phse_4:
    p4_index.append(cpos1[i])
for i in phse_5:
    p5_index.append(cpos1[i])
for i in phse_6:
    p6_index.append(cpos1[i])
for i in phse_7:
    p7_index.append(cpos1[i])
for i in phse_8:
    p8_index.append(cpos1[i])
for i in phse_9:
    p9_index.append(cpos1[i])

for i in phse_0:
    v0_index.append(cvel1[i])
for i in phse_1:
    v1_index.append(cvel1[i])
for i in phse_2:
    v2_index.append(cvel1[i])
for i in phse_3:
    v3_index.append(cvel1[i])
for i in phse_4:
    v4_index.append(cvel1[i])
for i in phse_5:
    v5_index.append(cvel1[i])
for i in phse_6:
    v6_index.append(cvel1[i])
for i in phse_7:
    v7_index.append(cvel1[i])
for i in phse_8:
    v8_index.append(cvel1[i])
for i in phse_9:
    v9_index.append(cvel1[i])
   
for i in phse_0:
    dp0_index.append(dpos1[i])
for i in phse_1:
    dp1_index.append(dpos1[i])
for i in phse_2:
    dp2_index.append(dpos1[i])
for i in phse_3:
    dp3_index.append(dpos1[i])
for i in phse_4:
    dp4_index.append(dpos1[i])
for i in phse_5:
    dp5_index.append(dpos1[i])
for i in phse_6:
    dp6_index.append(dpos1[i])
for i in phse_7:
    dp7_index.append(dpos1[i])
for i in phse_8:
    dp8_index.append(dpos1[i])
for i in phse_9:
    dp9_index.append(dpos1[i])

for i in phse_0:
    dv0_index.append(dvel1[i])
for i in phse_1:
    dv1_index.append(dvel1[i])
for i in phse_2:
    dv2_index.append(dvel1[i])
for i in phse_3:
    dv3_index.append(dvel1[i])
for i in phse_4:
    dv4_index.append(dvel1[i])
for i in phse_5:
    dv5_index.append(dvel1[i])
for i in phse_6:
    dv6_index.append(dvel1[i])
for i in phse_7:
    dv7_index.append(dvel1[i])
for i in phse_8:
    dv8_index.append(dvel1[i])
for i in phse_9:
    dv9_index.append(dvel1[i])
################################################
for i in phse_0:
    py0_index.append(cpos2[i])
for i in phse_1:
    py1_index.append(cpos2[i])
for i in phse_2:
    py2_index.append(cpos2[i])
for i in phse_3:
    py3_index.append(cpos2[i])
for i in phse_4:
    py4_index.append(cpos2[i])
for i in phse_5:
    py5_index.append(cpos2[i])
for i in phse_6:
    py6_index.append(cpos2[i])
for i in phse_7:
    py7_index.append(cpos2[i])
for i in phse_8:
    py8_index.append(cpos2[i])
for i in phse_9:
    py9_index.append(cpos2[i])

for i in phse_0:
    vy0_index.append(cvel2[i])
for i in phse_1:
    vy1_index.append(cvel2[i])
for i in phse_2:
    vy2_index.append(cvel2[i])
for i in phse_3:
    vy3_index.append(cvel2[i])
for i in phse_4:
    vy4_index.append(cvel2[i])
for i in phse_5:
    vy5_index.append(cvel2[i])
for i in phse_6:
    vy6_index.append(cvel2[i])
for i in phse_7:
    vy7_index.append(cvel2[i])
for i in phse_8:
    vy8_index.append(cvel2[i])
for i in phse_9:
    vy9_index.append(cvel2[i])
   
for i in phse_0:
    dpy0_index.append(dpos2[i])
for i in phse_1:
    dpy1_index.append(dpos2[i])
for i in phse_2:
    dpy2_index.append(dpos2[i])
for i in phse_3:
    dpy3_index.append(dpos2[i])
for i in phse_4:
    dpy4_index.append(dpos2[i])
for i in phse_5:
    dpy5_index.append(dpos2[i])
for i in phse_6:
    dpy6_index.append(dpos2[i])
for i in phse_7:
    dpy7_index.append(dpos2[i])
for i in phse_8:
    dpy8_index.append(dpos2[i])
for i in phse_9:
    dpy9_index.append(dpos2[i])

for i in phse_0:
    dvy0_index.append(dvel2[i])
for i in phse_1:
    dvy1_index.append(dvel2[i])
for i in phse_2:
    dvy2_index.append(dvel2[i])
for i in phse_3:
    dvy3_index.append(dvel2[i])
for i in phse_4:
    dvy4_index.append(dvel2[i])
for i in phse_5:
    dvy5_index.append(dvel2[i])
for i in phse_6:
    dvy6_index.append(dvel2[i])
for i in phse_7:
    dvy7_index.append(dvel2[i])
for i in phse_8:
    dvy8_index.append(dvel2[i])
for i in phse_9:
    dvy9_index.append(dvel2[i])

## plot 
fig = plt.figure(1)
plt.get_current_fig_manager().window.wm_geometry("960x1100+0+0")
fig.canvas.set_window_title('X phase plot')
plt.title('X')
plt.plot(p0_index, v0_index, '-', color = 'red', label = 'initiation')
plt.plot(dp0_index, dv0_index, '--', color = 'red')
plt.plot(p1_index, v1_index, '-', color = 'lightsalmon', label = 'lift up')
plt.plot(dp1_index, dv1_index, '--', color = 'lightsalmon')
plt.plot(p2_index, v2_index, '-', color = 'gold', label = 'double contact 1')
plt.plot(dp2_index, dv2_index, '--', color = 'gold')
plt.plot(p3_index, v3_index, '-', color = 'y', label = 'right swing start trans')
plt.plot(dp3_index, dv3_index, '--', color = 'y')
plt.plot(p4_index, v4_index, '-', color = 'limegreen', label = 'right swing')
plt.plot(dp4_index, dv4_index, '--', color = 'limegreen')
plt.plot(p5_index, v5_index, '-', color = 'forestgreen', label = 'right swing end trans')
plt.plot(dp5_index, dv5_index, '--', color = 'forestgreen')
plt.plot(p6_index, v6_index, '-', color = 'deepskyblue', label = 'double contact 2')
plt.plot(dp6_index, dv6_index, '--', color = 'deepskyblue')
plt.plot(p7_index, v7_index, '-', color = 'royalblue', label = 'left swing start trans')
plt.plot(dp7_index, dv7_index, '--', color = 'royalblue')
plt.plot(p8_index, v8_index, '-', color = 'violet', label = 'left swing')
plt.plot(dp8_index, dv8_index, '--', color = 'violet')
plt.plot(p9_index, v9_index, '-', color = 'blueviolet', label = 'left swing end trans')
plt.plot(dp9_index, dv9_index, '--', color = 'blueviolet')
plt.legend(loc='upper left')
plt.ylabel('velocity')
plt.xlabel('position')
plt.grid(True)

fig = plt.figure(2)
plt.get_current_fig_manager().window.wm_geometry("960x1100+1000+0")
fig.canvas.set_window_title('Y phase plot')
plt.title('Y')
plt.plot(py0_index, vy0_index, '-', color = 'red', label = 'initiation')
plt.plot(dpy0_index, dvy0_index, '--', color = 'red')
plt.plot(py1_index, vy1_index, '-', color = 'lightsalmon', label = 'lift up')
plt.plot(dpy1_index, dvy1_index, '--', color = 'lightsalmon')
plt.plot(py2_index, vy2_index, '-', color = 'gold', label = 'double contact 1')
plt.plot(dpy2_index, dvy2_index, '--', color = 'gold')
plt.plot(py3_index, vy3_index, '-', color = 'y', label = 'right swing start trans')
plt.plot(dpy3_index, dvy3_index, '--', color = 'y')
plt.plot(py4_index, vy4_index, '-', color = 'limegreen', label = 'right swing')
plt.plot(dpy4_index, dvy4_index, '--', color = 'limegreen')
plt.plot(py5_index, vy5_index, '-', color = 'forestgreen', label = 'right swing end trans')
plt.plot(dpy5_index, dvy5_index, '--', color = 'forestgreen')
plt.plot(py6_index, vy6_index, '-', color = 'deepskyblue', label = 'double contact 2')
plt.plot(dpy6_index, dvy6_index, '--', color = 'deepskyblue')
plt.plot(py7_index, vy7_index, '-', color = 'royalblue', label = 'left swing start trans')
plt.plot(dpy7_index, dvy7_index, '--', color = 'royalblue')
plt.plot(py8_index, vy8_index, '-', color = 'violet', label = 'left swing')
plt.plot(dpy8_index, dvy8_index, '--', color = 'violet')
plt.plot(py9_index, vy9_index, '-', color = 'blueviolet', label = 'left swing end trans')
plt.plot(dpy9_index, dvy9_index, '--', color = 'blueviolet')
plt.legend(loc='upper left')
plt.ylabel('velocity')
plt.xlabel('position')
plt.grid(True)

plt.show()
