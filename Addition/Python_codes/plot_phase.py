import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

def main():

    file_path = os.getcwd() + "/../../experiment_data_check/"

    # CoM data
    data_com_pos = \
            np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float))
    data_com_vel = \
            np.genfromtxt(file_path+'com_vel.txt', delimiter=None, dtype=(float))
    # foot data #
    data_rfoot_contact = \
            np.genfromtxt(file_path+'rfoot_contact.txt', delimiter=None, dtype=(float))
    data_rfoot_pos = \
            np.genfromtxt(file_path+'rfoot_pos.txt', delimiter=None, dtype=(float))
    data_rfoot_vel = \
            np.genfromtxt(file_path+'rfoot_vel.txt', delimiter=None, dtype=(float))
    data_rfoot_pos_des = \
            np.genfromtxt(file_path+'rfoot_pos_des.txt', delimiter=None, dtype=(float))
    data_rfoot_vel_des = \
            np.genfromtxt(file_path+'rfoot_vel_des.txt', delimiter=None, dtype=(float))
    data_lfoot_contact = \
            np.genfromtxt(file_path+'lfoot_contact.txt', delimiter=None, dtype=(float))
    data_lfoot_pos = \
            np.genfromtxt(file_path+'lfoot_pos.txt', delimiter=None, dtype=(float))
    data_lfoot_vel = \
            np.genfromtxt(file_path+'lfoot_vel.txt', delimiter=None, dtype=(float))
    data_lfoot_pos_des = \
            np.genfromtxt(file_path+'lfoot_pos_des.txt', delimiter=None, dtype=(float))
    data_lfoot_vel_des = \
            np.genfromtxt(file_path+'lfoot_vel_des.txt', delimiter=None, dtype=(float))
    data_phse = \
            np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
    data_body_pos_des = \
            np.genfromtxt(file_path+'body_pos_des.txt', delimiter=None, dtype=(float))
    # CoM
    data_x_com_pos = []
    data_x_com_vel = []
    data_y_com_pos = []
    data_y_com_vel = []
    for i in range(len(data_com_pos)):
        data_x_com_pos.append(data_com_pos[i][0])
        data_x_com_vel.append(data_com_vel[i][0])
        data_y_com_pos.append(data_com_pos[i][1])
        data_y_com_vel.append(data_com_vel[i][1])
    # desired phase data
    data_constant = []
    g = 9.80665
    for i in range(len(data_body_pos_des)):
        data_constant.append((data_body_pos_des[i][2])/g)
    print data_constant
    ## phase ##
    # swing
    phse_rswing = [0] # right leg swing period index
    phse_lswing = [0] # left leg swing period index
    for i in range(len(data_phse)-1):
            if data_phse[i] != data_phse[i+1] and data_phse[i+1]== 4:
                phse_rswing.append(i+1)
            elif data_phse[i] != data_phse[i+1] and data_phse[i-1]== 4:
                phse_rswing.append(i)
            elif data_phse[i] != data_phse[i+1] and data_phse[i+1]== 8:
                phse_lswing.append(i+1)
            elif data_phse[i] != data_phse[i+1] and data_phse[i-1]== 8:
                phse_lswing.append(i)
            else:
                pass
    # swing_start
    phse_rstart = [0]
    phse_lstart = [0]
    for i in range(len(data_phse)-1):
            if data_phse[i] != data_phse[i+1] and data_phse[i+1]== 3:
                phse_rstart.append(i+1)
            elif data_phse[i] != data_phse[i+1] and data_phse[i-1]== 3:
                phse_rstart.append(i)
            elif data_phse[i] != data_phse[i+1] and data_phse[i+1]== 7:
                phse_lstart.append(i+1)
            elif data_phse[i] != data_phse[i+1] and data_phse[i-1]== 7:
                phse_lstart.append(i)
            else:
                pass
    # swing_end
    phse_rend = [0]
    phse_lend = [0]
    for i in range(len(data_phse)-1):
            if data_phse[i] != data_phse[i+1] and data_phse[i+1]== 5:
                phse_rend.append(i+1)
            elif data_phse[i] != data_phse[i+1] and data_phse[i-1]== 5:
                phse_rend.append(i)
            elif data_phse[i] != data_phse[i+1] and data_phse[i+1]== 9:
                phse_lend.append(i+1)
            elif data_phse[i] != data_phse[i+1] and data_phse[i-1]== 9:
                phse_lend.append(i)
            else:
                pass
    print phse_lstart
    print phse_lend
    print phse_lswing

    # rfoot_pos
    data_x_rfoot_pos = []
    data_y_rfoot_pos = []
    for i in range(len(data_rfoot_pos)):
        data_x_rfoot_pos.append(data_rfoot_pos[i][0])
        data_y_rfoot_pos.append(data_rfoot_pos[i][1])
    # rfoot_vel
    data_x_rfoot_vel = []
    data_y_rfoot_vel = []
    for i in range(len(data_rfoot_vel)):
        data_x_rfoot_vel.append(data_rfoot_vel[i][0])
        data_y_rfoot_vel.append(data_rfoot_vel[i][1])
    # rfoot_pos_des
    data_x_rfoot_pos_des = []
    data_y_rfoot_pos_des = []
    for i in range(len(data_rfoot_pos_des)):
        data_x_rfoot_pos_des.append(data_rfoot_pos_des[i][0])
        data_y_rfoot_pos_des.append(data_rfoot_pos_des[i][1])
    # rfoot_vel_des
    data_x_rfoot_vel_des = []
    data_y_rfoot_vel_des = []
    for i in range(len(data_rfoot_vel_des)):
        data_x_rfoot_vel_des.append(data_rfoot_vel_des[i][0])
        data_y_rfoot_vel_des.append(data_rfoot_vel_des[i][1])
    # lfoot_pos
    data_x_lfoot_pos = []
    data_y_lfoot_pos = []
    for i in range(len(data_lfoot_pos)):
        data_x_lfoot_pos.append(data_lfoot_pos[i][0])
        data_y_lfoot_pos.append(data_lfoot_pos[i][1])
    # lfoot_vel
    data_x_lfoot_vel = []
    data_y_lfoot_vel = []
    for i in range(len(data_lfoot_vel)):
        data_x_lfoot_vel.append(data_lfoot_vel[i][0])
        data_y_lfoot_vel.append(data_lfoot_vel[i][1])
    # lfoot_pos_des
    data_x_lfoot_pos_des = []
    data_y_lfoot_pos_des = []
    for i in range(len(data_lfoot_pos_des)):
        data_x_lfoot_pos_des.append(data_lfoot_pos_des[i][0])
        data_y_lfoot_pos_des.append(data_lfoot_pos_des[i][1])
    # lfoot_vel_des
    data_x_lfoot_vel_des = []
    data_y_lfoot_vel_des = []
    for i in range(len(data_lfoot_vel_des)):
        data_x_lfoot_vel_des.append(data_lfoot_vel_des[i][0])
        data_y_lfoot_vel_des.append(data_lfoot_vel_des[i][1])
    
    # get trajectory range end index # step_idx = [0]
    # step_mid = []
    # count = 0
    # for i in range(len(data_rfoot_contact)):
        # if data_rfoot_contact[i] == 1 and data_rfoot_contact[i-1] == 0:
            # step_idx.append(i)
            # step_mid.append((step_idx[count])+((step_idx[count+1]-step_idx[count])//2))
            # count += 1
        # elif data_rfoot_contact[i] == 0 and data_rfoot_contact[i-1] == 1:
            # step_idx.append(i)
            # step_mid.append((step_idx[count])+((step_idx[count+1]-step_idx[count])//2))
            # count += 1
        # else:
            # pass
    # print "1st swing : %d~%d" %(step_idx[0],step_idx[1])
    # print "1st foot foot position index : %d" %(step_mid[0])

    # plot foot position/desired
    print len(data_x_com_pos)
    print phse_lstart[1]
    print phse_lstart[3]
    zeroHeight = [0]
    for i in range(1,6,2):
        fig = plt.figure(1)
        exec('plt.subplot(1,7,%d)'%(i))
        plt.get_current_fig_manager().window.wm_geometry("480x400+480+0")
        plt.plot(data_x_lfoot_pos[phse_rstart[i]], zeroHeight[0],'r*', \
                data_x_lfoot_pos_des[phse_rswing[i+2]], zeroHeight[0],'ko', \
                data_x_lfoot_pos[phse_rstart[i+2]], zeroHeight[0],'b+', \
                data_x_com_pos[phse_rswing[i]:phse_rswing[i+1]+2], \
                data_x_com_vel[phse_rswing[i]:phse_rswing[i+1]+2],'g-', \
                data_x_com_pos[phse_rswing[i+2]:phse_rswing[i+3]+2], \
                data_x_com_vel[phse_rswing[i+2]:phse_rswing[i+3]+2],'r-')
        plt.plot(data_x_com_pos[phse_rstart[i]:phse_rstart[i+1]+2], \
                data_x_com_vel[phse_rstart[i]:phse_rstart[i+1]+2], '-', color = 'lime')
        plt.plot(data_x_com_pos[phse_rend[i]:phse_rend[i+1]+2], \
                data_x_com_vel[phse_rend[i]:phse_rend[i+1]+2], '-', color = 'lime')
        plt.plot(data_x_com_pos[phse_rstart[i+2]:phse_rstart[i+3]+2], \
                data_x_com_vel[phse_rstart[i+2]:phse_rstart[i+3]+2], '-', color = 'magenta')
        plt.plot(data_x_com_pos[phse_rend[i+2]:phse_rend[i+1]+3], \
                data_x_com_vel[phse_rend[i+2]:phse_rend[i+1]+3], '-', color = 'magenta')
        plt.grid(True)
        exec('plt.subplot(1,7,%d)'%(i+1))
        plt.get_current_fig_manager().window.wm_geometry("480x400+480+0")
        plt.plot(data_x_rfoot_pos[phse_lstart[i]], zeroHeight[0],'r*', \
                data_x_rfoot_pos_des[phse_lstart[i+2]], zeroHeight[0],'ko', \
                data_x_rfoot_pos[phse_lstart[i+2]], zeroHeight[0],'b+', \
                data_x_com_pos[phse_lswing[i]:phse_lswing[i+1]+2], \
                data_x_com_vel[phse_lswing[i]:phse_lswing[i+1]+2],'g-', \
                data_x_com_pos[phse_lswing[i+2]:phse_lswing[i+3]+2], \
                data_x_com_vel[phse_lswing[i+2]:phse_lswing[i+3]+2],'r-')
        plt.plot(data_x_com_pos[phse_lstart[i]:phse_lstart[i+1]+2], \
                data_x_com_vel[phse_lstart[i]:phse_lstart[i+1]+2], '-', color = 'lime')
        plt.plot(data_x_com_pos[phse_lend[i]:phse_lend[i+1]+2], \
                data_x_com_vel[phse_lend[i]:phse_lend[i+1]+2], '-', color = 'lime')
        plt.plot(data_x_com_pos[phse_lstart[i+2]:phse_lstart[i+3]+2], \
                data_x_com_vel[phse_lstart[i+2]:phse_lstart[i+3]+2], '-', color = 'magenta')
        plt.plot(data_x_com_pos[phse_lend[i+2]:phse_lend[i+1]+3], \
                data_x_com_vel[phse_lend[i+2]:phse_lend[i+1]+3], '-', color = 'magenta')

        str_i = str(i)
        # exec('fig.canvas.set_window_title(''X %s foot position check'')'%(str_i))
        fig.canvas.set_window_title('X foot phase plot')
        # plt.xlabel('x')
        # plt.ylabel('x dot')
        # plt.legend('raw','des','next','com')
        plt.grid(True)


    plt.show()
    ##--------------------------------------------------------------------------------
    # PHASE MARKER #
main()
