import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot
num_figures = 4

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, \
        starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    file_path = os.getcwd() + "/../../experiment_data_check/"

    ## read files
    data_global_pos_offset = \
    np.genfromtxt(file_path+'global_pos_local.txt', delimiter=None, dtype=(float))
    data_estimated_com = \
    np.genfromtxt(file_path+'estimated_com_state.txt', delimiter=None, dtype=(float))
    data_com = \
    np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float))
    data_com_vel = \
    np.genfromtxt(file_path+'com_vel.txt', delimiter=None, dtype=(float))
    # data_body_des = \
    # np.genfromtxt(file_path+'body_pos_des.txt', delimiter=None, dtype=(float))
    # data_body_ori_des = \
    # np.genfromtxt(file_path+'body_ori_des.txt', delimiter=None, dtype=(float))
    # data_body_ori = \
    # np.genfromtxt(file_path+'body_ori.txt', delimiter=None, dtype=(float))
    # data_body_vel = \
    # np.genfromtxt(file_path+'body_vel.txt', delimiter=None, dtype=(float))
    data_qdot = \
    np.genfromtxt(file_path+'qdot.txt', delimiter=None, dtype=(float))
    data_q = \
    np.genfromtxt(file_path+'config.txt', delimiter=None, dtype=(float))
    data_LED = \
            np.genfromtxt(file_path+'LED_Pos.txt', delimiter=None, dtype=(float))
    data_led_body_vel = \
    np.genfromtxt(file_path+'Body_LED_vel.txt', delimiter=None, dtype=(float))
    # data_jjpos_body_pos = \
        # np.genfromtxt(file_path+'jjpos_body_pos.txt', delimiter=None, dtype=(float))
    data_jjvel_qdot = \
        np.genfromtxt(file_path+'jjvel_qdot.txt', delimiter=None, dtype=(float))
    data_jjvel_body_vel = np.copy(data_jjvel_qdot[:, 3:6]);
   # data_ekf_body_pos = \
    # np.genfromtxt(file_path+'ekf_o_r.txt', delimiter=None, dtype=(float))
    # data_ekf_body_vel = \
    # np.genfromtxt(file_path+'ekf_o_v.txt', delimiter=None, dtype=(float))
    # data_com_kin_vel = \
        # np.genfromtxt(file_path+'ekf_com_vel_kin.txt', delimiter=None, dtype=(float))
    data_est_com_vel = \
            np.genfromtxt(file_path+'est_com_vel.txt', delimiter=None, dtype=(float))
    data_est_mocap_body_vel = \
            np.genfromtxt(file_path+'est_mocap_body_vel.txt', delimiter=None, dtype=(float))
    data_est_mocap_body_pos = \
            np.genfromtxt(file_path+'est_mocap_body_pos.txt', delimiter=None, dtype=(float))
    data_ave_vel = \
            np.genfromtxt(file_path+'average_vel.txt', delimiter=None, dtype=(float))
    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

    data_body_ori = np.zeros(shape = (len(data_x), 4))
    data_body_ori[:, 1:4] = np.copy(data_q[:, 3:6]);
    data_body_ori[:, 0] = np.copy(data_q[:, 12]);
    
    data_com_global = data_com + data_global_pos_offset
    data_body_global = data_q[:,0:3] + data_global_pos_offset
    data_est_com_global = data_estimated_com[:,0:2] + \
                                    data_global_pos_offset[:, 0:2]

    st_idx = 1000
    end_idx = len(data_x) - 10
    # end_idx = st_idx + 3000
    data_x = data_x[st_idx:end_idx]

    # PHASE MARKER #
    data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
    data_phse = data_phse[st_idx:end_idx]
    # get phase.txt data #
    phseChange = []
    for i in range(0,len(data_x)-1):
            if data_phse[i] != data_phse[i+1]:
                phseChange.append(i)
            else:
                pass
    axes = plt.gca()

    # foot position
    rfoot_LED_idx = [6, 7];
    lfoot_LED_idx = [11, 12];
  
  # TEST
    data_LED_rfoot = ((data_LED[:, 3*rfoot_LED_idx[0]:3*rfoot_LED_idx[0]+3]))
    data_LED_lfoot = ((data_LED[:, 3*lfoot_LED_idx[0]:3*lfoot_LED_idx[0]+3]))

    data_LED_body = data_LED[:, 0:3] 
    data_LED_local_body_from_rfoot = data_LED[:, 0:3] - data_LED_rfoot
    data_LED_local_body_from_lfoot = data_LED[:, 0:3] - data_LED_lfoot

    data_LED_rfoot = ((data_LED[:, 3*rfoot_LED_idx[1]:3*rfoot_LED_idx[1]+3]))
    data_LED_lfoot = ((data_LED[:, 3*lfoot_LED_idx[1]:3*lfoot_LED_idx[1]+3]))

    data_LED_local_body_from_rfoot2 = data_LED[:, 0:3] - data_LED_rfoot
    data_LED_local_body_from_lfoot2 = data_LED[:, 0:3] - data_LED_lfoot


    ## plot global
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('body global pos')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_com_global[st_idx:end_idx,i-1], "c-", \
                # data_x, data_body_des[st_idx:end_idx,i-1], "r-", \
                data_x, data_body_global[st_idx:end_idx,i-1], "b-")
        plt.plot(data_x, data_global_pos_offset[st_idx:end_idx,i-1], "crimson")
        # if i != 3:
            # plt.plot(data_x, data_est_com_global[st_idx:end_idx,i-1], "k-")

        plt.plot(data_x, data_LED[st_idx:end_idx, i-1], color="orange")
        # plt.legend(('command', 'pos'), loc='upper left')
        plt.grid(True)
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    ## plot local body
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('body local pos')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_com[st_idx:end_idx,i-1], "c-", \
                # data_x, data_body_des[st_idx:end_idx,i-1], "r-", \
                data_x, data_q[st_idx:end_idx,i-1], "b-")
        # plt.plot(data_x, data_jjpos_body_pos[st_idx:end_idx, i-1], \
                # color="black", linewidth=1.5)
        # if i != 3:
            # plt.plot(data_x, data_estimated_com[st_idx:end_idx,i-1], "k-")

        # plt.plot(data_x, data_LED_body[st_idx:end_idx, i-1], color="orange")
        plt.plot(data_x, data_LED_local_body_from_rfoot[st_idx:end_idx, i-1], \
            color='indigo')
        plt.plot(data_x, data_LED_local_body_from_lfoot[st_idx:end_idx, i-1], \
            color='olive')
        # plt.plot(data_x, data_LED_local_body_from_rfoot2[st_idx:end_idx, i-1], \
            # color='indigo')
        # plt.plot(data_x, data_LED_local_body_from_lfoot2[st_idx:end_idx, i-1], \
            # color='olive')
        # plt.plot(data_x, data_est_mocap_body_pos[st_idx:end_idx, i-1], \
            # color='orange', linewidth=2.)
        # plt.legend(('command', 'pos'), loc='upper left')
        plt.grid(True)
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    # fig = plt.figure(2)
    # plt.get_current_fig_manager().window.wm_geometry("480x600+540+0")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + \
            "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))    
    fig.canvas.set_window_title('body vel')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_com_vel[st_idx:end_idx,i-1], "c-", linewidth=2)
        plt.plot(data_x, data_qdot[st_idx:end_idx,i-1], "b-")
        plt.plot(data_x, data_jjvel_body_vel[st_idx:end_idx, i-1], color="black", linewidth=1.5)
        # plt.plot(data_x, data_ekf_body_vel[st_idx:end_idx, i-1], "g-")
        # plt.plot(data_x, data_com_kin_vel[st_idx:end_idx, i-1], linewidth=1.5, color = "crimson")
        plt.plot(data_x, data_led_body_vel[st_idx:end_idx, i-1], color="orange", linewidth=4)
 
        if i != 3:
            # plt.plot(data_x, data_estimated_com[st_idx:end_idx,i-1+2], "k-")
            # plt.plot(data_x, data_est_com_vel[st_idx:end_idx, i-1], "g-", linewidth=2)
            plt.plot(data_x, data_est_mocap_body_vel[st_idx:end_idx, i-1], color="crimson", linewidth=2)
            plt.plot(data_x, data_ave_vel[st_idx:end_idx,i-1], linewidth=2, color="olive")

        plt.grid(True)
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
        
    # fig = plt.figure(3)
    # plt.get_current_fig_manager().window.wm_geometry("480x600+0+650")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('body ori (quaternion)')

    for i in range(1,5,1):
        ax1 = plt.subplot(4, 1, i)
        plt.plot(data_x, data_body_ori[st_idx:end_idx,i-1], "b-")
        plt.grid(True)

        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')

    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    # fig = plt.figure(4)
    # plt.get_current_fig_manager().window.wm_geometry("480x600+540+650")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('body ang vel')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_qdot[st_idx:end_idx,i-1+3], "r-")
        plt.grid(True)
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.xlabel('time (sec)')


if __name__ == "__main__":
    create_figures()
    plt.show()
