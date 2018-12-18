import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1


def create_figures(subfigure_width=480, subfigure_height=500, starting_figure_no=1, \
        starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    file_path = os.getcwd() + "/../../experiment_data_check/"

    ## read files  ************************************************************
    # CoM data
    data_global_pos_offset = \
        np.genfromtxt(file_path+'global_pos_local.txt', delimiter=None, dtype=(float))
    # data_global_pos_offset = \
        # np.genfromtxt(file_path+'global_jjpos_local.txt', delimiter=None, dtype=(float))
    data_estimated_com = \
        np.genfromtxt(file_path+'estimated_com_state.txt', delimiter=None, dtype=(float))
    data_com_pos = \
        np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float))
    data_com_vel = \
        np.genfromtxt(file_path+'com_vel.txt', delimiter=None, dtype=(float))
    data_config = \
        np.genfromtxt(file_path+'config.txt', delimiter=None, dtype=(float))
    # foot data #
    data_rfoot_contact = \
        np.genfromtxt(file_path+'rfoot_contact.txt', delimiter=None, dtype=(float))
    data_rfoot_pos = \
        np.genfromtxt(file_path+'rfoot_pos.txt', delimiter=None, dtype=(float))
    data_jj_rfoot_pos = \
        np.genfromtxt(file_path+'jjpos_rfoot_pos.txt', delimiter=None, dtype=(float))
    data_jj_lfoot_pos = \
        np.genfromtxt(file_path+'jjpos_lfoot_pos.txt', delimiter=None, dtype=(float))
    data_rfoot_pos_des = \
        np.genfromtxt(file_path+'rfoot_pos_des.txt', delimiter=None, dtype=(float))
    data_lfoot_contact = \
        np.genfromtxt(file_path+'lfoot_contact.txt', delimiter=None, dtype=(float))
    data_lfoot_pos = \
        np.genfromtxt(file_path+'lfoot_pos.txt', delimiter=None, dtype=(float))
    data_lfoot_pos_des = \
        np.genfromtxt(file_path+'lfoot_pos_des.txt', delimiter=None, dtype=(float))
    data_phse = \
        np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
    # data_ekf_body_pos = \
        # np.genfromtxt(file_path+'ekf_o_r.txt', delimiter=None, dtype=(float))
    # data_ekf_body_vel = \
        # np.genfromtxt(file_path+'ekf_o_v.txt', delimiter=None, dtype=(float))
    data_planner = \
        np.genfromtxt(file_path+'planner_data.txt', delimiter=None, dtype=(float))
    data_est_mocap_body_vel = \
            np.genfromtxt(file_path+'est_mocap_body_vel.txt', delimiter=None, dtype=(float))
    data_est_mocap_body_pos = \
            np.genfromtxt(file_path+'est_mocap_body_pos.txt', delimiter=None, dtype=(float))
    data_est_com_vel = \
            np.genfromtxt(file_path+'est_com_vel.txt', delimiter=None, dtype=(float))

    phase_swing_st_trans_idx = []
    phase_swing_st_idx = []
    phase_swing_end_trans_idx = []
    phase_double_idx = []

    for i in range(len(data_phse)-1):
            if data_phse[i] != data_phse[i+1] and (data_phse[i+1]== 3 or \
                    data_phse[i+1] == 7):
                phase_swing_st_trans_idx.append(i+1)
            elif data_phse[i] != data_phse[i+1] and (data_phse[i+1]== 4 or \
                    data_phse[i+1] == 8):
                phase_swing_st_idx.append(i+1)
            elif data_phse[i] != data_phse[i+1] and (data_phse[i+1]== 5 or \
                    data_phse[i+1] == 9):
                phase_swing_end_trans_idx.append(i+1)
            elif data_phse[i] != data_phse[i+1] and (data_phse[i+1]== 6 or \
                    data_phse[i+1] == 2):
                phase_double_idx.append(i+1)
            else:
                pass
    
    print phase_swing_st_trans_idx
    print phase_swing_st_idx
    print phase_swing_end_trans_idx
    print phase_double_idx

    stance_foot_loc = []
    for i in range(len(data_global_pos_offset)-1):
        if (data_global_pos_offset[i,1] != data_global_pos_offset[i+1,1]):
            stance_foot_loc.append(data_global_pos_offset[i,:])

    np_stancefoot = np.array(stance_foot_loc)

    data_com_pos_global = data_com_pos + data_global_pos_offset
    data_mocap_pos_global = data_est_mocap_body_pos + data_global_pos_offset
    data_body_global = data_config[:,0:3] + data_global_pos_offset
    data_estimated_com_global = data_estimated_com
    data_estimated_com_global[:,0:2] += data_global_pos_offset[:, 0:2]
    x_pos_offset = 0.0
    st_step = 50
    num_steps = 8

    lin_width = 3
    lin_width_thin = 1
    fig_width = 480
    fig_height = 400
    x_lim = 1500
    x_pos = -fig_width
    y_pos = 0
    for i in range(num_steps):
        x_pos += fig_width
        if (x_pos > x_lim):
            y_pos = y_pos + fig_height
            x_pos = 0

        fig_size_loc = str(fig_width) + "x" + str(fig_height) + "+" + str(x_pos) + "+" + str(y_pos)
        fig = plt.figure(figure_number)
        plt.get_current_fig_manager().window.wm_geometry(fig_size_loc)
        plt.grid(True)

        ### Indexing
        trans1_st_idx = phase_swing_st_trans_idx[st_step+i];
        trans1_end_idx = phase_swing_st_idx[st_step + i];
        # swing
        swing_st_idx = phase_swing_st_idx[st_step + i]-1;
        swing_end_idx = phase_swing_end_trans_idx[st_step + i]+3;
        # trans 2
        trans2_st_idx = phase_swing_end_trans_idx[st_step + i];
        trans2_end_idx = phase_double_idx[st_step+i + 1];
        # double
        double_st_idx = phase_double_idx[st_step+i+1];
        double_end_idx = phase_swing_st_trans_idx[st_step+i+1];
        # next swing trans
        trans3_st_idx = phase_swing_st_trans_idx[st_step+i + 1];
        trans3_end_idx = phase_swing_st_idx[st_step + i +1];
        # next swing
        nx_swing_st_idx = phase_swing_st_idx[st_step + i + 1]-1;
        nx_swing_end_idx = phase_swing_end_trans_idx[st_step + i + 1]+3;

         # x *******************************************************************
        plot_axis = 0
        ## Phase trajectory #############
        plt.plot(data_com_pos_global[swing_st_idx:swing_end_idx, plot_axis], \
                data_com_vel[swing_st_idx:swing_end_idx, plot_axis], \
                linewidth=lin_width, color='blue')
        plt.plot(data_com_pos_global[double_st_idx:double_end_idx, plot_axis], \
                data_com_vel[double_st_idx:double_end_idx, plot_axis], \
                linewidth=lin_width, color='black')
        plt.plot(data_com_pos_global[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                data_com_vel[nx_swing_st_idx:nx_swing_end_idx, plot_axis], linewidth=lin_width, color='indigo')
        ## End of Phase trajectory #############

        # plt.plot(data_com_pos_global[swing_st_idx:swing_end_idx, plot_axis], \
                # data_est_com_vel[swing_st_idx:swing_end_idx, plot_axis], \
                # linewidth=lin_width, color='olive')
        # plt.plot(data_com_pos_global[double_st_idx:double_end_idx, plot_axis], \
                # data_est_com_vel[double_st_idx:double_end_idx, plot_axis], \
                # linewidth=lin_width, color='cyan')
        # plt.plot(data_com_pos_global[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                # data_est_com_vel[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                # linewidth=lin_width, color='orange')

        #current swing
        plt.plot(data_body_global[swing_st_idx:swing_end_idx, plot_axis], \
                data_est_mocap_body_vel[swing_st_idx:swing_end_idx, plot_axis], \
                linewidth=lin_width, color='crimson')
        # next swing
        plt.plot(data_body_global[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                data_est_mocap_body_vel[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                linewidth=lin_width, color='cyan')



        # Body pos
        # plt.plot(data_body_global[trans1_st_idx:trans1_end_idx, plot_axis], \
                # data_com_vel[trans1_st_idx:trans1_end_idx, plot_axis], linewidth=lin_width, color='blue')
        # plt.plot(data_body_global[swing_st_idx:swing_end_idx, plot_axis], \
                # data_com_vel[swing_st_idx:swing_end_idx, plot_axis], linewidth=lin_width, color='olive')
        # plt.plot(data_body_global[trans2_st_idx:trans2_end_idx, plot_axis], \
                # data_com_vel[trans2_st_idx:trans2_end_idx, plot_axis], linewidth=lin_width, color='orange')

        # plt.plot(data_ekf_body_pos[trans1_st_idx:trans1_end_idx, plot_axis], \
                # data_ekf_body_vel[trans1_st_idx:trans1_end_idx, plot_axis], linewidth=lin_width, color='tomato')
        # plt.plot(data_ekf_body_pos[swing_st_idx:swing_end_idx, plot_axis], \
                # data_ekf_body_vel[swing_st_idx:swing_end_idx, plot_axis], linewidth=lin_width, color='crimson')
        # plt.plot(data_ekf_body_pos[trans2_st_idx:trans2_end_idx, plot_axis], \
                # data_ekf_body_vel[trans2_st_idx:trans2_end_idx, plot_axis], linewidth=lin_width, color='deeppink')

        ## Estimated (Steven)
        # plt.plot(data_com_pos_global[swing_st_idx:swing_end_idx, plot_axis], \
                # data_ekf_body_vel[swing_st_idx:swing_end_idx, plot_axis], linewidth=lin_width, color='orange')
        # plt.plot(data_com_pos_global[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                # data_ekf_body_vel[nx_swing_st_idx:nx_swing_end_idx, plot_axis], linewidth=lin_width, color='olive')
 
       

        ### planner choice #########################################################
        # planning start com state
        plt.scatter(data_planner[st_step+i, plot_axis] + x_pos_offset, \
                data_planner[st_step+i, 2 + plot_axis], \
                s = 90, facecolors='none', edgecolor='sienna')
        plt.scatter(data_planner[st_step+i, 4 + plot_axis] + x_pos_offset, data_planner[st_step+i, 6 + plot_axis], \
                s = 60, facecolors='none', edgecolor='green')
        # Foot location
        plt.scatter(data_planner[st_step + i, 8], 0, s=80, facecolors='none',\
                edgecolor='r')
        plt.plot(np_stancefoot[st_step + i, 0], 0, '*', color='orange', markersize=12) # current stance foot
        plt.plot(np_stancefoot[st_step + i+1, 0], 0, \
                'b+', markersize = 15, linewidth=4) # next stance foot (landing location)

        # if (st_step+i)%2 ==1:
            # landing_loc = data_jj_lfoot_pos[swing_end_idx - 3, plot_axis] + np_stancefoot[st_step+i,0]
        # else:
            # landing_loc = data_jj_rfoot_pos[swing_end_idx-3, plot_axis] + np_stancefoot[st_step+i,0]

        # plt.plot(landing_loc, 0, \
                # '+', color='black', markersize = 17, linewidth=6)
        ### END of planner choice #########################################################
        figure_number += 1;


        # y *********************************************
        fig = plt.figure(figure_number)
        x_pos += fig_width
        fig_size_loc = str(fig_width) + "x" + str(fig_height) + "+" + str(x_pos) + "+" + str(y_pos)
        plt.get_current_fig_manager().window.wm_geometry(fig_size_loc)
        plt.grid(True)
        plot_axis = 1
        # current swing
        plt.plot(data_com_pos_global[swing_st_idx:swing_end_idx, plot_axis], \
                data_com_vel[swing_st_idx:swing_end_idx, plot_axis], linewidth=lin_width_thin, color='blue')
        # double 
        plt.plot(data_com_pos_global[double_st_idx:double_end_idx, plot_axis], \
                data_com_vel[double_st_idx:double_end_idx, plot_axis], linewidth=lin_width_thin, color='black')
 
        # next swing
        plt.plot(data_com_pos_global[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                data_com_vel[nx_swing_st_idx:nx_swing_end_idx, plot_axis], linewidth=lin_width_thin, color='indigo')

        ## Estimated (DHKim) 
        #current swing
        # plt.plot(data_com_pos_global[swing_st_idx:swing_end_idx, plot_axis], \
                # data_est_com_vel[swing_st_idx:swing_end_idx, plot_axis], \
                # linewidth=lin_width, color='olive')
        # next swing
        # plt.plot(data_com_pos_global[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                # data_est_com_vel[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                # linewidth=lin_width, color='orange')

        #current swing
        plt.plot(data_body_global[swing_st_idx:swing_end_idx, plot_axis], \
                data_est_mocap_body_vel[swing_st_idx:swing_end_idx, plot_axis], \
                linewidth=lin_width, color='crimson')
        # next swing
        plt.plot(data_body_global[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                data_est_mocap_body_vel[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                linewidth=lin_width, color='cyan')


       # plt.plot(data_estimated_com_global[trans1_st_idx:trans1_end_idx, plot_axis], \
                # data_estimated_com_global[trans1_st_idx:trans1_end_idx, 2 + plot_axis], linewidth=lin_width, color='olive')
        # plt.plot(data_estimated_com_global[swing_st_idx:swing_end_idx, plot_axis], \
                # data_estimated_com_global[swing_st_idx:swing_end_idx, 2 + plot_axis], linewidth=lin_width, color='orange')
        # plt.plot(data_estimated_com_global[trans2_st_idx:trans2_end_idx, plot_axis], \
                # data_estimated_com_global[trans2_st_idx:trans2_end_idx, 2 + plot_axis], linewidth=lin_width, color='olive')
 
        # Estimated (Steven)
        # plt.plot(data_body_global[swing_st_idx:swing_end_idx, plot_axis], \
                # data_ekf_body_vel[swing_st_idx:swing_end_idx, plot_axis], linewidth=lin_width, color='orange')
        # plt.plot(data_body_global[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                # data_ekf_body_vel[nx_swing_st_idx:nx_swing_end_idx, plot_axis], linewidth=lin_width, color='olive')
        # plt.plot(data_ekf_body_pos[swing_st_idx:swing_end_idx, plot_axis], \
                # data_ekf_body_vel[swing_st_idx:swing_end_idx, plot_axis], linewidth=lin_width, color='orange')
        # plt.plot(data_ekf_body_pos[nx_swing_st_idx:nx_swing_end_idx, plot_axis], \
                # data_ekf_body_vel[nx_swing_st_idx:nx_swing_end_idx, plot_axis], linewidth=lin_width, color='olive')
    
        # planner choice
        # planning start com state
        plt.scatter(data_planner[st_step+i, plot_axis], data_planner[st_step+i, 2 + plot_axis], \
                s = 90, facecolors='none', edgecolor='sienna')
        plt.scatter(data_planner[st_step+i, 4 + plot_axis], data_planner[st_step+i, 6 + plot_axis], \
                s = 60, facecolors='none', edgecolor='green')
        # Foot location
        plt.scatter(data_planner[st_step + i, 8+plot_axis], 0, s=80, facecolors='none', edgecolor='r') # planned location
        plt.plot(np_stancefoot[st_step + i, plot_axis], 0, '*', color='orange', markersize=12) # current stance foot
        plt.plot(np_stancefoot[st_step + i+1, plot_axis], 0, 'b+', markersize=12) # next stance foot (landing location)
        # if (st_step+i)%2 ==1:
            # landing_loc = data_jj_lfoot_pos[swing_end_idx - 3, plot_axis] + np_stancefoot[st_step+i, plot_axis]
        # else:
            # landing_loc = data_jj_rfoot_pos[swing_end_idx-3, plot_axis] + np_stancefoot[st_step+i, plot_axis]

        # plt.plot(landing_loc, 0, \
                # '+', color='black', markersize = 17, linewidth=7)


        figure_number += 1;

if __name__ == "__main__":
   create_figures()
   plt.show()
