import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot
num_figures = 8

sim_data_available = False
kin_com_data_available = False
filtered_ang_vel_data_available = False
rot_imu_vel_data_available = False

plot_contact_switches = False

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY, use_custom_layout=False):
    global sim_data_available
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    file_path = os.getcwd() + "/../../experiment_data_check/"

    ## read files
    data_body_pos_des = \
    np.genfromtxt(file_path+'body_pos_des.txt', delimiter=None, dtype=(float))    
    data_body_vel_des = \
    np.genfromtxt(file_path+'body_vel_des.txt', delimiter=None, dtype=(float))
    data_body_ori_des = \
    np.genfromtxt(file_path+'body_ori_des.txt', delimiter=None, dtype=(float)) 

    data_sim_imu_pos = None
    data_sim_imu_vel = None    
    try:
        data_sim_imu_pos = \
        np.genfromtxt(file_path+'sim_imu_pos.txt', delimiter=None, dtype=(float))    
        data_sim_imu_vel = \
        np.genfromtxt(file_path+'sim_imu_vel.txt', delimiter=None, dtype=(float))
        sim_data_available = True
    except:
        print "\n Note: simulated data for position and/or velocity is not available. Will not plot simulated ground truth data\n"
        sim_data_available = False


    data_com = \
    np.genfromtxt(file_path+'com_pos.txt', delimiter=None, dtype=(float))
    data_kinematics_com_vel = \
    np.genfromtxt(file_path+'com_vel.txt', delimiter=None, dtype=(float))
 
    data_body_pos = \
    np.genfromtxt(file_path+'ekf_o_r.txt', delimiter=None, dtype=(float))
    data_body_vel = \
    np.genfromtxt(file_path+'ekf_o_v.txt', delimiter=None, dtype=(float))
    data_body_ori = \
    np.genfromtxt(file_path+'ekf_o_q_b.txt', delimiter=None, dtype=(float))

    data_body_kin_vel = \
    np.genfromtxt(file_path+'ekf_body_vel_kin.txt', delimiter=None, dtype=(float))

    data_com_kin_vel = None
    try:
        data_com_kin_vel = \
        np.genfromtxt(file_path+'ekf_com_vel_kin.txt', delimiter=None, dtype=(float))
        kin_com_data_available = True
    except:
        print "\n Note: EKF com vel data from kinematicsd not available\n"
        kin_com_data_available = False

    data_accumulated_body_ori = \
    np.genfromtxt(file_path+'body_ori.txt', delimiter=None, dtype=(float))


    data_z_lfoot_pos = \
    np.genfromtxt(file_path+'ekf_z_l_foot_B.txt', delimiter=None, dtype=(float))
    data_z_rfoot_pos = \
    np.genfromtxt(file_path+'ekf_z_r_foot_B.txt', delimiter=None, dtype=(float))

    data_p_lfoot_pos = \
    np.genfromtxt(file_path+'ekf_p_left_B.txt', delimiter=None, dtype=(float))
    data_p_rfoot_pos = \
    np.genfromtxt(file_path+'ekf_p_right_B.txt', delimiter=None, dtype=(float))

    data_world_p_lfoot_pos = \
    np.genfromtxt(file_path+'ekf_p_left_O.txt', delimiter=None, dtype=(float))
    data_world_p_rfoot_pos = \
    np.genfromtxt(file_path+'ekf_p_right_O.txt', delimiter=None, dtype=(float))


    data_f_imu = \
    np.genfromtxt(file_path+'ekf_f_imu.txt', delimiter=None, dtype=(float))
    data_bias_f_imu = \
    np.genfromtxt(file_path+'ekf_B_bf.txt', delimiter=None, dtype=(float))


    data_omega_imu = \
    np.genfromtxt(file_path+'imu_ang_vel.txt', delimiter=None, dtype=(float))    
    data_bias_omega_imu = \
    np.genfromtxt(file_path+'ekf_B_bw.txt', delimiter=None, dtype=(float))    


    data_filtered_ang_vel = None
    try:
        data_filtered_ang_vel = \
        np.genfromtxt(file_path+'filtered_ang_vel.txt', delimiter=None, dtype=(float))    
        filtered_ang_vel_data_available = True
        print "Filtered angular velocity data available"
    except:
        filtered_ang_vel_data_available = False
        print "Note: Filtered angular velocity data is not available"

    data_meas_diff = \
    np.genfromtxt(file_path+'ekf_measured_diff.txt', delimiter=None, dtype=(float))    



    try:
        data_rot_imu_vel = \
        np.genfromtxt(file_path+'ekf_imu_vel.txt', delimiter=None, dtype=(float))
        rot_imu_vel_data_available = True
    except:
        print "\n Note: EKF com vel data from kinematicsd not available\n"
        rot_imu_vel_data_available = False


              


    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

    st_idx = 10
    end_idx = len(data_x) - 1
    data_x = data_x[st_idx:end_idx]

    # PHASE MARKER #
    data_phse = np.genfromtxt(file_path+'phase.txt', delimiter=None, dtype=(float))
    # get phase.txt data #
    phseChange = []
    for i in range(0,len(data_x)-1):
            if data_phse[i] != data_phse[i+1]:
                phseChange.append(i - st_idx)
            else:
                pass
    axes = plt.gca()



    # LeftFoot Contact Signal #
    data_lf_contact = np.genfromtxt(file_path+'lfoot_contact.txt', delimiter=None, dtype=(float))
    data_lf_contact = data_lf_contact[st_idx:end_idx]    
    lf_contact_index_change = []
    lf_contact_index_change.append(0)    
    lf_contact_index_change.append(1)        
    for i in range(2, len(data_x)):
        if data_lf_contact[i] != data_lf_contact[i-1]:
            lf_contact_index_change.append(i)


    # RightFoot Contact Signal #
    data_rf_contact = np.genfromtxt(file_path+'rfoot_contact.txt', delimiter=None, dtype=(float))
    data_rf_contact = data_rf_contact[st_idx:end_idx]
    rf_contact_index_change = []
    rf_contact_index_change.append(0)    
    rf_contact_index_change.append(1)        
    for i in range(2, len(data_x)):
        if data_rf_contact[i] != data_rf_contact[i-1]:
            rf_contact_index_change.append(i)            



 
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    fig.canvas.set_window_title('World body world pos(m)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        if sim_data_available:
            plt.plot(data_x, data_sim_imu_pos[st_idx:end_idx,i-1], "r-")
        plt.plot(data_x, data_body_pos[st_idx:end_idx,i-1], "b-")

        # plt.plot(data_x, data_body_pos[st_idx:end_idx,i-1], "b-", \
        #          data_x, data_com[st_idx:end_idx, i-1], "c-")

        if plot_contact_switches:
            # Plot Left Foot Contact 
            for j in range(1, len(lf_contact_index_change)):
                t_start = data_x[lf_contact_index_change[j-1]]
                t_end = data_x[lf_contact_index_change[j]]            
                val_start = data_lf_contact[lf_contact_index_change[j-1]]
                val_end = data_lf_contact[lf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_start)*ax1.get_ylim()[0]*0.9
                y_end = val_end * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_end)*ax1.get_ylim()[0]*0.9          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='orange')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'L%d'%(data_lf_contact[lf_contact_index_change[j-1]]),color='orange')


            # Plot Right Foot Contact 
            for j in range(1, len(rf_contact_index_change)):
                t_start = data_x[rf_contact_index_change[j-1]]
                t_end = data_x[rf_contact_index_change[j]]            
                val_start = data_rf_contact[rf_contact_index_change[j-1]]
                val_end = data_rf_contact[rf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0  + (1 - val_start)*ax1.get_ylim()[0]
                y_end = val_end * ax1.get_ylim()[1]/2.0  + (1 - val_end)*ax1.get_ylim()[0]          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'R%d'%(data_rf_contact[rf_contact_index_change[j-1]]),color='magenta')



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
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))    
    fig.canvas.set_window_title('World body vel(m/s)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)

        if sim_data_available:
            plt.plot(data_x, data_sim_imu_vel[st_idx:end_idx,i-1], "r-")

        # plt.plot(data_x, data_body_vel[st_idx:end_idx, i-1], "b-", \
        #          data_x, data_kinematics_com_vel[st_idx:end_idx, i-1], "c-")

        plt.plot(data_x, data_body_vel[st_idx:end_idx, i-1], "b-")
        plt.plot(data_x, data_body_kin_vel[st_idx:end_idx, i-1], color="orange")
        
        if kin_com_data_available:
            plt.plot(data_x, data_com_kin_vel[st_idx:end_idx, i-1], color="black")

        # if rot_imu_vel_data_available:
        #     plt.plot(data_x, data_rot_imu_vel[st_idx:end_idx,i-1], color="olive")

        if plot_contact_switches:
            # Plot Left Foot Contact 
            for j in range(1, len(lf_contact_index_change)):
                t_start = data_x[lf_contact_index_change[j-1]]
                t_end = data_x[lf_contact_index_change[j]]            
                val_start = data_lf_contact[lf_contact_index_change[j-1]]
                val_end = data_lf_contact[lf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_start)*ax1.get_ylim()[0]*0.9
                y_end = val_end * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_end)*ax1.get_ylim()[0]*0.9          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='orange')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'L%d'%(data_lf_contact[lf_contact_index_change[j-1]]),color='orange')


            # Plot Right Foot Contact 
            for j in range(1, len(rf_contact_index_change)):
                t_start = data_x[rf_contact_index_change[j-1]]
                t_end = data_x[rf_contact_index_change[j]]            
                val_start = data_rf_contact[rf_contact_index_change[j-1]]
                val_end = data_rf_contact[rf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0  + (1 - val_start)*ax1.get_ylim()[0]
                y_end = val_end * ax1.get_ylim()[1]/2.0  + (1 - val_end)*ax1.get_ylim()[0]          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'R%d'%(data_rf_contact[rf_contact_index_change[j-1]]),color='magenta')

        plt.grid(True)
        for j in phseChange:
            # phase line
            plt.axvline(x=data_x[j],color='indigo',linestyle='-')
            # phase number
            plt.text(data_x[j],ax1.get_ylim()[1],'%d'%(data_phse[j]),color='indigo')
        plt.xlabel('time (sec)')
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1


    # fig = plt.figure(3)
    # plt.get_current_fig_manager().window.wm_geometry("480x600+0+650")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('World body ori (quaternion)')
    for i in range(1,5,1):
        ax1 = plt.subplot(4, 1, i)
        plt.plot(data_x, data_body_ori_des[st_idx:end_idx,i-1], "r-" , \
                data_x, data_body_ori[st_idx:end_idx,i-1], "b-", \
                data_x, data_accumulated_body_ori[st_idx:end_idx, i-1], "c-")
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1


    # plt.get_current_fig_manager().window.wm_geometry("480x600+0+650")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('body imu input and bias (acc)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_f_imu[st_idx:end_idx,i-1], "b-" , \
                data_x, data_bias_f_imu[st_idx:end_idx,i-1], "c-")
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    # Use another row
    if use_custom_layout:
        col_index = 0
        row_index = 1

    # plt.get_current_fig_manager().window.wm_geometry("480x600+0+650")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('body omega input and bias (ang vel)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_omega_imu[st_idx:end_idx,i-1], "b-" , \
                data_x, data_bias_omega_imu[st_idx:end_idx,i-1], "c-")

        if filtered_ang_vel_data_available:
            plt.plot(data_x, data_filtered_ang_vel[st_idx:end_idx,i-1], color="orange")


        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1


    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('measured diff (m)')
    for i in range(1,7,1):
        ax1 = plt.subplot(6, 1, i)
        plt.plot(data_x, 0*data_meas_diff[st_idx:end_idx,i-1], "r-")
        plt.plot(data_x, data_meas_diff[st_idx:end_idx,i-1], "b-")

        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1


    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('Body frame lfoot position (m)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_z_lfoot_pos[st_idx:end_idx,i-1], "r-")
        plt.plot(data_x, data_p_lfoot_pos[st_idx:end_idx,i-1], "b-")        

        if plot_contact_switches:
            # Plot Left Foot Contact 
            for j in range(1, len(lf_contact_index_change)):
                t_start = data_x[lf_contact_index_change[j-1]]
                t_end = data_x[lf_contact_index_change[j]]            
                val_start = data_lf_contact[lf_contact_index_change[j-1]]
                val_end = data_lf_contact[lf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_start)*ax1.get_ylim()[0]*0.9
                y_end = val_end * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_end)*ax1.get_ylim()[0]*0.9          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='orange')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'L%d'%(data_lf_contact[lf_contact_index_change[j-1]]),color='orange')


            # Plot Right Foot Contact 
            for j in range(1, len(rf_contact_index_change)):
                t_start = data_x[rf_contact_index_change[j-1]]
                t_end = data_x[rf_contact_index_change[j]]            
                val_start = data_rf_contact[rf_contact_index_change[j-1]]
                val_end = data_rf_contact[rf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0  + (1 - val_start)*ax1.get_ylim()[0]
                y_end = val_end * ax1.get_ylim()[1]/2.0  + (1 - val_end)*ax1.get_ylim()[0]          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'R%d'%(data_rf_contact[rf_contact_index_change[j-1]]),color='magenta')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('Body frame rfoot position (m)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_z_rfoot_pos[st_idx:end_idx,i-1], "r-")
        plt.plot(data_x, data_p_rfoot_pos[st_idx:end_idx,i-1], "b-")        

        if plot_contact_switches:
            # Plot Left Foot Contact 
            for j in range(1, len(lf_contact_index_change)):
                t_start = data_x[lf_contact_index_change[j-1]]
                t_end = data_x[lf_contact_index_change[j]]            
                val_start = data_lf_contact[lf_contact_index_change[j-1]]
                val_end = data_lf_contact[lf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_start)*ax1.get_ylim()[0]*0.9
                y_end = val_end * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_end)*ax1.get_ylim()[0]*0.9          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='orange')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'L%d'%(data_lf_contact[lf_contact_index_change[j-1]]),color='orange')


            # Plot Right Foot Contact 
            for j in range(1, len(rf_contact_index_change)):
                t_start = data_x[rf_contact_index_change[j-1]]
                t_end = data_x[rf_contact_index_change[j]]            
                val_start = data_rf_contact[rf_contact_index_change[j-1]]
                val_end = data_rf_contact[rf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0  + (1 - val_start)*ax1.get_ylim()[0]
                y_end = val_end * ax1.get_ylim()[1]/2.0  + (1 - val_end)*ax1.get_ylim()[0]          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'R%d'%(data_rf_contact[rf_contact_index_change[j-1]]),color='magenta')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1        

    # Use another row
    if use_custom_layout:
        col_index = 0
        row_index = 2

    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('World frame lfoot position (m)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_world_p_lfoot_pos[st_idx:end_idx,i-1], "b-")

        if plot_contact_switches:
            # Plot Left Foot Contact 
            for j in range(1, len(lf_contact_index_change)):
                t_start = data_x[lf_contact_index_change[j-1]]
                t_end = data_x[lf_contact_index_change[j]]            
                val_start = data_lf_contact[lf_contact_index_change[j-1]]
                val_end = data_lf_contact[lf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_start)*ax1.get_ylim()[0]*0.9
                y_end = val_end * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_end)*ax1.get_ylim()[0]*0.9          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='orange')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'L%d'%(data_lf_contact[lf_contact_index_change[j-1]]),color='orange')


            # Plot Right Foot Contact 
            for j in range(1, len(rf_contact_index_change)):
                t_start = data_x[rf_contact_index_change[j-1]]
                t_end = data_x[rf_contact_index_change[j]]            
                val_start = data_rf_contact[rf_contact_index_change[j-1]]
                val_end = data_rf_contact[rf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0  + (1 - val_start)*ax1.get_ylim()[0]
                y_end = val_end * ax1.get_ylim()[1]/2.0  + (1 - val_end)*ax1.get_ylim()[0]          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'R%d'%(data_rf_contact[rf_contact_index_change[j-1]]),color='magenta')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1  

    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))        
    fig.canvas.set_window_title('World frame rfoot position (m)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot(data_x, data_world_p_rfoot_pos[st_idx:end_idx,i-1], "b-")

        if plot_contact_switches:
            # Plot Left Foot Contact 
            for j in range(1, len(lf_contact_index_change)):
                t_start = data_x[lf_contact_index_change[j-1]]
                t_end = data_x[lf_contact_index_change[j]]            
                val_start = data_lf_contact[lf_contact_index_change[j-1]]
                val_end = data_lf_contact[lf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_start)*ax1.get_ylim()[0]*0.9
                y_end = val_end * ax1.get_ylim()[1]/2.0 * 0.9 + (1 - val_end)*ax1.get_ylim()[0]*0.9          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='orange')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'L%d'%(data_lf_contact[lf_contact_index_change[j-1]]),color='orange')


            # Plot Right Foot Contact 
            for j in range(1, len(rf_contact_index_change)):
                t_start = data_x[rf_contact_index_change[j-1]]
                t_end = data_x[rf_contact_index_change[j]]            
                val_start = data_rf_contact[rf_contact_index_change[j-1]]
                val_end = data_rf_contact[rf_contact_index_change[j]]            
                y_start = val_start * ax1.get_ylim()[1]/2.0  + (1 - val_start)*ax1.get_ylim()[0]
                y_end = val_end * ax1.get_ylim()[1]/2.0  + (1 - val_end)*ax1.get_ylim()[0]          

                # Plot Square Wave
                plt.plot([t_start, t_end, t_end], [y_start, y_start, y_end], color='magenta')
                plt.text((t_start+(t_end-t_start)/2.0), y_start,'R%d'%(data_rf_contact[rf_contact_index_change[j-1]]),color='magenta')

        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1               





if __name__ == "__main__":
    create_figures(subfigure_height=500, use_custom_layout=True)
    plt.show()
