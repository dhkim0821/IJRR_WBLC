import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os

# Plot configuration
PLOT_VERTICALLY = 0
PLOT_HORIZONTALLY = 1

# number of figures in this plot
num_figures = 5

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index


    file_path = os.getcwd() + "/../../experiment_data_check/"

    ## read files
    data_bus_current = \
    np.genfromtxt(file_path+'bus_current.txt', delimiter=None, dtype=(float))
    data_bus_voltage = \
    np.genfromtxt(file_path+'bus_voltage.txt', delimiter=None, dtype=(float))
    data_x = np.genfromtxt(file_path+'time.txt', delimiter='\n', dtype=(float))

    data_bus_power = np.multiply(data_bus_current, data_bus_voltage)
    
    tot_len = len(data_x)
    num_joint = 6

    data_bus_tot_power = np.zeros((tot_len, 1))
    for i in range(0, tot_len, 1):
        for j in range(0, 5, 1):
            data_bus_tot_power[i, 0] += abs(data_bus_power[i, j])


    st_idx = 10
    end_idx = len(data_x) - 10
    data_x = data_x[st_idx:end_idx]

    
    ## plot bus current
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))    
    fig.canvas.set_window_title('bus (right_leg)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot( \
                data_x,  data_bus_current[st_idx:end_idx, i-1], "r-", \
                data_x, data_bus_voltage[st_idx:end_idx, i-1], "k-", \
                data_x, data_bus_power[st_idx:end_idx, i-1], "c-");
        # plt.legend(('command', 'pos'), loc='upper left')
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    ## plot bus current
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))       
    fig.canvas.set_window_title('bus (left_leg)')
    for i in range(1,4,1):
        ax1 = plt.subplot(3, 1, i)
        plt.plot( \
                data_x,  data_bus_current[st_idx:end_idx, i-1], "r-", \
                data_x, data_bus_voltage[st_idx:end_idx, i-1], "k-", \
                data_x, data_bus_power[st_idx:end_idx, i-1], "c-");
        plt.grid(True)
    plt.xlabel('time (sec)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1

    current_sum = data_bus_current[st_idx:end_idx, 0];
    for i in range(1,6,1):
        current_sum = current_sum + abs(data_bus_current[st_idx:end_idx, i]);

    # fig = plt.figure(3)
    # plt.get_current_fig_manager().window.wm_geometry("480x300+800+0")
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))    
    fig.canvas.set_window_title('sum of bus current')
    plt.plot(data_x, data_bus_tot_power[st_idx:end_idx], "r-")
    plt.xlabel('time (sec)')

if __name__ == "__main__":
    create_figures()
    plt.show()


