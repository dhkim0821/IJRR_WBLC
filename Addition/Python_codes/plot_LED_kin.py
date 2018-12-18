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

def create_figures(subfigure_width=480, subfigure_height=600, starting_figure_no=1, starting_col_index = 0, starting_row_index=0, plot_configuration=PLOT_HORIZONTALLY):
    figure_number = starting_figure_no
    col_index = starting_col_index
    row_index = starting_row_index

    ## read files --------------------------------------------------------------------
    file_path = os.getcwd() + "/../../experiment_data_check/"

    data_LED = \
            np.genfromtxt(file_path+'LED_Pos.txt', delimiter=None, dtype=(float))
    data_LED_kin = \
            np.genfromtxt(file_path+'LED_Kin_Pos.txt', delimiter=None, dtype=(float))
    st_idx = 4000;
    end_idx = len(data_LED) - 1000
    # Select LED 
    # (0, 1, 2): Body
    # (3, 4, 5, 6, 7): Right Leg
    # (8, 9, 10, 11, 12): Left Leg
    # LED_idx = [3, 4]; # right thigh
    # LED_idx = [5, 6, 7]; # right shank
    # LED_idx = [8, 9]; #left thigh
    LED_idx = [10, 11, 12]; #left shank
    origin_x_offset_LED = 0.075    
    # Plot Figure --------------------------------------------------------------------
    ## plot X (YZ plane)
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    #plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
    fig.canvas.set_window_title('YZ plane (X axis)')
    for i in LED_idx:
        plt.plot(data_LED[st_idx:end_idx, 3*i + 1], data_LED[st_idx:end_idx, 3*i+2], 'r*')
        plt.plot(data_LED_kin[st_idx:end_idx, 3*i + 1], data_LED_kin[st_idx:end_idx, 3*i+2], 'b*')
        # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
    plt.xlabel('Y (m)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
    #----------------------------------------------------------------------------------

    # Plot Figure --------------------------------------------------------------------
    ## plot Y axis (XZ plane)
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    #plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
    fig.canvas.set_window_title('XZ plane (Y axis)')
    for i in LED_idx:
        plt.plot(data_LED[st_idx:end_idx, 3*i + 0] + origin_x_offset_LED, data_LED[st_idx:end_idx, 3*i+2], 'r*')
        plt.plot(data_LED_kin[st_idx:end_idx, 3*i + 0], data_LED_kin[st_idx:end_idx, 3*i+2], 'b*')
        # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
    plt.xlabel('X(m)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
    #----------------------------------------------------------------------------------

    # Plot Figure --------------------------------------------------------------------
    ## plot Z axis (XY plane)
    fig = plt.figure(figure_number)
    plt.get_current_fig_manager().window.wm_geometry(str(subfigure_width) + "x" + str(subfigure_height) +  "+" + str(subfigure_width*col_index) + "+" + str(subfigure_height*row_index))
    #plt.get_current_fig_manager().window.wm_geometry("480x600+0+0")
    fig.canvas.set_window_title('XY plane (Z axis)')
    for i in LED_idx:
        plt.plot(data_LED[st_idx:end_idx, 3*i + 0] + origin_x_offset_LED, data_LED[st_idx:end_idx, 3*i+1], 'r*')
        plt.plot(data_LED_kin[st_idx:end_idx, 3*i + 0], data_LED_kin[st_idx:end_idx, 3*i+1], 'b*')
        # plt.legend(('command', 'pos'), loc='upper left')
    plt.grid(True)
    plt.xlabel('X (m)')
    ## increment figure number and index
    figure_number += 1
    if plot_configuration == PLOT_HORIZONTALLY:
        col_index += 1
    elif plot_configuration == PLOT_VERTICALLY:
        row_index +=1
    #----------------------------------------------------------------------------------


if __name__ == "__main__":
    create_figures()
    plt.show()

