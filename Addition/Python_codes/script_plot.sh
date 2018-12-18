#!/bin/bash
plot_these=(plot_joint 
	plot_body 
	plot_rfoot
	plot_lfoot
    plot_torque)

python plot_multiple.py  ${plot_these[*]}
