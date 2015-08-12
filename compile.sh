#!/bin/bash
# what else?
clear
printf "\nCompiling\n\n"

g++ main.cpp joystick_input.c globals.c galil_control.cpp joystick_command.cpp user_interface.cpp visual_feedback.cpp socket_communication.cpp force_feedback.cpp atinano.c kinematics_staubli.cpp motion_controller.cpp camera_calibration.cpp sleep_time.c -lGalil -lm -lrt -lnative -lrtdk -lcomedi -latift-0.1 -lgsl -lgslcblas -Wall -O2 `xeno-config --skin=native --cflags` `xeno-config --skin=native --ldflags` `pkg-config --cflags --libs opencv`

