#!/bin/bash

rosrun earth_rover_localization get_first_RTK.py $(rospack find earth_rover_localization)/cfg
roslaunch earth_rover_localization start_localization_node.launch
