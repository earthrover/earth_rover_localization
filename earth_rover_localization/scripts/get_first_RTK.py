#!/usr/bin/env python

################################################################
# Copyright (C) 2019 Earth Rover Limited. All rights reserved. #
################################################################

from __future__ import print_function
import sys
import os

import rospy
from sensor_msgs.msg import NavSatFix

# init ros node
rospy.init_node('first_RTK')

opath = sys.argv[1]

# Subscribe once to RTK topic and get first msg
msg = rospy.wait_for_message('/piksi_receiver/navsatfix_best_fix', NavSatFix, timeout=None)

# Write GPS corrdinates to yaml cfg file
text_file = open(opath+"/datum.yaml", "w")
text_file.write("datum: [" + str(msg.latitude) + ", " + str(msg.longitude) + ", 0]")
text_file.close()

text_file = open(opath+"/local_xy_origins.yaml", "w")
text_file.write("local_xy_origins: [{name: swri, latitude: " + str(msg.latitude) + ", longitude: " + str(msg.longitude) + ", altitude: " + str(msg.altitude) + ", heading: 0}]")
text_file.close()
