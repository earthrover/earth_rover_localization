#!/usr/bin/env python

################################################################
# Copyright (C) 2019 Earth Rover Limited. All rights reserved. #
################################################################

from __future__ import print_function
import sys
import os

import rospy
from sensor_msgs.msg import NavSatFix

class FirstRTK():
    def __init__(self, opath):
        self.opath = opath
        # Subscribe once to RTK topic and get first msg
        msg = rospy.wait_for_message('/piksi_receiver/navsatfix_best_fix', NavSatFix, timeout=None)
        
        # Write GPS corrdinates to yaml cfg file
        text_file = open(self.opath+"/enu_origin.yaml", "w")
        text_file.write("latitude0_deg: " + str(msg.latitude) + '\r\n')
        text_file.close()

        text_file = open(self.opath+"/enu_origin.yaml", "a")
        text_file.write("longitude0_deg: " + str(msg.longitude) + '\r\n')
        text_file.write("altitude0: " + str(msg.altitude) + '\r\n')
        text_file.close()

if __name__ == "__main__":
    
    # get passed arguments:
    input_args = rospy.myargv(argv=sys.argv)
    output_path = input_args[1]

    # init ros node
    rospy.init_node('first_RTK')
    listen_rtk = FirstRTK(opath=output_path)
