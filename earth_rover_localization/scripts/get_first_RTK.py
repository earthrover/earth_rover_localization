#!/usr/bin/env python3

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
        sub = rospy.Subscriber('/piksi_receiver/navsatfix_best_fix', NavSatFix, callback)

    def callback(self, msg):
        text_file = open(self.opath/+"enu_origin.yaml", "w")
        text_file.write("latitude0_deg: " + msg.latitude)
        text_file.close()

        text_file = open(self.opath/+"enu_origin.yaml", "a")
        text_file.write("longitude0_deg: " + msg.longitude)
        text_file.write("altitude0: " + msg.altitude)
        text_file.close()

    # While loop
    def main(self):
        try:
            rospy.spin()
        except (rospy.ROSInterruptException):
            pass

if __name__ == "__main__":

    # get passed arguments:
    input_args = rospy.myargv(argv=sys.argv)
    output_path = input_args[1]

    # init ros node
    rospy.init_node('first_RTK')
    listen_rtk = FirstRTK(opath=output_path)

    listen_rtk.main()
