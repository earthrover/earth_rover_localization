#!/usr/bin/env python

# This node waits a specified wait_time once it receives a GNNS msg and makes sure RTK fix is available to launch autonomous_rover.launcg
# Author: Xavier Ruiz
# Email: xavier.ruiz@earthrover.farm

import os
import rospy
from sensor_msgs.msg import NavSatFix

def init():
    count = 0
    rtk_fix = False
    # init ros node
    rospy.init_node('wait_RTK_fix')

    # wait for GNSS convergence
    wait_time = rospy.get_param('/wait_RTK_fix/wait_localization', '10')
    msg = rospy.wait_for_message('/piksi_receiver/navsatfix_best_fix', NavSatFix, timeout=None)
    rospy.sleep(float(wait_time))

    while not rtk_fix:
        msg = rospy.wait_for_message('/piksi_receiver/navsatfix_best_fix', NavSatFix, timeout=None)
        if msg.position_covariance[0] == 0.0049:
            count += 1
            print(count)
        if count == 10:
            rtk_fix = True
    autonomousLaunch()


def autonomousLaunch():
    HOME = os.getenv('HOME')
    cmd = "roslaunch" + " " + HOME + "/earth_rover_ws/src/earth_rover_ugv/earth_rover_autonomous_rover/launch/autonomous_rover.launch"
    reason = "Got RTK fix"
    os.system(cmd);
    rospy.signal_shutdown(reason)

if __name__ == '__main__':
    init()
