#!/usr/bin/env python

import rospy
import time
import os
from sensor_msgs.msg import PointCloud2

class Monitor:
    def __init__(self):
        self.c1_msgs = 0
        self.c2_msgs = 0
        rospy.init_node('realsense_reboot_test')
        self.logfile = os.environ['HOME']+'/'+rospy.get_param("~logfile")
        if os.path.isfile(self.logfile) and (not os.access(self.logfile, os.W_OK)):
            rospy.logerr("Do not have write previleges on logfile: %s",self.logfile)
            exit(-1)
        # Maximum number of iterations to consider
        self.itr_limit = rospy.get_param("~itreration_limit")
        # Number of lines written to file indicate the current iteration
        self.current_itr = 1
        if os.path.isfile(self.logfile):
            with open(self.logfile,'r') as lf:
                for line in lf:
                    self.current_itr += 1
        self.sleep_time = rospy.get_param("~sleep",15)*60 # sleep param is in minutes
        self.timeout = rospy.get_param("~timeout",10)*60 # timeout param is in minutes
        self.start_t = time.time()
        self.strobe_limit = 10
        rospy.Subscriber("depth_cloud_cam1", PointCloud2, self.cam1_callback)
        rospy.Subscriber("depth_cloud_cam2", PointCloud2, self.cam2_callback)

    def cam1_callback(self,data):
        if "optical_frame" in data.header.frame_id:
            self.c1_msgs += 1

    def cam2_callback(self,data):
        if "optical_frame" in data.header.frame_id:
            self.c2_msgs += 1

    def monitor(self):
        # If number of lines exceed the iteration limit, exit
        if self.current_itr>self.itr_limit:
            return
        # Check for minimum number of messages set by strobe_limit until timeout
        while not rospy.is_shutdown():
            if self.c1_msgs>self.strobe_limit and self.c2_msgs>self.strobe_limit:
                # Breaking off success
                break
            if time.time()-self.start_t > self.timeout:
                # Breaking off faliure
                break
            time.sleep(1)
        # Compute and append the result to a logfile
        success = self.c1_msgs>self.strobe_limit and self.c2_msgs>self.strobe_limit
        with open(self.logfile,'a+') as lf:
            lf.write("Result (%d) : %s : %d,%d\n"%(self.current_itr, 'Success' if success else 'Failed', self.c1_msgs, self.c2_msgs))

        # Sleep a certain amount of time before rebooting the system
        time.sleep(self.sleep_time)

        # Reboot the system
        os.system('sudo reboot')

if __name__ == '__main__':
    m = Monitor()
    m.monitor()
