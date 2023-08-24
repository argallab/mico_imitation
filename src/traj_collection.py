#!/usr/bin/python

import rospy
import rospy.rostime as rostime
from geometry_msgs.msg import Pose, Point, Quaternion
import threading

import tf2_ros
import numpy as np
npa = np.array

class CollectNode:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = True
        self.runningCV = threading.Condition()

        self.rate = rospy.Rate(10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pub = rospy.Publisher('eef_pose', Pose, queue_size=10)

        self.eef_position = npa([0]*3, dtype ='f')
        self.eef_quat = npa([0]*4, dtype='f')

        self.model_source_frameid = "mico_link_base"
        self.model_target_frameid = "mico_end_effector"

        tfpose = self.buffer.lookup_transform(self.model_source_frameid, self.model_target_frameid, rospy.Time(0), rospy.Duration(10))
        self.eef_position = npa([tfpose.transform.translation.x, tfpose.transform.translation.y, tfpose.transform.translation.z], dtype = 'f')
        self.eef_quat = npa([tfpose.transform.rotation.x, tfpose.transform.rotation.y, tfpose.transform.rotation.z, tfpose.transform.rotation.w], dtype='f')

    def update_current_position(self):
        # (self.eef_position, self.eef_quat) = self.listener.lookupTransform(self.model_source_frameid,self.model_target_frameid, rostime.Time(0))
        # self.eef_position = npa(self.eef_position)
        # self.eef_quat = npa(self.eef_quat)

        tfpose = self.buffer.lookup_transform(self.model_source_frameid, self.model_target_frameid, rospy.Time(0), rospy.Duration(10))
        self.eef_position = npa([tfpose.transform.translation.x, tfpose.transform.translation.y, tfpose.transform.translation.z], dtype = 'f')
        # print "CURR POSITION ", self.eef_position
        self.eef_quat = npa([tfpose.transform.rotation.x, tfpose.transform.rotation.y, tfpose.transform.rotation.z, tfpose.transform.rotation.w], dtype='f')
        self.pub.publish(Point(self.eef_position[0], self.eef_position[1], self.eef_position[2]), Quaternion(self.eef_quat[0], self.eef_quat[1], self.eef_quat[2], self.eef_quat[3]))

    def spin(self):
        rospy.loginfo("Running!")
        try:
            while not rospy.is_shutdown():
                self.runningCV.acquire()
                if self.running:
                    self.step()
                    self.rate.sleep()
                else:
                    self.runningCV.wait(1.0)
                self.runningCV.release()

        except KeyboardInterrupt:
            # self.file_handle.close()
            rospy.logdebug('Keyboard interrupt, shutting down')
            rospy.core.signal_shutdown('Keyboard interrupt')
			
    def step(self):
        self.update_current_position()

if __name__ == '__main__':
	rospy.init_node("mico_collect_node")
	n =  CollectNode()
	n.spin() # might have to switch this out with "b.spin() which keep the thread alive. Similar SEDS blend node. "

