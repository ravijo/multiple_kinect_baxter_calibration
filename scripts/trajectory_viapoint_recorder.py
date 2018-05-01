#!/usr/bin/env python
# -*- coding: utf-8 -*-

# trajectory_viapoint_recorder.py: records trajectory viapoint into csv file
# Author: Ravi Joshi
# Date: 2018/04/12

# import modules
import rospy
import numpy as np
from baxter_interface import Limb, Navigator

class TrajectoryViapointRecorder():
    def __init__(self, limb_name, file_name):
        self.limb = Limb(limb_name)
        self.trajectory = []
        self.file_name = file_name
        self.file_header = self.limb.joint_names()

        self.dt = rospy.Duration(secs=2)
        self.previous_time = rospy.Time.now()

        self.limb_nav = Navigator(limb_name)
        self.limb_nav.button2_changed.connect(self.limb_nav_button_pressed)

        rospy.on_shutdown(self.save_data)
        rospy.spin()

    def save_data(self):
        file_header = ','.join(x for x in self.file_header)
        np.savetxt(self.file_name, self.trajectory, header=file_header,
                   delimiter=',', fmt='%.4f', comments='')
        rospy.loginfo('Trajectory have been successfully saved to %s' % self.file_name)

    def limb_nav_button_pressed(self, state):
        now = rospy.Time.now()
        if (now - self.previous_time > self.dt) and state:
            self.limb_nav.inner_led = state
            self.record()
        elif not state:
            self.limb_nav.inner_led = state
            self.previous_time = now

    def record(self):
        joint_angles_dict = self.limb.joint_angles()
        joint_angles = [joint_angles_dict[joint_name]
                        for joint_name in self.file_header]
        self.trajectory.append(joint_angles)
        rospy.loginfo('%d samples collected.' % len(self.trajectory))


if __name__ == '__main__':
    rospy.init_node('trajectory_viapoint_recorder_node', anonymous=True)
    limb = rospy.get_param('~limb', 'right')
    file_name = rospy.get_param('~file', None)

    if file_name is None:
        rospy.logerr('File name is not provided. Please use _file:=baxter.csv')
    else:
        rospy.loginfo("Limb: '%s', file name: '%s'" % (limb, file_name))
        TrajectoryViapointRecorder(limb, file_name)
