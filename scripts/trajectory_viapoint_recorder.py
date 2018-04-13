#!/usr/bin/env python
# -*- coding: utf-8 -*-

# trajectory_viapoint_recorder.py: records trajectory viapoint into csv file
# Author: Ravi Joshi
# Date: 2018/04/12

# import modules
import rospy
import numpy as np
from std_msgs.msg import Bool
from baxter_interface import Limb


class TrajectoryViapointRecorder():
    def __init__(self, limb, file_name):
        arm = Limb(limb)
        self.trajectory = []
        self.file_name = file_name
        self.file_header = arm.joint_names()

        rospy.on_shutdown(self.save_data)

        while not rospy.is_shutdown():
            raw_input('Press Enter to record this point...')
            joint_angles_dict = arm.joint_angles()
            joint_angles = [joint_angles_dict[joint_name]
                            for joint_name in self.file_header]
            self.trajectory.append(joint_angles)
            rospy.loginfo('%s via-points collected.' % len(self.trajectory))

    def save_data(self):
        file_header = ','.join(x for x in self.file_header)
        np.savetxt(self.file_name, self.trajectory, header=file_header,
                   delimiter=',', fmt='%.4f', comments='')
        rospy.loginfo(
            'Trajectory via-points have been successfully saved to %s' % self.file_name)


if __name__ == '__main__':
    rospy.init_node('trajectory_viapoint_recorder_node', anonymous=True)
    limb = 'right'
    file_name = 'viapoints'
    TrajectoryViapointRecorder(limb, file_name)
