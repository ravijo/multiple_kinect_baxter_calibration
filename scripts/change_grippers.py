#!/usr/bin/env python
# -*- coding: utf-8 -*-

# change_grippers.py: change grippers to the one which we are using now
# Author: Ravi Joshi
# Date: 2017/11/17

# import modules
import os
import rospy
import signal
import subprocess

sleep_duration = 4  # seconds


def main():
    # initialize ros node
    rospy.init_node('change_grippers', anonymous=True)

    rospy.loginfo('Please wait for sometime. Change grippers process should take %s seconds approximately.' % (
        sleep_duration * 2))

    left_cmd = 'rosrun baxter_examples send_urdf_fragment.py -f $(rospack find baxter_description)/urdf/left_end_effector.urdf.xacro -l left_hand -j left_gripper_base'
    right_cmd = 'rosrun baxter_examples send_urdf_fragment.py -f $(rospack find baxter_description)/urdf/right_end_effector.urdf.xacro -l right_hand -j right_gripper_base'

    all_cmd = [left_cmd, right_cmd]

    # source: https://stackoverflow.com/a/4791612
    for cmd in all_cmd:
        p = subprocess.Popen(
            cmd, shell=True, stderr=subprocess.PIPE, preexec_fn=os.setsid)
        # let the parameters reach to baxter controller
        rospy.sleep(sleep_duration)
        # Send the signal to all the process groups
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)

    rospy.loginfo('Change grippers finished.')


if __name__ == '__main__':
    main()
