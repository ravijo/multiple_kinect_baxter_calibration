#!/usr/bin/env python
# -*- coding: utf-8 -*-

# baxter_controller.py: baxter arm  motion controller for calibration
# Author: Ravi Joshi
# Date: 2018/04/12

# import modules
import rospy
import numpy as np
from baxter_interface import Limb
from multiple_kinect_baxter_calibration.srv import move_arm_to_waypoint

class BaxterController():
    def __init__(self, limb_name, trajectory):
        # crate a limb instance control the baxter arm
        self.limb = Limb(limb_name)

        # numpy joint angle trajectory of nx7 shape
        self.trajectory = trajectory

        # index variable to keep track of current row in numpy array
        self.trajectory_index = 0

        # store the joint names since
        # it will be used later while commanding the limb
        self.joint_names = self.limb.joint_names()

        # define a service called 'move_arm_to_waypoint'
        service = rospy.Service('move_arm_to_waypoint', move_arm_to_waypoint, self.handle_move_arm_to_waypoint)

        # let the ros stay awake and serve the request
        rospy.spin()

    def handle_move_arm_to_waypoint(self, request):
        # check if the trajectory finished
        trajectory_finished = self.trajectory_index >= self.trajectory.shape[0]

        # if trajectory isn't finished
        if not trajectory_finished:
            # get the latest joint values from given trajectory
            joint_values = trajectory[self.trajectory_index, :]

            # create a dictionary of joint names and values
            joint_command = dict(zip(self.joint_names, joint_values))

            # move the limb to given joint angle
            self.limb.move_to_joint_positions(joint_command)

            # increment the counter
            self.trajectory_index += 1
        return trajectory_finished

# get the sensor_name as the first word between two leftmost slash chacters
def get_sensor_name(topic):
    import re
    all_slash = [m.start() for m in re.finditer('/', topic)]
    sensor_name = topic[all_slash[0] + 1 : all_slash[1]]
    return sensor_name

# read csv file. return data and header separately
def read_csv(file_name):
    # load the data files
    data = np.genfromtxt(file_name, delimiter=',', names=True)

    # get single header string
    header = data.dtype.names

    # convert structured array to numpy array
    data = data.view(np.float).reshape(data.shape + (-1,))
    return data, header

if __name__ == '__main__':
    rospy.init_node('baxter_controller_node', anonymous=True)

    # get the parameters
    limb = rospy.get_param('~limb')
    topic = rospy.get_param('~topic')
    file_name = rospy.get_param('~%s_trajectory' % get_sensor_name(topic))
    trajectory, header = read_csv(file_name)

    # check if wrong trajectory file is provided
    if not all(x.startswith(limb) for x in header):
        rospy.logerr("Provided limb '%s' doesn't match with trajectory file. Trajectory file: \n'%s'" % (limb, file_name))
    else:
        BaxterController(limb, trajectory)
