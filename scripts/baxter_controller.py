#!/usr/bin/env python
# -*- coding: utf-8 -*-

# baxter_controller.py: baxter arm  motion controller for calibration
# Author: Ravi Joshi
# Date: 2018/04/12

# import modules
import rospy
import numpy as np
import std_srvs.srv
from baxter_interface import Limb

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
        self.service = rospy.Service('move_arm_to_waypoint', std_srvs.srv.Trigger, self.handle_move_arm_to_waypoint)

        # flag to set when trajectory is finished
        self.trajectory_finished = False

    def spin(self):
        # let the ros stay awake and serve the request
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.trajectory_finished:
            rate.sleep()

        # wait some time before stopping the node so that the service request returns if any
        rospy.sleep(1)
        self.service.shutdown()
        rospy.logdebug('Shutting down the baxter controller node')
        rospy.signal_shutdown('User requested')

    def handle_move_arm_to_waypoint(self, request):
        # create a response object for the trigger service
        response = std_srvs.srv.TriggerResponse()

        # check if the trajectory finished
        trajectory_finished = self.trajectory_index >= self.trajectory.shape[0]

        # if trajectory isn't finished
        if not trajectory_finished:
            # get the latest joint values from given trajectory
            joint_values = trajectory[self.trajectory_index, :]

            # create command, i.e., a dictionary of joint names and values
            command = dict(zip(self.joint_names, joint_values))

            # move the limb to given joint angle
            try:
                self.limb.move_to_joint_positions(command)
                response.message = 'Successfully moved arm to the following waypoint %s' % command
            except rospy.exceptions.ROSException:
                response.message = 'Error while moving arm to the following waypoint %s' % command
            finally:
                # increment the counter
                self.trajectory_index += 1
        else:
            response.message = 'Arm trajectory is finished already'

        # set the success parameter of the response object
        response.success = trajectory_finished

        # set the flag just before returning from the function so that it is
        # almost certain that the service request is returned successfully
        self.trajectory_finished = trajectory_finished
        return response

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
        controller = BaxterController(limb, trajectory)
        controller.spin()
