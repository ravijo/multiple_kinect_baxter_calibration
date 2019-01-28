#!/usr/bin/env python
# -*- coding: utf-8 -*-

# robot_controller.py: Baxter arm motion controller for calibration
# Author: Ravi Joshi
# Date: 2018/04/12

# import modules
import rospy
import numpy as np
from baxter_interface import Limb
from tf.transformations import quaternion_matrix
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from multiple_kinect_baxter_calibration.srv import GetEEPosition, GetEEPositionRequest, GetEEPositionResponse


class BaxterController():
    def __init__(self, limb_name, trajectory, offset, radius):
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
        self.move_arm_to_waypoint_service = rospy.Service('move_arm_to_waypoint',
                                                          Trigger,
                                                          self.handle_move_arm_to_waypoint)

        # define another service called 'get_ee_pose'
        self.get_ee_position_service = rospy.Service('get_ee_position',
                                                     GetEEPosition,
                                                     self.handle_get_ee_position_service)

        # flag to set when trajectory is finished
        self.trajectory_finished = False

        # define 4x4 transformation for marker wrt end-effector
        # considering no rotation in 'marker_wrt_ee' transformation matrix
        self.marker_wrt_ee = np.eye(4)
        self.marker_wrt_ee[2, -1] = offset + radius  # in z direction only

    def spin(self):
        # let the ros stay awake and serve the request
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.trajectory_finished:
            rate.sleep()

        # wait some time before stopping the node so that the service request returns if any
        rospy.sleep(1)
        self.get_ee_position_service.shutdown()
        self.move_arm_to_waypoint_service.shutdown()
        rospy.logdebug('Shutting down the baxter controller node')
        rospy.signal_shutdown('User requested')

    def handle_get_ee_position_service(self, request):
        # create a response object for the 'GetEEPose' service
        ee_pose = self.limb.endpoint_pose()

        # get rotation matrix from quaternion
        ee_wrt_robot = quaternion_matrix(ee_pose['orientation'])
        ee_wrt_robot[:-1, -1] = ee_pose['position']  # update the translation

        # marker wrt robot = marker_wrt_ee * ee_wrt_robot
        marker_wrt_robot = ee_wrt_robot.dot(self.marker_wrt_ee)
        response = GetEEPositionResponse()
        response.position.x = marker_wrt_robot[0, -1]
        response.position.y = marker_wrt_robot[1, -1]
        response.position.z = marker_wrt_robot[2, -1]
        return response

    def handle_move_arm_to_waypoint(self, request):
        # create a response object for the trigger service
        response = TriggerResponse()

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


def get_sensor_name(topic):
    '''
    # get the sensor_name as the first word between two leftmost slash chacters
    '''

    import re
    all_slash = [m.start() for m in re.finditer('/', topic)]
    sensor_name = topic[all_slash[0] + 1: all_slash[1]]
    return sensor_name


def read_csv(file_name):
    '''
    # read csv file. return data and header separately
    '''

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
    offset = rospy.get_param('~offset')
    radius = rospy.get_param('~radius')

    trajectory, header = read_csv(file_name)

    # check if wrong trajectory file is provided
    if not all(x.startswith(limb) for x in header):
        rospy.logerr("Provided limb '%s' doesn't match with trajectory file. Trajectory file: \n'%s'" % (
            limb, file_name))
    else:
        controller = BaxterController(limb, trajectory, offset, radius)
        controller.spin()
