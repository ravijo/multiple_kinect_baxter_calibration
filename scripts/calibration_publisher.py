#!/usr/bin/env python
# -*- coding: utf-8 -*-

# calibration_publisher.py: Code to publish calibration matrix
# Author: Nishanth Koganti, Ravi Joshi
# Date: 2017/11/07
# Source: https://github.com/osrf/baxter_demos/blob/master/scripts/get_ar_calib.py

# import modules
import os
import rospy
import yaml
import tf2_ros
from geometry_msgs.msg import TransformStamped

def get_kinects_from_data(data):
    kinects = [x.strip(' ') for x in data.lstrip(' [,').rstrip(']').split(', ')]
    return kinects

def main():
    # initialize ros node
    rospy.init_node('publish_calibration', anonymous=True)

    calibration = rospy.get_param('~calibration')
    calibration = get_kinects_from_data(calibration)
    data_dir = rospy.get_param('~data_dir')

    # create tf listener and broadcaster
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    all_transformations = []
    for calib in calibration:
        # load calibration file
        param_file = os.path.join(data_dir, ('baxter_%s_calibration.yaml' % calib))
        rospy.loginfo('Reading file:\n%s\n' % param_file)

        with open(param_file, 'r') as f:
            params = yaml.load(f)

        # parameter initialization
        rot = params['rot']
        child = params['child']
        trans = params['trans']
        parent = params['parent']

        static_transform = TransformStamped()

        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = parent
        static_transform.child_frame_id = child

        static_transform.transform.translation.x = trans[0]
        static_transform.transform.translation.y = trans[1]
        static_transform.transform.translation.z = trans[2]
        static_transform.transform.rotation.x = rot[0]
        static_transform.transform.rotation.y = rot[1]
        static_transform.transform.rotation.z = rot[2]
        static_transform.transform.rotation.w = rot[3]

        all_transformations.append(static_transform)

    # src: https://answers.ros.org/question/261815
    # Publish static transformation
    tf_broadcaster.sendTransform(all_transformations)
    rospy.spin()


if __name__ == '__main__':
    main()
