#!/usr/bin/env python
# -*- coding: utf-8 -*-

# calibration_publisher.py: Code to publish calibration matrix
# Author: Ravi Joshi, Nishanth Koganti
# Date: 2017/11/07
# Source: https://github.com/osrf/baxter_demos/blob/master/scripts/get_ar_calib.py

# import modules
import rospy
import yaml
import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    # initialize ros node
    rospy.init_node('publish_calibration', anonymous=True)

    # load calibration file
    paramFile = rospy.get_param('~parameters_file')

    with open(paramFile, 'r') as f:
        params = yaml.load(f)

    # parameter initialization
    rot = params['rot']
    child = params['child']
    trans = params['trans']
    parent = params['parent']

    # create tf listener and broadcaster
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = parent
    static_transformStamped.child_frame_id = child

    static_transformStamped.transform.translation.x = trans[0]
    static_transformStamped.transform.translation.y = trans[1]
    static_transformStamped.transform.translation.z = trans[2]
    static_transformStamped.transform.rotation.x = rot[0]
    static_transformStamped.transform.rotation.y = rot[1]
    static_transformStamped.transform.rotation.z = rot[2]
    static_transformStamped.transform.rotation.w = rot[3]

    # Publish static transformation
    tf_broadcaster.sendTransform(static_transformStamped)
    rospy.spin()


if __name__ == '__main__':
    main()
