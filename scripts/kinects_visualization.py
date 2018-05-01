#!/usr/bin/env python
# -*- coding: utf-8 -*-

# kinects_visualization.py: kinects visualization
# Author: Ravi Joshi
# Date: 2018/04/05

# import modules
import tf
import rospy
from math import pi
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Quaternion
from tf.transformations import quaternion_about_axis
from visualization_msgs.msg import Marker, MarkerArray


class KinectsVisualization():
    def __init__(self, base_frame_id, pc1_frame_id, pc2_frame_id, pc3_frame_id, colors, scale, freq):
        self.ns = 'kinects_visualization'

        self.base_frame_id = base_frame_id
        self.pc1_frame_id = pc1_frame_id
        self.pc2_frame_id = pc2_frame_id
        self.pc3_frame_id = pc3_frame_id

        # colors must be given in following order '[#FF0000, #00FF00, #0000FF]'
        kinect_colors = [self.hex_to_rgba(x.strip(' ')) for x in colors.lstrip(' [,').rstrip(']').split(', ')]

        kinect1 = self.create_kinect(0, kinect_colors[0], float(scale), self.pc1_frame_id)
        kinect2 = self.create_kinect(1, kinect_colors[1], float(scale), self.pc2_frame_id)
        kinect3 = self.create_kinect(2, kinect_colors[2], float(scale), self.pc3_frame_id)

        kinect_array = MarkerArray()
        kinect_array.markers.append(kinect1)
        kinect_array.markers.append(kinect2)
        kinect_array.markers.append(kinect3)

        rate = rospy.Rate(float(freq))

        kinect_pub = rospy.Publisher(self.ns, MarkerArray, queue_size=2)
        while not rospy.is_shutdown():
            kinect_pub.publish(kinect_array)
            rate.sleep()

    # src: https://stackoverflow.com/a/29643643/1175065
    def hex_to_rgba(self, hex_color):
        # remove hash sign from the beginning of color code
        hex_color = hex_color.lstrip('#')
        # we need colors in 0 to 1 range
        rgb_color = tuple(float(int(hex_color[i:i + 2], 16) / 255.0) for i in (0, 2, 4))
        return ColorRGBA(rgb_color[0], rgb_color[1], rgb_color[2], 1)

    def create_kinect(self, index, color, scale, frame_id):
        def rotate_about_y_axis():
            yaxis = [0, 1, 0]
            beta = pi / 2
            qy = quaternion_about_axis(beta, yaxis)
            return Quaternion(qy[0], qy[1], qy[2], qy[3])

        def create_marker(index, color, marker_type, time, frame_id):
            marker = Marker()
            marker.id = index
            marker.ns = self.ns
            marker.color = color
            marker.action = Marker.ADD
            marker.type = marker_type
            marker.header.stamp = time
            marker.header.frame_id = frame_id
            marker.lifetime = rospy.Duration(0)  # forever (static markers)
            return marker

        kinect = create_marker(index, color, Marker.CUBE, rospy.Time.now(), frame_id)
        # kinect v2 dimensions are 66 mm width *  43 mm height * 249 mm length
        kinect.scale = Vector3(scale * 0.066, scale * 0.043, scale * 0.249)
        kinect.pose.orientation = rotate_about_y_axis()
        return kinect


if __name__ == '__main__':
    # initialize ros node
    rospy.init_node('kinects_visualization', anonymous=True)

    base_frame_id = rospy.get_param('~base_frame_id')
    pc1_frame_id = rospy.get_param('~pc1_frame_id')
    pc2_frame_id = rospy.get_param('~pc2_frame_id')
    pc3_frame_id = rospy.get_param('~pc3_frame_id')
    colors = rospy.get_param('~colors')
    scale = rospy.get_param('~scale')
    freq = rospy.get_param('~freq')

    KinectsVisualization(base_frame_id, pc1_frame_id, pc2_frame_id, pc3_frame_id, colors, scale, freq)
