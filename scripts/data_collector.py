#!/usr/bin/env python

# data_collector.py: Code to track AR marker with respect to Kinect and Baxter
# Author: Ravi Joshi
# Date: 2017/03/23

import rospy
import numpy as np
from baxter_interface import Limb
from ar_track_alvar_msgs.msg import AlvarMarkers
from baxter_controller import get_sensor_name
from std_srvs.srv import Trigger


class DataCollector():
    def __init__(self):
        # initialize ros node
        rospy.init_node('data_collector', anonymous=True)

        limb = rospy.get_param('~limb')
        self.max_samples = rospy.get_param('~max_samples')
        self.wait_time = rospy.get_param('~wait_time')
        self.data_dir = rospy.get_param('~data_dir')
        self.cam_image_topic = rospy.get_param('~cam_image_topic')

        self.markers = None
        self.baxter_arm_postion = None

        # create empty lists to save tracked data
        self.position_wrt_camera = []
        self.position_wrt_baxter = []

        self.baxter_arm = Limb(limb)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_callback)

    # write to tracking data to file
    def save_data(self):
        if not self.position_wrt_camera:
            rospy.logerr("Couldn't record any data. Run the program again.")
            return

        sensor_name = get_sensor_name(self.cam_image_topic)
        file_name = self.data_dir + "/baxter_" + sensor_name + "_position_ar.csv"
        trajectory = np.hstack(
            (np.array(self.position_wrt_baxter),  np.array(self.position_wrt_camera)))
        header = "baxter_x,baxter_y,baxter_z,camera_x,camera_y,camera_z"

        rospy.loginfo(
            'Saving collected data in following file: \n %s' % file_name)
        np.savetxt(file_name, trajectory, header=header,
                   delimiter=',', fmt='%.6f', comments='')

    def process_latest_data(self):
        marker_len = len(self.markers)

        if marker_len < 1:
            rospy.logwarn('No marker detected.')
            return False

        elif marker_len > 1:
            rospy.logwarn('Multiple markers detected')
            return False

        else:
            marker_position = self.markers[0]
            marker_position = [marker_position.x,
                               marker_position.y, 
                               marker_position.z]

            # ar marker frame wrt kinect
            self.position_wrt_camera.append(marker_position)

            # ar frame wrt baxter base since
            # ar marker is attached to the end-effector
            self.position_wrt_baxter.append(list(self.baxter_arm_postion))
            return True

    def marker_callback(self, data):
        detected_markers = [marker.pose.pose.position for marker in data.markers if marker.id is not 255]
        baxter_arm_postion = self.baxter_arm.endpoint_pose()['position']
        self.markers = detected_markers
        self.baxter_arm_postion = baxter_arm_postion

    def collect(self):
        rate = rospy.Rate(1.0 / self.wait_time)
        move_arm_service = '/move_arm_to_waypoint'

        rospy.loginfo('Waiting for service %s' % move_arm_service)
        rospy.wait_for_service(move_arm_service)
        rospy.loginfo('Service found %s' % move_arm_service)

        move_arm = rospy.ServiceProxy(move_arm_service, Trigger)

        keep_running = True
        while not rospy.is_shutdown() or keep_running:
            '''
            // -------------------------------------------------------------------- //
            // ------------------------------ STEP 1 ------------------------------ //
            // -------------------------------------------------------------------- //
            // move the baxter arm and wait for the arm to stop
            '''
            # call the service and wait for it. call again if failed
            service_response = move_arm()

            # the service has returned successfully
            rospy.logdebug('Service %s has returned successfully with the following message %s' % (
                move_arm_service, service_response.message))

            # exit and save the tracking data if the trajectory is finished
            if service_response.success:
                self.save_data()

                # stop the node by giving a message
                rospy.loginfo('Finished. Press Ctrl+C to exit.')
                keep_running = False
                break

            '''
            // -------------------------------------------------------------------- //
            // ------------------------------ STEP 2 ------------------------------ //
            // -------------------------------------------------------------------- //
            // wait for a while so that the baxter arm stops shaking
            '''
            rate.sleep()

            '''
            // -------------------------------------------------------------------- //
            // ------------------------------ STEP 3 ------------------------------ //
            // -------------------------------------------------------------------- //
            // start processing the latest ar marker data
            // grab the latest ar marker data. repeat it 'max_samples' times
            '''
            success_count = 0
            for _ in range(self.max_samples):
                success_count += self.process_latest_data()
                # wait for 'marker_callback()' to be called
                rospy.sleep(0.1)  # sleep for 0.1 second (100 ms)

            rospy.loginfo('Successfully detected ar marker %d times out of %d' % (
                success_count, self.max_samples))


if __name__ == '__main__':
    dc = DataCollector()
    dc.collect()
