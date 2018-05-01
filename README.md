# multiple_kinect_baxter_calibration

```
rosrun multiple_kinect_baxter_calibration view_image _file:=scene.jpg
rosrun multiple_kinect_baxter_calibration view_cloud _file:=scene.pcd
rosrun multiple_kinect_baxter_calibration save_pcd _topic:="/kinect1/sd/points"
rosrun multiple_kinect_baxter_calibration view_cloud_realtime _topic:="/kinect1/sd/points"
rosrun multiple_kinect_baxter_calibration sphere_detector_test
roslaunch multiple_kinect_baxter_calibration calibration_data_collector.launch topic:=/kinect1/sd/points
roslaunch multiple_kinect_baxter_calibration calibration_publisher.launch calibration:="[kinect1, kinect2, kinect3]"
```

Installation
```
roscd multiple_kinect_baxter_calibration/scripts
chmod +x *.py
catkin_make
```
