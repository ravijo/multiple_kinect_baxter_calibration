# multiple_kinect_baxter_calibration

```
rosrun multiple_kinect_baxter_calibration view_image _file:=scene.jpg
rosrun multiple_kinect_baxter_calibration view_cloud _file:=scene.pcd
rosrun multiple_kinect_baxter_calibration save_pcd _topic:="/kinect1/sd/points"
rosrun multiple_kinect_baxter_calibration view_cloud_realtime _topic:="/kinect1/sd/points"
rosrun multiple_kinect_baxter_calibration sphere_detector_test
roslaunch multiple_kinect_baxter_calibration calibration_data_collector.launch topic:=/kinect1/sd/points
roslaunch multiple_kinect_baxter_calibration calibration_compute.launch kinect:=kinect1
roslaunch multiple_kinect_baxter_calibration calibration_publisher.launch calibration:="[kinect1, kinect2, kinect3]"
```
For kinect_anywhere
```
roslaunch kinect_anywhere kinect_anywhere.launch color:=false body:=true pointcloud:=true kinect_frame:=kinect1_link
roslaunch multiple_kinect_baxter_calibration calibration_data_collector.launch topic:=/kinect_anywhere/point_cloud/points2
roslaunch multiple_kinect_baxter_calibration calibration_compute.launch kinect:=kinect_anywhere

Open file and make child: kinect1_link

roslaunch multiple_kinect_baxter_calibration calibration_publisher.launch calibration:="[kinect_anywhere]"
```

Installation
```
roscd multiple_kinect_baxter_calibration/scripts
chmod +x *.py
catkin_make
```
