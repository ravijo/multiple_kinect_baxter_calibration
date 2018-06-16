# multiple_kinect_baxter_calibration
This repository contains code for autonomous calibration of Baxter robot and Kinect camera. It supports multiple Kinect sensors.

## Dependencies
* [Baxter SDK](https://github.com/RethinkRobotics/baxter)
  * Steps to install Baxter SDK can be found [here](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)
* [iai_kinect2](https://github.com/code-iai/iai_kinect2)
  * Tools for using the Kinect v2 in ROS
* [kinect_anywhere](https://github.com/ravijo/kinect_anywhere)
  * If you want to use Kinect v2 on windows 

## Installation
1. Download or clone the repository to the source directory of ROS workspace
1. Compile the workspace by using `catkin_make`
1. Mark the python scripts executable by using command below-
```
roscd multiple_kinect_baxter_calibration/scripts
chmod +x *.py
```
## Steps for calibration
There are following three steps of the calibration process-
1. Place the marker on Baxter arm
1. Define a trajectory of Baxter arm
1. Collect the data
1. Compute the calibration
1. Publish the calibration

Below are the details of each step.

### Place the marker on Baxter arm
We are using a green colored sphere as a marker as shown below-
![setup](files/docs/setup.jpg)

### Define a trajectory of Baxter arm
Each Baxter Kinect setup varies due to the the location of camera. Hence, prior to data collection step, we must need to define a trajectory of Baxter arm by following steps mentioned below-
* Record way-points of the arm trajectory by executing the following script. 
```
rosrun multiple_kinect_baxter_calibration trajectory_waypoints_recorder.py _file:=baxter.csv _limb:=right
```
* Make sure to enable the [Zero-G mode](http://sdk.rethinkrobotics.com/wiki/Zero-G_Mode) so that the arm can be moved easily to any location by grasping the cuff over its groove. 
* Press the `Baxter` button on the arm in order to record the way-point.
* Record 20 different way-points. Press <kbd>CTRL</kbd>+<kbd>C</kbd> to stop the recording process.

### Collect the data
* Start the kinect by using following command-
  * For iai_kinect2: `roslaunch kinect2_bridge kinect2_bridge.launch`
  * For kinect_anywhere: `roslaunch kinect_anywhere kinect_anywhere.launch pointcloud:=true kinect_frame:=kinect2_link`
* Start collecting the data by using following command-
```
roslaunch multiple_kinect_baxter_calibration calibration_data_collector.launch topic:=/kinect2/sd/points kinect2_trajectory:=/home/ravi/ros_ws/src/multiple_kinect_baxter_calibration)/files/baxter.csv
```

### Compute the calibration
```
roslaunch multiple_kinect_baxter_calibration calibration_compute.launch kinect:=kinect2
```

### Publish the calibration
```
roslaunch multiple_kinect_baxter_calibration calibration_publisher.launch calibration:="[kinect2]"
```

## Other utility files
### view_cloud_realtime
To view the point cloud data in real-time
```
rosrun multiple_kinect_baxter_calibration view_cloud_realtime _topic:="/kinect2/sd/points"
```
### save_pcd
To save the point cloud data in a PCD file
```
rosrun multiple_kinect_baxter_calibration save_pcd _topic:="/kinect2/sd/points"
```
### view_pcd
To visualize the stored point cloud file
```
rosrun multiple_kinect_baxter_calibration view_pcd _file:=scene.pcd
```
Please press <kbd>j</kbd> to take screenshot of the current scene.

### segment_image
To find out the HSV range of the colored marker in the image
```
rosrun multiple_kinect_baxter_calibration segment_image _file:=scene.jpg
```
### view_image
To find out the RGB and HSV value of any pixel in the image
```
rosrun multiple_kinect_baxter_calibration view_image _file:=scene.jpg
```
## sphere_detector_test
To test whether sphere segmentation is working or not
```
rosrun multiple_kinect_baxter_calibration sphere_detector_test _file:=scene.pcd
```

## For three Kinects running on iai_kinect2
1. Initialize project and start all the kinects
```
roslaunch multiple_kinect_baxter_calibration init.launch
```
2. Collect the data for `kinect1`
```
roslaunch multiple_kinect_baxter_calibration calibration_data_collector.launch topic:=/kinect1/sd/points
```
3. Compute the calibration for `kinect1`
```
roslaunch multiple_kinect_baxter_calibration calibration_compute.launch kinect:=kinect1
```
4. Repeate steps 2 and 3 for `kinect2` and `kinect3`
5. Publish the calibration data
```
roslaunch multiple_kinect_baxter_calibration calibration_publisher.launch calibration:="[kinect1, kinect2, kinect3]"
```

## For kinect_anywhere
```
roslaunch kinect_anywhere kinect_anywhere.launch color:=false body:=true pointcloud:=true kinect_frame:=kinect1_link
roslaunch multiple_kinect_baxter_calibration calibration_data_collector.launch topic:=/kinect_anywhere/point_cloud/points2
roslaunch multiple_kinect_baxter_calibration calibration_compute.launch kinect:=kinect_anywhere
```
Open calibration file and modify child to `kinect1_link`
```
roslaunch multiple_kinect_baxter_calibration calibration_publisher.launch calibration:="[kinect_anywhere]"
```

 
