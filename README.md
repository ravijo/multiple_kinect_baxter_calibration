# multiple_kinect_baxter_calibration
This repository contains code for calibration of Baxter robot and Kinect camera. It supports multiple Kinect sensors. It can be used (after minor modification) in other ROS supported robots as well.

## Dependencies
* [Baxter SDK](https://github.com/RethinkRobotics/baxter)
  * Steps to install Baxter SDK can be found [here](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)
* [ar_track_alvar](http://wiki.ros.org/ar_track_alvar)
  * Use following command to install ar_track_alvar package `sudo apt-get install ros-indigo-ar-track-alvar`
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
1. Record way-points of the arm trajectory by executing the following script. 
```
rosrun multiple_kinect_baxter_calibration trajectory_waypoints_recorder.py _file:=baxter.csv _limb:=right
```
Following are the valid parameters for this script-
  * `_file:=` [type: string] filename to store all the way-points as csv.
    * Default value: No value
  * `_limb:=` [type: string] name of the baxter arm, in which the marker is attached. The value of this parameter can only be `left` or `right`
    * Default value: right

2. Make sure to enable the [Zero-G mode](http://sdk.rethinkrobotics.com/wiki/Zero-G_Mode) so that the arm can be moved easily to any location by grasping the cuff over its groove. 
3. Press the `Baxter` button on the arm in order to record the way-point.
4. Record 10 (higher is better) different way-points. Press <kbd>CTRL</kbd>+<kbd>C</kbd> to stop the recording process.

### Collect the data
1. Start the kinect by using following command-
  * For iai_kinect2: `roslaunch kinect2_bridge kinect2_bridge.launch`
    *  Following are the valid parameters for this script. [Check here](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge#usage).
  * For kinect_anywhere: `roslaunch kinect_anywhere kinect_anywhere.launch pointcloud:=true kinect_frame:=kinect2_link`
    *  Following are the valid parameters for this script. [Check here](https://github.com/ravijo/kinect_anywhere#steps-to-run).
2. Start collecting the data by using following command-
```
roslaunch multiple_kinect_baxter_calibration calibration_data_collector.launch topic:=/kinect2/sd/points kinect2_trajectory:=/home/ravi/ros_ws/src/multiple_kinect_baxter_calibration/files/baxter.csv
```
Following are the valid parameters for this script-
  * `topic:=` [type: string] rostopic for receving point cloud
    * Default value: No value
  * `limb:=` [type: string] limb used in calibration process
    * Default value: right
  * `log:=` [type: string] log level parameter. It must be one of the following- Info, Debug, Warn, Error, Fatal.
    * Default value: Info
  * `kinect1_trajectory:=` [type: string] full path to the baxter arm pre-defined trajectory for kinect1
    * Default value: /home/ravi/ros_ws/src/multiple_kinect_baxter_calibration/files/viapoints.csv
  * `kinect2_trajectory:=` [type: string] full path to the baxter arm pre-defined trajectory for kinect2
    * Default value: /home/ravi/ros_ws/src/multiple_kinect_baxter_calibration/files/viapoints.csv
  * `kinect3_trajectory:=` [type: string] full path to the baxter arm pre-defined trajectory for kinect3
    * Default value: /home/ravi/ros_ws/src/multiple_kinect_baxter_calibration/files/viapoints.csv
  * `kinect_anywhere_trajectory:=` [type: string] full path to the baxter arm pre-defined trajectory for kinect anywhere
    * Default value: /home/ravi/ros_ws/src/multiple_kinect_baxter_calibration/files/viapoints.csv
  * `min_hsv:=` [type: string] minimum HSV value for sphere segmentation as `min_hsv:="[40, 50, 60]"`
    * Default value: "[40, 50, 60]"
  * `max_hsv:=` [type: string] maximum HSV value for sphere segmentation as `max_hsv:="[60, 200, 255]"`
    * Default value: "[60, 200, 255]"
  * `radius:=` [type: float] radius of sphere (in meter)
    * Default value: 0.05
  * `offset:=` [type: float] length of stick to hold the sphere (in meter)
    * Default value: 0.0343
  * `k_neighbors:=` [type: int] number of 'k' nearest neighbors to use for feature estimation
    * Default value: 10
  * `weight:=` [type: double] normal angular distance weight
    * Default value: 0.05
  * `max_itr:=` [type: int] maximum number of iterations before giving up
    * Default value: 1000
  * `d_thresh:=` [type: double] distance to the model threshold
    * Default value: 0.005
  * `prob:=` [type: double] probability of choosing at least one sample free from outliers
    * Default value: 0.99999
  * `tolerance:=` [type: double] tolerance in radius (in meters)
    * Default value: 0.01
  * `epsilon:=` [type: double] angle epsilon (delta) threshold (in degree)
    * Default value: 15
  * `data_dir:=` [type: string] directory for saving tracking data
    * Default value: /home/ravi/ros_ws/src/multiple_kinect_baxter_calibration/files
  * `queue_size:=` [type: int] queue_size for the subscribers
    * Default value: 1
  * `wait_time:=` [type: double] wait time to stablize arm before capturing point cloud (in seconds)
    * Default value: 2
  * `max_samples:=` [type: int] maximum number of samples at any waypoint
    * Default value: 5
  * `min_z:=` [type: float] minimum z coordinate value of point cloud w.r.t. camera
    * Default value: 0.5
  * `max_z:=` [type: float] maximum z coordinate value of point cloud w.r.t. camera
    * Default value: 5.0
  * `title_bar_height:=` [type: int] height of the title bar in point cloud visualizer window (in pixel)
    * Default value: 10
  
### Compute the calibration
```
roslaunch multiple_kinect_baxter_calibration calibration_compute.launch kinect:=kinect2
```
Following are the valid parameters for this script-
  * `data_dir:=` [type: string] directory of baxter_trajectory and kinect_trajectory file
    * Default value: /home/ravi/ros_ws/src/multiple_kinect_baxter_calibration/files/
  * `kinect:=` [type: string] name/id of the kinect as `kinect:=kinect1`
    * Default value: No value

### Publish the calibration
```
roslaunch multiple_kinect_baxter_calibration calibration_publisher.launch calibration:="[kinect2]"
```
Following are the valid parameters for this script-
  * `data_dir:=` [type: string] directory of baxter_trajectory and kinect_trajectory file
    * Default value: /home/ravi/ros_ws/src/multiple_kinect_baxter_calibration/files/
  * `calibration:=` [type: string] all the names/ids of the kinects as `calibration:="[kinect1, kinect2, kinect3]"`
    * Default value: "[kinect1, kinect2, kinect3]"

## Other utility files
### view_cloud_realtime
To view the point cloud data in real-time
```
rosrun multiple_kinect_baxter_calibration view_cloud_realtime _topic:="/kinect2/sd/points"
```
Following are the valid parameters for this script-
  * `_topic:=` [type: string] rostopic for subscribing to point cloud
    * Default value: "/kinect1/sd/points"
  * `_source:=` [type: string] source of the point cloud. It can be `Windows` or `Linux`
    * Default value: Linux
  
### save_pcd
To save the point cloud data in a PCD file
```
rosrun multiple_kinect_baxter_calibration save_pcd _topic:="/kinect2/sd/points"
```
Following are the valid parameters for this script-
  * `_topic:=` [type: string] rostopic for subscribing to point cloud
    * Default value: "/kinect1/sd/points"

### view_pcd
To visualize the stored point cloud file
```
rosrun multiple_kinect_baxter_calibration view_pcd _file:=scene.pcd
```
Following are the valid parameters for this script-
  * `_file:=` [type: string] full path of the point cloud file
    * Default value: "/home/ravi/ros_ws/src/multiple_kinect_baxter_calibration/files/scene.pcd"
  * `_source:=` [type: string] source of the point cloud. It can be `Windows` or `Linux`
    * Default value: Linux

Please press <kbd>j</kbd> to take screenshot of the current scene.

### segment_image
To find out the HSV range of the colored marker in the image
```
rosrun multiple_kinect_baxter_calibration segment_image _file:=scene.jpg
```
Following are the valid parameters for this script-
  * `_file:=` [type: string] full path of the image file
    * Default value: No value

### view_image
To find out the RGB and HSV value of any pixel in the image
```
rosrun multiple_kinect_baxter_calibration view_image _file:=scene.jpg
```
Following are the valid parameters for this script-
  * `_file:=` [type: string] full path of the image file
    * Default value: No value

## sphere_detector_test
To test whether sphere segmentation is working or not
```
rosrun multiple_kinect_baxter_calibration sphere_detector_test _file:=scene.pcd
```
Following are the valid parameters for this script-
  * `_file:=` [type: string] full path of the point cloud file
    * Default value: "/home/ravi/ros_ws/src/multiple_kinect_baxter_calibration/files/scene.pcd"
  * `_r:=` [type: float] radius of sphere (in meter)
    * Default value: 0.05
  * `_min_h:=` [type: int] minimum hue for sphere segmentation
    * Default value: 40
  * `_min_s:=` [type: int] minimum saturation for sphere segmentation
    * Default value: 50
  * `_min_v:=` [type: int] minimum value for sphere segmentation
    * Default value: 60
  * `_max_h:=` [type: int] maximum hue for sphere segmentation
    * Default value: 60
  * `_max_s:=` [type: int] maximum saturation for sphere segmentation
    * Default value: 200
  * `_max_v:=` [type: int] maximum value for sphere segmentation
    * Default value: 255
  * `_source:=` [type: string] source of the point cloud. It can be `Windows` or `Linux`
    * Default value: Linux

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

## Issues (or Error Reporting)
Please check [here](https://github.com/ravijo/multiple_kinect_baxter_calibration/issues) and create issues accordingly.
 
