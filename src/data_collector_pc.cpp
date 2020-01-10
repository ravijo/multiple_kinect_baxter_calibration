/**
 * data_collector.cpp: class file for collecting data from single depth camera
 * Author: Ravi Joshi
 * Date: 2018/02/17
 */

#include <vector>

// utility header
#include <utility.h>

// ros headers
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

// sphere_detector header
#include <sphere_detector.h>

// vtk header
#include <vtkCamera.h>

// we are using trigger service for implementing 'move_arm_to_waypoint' service
#include <std_srvs/Trigger.h>
#include <multiple_kinect_baxter_calibration/GetEEPosition.h>

// for thread related support
#include <boost/thread.hpp>

// name for 'move_arm_to_waypoint' service
#define MOVE_ARM_SERVICE "/move_arm_to_waypoint"
#define GET_POSITION_SERVICE "/get_ee_position"

// define various ids for visualization
#define CLOUD_ID "point_cloud"
#define SEGMENTED_CLOUD_ID "segmented_cloud"
#define SPHERE_ID "detected_sphere"
#define BLUE_COLOR 0.2, 0.3, 1.0

class DataCollector
{
private:
  // queue_size for the subscribers
  int queue_size;

  // maximum number of samples at any waypoint
  int max_samples;

  // minimum and maximum z coordinate value of point cloud w.r.t. camera
  float min_z, max_z;

  // height of the title bar in point cloud visualizer window (in pixel)
  int title_bar_height;

  // service for moving baxter arm to waypoint
  ros::ServiceClient move_arm;
  ros::ServiceClient get_position;

  // ros node handle
  ros::NodeHandle nh;

  // ros asynchronous spinner. instead of a blocking spin() call,
  // it has start() and stop() calls.
  boost::shared_ptr<ros::AsyncSpinner> spinner;

  // various subscribers
  ros::Subscriber point_cloud_sub;
  ros::Subscriber baxter_arm_sub;

  // wait time to stablize arm before capturing point cloud (seconds)
  ros::Duration wait_duration;

  // wait time after capturing point cloud (in seconds)
  ros::Duration sleep_duration;

  // camera onject for point cloud visualization
  pcl::visualization::Camera camera;

  // pointer to sphere detector library
  boost::shared_ptr<pcl_utility::SphereDetector> sphere_detector;

  // various tracking data
  // position_wrt_baxter: position of the shpere w.r.t. baxter
  // position_wrt_camera: position of the shpere w.r.t. camera
  std::vector<std::vector<float> > position_wrt_baxter;
  std::vector<std::vector<float> > position_wrt_camera;

  // pc_topic: rostopic for point cloud
  // data_dir: directory for saving tracking data
  // sensor_name: name of the depth sensor
  std::string pc_topic, data_dir, sensor_name;

  // container to store point cloud visualizers
  std::vector<pcl::visualization::PCLVisualizer*> pc_viewers;

  // latest available point cloud and ee data pointers
  sensor_msgs::PointCloud2ConstPtr pc_msg_ptr;
  geometry_msgs::PoseConstPtr ee_pose_ptr;

  // the point cloud from iai_kinect2 (kinect camera) was found misleading
  // it says that the point cloud is 'rgb', but in reality it is 'rgba'.
  // hence we made utility::getPointCloudFromMsg function to take care of it
  // however the point cloud from realsense-ros (realsense camera) is also
  // giving wrong information. Due to that PCL throws the following error
  // Failed to find match for field 'rgb'.
  // The workaround is give at the following URL
  // https://github.com/IntelRealSense/realsense-ros/issues/680#issuecomment-543492819
  // Make sure to follow the above URL.
  // Due to these complications for now we are treating realsense camera
  // separately
  bool is_realsense_camera;

  // function to save the tracking data into a csv file
  void saveTrackingData();

  // initialization of various parameter etc.
  void init();

  // set log level of ros node
  void setLoggerLevel(std::string level);

  // function for setting the positions of visualizers
  void setWindowPosition(int index);

  // function to record the sphere center w.r.t. depth sensor
  void recordSpherePositionWrtCamera(
      const pcl::ModelCoefficients& sphere_coefficients);

  // function to record the sphere center w.r.t. baxter
  void recordSpherePositionWrtBaxter(const geometry_msgs::Point ee_position);

  // callback function for point cloud
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg);

  // function to track the sphere center in the latest available data
  // returns true: if tracking is successfull, false otherwise
  bool processLatestData();

public:
  // default constructor
  DataCollector();

  // main function
  void spin();
};

// function to record the sphere center w.r.t. baxter
void DataCollector::recordSpherePositionWrtBaxter(
    const geometry_msgs::Point ee_position)
{
  // create vector from position
  std::vector<float> baxter;
  baxter.push_back(ee_position.x);
  baxter.push_back(ee_position.y);
  baxter.push_back(ee_position.z);

  // store it
  position_wrt_baxter.push_back(baxter);
}

// function to record the sphere center w.r.t. depth sensor
void DataCollector::recordSpherePositionWrtCamera(
    const pcl::ModelCoefficients& sphere_coefficients)
{
  // get the position of center of sphere in a vector
  std::vector<float> camera;
  for (int i = 0; i < 3; i++)
    camera.push_back(sphere_coefficients.values[i]);

  // store it
  position_wrt_camera.push_back(camera);
}

// callback function for point cloud
void DataCollector::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
  // store boost::shared_ptr pointer of latest received data
  pc_msg_ptr = pc_msg;

  // print single message after 2 seconds (to keep the terminal cleaner)
  ROS_DEBUG_THROTTLE(2, "Callback received: point cloud data");
}

// function to save the tracking data into a csv file
void DataCollector::saveTrackingData()
{
  std::string file_name = data_dir + "/baxter_" + sensor_name + "_pc.csv";
  std::string header = "baxter_x,baxter_y,baxter_z,camera_x,camera_y,camera_z";

  std::vector<std::vector<float> > trajectory =
      utility::hstack(position_wrt_baxter, position_wrt_camera);

  ROS_INFO_STREAM("Saving collected data in following file: \n'" << file_name
                                                                 << "'\n");

  std::string err;
  if (!utility::writeCSV(file_name, header, trajectory, err))
    ROS_ERROR_STREAM("Unable to save tracking data to CSV file. " << err);
}

// function to track the sphere center in the latest available data
// returns true: if tracking is successfull, false otherwise
bool DataCollector::processLatestData()
{
  // return if we haven't received the data yet
  // source: https://stackoverflow.com/a/5610531/1175065
  if (!pc_msg_ptr)
    return false;

  multiple_kinect_baxter_calibration::GetEEPosition service;

  // call the service and wait for it. call again if failed
  if (!get_position.call(service))
  {
    ROS_ERROR_STREAM("Failed to call service " << GET_POSITION_SERVICE);
  }

  // the service has returned successfully
  ROS_DEBUG_STREAM("Service " << GET_POSITION_SERVICE
                              << " has returned successfully.");

  // create point cloud object retrieved from
  // boost::shared_ptr. our assumption is that the stored
  // point cloud and end-effector messages belong to the
  // same timestamp. note that we are using 'const' keyword.
  const sensor_msgs::PointCloud2 pc_msg = *pc_msg_ptr;
  const geometry_msgs::Point ee_position = service.response.position;

  // set windows position
  for (size_t i = 0; i < 2; i++)
    setWindowPosition(i);

  // convert sensor_msgs::PointCloud2 to pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  if (is_realsense_camera)
    pcl::fromROSMsg(pc_msg, *cloud);
  else
    utility::getPointCloudFromMsg(pc_msg, *cloud, min_z, max_z);

  // show the caputed point cloud
  //if (!pc_viewers.at(0)->updatePointCloud(cloud, CLOUD_ID))
  //  pc_viewers.at(0)->addPointCloud(cloud, CLOUD_ID);
  //pc_viewers.at(0)->spinOnce();

  // clear the view
  // we could use update method but it seems buggy in PCL 1.7.2
  for (size_t i = 0; i < 2; i++)
  {
    pc_viewers.at(i)->removeAllShapes();
    pc_viewers.at(i)->removeAllPointClouds();
  }

  // show the caputed point cloud
  pc_viewers.at(0)->addPointCloud(cloud, CLOUD_ID);

  // declare the sphere coefficients consisting of position of the center
  // and radius of the sphere
  pcl::ModelCoefficients sphere_coff;

  // declare the segmented point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  // segment sphere from the captured point cloud
  // get the segmented point cloud and the sphere coefficients
  bool success = sphere_detector->segmentSphere(pc_viewers.at(0), cloud,
                                                segmented_cloud, sphere_coff);

  // add segmented point cloud if not added previously, update otherwise
  //if (!pc_viewers.at(1)->updatePointCloud(segmented_cloud, SEGMENTED_CLOUD_ID))
  pc_viewers.at(1)->addPointCloud(segmented_cloud, SEGMENTED_CLOUD_ID);

  // force the visualizer to update the view
  pc_viewers.at(1)->spinOnce();

  if (success)
  {
    ROS_DEBUG_STREAM("Sphere detection successfull");

    // record the sphere center position
    recordSpherePositionWrtBaxter(ee_position);
    recordSpherePositionWrtCamera(sphere_coff);

    // initialize detected sphere from the sphere coefficients
    pcl::PointXYZ detected_sphere(sphere_coff.values[0], sphere_coff.values[1],
                                  sphere_coff.values[2]);

    // add detected sphere if not added previously, update otherwise
    //if (!pc_viewers.at(1)->updateSphere(detected_sphere, sphere_coff.values[3],
    //                                    BLUE_COLOR, SPHERE_ID))
    pc_viewers.at(1)->addSphere(detected_sphere, sphere_coff.values[3],
                                BLUE_COLOR, SPHERE_ID);
    // force the visualizer to update the view
    //pc_viewers.at(1)->spinOnce(1, true);
    pc_viewers.at(1)->spinOnce();
  }
  else
  {
    ROS_WARN_STREAM("Sphere detection failed");

    // remove the previously detected sphere in case of failure
    // pc_viewers.at(1)->removeShape(SPHERE_ID);

    /*
    std::string seg = "segme_" + utility::to_string(index) + ".pcd";
    std::string clo = "cloud_" + utility::to_string(index) + ".pcd";

    pcl::io::savePCDFileASCII(seg, *segmented_cloud);
    pcl::io::savePCDFileASCII(clo, *cloud);
    */
  }

  return success;
}

// set log level of ros node
// source: http://wiki.ros.org/rosconsole#Changing_Logger_Levels
void DataCollector::setLoggerLevel(std::string level)
{
  ROS_INFO_STREAM("Trying to set logger level to " << level);

  // in-place converision to lowercase
  // source: https://stackoverflow.com/a/313988/1175065
  boost::algorithm::to_lower(level);

  if (level == "info")
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Info);
  else if (level == "debug")
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);
  else if (level == "warn")
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Warn);
  else if (level == "error")
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Error);
  else if (level == "fatal")
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Fatal);
  else
    ROS_ERROR_STREAM("Invalid logger level provided. It should be one of the "
                     "following- Info, Debug, Warn, Error, Fatal.");

  // after changing the verbosity levels we must call the following
  ros::console::notifyLoggerLevelsChanged();
}

// initialization of various parameter etc.
void DataCollector::init()
{
  // SAC_RANSAC parameter
  int k_neighbors, max_itr;

  // minimum and maximum HSV values for sphere segmentation
  std::string min_hsv, max_hsv;

  // limb used in calibration process
  std::string limb;

  // SAC_RANSAC parameter
  double weight, d_thresh, prob, epsilon;

  // SAC_RANSAC parameter (tolerance of radius in meters)
  double tolerance;

  // SAC_RANSAC parameter (radius of sphere in meters)
  float sphere_radius;

  // get the parameters for sphere segmentation
  nh.getParam("min_hsv", min_hsv);
  nh.getParam("max_hsv", max_hsv);
  nh.getParam("radius", sphere_radius);

  // parameters for SAC_RANSAC
  nh.getParam("k_neighbors", k_neighbors);
  nh.getParam("max_itr", max_itr);
  nh.getParam("weight", weight);
  nh.getParam("d_thresh", d_thresh);
  nh.getParam("prob", prob);
  nh.getParam("tolerance", tolerance);
  nh.getParam("epsilon", epsilon);

  nh.getParam("limb", limb);

  nh.getParam("data_dir", data_dir);
  nh.getParam("topic", pc_topic);

  // get the 'queue_size' for the subscribers
  nh.getParam("queue_size", queue_size);

  // wait time to stablize arm before capturing point cloud (seconds)
  double wait_time, loop_sleep;
  nh.getParam("wait_time", wait_time);
  nh.getParam("loop_sleep", loop_sleep);
  wait_duration = ros::Duration(wait_time);
  sleep_duration = ros::Duration(loop_sleep);

  // get the parameter 'max_samples'
  nh.getParam("max_samples", max_samples);

  // the logger level parameter
  std::string level;
  nh.getParam("log", level);
  setLoggerLevel(level);

  // minimum and maximum z coordinate value of point cloud w.r.t. camera
  nh.getParam("min_z", min_z);
  nh.getParam("max_z", max_z);

  // height of the title bar in point cloud visualizer window (pixel)
  nh.getParam("title_bar_height", title_bar_height);

  // get the sensor_name as the first word between two leftmost slash chacters
  std::vector<int> all_slash = utility::find_all(pc_topic, "/");
  sensor_name = pc_topic.substr(all_slash[0] + 1, all_slash[1] - 1);

  is_realsense_camera = boost::starts_with(sensor_name, "camera");

  // we used string to declare an array
  // now we need to convert it and get the integers
  std::vector<int> min_hsv_values = utility::stringToArray(min_hsv);
  std::vector<int> max_hsv_values = utility::stringToArray(max_hsv);

  // initialize sphere detector
  pcl_utility::RansacParams ransac_params(k_neighbors, max_itr, weight,
                                          d_thresh, prob, tolerance, epsilon);
  sphere_detector.reset(new pcl_utility::SphereDetector(
      sphere_radius, &min_hsv_values, &max_hsv_values, &ransac_params));

  // set the camera file based on 'sensor_name'
  std::string cam_file = boost::starts_with(sensor_name, "kinect_anywhere") ?
                             data_dir + "/kinect_anywhere.cam" :
                             data_dir + "/libfreenect.cam";

  // initialize point cloud viewers and related parameters
  std::vector<std::string> cam_param;
  bool result = utility::loadCameraParametersPCL(cam_file, cam_param);
  result = result && utility::getCameraParametersPCL(cam_param, camera);
  ROS_DEBUG_STREAM("'utility::loadCameraParametersPCL' returned " << result);

  for (size_t i = 0; i < 2; i++)
  {
    // define the visualizer window name
    std::string window_name = i < 1 ? "Point Cloud (" + sensor_name + ")" :
                                      "Segmented Cloud (" + sensor_name + ")";

    // create point cloud visualizer
    pcl::visualization::PCLVisualizer* pc_viewer(
        new pcl::visualization::PCLVisualizer(window_name));

    // set the view parameters
    pc_viewer->initCameraParameters();
    pc_viewer->setCameraParameters(camera);
    pc_viewer->spinOnce(10);

    // store it
    pc_viewers.push_back(pc_viewer);
  }

  // use orthographic views
  // pc_viewers.at(0)->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
  // pc_viewers.at(0)->setCameraParameters(camera);

  // set the position of visualizers
  for (size_t i = 0; i < 2; i++)
    setWindowPosition(i);
}

// let the ROS stay awake and we collect the data
void DataCollector::spin()
{
  // make sure we have the data. so we need to start the spinner
  spinner->start();

  // wait for the 'move_arm_to_waypoint' service to be advertised
  ROS_DEBUG_STREAM("Waiting for service " << MOVE_ARM_SERVICE);
  ros::service::waitForService(MOVE_ARM_SERVICE, ros::Duration(-1));

  // create a client for the 'move_arm_to_waypoint' service
  ROS_DEBUG_STREAM("Creating client for service " << MOVE_ARM_SERVICE);
  move_arm = nh.serviceClient<std_srvs::Trigger>(MOVE_ARM_SERVICE);

  // keep running in an infinite loop
  // exit only when trajectory is finished
  while (ros::ok())
  {
    // -------------------------------------------------------------------- //
    // ------------------------------ STEP 1 ------------------------------ //
    // -------------------------------------------------------------------- //
    // move the baxter arm and wait for the arm to stop
    std_srvs::Trigger service;

    // call the service and wait for it. call again if failed
    if (!move_arm.call(service))
    {
      ROS_ERROR_STREAM("Failed to call service " << MOVE_ARM_SERVICE);
      continue;
    }

    // the service has returned successfully
    bool trajectory_finished = service.response.success;
    ROS_DEBUG_STREAM(
        "Service " << MOVE_ARM_SERVICE
                   << " has returned successfully with the following message: "
                   << service.response.message);

    // exit and save the tracking data if the trajectory is finished
    if (trajectory_finished)
    {
      // throw error if we couldn't track the sphere even once, save otherwise
      if (position_wrt_camera.empty())
        ROS_ERROR_STREAM("Couldn't record any data. Run the program again.");
      else
        saveTrackingData();

      // stop the node by giving a message
      ROS_INFO_STREAM("Exiting now...");
      ros::shutdown();
      break;
    }

    // -------------------------------------------------------------------- //
    // ------------------------------ STEP 2 ------------------------------ //
    // -------------------------------------------------------------------- //
    // wait for a while so that the baxter arm stops shaking
    wait_duration.sleep();

    // -------------------------------------------------------------------- //
    // ------------------------------ STEP 3 ------------------------------ //
    // -------------------------------------------------------------------- //
    // start processing the latest point cloud
    // grab the latest point cloud. repeat it 'max_samples' times
    int success_count = 0;
    for (size_t i = 0; i < max_samples; i++)
    {
      bool status = processLatestData();
      success_count += status;  // divide by sizeof(bool)
      sleep_duration.sleep();   // wait for callback to be called. sleep for 0.2
                                // second (200 ms)
    }
    ROS_INFO_STREAM("Successfully detected sphere "
                    << success_count << " times out of " << max_samples);
  }
}

// function for setting the positions of visualizers
void DataCollector::setWindowPosition(int index)
{
  int width = camera.window_size[0];
  int height = camera.window_size[1];

  int x = index * width;
  int y = 0;

  if (index >= 1)
  {
    x = (index - 1) * width;
    y = height + title_bar_height;
  }

  // set the window width and height
  pc_viewers.at(index)->setSize(width, height);

  // set the window position in screen
  pc_viewers.at(index)->setPosition(x, y);
}

// default constructor
DataCollector::DataCollector()
{
  // assign nodehandle with relative namespace
  nh = ros::NodeHandle("~");

  // initialize of the parameter
  init();

  // create subscriber for the point cloud data and end-effector pose
  point_cloud_sub = nh.subscribe(pc_topic, queue_size,
                                 &DataCollector::pointCloudCallback, this);

  // wait for the 'move_arm_to_waypoint' service to be advertised
  ROS_DEBUG_STREAM("Waiting for service " << GET_POSITION_SERVICE);
  ros::service::waitForService(GET_POSITION_SERVICE, ros::Duration(-1));

  // create a client for the 'move_arm_to_waypoint' service
  ROS_DEBUG_STREAM("Creating client for service " << GET_POSITION_SERVICE);
  get_position =
      nh.serviceClient<multiple_kinect_baxter_calibration::GetEEPosition>(
          GET_POSITION_SERVICE);

  // get the concurrent hardware thread count
  const int spported_threads = boost::thread::hardware_concurrency();
  ROS_DEBUG_STREAM("Number of heardware supported threads for this CPU is "
                   << spported_threads);

  // we need one additional thread for point cloud data subscriber
  // which is asynchronous
  int desired_threads = 1;

  // show warning to user
  if (desired_threads > spported_threads)
    ROS_WARN_STREAM(
        "It is suggested to upgrade the CPU for better performance");
  else
    ROS_DEBUG_STREAM("Initializing " << desired_threads
                                     << " additional threads");

  // lazy initialization for boost::shared_ptr
  // source: https://stackoverflow.com/a/12997218/1175065
  spinner.reset(new ros::AsyncSpinner(desired_threads));
}

int main(int argc, char** argv)
{
  // create ros node with random suffix
  ros::init(argc, argv, "data_collector_node",
            ros::init_options::AnonymousName);

  // create and instance of 'DataCollector'
  DataCollector dc;
  dc.spin();

  return 0;
}
