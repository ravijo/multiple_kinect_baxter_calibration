/**
 * data_collector.cpp: class file for collecting data from single kinect
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
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

// sphere_detector header
#include <sphere_detector.h>

// baxter header
#include <baxter_core_msgs/EndpointState.h>

// vtk header
#include <vtkCamera.h>

// number of kinects used
#define KINECT_COUNT 1

// baxter arm motion state (moving:0, stop:1, finished:2)
#define MOVING 0
#define STOP 1
#define FINISHED 2

class DataCollector {
private:
  int queue_size;
  double tolerance;
  float sphere_radius;
  int baxter_arm_motion_state; // moving:0, stop:1, finished:2
  Eigen::Matrix4d t_ball_wrt_ee;
  std_msgs::Bool still_processing;
  pcl::visualization::Camera camera;
  ros::Subscriber baxter_arm_motion_status_sub;
  pcl_project::SphereDetector* sphere_detector;
  ros::Publisher data_collection_progress_pub;
  std::vector<std::vector<float> > position_wrt_baxter;
  std::vector<std::vector<float> > position_wrt_kinect;
  std::string pc_topic, ee_topic, data_dir, file_suffix;
  std::vector<pcl::visualization::PCLVisualizer*> pc_viewers;

  void saveData();

  void init(ros::NodeHandle nh);

  void setWindowPosition(int index);

  void baxterArmMotionStatusCallback(const std_msgs::Int8::ConstPtr& msg);

  void recordBallPositionWrtKinect(
      pcl::ModelCoefficients& sphere_coefficients);

  void recordBallPositionWrtBaxter(
      baxter_core_msgs::EndpointStateConstPtr ee_msg);

  void callback(const baxter_core_msgs::EndpointStateConstPtr& ee_msg,
      const sensor_msgs::PointCloud2ConstPtr& pc_msg);
public:
  DataCollector();
};

void DataCollector::recordBallPositionWrtBaxter(
    baxter_core_msgs::EndpointStateConstPtr ee_msg) {
  Eigen::Quaterniond q(ee_msg->pose.orientation.w, ee_msg->pose.orientation.x,
      ee_msg->pose.orientation.y, ee_msg->pose.orientation.z);

  Eigen::Matrix4d t_ee_wrt_base;
  // identity to make bottom row of matrix 0, 0, 0, 1
  t_ee_wrt_base.setIdentity();
  t_ee_wrt_base.block<3, 3>(0, 0) = q.toRotationMatrix();
  t_ee_wrt_base(0, 3) = ee_msg->pose.position.x;
  t_ee_wrt_base(1, 3) = ee_msg->pose.position.y;
  t_ee_wrt_base(2, 3) = ee_msg->pose.position.z;

  Eigen::Matrix4d t = t_ee_wrt_base * t_ball_wrt_ee;

  std::vector<float> baxter;
  for (int i = 0; i < 3; i++)
    baxter.push_back(t(i, 3));

  position_wrt_baxter.push_back(baxter);
}

void DataCollector::recordBallPositionWrtKinect(
    pcl::ModelCoefficients& sphere_coefficients) {
  std::vector<float> kinect;
  for (int i = 0; i < 3; i++)
    kinect.push_back(sphere_coefficients.values[i]);

  position_wrt_kinect.push_back(kinect);
}

void DataCollector::baxterArmMotionStatusCallback(
    const std_msgs::Int8::ConstPtr& msg) {
  baxter_arm_motion_state = msg->data;
}

void DataCollector::saveData() {
  std::string file_name = data_dir + "/position_wrt_baxter_" + file_suffix + ".csv";
  std::string header = "baxter_x,baxter_y,baxter_z,kinect_x,kinect_y,kinect_z";
  std::vector<std::vector<float> > trajectory = utility::hstack(position_wrt_baxter, position_wrt_kinect);

  ROS_INFO_STREAM("Saving collected data in following file: \n" << file_name << "\n");

  std::string err;
  if (!utility::writeCSV(file_name, header, trajectory, err))
    ROS_ERROR_STREAM("Unable to save tracking data to CSV file. " << err);
}

void DataCollector::callback(
    const baxter_core_msgs::EndpointStateConstPtr& ee_msg,
    const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
  // exit and save the data
  if (baxter_arm_motion_state == FINISHED) {
    if (position_wrt_kinect.empty())
      ROS_ERROR_STREAM(
          "Couldn't record any data. Run the program again.");
    else
      saveData();

    ROS_INFO_STREAM("Exiting now...");
    ros::shutdown();
    return;
  }

  // don't proceed further if robot is moving or if we are processing the existing data
  if (still_processing.data || (baxter_arm_motion_state == MOVING))
    return;

  still_processing.data = true; // we are about to start processing
  data_collection_progress_pub.publish(still_processing);

  // set windows position
  for (size_t i = 0; i < 2 * KINECT_COUNT; i++)
    setWindowPosition(i);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  utility::getPointCloudFromMsg(pc_msg, *cloud);

  // show caputed point cloud
  if (!pc_viewers.at(0)->updatePointCloud(cloud, "cloud"))
      pc_viewers.at(0)->addPointCloud(cloud, "cloud");
  pc_viewers.at(0)->spinOnce();

  pcl::ModelCoefficients sphere_coff;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  bool success = sphere_detector->segmentSphere(pc_viewers.at(0), cloud,
      segmented_cloud, sphere_coff);

  // show Segmented point cloud
  if (!pc_viewers.at(1)->updatePointCloud(segmented_cloud, "segmented_cloud"))
    pc_viewers.at(1)->addPointCloud(segmented_cloud, "segmented_cloud");
  pc_viewers.at(1)->spinOnce();

  if (success) {
    ROS_INFO_STREAM("Sphere detection successfull");

    recordBallPositionWrtBaxter(ee_msg);
    recordBallPositionWrtKinect(sphere_coff);

    pcl::PointXYZ detected_sphere(sphere_coff.values[0],
        sphere_coff.values[1], sphere_coff.values[2]);
    if (!pc_viewers.at(1)->updateSphere(detected_sphere,
        sphere_coff.values[3], 0.2, 0.3, 1.0, "detected_sphere"))
      pc_viewers.at(1)->addSphere(detected_sphere, sphere_coff.values[3],
          0.2, 0.3, 1.0, "detected_sphere");

    pc_viewers.at(1)->spinOnce();
  } else {
    ROS_WARN_STREAM("Sphere detection failed");

    /*
    std::string seg = "segme_" + utility::to_string(index) + ".pcd";
    std::string clo = "cloud_" + utility::to_string(index) + ".pcd";

    pcl::io::savePCDFileASCII(seg, *segmented_cloud);
    pcl::io::savePCDFileASCII(clo, *cloud);
    */
  }

  still_processing.data = false; // we have done the processing
  data_collection_progress_pub.publish(still_processing);
}

void DataCollector::init(ros::NodeHandle nh) {
  float offset;
  int k_neighbors, max_itr;
  std::string min_hsv, max_hsv, cam_file, limb;
  double weight, d_thresh, prob, epsilon;

  nh.getParam("min_hsv", min_hsv);
  nh.getParam("max_hsv", max_hsv);
  nh.getParam("radius", sphere_radius);
  nh.getParam("offset", offset);

  // parameters for SAC_RANSAC
  nh.getParam("k_neighbors", k_neighbors);
  nh.getParam("max_itr", max_itr);
  nh.getParam("weight", weight);
  nh.getParam("d_thresh", d_thresh);
  nh.getParam("prob", prob);
  nh.getParam("tolerance", tolerance);
  nh.getParam("epsilon", epsilon);

  nh.getParam("cam_file", cam_file);
  nh.getParam("limb", limb);

  // define end-effector topic
  ee_topic = "/robot/limb/" + limb + "/endpoint_state";

  nh.getParam("data_dir", data_dir);
  nh.getParam("queue_size", queue_size);

  nh.getParam("topic", pc_topic);

  std::vector<int>  all_slash = utility::find_all(pc_topic, "/");
  file_suffix = pc_topic.substr(all_slash[0] + 1, all_slash[1] - 1);

  std::vector<int> min_hsv_values = utility::stringToArray(min_hsv);
  std::vector<int> max_hsv_values = utility::stringToArray(max_hsv);

  // initialize sphere detector
  pcl_project::RansacParams ransac_params(k_neighbors, max_itr, weight,
      d_thresh, prob, tolerance, epsilon);
  sphere_detector = new pcl_project::SphereDetector(sphere_radius, &min_hsv_values,
      &max_hsv_values, &ransac_params);

  // define homogeneous transformation matrix for sphere
  t_ball_wrt_ee.setIdentity(); // considering no rotation
  t_ball_wrt_ee(2, 3) = sphere_radius + offset; // distance (m) of the ball center from ee

  baxter_arm_motion_state = 0; //moving:0, stop:1, finished:2
  still_processing.data = false;

  // initialize point cloud viewers and related parameters
  std::vector<std::string> cam_param;
  bool result = utility::loadCameraParametersPCL(cam_file, cam_param);
  result = result && utility::getCameraParametersPCL(cam_param, camera);
  ROS_DEBUG_STREAM("loadCameraParametersPCL returned " << result);

  for (size_t i = 0; i < 2 * KINECT_COUNT; i++) {
    std::string window_name =
        i < KINECT_COUNT ?
            "Point Cloud (" + file_suffix + ")" :
            "Segmented Cloud (" + file_suffix + ")";
    pcl::visualization::PCLVisualizer* pc_viewer(
        new pcl::visualization::PCLVisualizer(window_name));

    pc_viewer->initCameraParameters();
    pc_viewer->setCameraParameters(camera);
    pc_viewer->spinOnce(10);

    pc_viewers.push_back(pc_viewer); // store it
  }

  // use orthographic views
  //pc_viewers.at(0)->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
  //pc_viewers.at(0)->setCameraParameters(camera);

  for (size_t i = 0; i < 2 * KINECT_COUNT; i++)
    setWindowPosition(i);
}

void DataCollector::setWindowPosition(int index) {
  int width = camera.window_size[0];
  int height = camera.window_size[1];

  int x = index * width;
  int y = 0;

  if (index >= KINECT_COUNT) {
    x = (index - KINECT_COUNT) * width;
    y = height;
  }

  pc_viewers.at(index)->setSize(width, height);
  pc_viewers.at(index)->setPosition(x, y);
}

DataCollector::DataCollector() {
  ros::NodeHandle nh("~");
  init(nh);

  message_filters::Subscriber < baxter_core_msgs::EndpointState
      > baxter_arm_sub(nh, ee_topic, 1);
  message_filters::Subscriber < sensor_msgs::PointCloud2
      > point_cloud_sub(nh, pc_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<
      baxter_core_msgs::EndpointState, sensor_msgs::PointCloud2> SyncPolicy;

  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(queue_size),
      baxter_arm_sub, point_cloud_sub);
  sync.registerCallback(boost::bind(&DataCollector::callback, this, _1, _2));

  std::string data_collection_progress_topic =
      "/multiple_kinect_baxter_calibration/is_data_collection_happening";
  std::string baxter_arm_motion_status_topic =
      "/multiple_kinect_baxter_calibration/baxter_arm_motion_status";

  data_collection_progress_pub = nh.advertise < std_msgs::Bool
      > (data_collection_progress_topic, 10);
  baxter_arm_motion_status_sub = nh.subscribe(baxter_arm_motion_status_topic,
      10, &DataCollector::baxterArmMotionStatusCallback, this);
  ros::spin();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_collector_node",
      ros::init_options::AnonymousName);
  DataCollector dc;
  return 0;
}
