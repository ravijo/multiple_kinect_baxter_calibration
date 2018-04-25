/**
 * sphere_detector_test.cpp: test file for sphere_detector
 * Author: Ravi Joshi
 * Date: 2018/02/20
 */

// ros headers
#include <ros/ros.h>
#include <ros/package.h>

// utility header
#include <utility.h>

// sphere detector header
#include <sphere_detector.h>

// vtk header
#include <vtkCamera.h>

/*
// small yellow ball parameters
float default_sphere_radius = 0.034; // m
int default_min_h = 5, default_min_s = 20, default_min_v = 150;
int default_max_h = 40, default_max_s = 190, default_max_v = 255;
*/

// Polystyrene green ball parameters
float default_sphere_radius = 0.05; // m
int default_min_h = 40, default_min_s = 50,  default_min_v = 60;
int default_max_h = 60, default_max_s = 200, default_max_v = 255;

int main(int argc, char * * argv) {
  ros::init(argc, argv, "sphere_detector_test",
      ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  std::string package_path =
  ros::package::getPath("multiple_kinect_baxter_calibration");

  std::string pcd_file;
  if (nh.getParam("file", pcd_file)) {
    ROS_INFO_STREAM("PCD file is '" << pcd_file << "'");
  } else {
    pcd_file = package_path + "/files/scene.pcd";
    ROS_WARN_STREAM(
        "PCD file is not provided. Using '" << pcd_file
            << "' as default PCD file");
  }

  float sphere_radius;
  if (nh.getParam("r", sphere_radius)) {
    ROS_INFO_STREAM("Sphere radius is " << sphere_radius);
  } else {
    sphere_radius = default_sphere_radius;
    ROS_WARN_STREAM(
        "Sphere radius is not provided. Using " << sphere_radius
            << " as default value");
  }

  int min_h, min_s, min_v;
  if (nh.getParam("min_h", min_h)) {
    ROS_INFO_STREAM("Minimum hue is " << min_h);
  } else {
    min_h = default_min_h;
    ROS_WARN_STREAM(
        "Minimum hue is not provided. Using " << min_h
            << " as default value");
  }

  if (nh.getParam("min_s", min_s)) {
    ROS_INFO_STREAM("Minimum saturation is " << min_s);
  } else {
    min_s = default_min_s;
    ROS_WARN_STREAM(
        "Minimum saturation is not provided. Using " << min_s
            << " as default value");
  }

  if (nh.getParam("min_v", min_v)) {
    ROS_INFO_STREAM("Minimum value is " << min_v);
  } else {
    min_v = default_min_v;
    ROS_WARN_STREAM(
        "Minimum value is not provided. Using " << min_v
            << " as default value");
  }

  int max_h, max_s, max_v;
  if (nh.getParam("max_h", max_h)) {
    ROS_INFO_STREAM("Maximum hue is " << max_h);
  } else {
    max_h = default_max_h;
    ROS_WARN_STREAM(
        "Maximum hue is not provided. Using " << max_h
            << " as default value");
  }

  if (nh.getParam("max_s", max_s)) {
    ROS_INFO_STREAM("Maximum saturation is " << max_s);
  } else {
    max_s = default_max_s;
    ROS_WARN_STREAM(
        "Maximum saturation is not provided. Using " << max_s
            << " as default value");
  }

  if (nh.getParam("max_v", max_v)) {
    ROS_INFO_STREAM("Maximum value is " << max_v);
  } else {
    max_v = default_max_v;
    ROS_WARN_STREAM(
        "Maximum value is not provided. Using " << max_v
            << " as default value");
  }

  std::vector<int> min_hsv_values(3);
  std::vector<int> max_hsv_values(3);

  min_hsv_values[0] = min_h;
  max_hsv_values[0] = max_h;
  min_hsv_values[1] = min_s;
  max_hsv_values[1] = max_s;
  min_hsv_values[2] = min_v;
  max_hsv_values[2] = max_v;

  // use default params
  pcl_project::RansacParams ransac_params;
  // change ransac_params as following
  /*
   ransac_params.k_neighbors = k_neighbors;
   ransac_params.max_itr = max_itr;
   ransac_params.weight = weight;
   ransac_params.d_thresh = d_thresh;
   ransac_params.prob = prob;
   ransac_params.tolerance = tolerance;
   ransac_params.epsilon = epsilon;
   */

  pcl_project::SphereDetector sphereDetector(sphere_radius, &min_hsv_values,
      &max_hsv_values, &ransac_params);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  // load PCD file and check if invalid file is provided
  if (pcl::io::loadPCDFile < pcl::PointXYZRGB > (pcd_file, *cloud) == -1) {
    ROS_ERROR_STREAM("Couldn't read PCD file " << pcd_file);
    return -1;
  }

  std::string cam_file;
  std::string source;
  if (nh.getParam("source", source) && boost::starts_with(boost::algorithm::to_lower_copy(source), "w")) {
    // if source is 'Windows'
    cam_file = package_path + "/files/kinect_anywhere.cam";
  } else {
    // if source is 'Linux'
    cam_file = package_path + "/files/libfreenect.cam";
  }
  ROS_INFO_STREAM("cam_file is '" << cam_file << "'");

  pcl::visualization::Camera camera;
  std::vector<std::string> cam_param;
  bool result = utility::loadCameraParametersPCL(cam_file, cam_param);
  result = result && utility::getCameraParametersPCL(cam_param, camera);
  ROS_DEBUG_STREAM("loadCameraParametersPCL returned " << result);

  pcl::visualization::PCLVisualizer pcd_viewer("Point Cloud");
  pcl::visualization::PCLVisualizer seg_viewer("Segmented Cloud");

  pcd_viewer.setPosition(0, 0);
  seg_viewer.setPosition(0, camera.window_size[1]);
  pcd_viewer.setSize(camera.window_size[0], camera.window_size[1]);
  seg_viewer.setSize(camera.window_size[0], camera.window_size[1]);
  seg_viewer.spinOnce();

  pcd_viewer.addPointCloud(cloud, "cloud");

  pcd_viewer.initCameraParameters();
  pcd_viewer.getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
  pcd_viewer.setCameraParameters(camera);
  seg_viewer.initCameraParameters();
  seg_viewer.setCameraParameters(camera);

  // force re-drawing
  pcd_viewer.spinOnce(100, true);
  seg_viewer.spinOnce();

  // sleep for four seconds (camera parameters takes time)
  ros::Duration(4).sleep();

  pcl::ModelCoefficients sphere_coff;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  bool status = sphereDetector.segmentSphere(&pcd_viewer, cloud,
      segmented_cloud, sphere_coff);

  seg_viewer.addPointCloud(segmented_cloud, "segmented_cloud");

  if (status) {
    ROS_INFO_STREAM("Sphere detection successfull");
    pcl::PointXYZ detected_sphere(sphere_coff.values[0],
        sphere_coff.values[1], sphere_coff.values[2]);
    seg_viewer.addSphere(detected_sphere, sphere_coff.values[3], 0.2,
        0.3, 1.0, "detected_sphere");
    seg_viewer.addText3D("Sphere", detected_sphere, 0.05, 0.2, 0.3, 1.0, "sphere_text");
  } else {
    ROS_WARN_STREAM("Sphere detection failed");
  }

  seg_viewer.spin();

  //pcl::io::savePCDFileASCII("segmented_cloud.pcd", *segmented_cloud);
  return 0;
}
