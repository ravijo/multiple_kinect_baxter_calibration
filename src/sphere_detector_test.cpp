/**
* sphere_detector_test.cpp: test file for sphere_detector
* Author: Ravi Joshi
* Date: 2018/02/20
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <sphere_detector.h>

float minH = 60, minS = 0.6, minV = 0.3;
float maxH = 64, maxS = 0.8, maxV = 1.0;
float radius = 0.034; // m

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sphere_detector_test", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  std::string pcdFileName = ros::package::getPath("multiple_kinect_baxter_calibration") + "/files/sphere_libfreenect2.pcd";

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile(pcdFileName, *rgbCloud);

  std::vector<float> min_hsv_values(3);
  std::vector<float> max_hsv_values(3);

  min_hsv_values[0] = minH; max_hsv_values[0] = maxH;
  min_hsv_values[1] = minS; max_hsv_values[1] = maxS;
  min_hsv_values[2] = minV; max_hsv_values[2] = maxV;

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

  pcl_project::SphereDetector sphereDetector(radius, &min_hsv_values, &max_hsv_values, &ransac_params);

  pcl::ModelCoefficients coefficients;
  bool status = sphereDetector.segmentSphere(rgbCloud, coefficients);

  if (status)
  {
      std::cout << coefficients.values[0] << ", " << coefficients.values[1] << ", "  << coefficients.values[2] << ", " << coefficients.values[3] << std::endl;
      pcl::PointXYZ detectedSphere(coefficients.values[0], coefficients.values[1], coefficients.values[2]);

      pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
      viewer.addPointCloud(rgbCloud, "rgb_cloud");
      viewer.addSphere(detectedSphere, coefficients.values[3], 255, 0, 0, "detected_sphere");
      viewer.initCameraParameters();
      bool result = viewer.getCameraParameters(argc, argv);
      viewer.spin();
  }
  else
  {
      std::cout << "Failed" << std::endl;
  }

  return 0;
}
