/**
* sphere_detector_test.cpp: test file for sphere_detector
* Author: Ravi Joshi
* Date: 2018/02/20
*/

#include <ros/ros.h>
#include <utility.h>
#include <ros/package.h>
#include <sphere_detector.h>
//#include <opencv2/opencv.hpp>

float minH = 50, minS = 0.2, minV = 0.5;
float maxH = 70, maxS = 1.0, maxV = 1.0;
float radius = 0.034; // m

/*
// src: https://stackoverflow.com/a/15009815/1175065
cv::Mat equalizeIntensity(const cv::Mat& inputImage)
{
    if(inputImage.channels() >= 3)
    {
        cv::Mat ycrcb;

        cv::cvtColor(inputImage, ycrcb, CV_BGR2YCrCb);

        std::vector<cv::Mat> channels;
        cv::split(ycrcb, channels);

        cv::equalizeHist(channels[0], channels[0]);

        cv::Mat result;
        cv::merge(channels, ycrcb);

        cv::cvtColor(ycrcb, result, CV_YCrCb2BGR);

        return result;
    }
    return cv::Mat();
}
*/

int main(int argc, char** argv)
{
//  ros::init(argc, argv, "sphere_detector_test", ros::init_options::AnonymousName);
//  ros::NodeHandle n;


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

  //std::string pcdFileName = ros::package::getPath("multiple_kinect_baxter_calibration") + "/files/sphere_libfreenect2.pcd";

  int total_frames = 44;
  int total_kinect = 3;

  int success[total_kinect];
  for (size_t i = 0; i < total_kinect; i++)
    success[i] = 0;

  for (size_t i = 0; i < total_frames; i++)
  {
    for (size_t j = 0; j < total_kinect; j++)
    {
      std::string pcdFileName = "/home/tom/Documents/ravi/Recycle_Bin/no-light/frame_"+ utility::to_string(i) + "_kinect_"+ utility::to_string(j) + ".pcd";

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::io::loadPCDFile(pcdFileName, *cloud);

      /*
      cv::Mat src = cv::Mat(cloud->width, cloud->height, CV_8UC3);
      for (size_t i = 0; i < cloud->points.size(); i++)
      {
        pcl::PointXYZRGB point = cloud->points[i];
        src.at<cv::Vec3b>(i, cloud->height)[0] = point.b;
        src.at<cv::Vec3b>(i, cloud->height)[1] = point.g;
        src.at<cv::Vec3b>(i, cloud->height)[2] = point.r;
      }

      cv::Mat dst = equalizeIntensity(src);

      for (size_t i = 0; i < cloud->points.size(); i++)
      {
        cloud->points[i].r = dst.at<cv::Vec3b>(i, cloud->height)[2];
        cloud->points[i].g = dst.at<cv::Vec3b>(i, cloud->height)[1];
        cloud->points[i].b = dst.at<cv::Vec3b>(i, cloud->height)[0];
      }
      */

      pcl::ModelCoefficients coefficients;
      bool status = sphereDetector.segmentSphere(cloud, coefficients);
      if(status) success[j] += 1;
    }
  }


  std::cout << "total_frames: " << total_frames << ". success kinect 1: " << success[0] << ", kinect 2: " << success[1] << ", kinect 3: " << success[2] << std::endl;


  /*
  if (status)
  {
      //std::cout << coefficients.values[0] << ", " << coefficients.values[1] << ", "  << coefficients.values[2] << ", " << coefficients.values[3] << std::endl;
      pcl::PointXYZ detectedSphere(coefficients.values[0], coefficients.values[1], coefficients.values[2]);

      pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
      viewer.addPointCloud(rgbCloud, "rgb_cloud");
      viewer.addSphere(detectedSphere, coefficients.values[3], 0, 255, 0, "detected_sphere");
      viewer.initCameraParameters();
      bool result = viewer.getCameraParameters(argc, argv);
      viewer.spin();
  }
  else
  {
      std::cout << "Failed" << std::endl;
  }
  */

  return 0;
}
