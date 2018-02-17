/**
* sphere_detector.h: header file for sphere_detector
* Author: Ravi Joshi
* Date: 2018/02/09
*/

#ifndef MULTIPLE_KINECT_BAXTER_CALIBRATION_SPHERE_DETECTOR_H_
#define MULTIPLE_KINECT_BAXTER_CALIBRATION_SPHERE_DETECTOR_H_

#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace pcl_project
{
  class RansacParams
  {
    public:
      int k_neighbors, max_itr;
      double weight, d_thresh, prob, tolerance, epsilon;
      RansacParams(int k_neighbors_ransac = 10, int max_itr_ransac = 1000,
          double weight_ransac = 0.05, double d_thresh_ransac = 0.005, double prob_ransac = 0.99999,
          double tolerance_ransac = 0.02, double epsilon_ransac = 15);
  };

  RansacParams::RansacParams(int k_neighbors_ransac, int max_itr_ransac, double weight_ransac,
      double d_thresh_ransac, double prob_ransac, double tolerance_ransac, double epsilon_ransac)
      : k_neighbors(k_neighbors_ransac)
      , max_itr(max_itr_ransac)
      , weight(weight_ransac)
      , d_thresh(d_thresh_ransac)
      , prob(prob_ransac)
      , tolerance(tolerance_ransac)
      , epsilon(epsilon_ransac){}

  class SphereDetector
  {
    public:

      /*
      * Constructor of SphereDetector class
      * Input:
      *   float sphere_radius = radius of the sphere in m
      *   std::vector<float>* min_hsv= minimum HSV values
      *   std::vector<float>* max_hsv = maximum HSV values
      *   RansacParams* ransac_params = Ransac params
      */
      SphereDetector(float sphere_radius, std::vector<float>* min_hsv, std::vector<float>* max_hsv, RansacParams* ransac_params);

      /*
      * This function segments the sphere from point cloud
      * Input:
      *   pcl::PointCloud<pcl::PointXYZRGB>& raw_cloud = point cloud for sphere detection
      * Output:
      *   pcl::ModelCoefficients& coefficients = coefficients for detected sphere
      * Returns:
      *   true if sphere is identified, false otherwise
      */
      bool segmentSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud, pcl::ModelCoefficients& coefficients);

    private:
      void initHsvFilter();
      void initSphereDetector();

      inline void PointCloudXYZRGBtoXYZHSV(pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZHSV>& out);
      inline void PointCloudXYZHSVtoXYZRGB(pcl::PointCloud<pcl::PointXYZHSV>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);

      pcl::ConditionalRemoval<pcl::PointXYZHSV> hsv_filter;
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
      pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
      float radius, minH, maxH, minS, maxS, minV, maxV;
      int k_neighbors, max_itr;
      double weight, d_thresh, prob, tolerance, epsilon;
  };
}

#endif        /* MULTIPLE_KINECT_BAXTER_CALIBRATION_SPHERE_DETECTOR_H_ */
