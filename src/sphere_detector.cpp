/**
* sphere_detector.cpp: class file for sphere_detector
* Author: Ravi Joshi
* Date: 2018/02/20
*/

#include <utility.h>
#include <sphere_detector.h>

namespace pcl_project
{
  SphereDetector::SphereDetector(float sphere_radius, std::vector<float>* min_hsv, std::vector<float>* max_hsv, RansacParams* ransac_params)
  {
    radius = sphere_radius;

    minH = min_hsv->at(0);
    minS = min_hsv->at(1);
    minV = min_hsv->at(2);
    maxH = max_hsv->at(0);
    maxS = max_hsv->at(1);
    maxV = max_hsv->at(2);

    prob        = ransac_params->prob;
    weight      = ransac_params->weight;
    epsilon     = ransac_params->epsilon;
    max_itr     = ransac_params->max_itr;
    d_thresh    = ransac_params->d_thresh;
    tolerance   = ransac_params->tolerance;
    k_neighbors = ransac_params->k_neighbors;

    initHsvFilter();
    initSphereDetector();
  }

  void SphereDetector::initHsvFilter()
  {
    pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZHSV>());
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::Ptr(new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::GT, minH)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::Ptr(new pcl::FieldComparison<pcl::PointXYZHSV>("h", pcl::ComparisonOps::LT, maxH)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::Ptr(new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::GT, minS)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::Ptr(new pcl::FieldComparison<pcl::PointXYZHSV>("s", pcl::ComparisonOps::LT, maxS)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::Ptr(new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::GT, minV)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZHSV>::Ptr(new pcl::FieldComparison<pcl::PointXYZHSV>("v", pcl::ComparisonOps::LT, maxV)));
    hsv_filter.setCondition(condition);
  }

  void SphereDetector::initSphereDetector()
  {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    ne.setKSearch(k_neighbors);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
    seg.setNormalDistanceWeight(weight);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_itr);
    seg.setDistanceThreshold(d_thresh);
    seg.setProbability(prob);
    seg.setRadiusLimits(radius - tolerance, radius + tolerance);
    seg.setEpsAngle(pcl::deg2rad(epsilon));
  }

  bool SphereDetector::segmentSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud, pcl::ModelCoefficients& coefficients)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_flitered_cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    utility::PointCloudXYZRGBtoXYZHSV(*raw_cloud, *hsv_cloud);
    hsv_filter.setInputCloud(hsv_cloud);
    hsv_filter.filter(*hsv_flitered_cloud);
    utility::PointCloudXYZHSVtoXYZRGB(*hsv_flitered_cloud, *rgb_cloud);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(rgb_cloud);
    ne.compute(*cloud_normals);
    seg.setInputCloud(rgb_cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, coefficients);

    return (inliers->indices.size() > 0);
  }
}
