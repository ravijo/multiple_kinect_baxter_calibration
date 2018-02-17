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

  inline void SphereDetector::PointCloudXYZRGBtoXYZHSV(pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZHSV>& out)
  {
    out.width = in.width;
    out.height = in.height;
    for (size_t i = 0; i < in.points.size(); i++)
    {
        pcl::PointXYZHSV p;
        pcl::PointXYZRGBtoXYZHSV(in.points[i], p);
        p.x = in.points[i].x; p.y = in.points[i].y; p.z = in.points[i].z; // bug in PCL 1.7
        out.points.push_back(p);
    }
  }

  inline void SphereDetector::PointCloudXYZHSVtoXYZRGB(pcl::PointCloud<pcl::PointXYZHSV>& in, pcl::PointCloud<pcl::PointXYZRGB>& out)
  {
    out.width = in.width;
    out.height = in.height;
    for (size_t i = 0; i < in.points.size(); i++)
    {
        pcl::PointXYZRGB p;
        pcl::PointXYZHSVtoXYZRGB(in.points[i], p);
        out.points.push_back(p);
    }
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

    PointCloudXYZRGBtoXYZHSV(*raw_cloud, *hsv_cloud);
    hsv_filter.setInputCloud(hsv_cloud);
    hsv_filter.filter(*hsv_flitered_cloud);
    PointCloudXYZHSVtoXYZRGB(*hsv_flitered_cloud, *rgb_cloud);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(rgb_cloud);
    ne.compute(*cloud_normals);
    seg.setInputCloud(rgb_cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, coefficients);

    return (inliers->indices.size() > 0);
  }
}
