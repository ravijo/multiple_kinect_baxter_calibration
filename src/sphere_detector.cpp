/**
 * sphere_detector.cpp: class file for detecting sphere
 * Author: Ravi Joshi
 * Date: 2018/02/20
 */

#include <utility.h>
#include <sphere_detector.h>

namespace pcl_project {
SphereDetector::SphereDetector(float sphere_radius, std::vector<int>* min_hsv,
    std::vector<int>* max_hsv, RansacParams* ransac_params,
    int overflow_offset, float sphere_z_min, float sphere_z_max) {
  radius = sphere_radius;

  min_hsv_ptr = new cv::Scalar(min_hsv->at(0), min_hsv->at(1),
      min_hsv->at(2));
  max_hsv_ptr = new cv::Scalar(max_hsv->at(0), max_hsv->at(1),
      max_hsv->at(2));

  prob = ransac_params->prob;
  weight = ransac_params->weight;
  epsilon = ransac_params->epsilon;
  max_itr = ransac_params->max_itr;
  d_thresh = ransac_params->d_thresh;
  tolerance = ransac_params->tolerance;
  k_neighbors = ransac_params->k_neighbors;

  offset = overflow_offset;

  sphere_z_min_max.x = sphere_z_min;
  sphere_z_min_max.y = sphere_z_max;

  initSphereDetector();
}

/*
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
 */

void SphereDetector::initSphereDetector() {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
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

cv::Rect SphereDetector::getBoundingRect(cv::Mat image) {
  // smooth image
  cv::GaussianBlur(image, image, cv::Size(5, 5), 0, 0);

  cv::Mat hsv_image, binary_image;
  cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV); // convert to hsv

  // remove pixels outside given range
  cv::inRange(hsv_image, *min_hsv_ptr, *max_hsv_ptr, binary_image);

  hsv_image.release(); // remove hsv_image from memory

  cv::Vec2d largest_contour(0, 0); // index, area

  // find contours from binary image
  cv::vector < cv::vector<cv::Point> > contours;
  findContours(binary_image, contours, CV_RETR_EXTERNAL,
      CV_CHAIN_APPROX_SIMPLE); // find contours

  binary_image.release(); // remove binary_image from memory

  // find largest contour
  for (int i = 0; i < contours.size(); i++) {
    double area = contourArea(cv::Mat(contours[i]));
    if (area > largest_contour[1]) {
      largest_contour[1] = area;
      largest_contour[0] = i;
    }
  }

  cv::Rect bound_rect = boundingRect(cv::Mat(contours[largest_contour[0]]));
  cv::Rect outer_rect(bound_rect);
  outer_rect.x -= bound_rect.width;
  outer_rect.y -= bound_rect.height;
  outer_rect.width = 3 * bound_rect.width;
  outer_rect.height = 3 * bound_rect.height;

  return outer_rect;
}

bool SphereDetector::convertMouseCoordsTo3DPC(vtkRenderWindowInteractor* iren,
    vtkPointPicker* point_picker, int mouse_x, int mouse_y,
    Eigen::Vector4f& point) {
  vtkRenderer* ren = iren->FindPokedRenderer(mouse_x, mouse_y);
  point_picker->Pick(mouse_x, mouse_y, 0.0, ren);

  int id = static_cast<int>(point_picker->GetPointId());
  if (id != -1 && point_picker->GetDataSet() != NULL) {
    double p[3];
    point_picker->GetDataSet()->GetPoint(id, p);
    for (size_t i = 0; i < 3; i++)
      point(i) = p[i];
    return true;
  } else
    return false;
}

bool SphereDetector::get3DMinMaxRange(pcl::visualization::PCLVisualizer* viewer,
    cv::Size img, cv::Rect rect, Eigen::Vector4f& min_3d,
    Eigen::Vector4f& max_3d) {
  vtkRenderWindowInteractor* iren =
      viewer->getRenderWindow()->GetInteractor();
  vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast(
      iren->GetPicker());

  // check if given bounding box is outside the window
  int start_x = rect.x < 0 ? offset : rect.x;
  int start_y = rect.y < 0 ? img.height - offset : img.height - rect.y;

  int end_x =
      rect.x + rect.width > img.width ?
          img.width - offset : rect.x + rect.width;
  int end_y =
      rect.y + rect.height > img.height ?
          offset : img.height - (rect.y + rect.height);

  Eigen::Vector4f s1, s2;
  bool status = convertMouseCoordsTo3DPC(iren, point_picker, start_x, start_y,
      s1);
  status = status
      && convertMouseCoordsTo3DPC(iren, point_picker, end_x, end_y, s2);

  //find minimum and maximum x y values
  if (s1(0) < s2(0)) {
    min_3d(0) = s1(0);
    max_3d(0) = s2(0);
  }

  if (s1(1) < s2(1)) {
    min_3d(1) = s1(1);
    max_3d(1) = s2(1);
  }

  min_3d(2) = sphere_z_min_max.x;
  max_3d(2) = sphere_z_min_max.y;

  min_3d(3) = 1.0;
  max_3d(3) = 1.0;

  return status;
}

void SphereDetector::crop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out, Eigen::Vector4f& min,
    Eigen::Vector4f& max) {
  box_filter.setMin(min);
  box_filter.setMax(max);
  box_filter.setInputCloud(in);
  box_filter.filter(*out);
}

bool SphereDetector::segmentSphere(pcl::visualization::PCLVisualizer* viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud,
    pcl::ModelCoefficients& coefficients) {
  cv::Mat image = utility::getImageFromPCLViewer(viewer);

  //src: https://stackoverflow.com/a/14028277
  cv::Size image_size = image.size();

  cv::Rect boundary = getBoundingRect(image);
  image.release(); // remove image from memory

  Eigen::Vector4f min_3d, max_3d;
  bool status = get3DMinMaxRange(viewer, image_size, boundary, min_3d,
      max_3d);

  crop(raw_cloud, filtered_cloud, min_3d, max_3d);

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*filtered_cloud, *xyz_cloud);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(xyz_cloud);
  ne.compute(*cloud_normals);
  seg.setInputCloud(xyz_cloud);
  seg.setInputNormals(cloud_normals);
  seg.segment(*inliers, coefficients);

  return (inliers->indices.size() > 0);
}
}
