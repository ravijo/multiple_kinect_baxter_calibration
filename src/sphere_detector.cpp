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
    int overflow_offset) {
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
  //cv::imwrite( "binary_image.jpg", binary_image); //just for checking

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

  double largest_contour_area = largest_contour[1];

  cv::Rect bound_rect;

  /*
  * calculate bouding rectangle only if
  * area associated with largest contour is more than zero
  */
  if(largest_contour[1] > 0)
      bound_rect = boundingRect(cv::Mat(contours[largest_contour[0]]));

  cv::Rect outer_rect(bound_rect);
  outer_rect.x -= bound_rect.width;
  outer_rect.y -= bound_rect.height;
  outer_rect.width = 3 * bound_rect.width;
  outer_rect.height = 3 * bound_rect.height;

  /*
  std::cout << "outer_rect " << outer_rect << std::endl;
  cv::imwrite( "binary_image.jpg", binary_image);

  rectangle( binary_image, bound_rect.tl(), bound_rect.br(), cv::Scalar(255, 255, 255), 2, 8, 0 );
  rectangle( binary_image, outer_rect.tl(), outer_rect.br(), cv::Scalar(255, 255, 255), 2, 8, 0 );
  cv::imwrite( "rect_binary_image.jpg", binary_image);
  */

  return outer_rect;
}

bool SphereDetector::convertMouseCoordsTo3DPC(vtkRenderWindowInteractor* iren,
    vtkPointPicker* point_picker, int mouse_x, int mouse_y,
    Eigen::Vector3f& point) {
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

void SphereDetector::get2DPoints(int points[])
{
  int x_1 = points[0], y_1 = points[1];
  int x_2 = points[2], y_2 = points[3];

  int x_e = mid(x_1, x_2),  y_e = mid(y_1, y_2);
  points[4] = x_e, points[5] = y_e;

  int x_a = x_e, y_a = y_1;
  points[6] = x_e, points[7] = y_e;

  int x_b = x_2, y_b = y_e;
  points[8] = x_e, points[9] = y_e;

  int x_c = x_e, y_c = y_2;
  points[10] = x_e, points[11] = y_e;

  int x_d = x_1, y_d = y_e;
  points[12] = x_e, points[13] = y_e;

  int x_g = mid(x_2, x_e), y_g = mid(y_2, y_e);
  points[14] = x_e, points[15] = y_e;

  int x_h = mid(x_d, x_c), y_h = mid(y_d, y_c);
  points[16] = x_e, points[17] = y_e;

  int x_i = mid(x_a, x_d), y_i = mid(y_a, y_d);
  points[18] = x_e, points[19] = y_e;

  int x_j = mid(x_a, x_e), y_j = mid(y_a, y_e);
  points[20] = x_e, points[21] = y_e;

  int x_k = mid(x_b, x_e), y_k = mid(y_b, y_e);
  points[22] = x_e, points[23] = y_e;

  int x_f = mid(x_a, x_b), y_f = mid(y_a, y_b);
  points[24] = x_e, points[25] = y_e;

  int x_l = mid(x_c, x_e), y_l = mid(y_c, y_e);
  points[26] = x_e, points[27] = y_e;

  int x_m = mid(x_d, x_e), y_m = mid(y_d, y_e);
  points[28] = x_e, points[29] = y_e;
}

void SphereDetector::get3DMinMaxRange(pcl::visualization::PCLVisualizer* viewer,
    cv::Size img, cv::Rect rect, Eigen::Vector4f& min_3d,
    Eigen::Vector4f& max_3d) {
  vtkRenderWindowInteractor* iren =
      viewer->getRenderWindow()->GetInteractor();
  vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast(
      iren->GetPicker());

  // check if given bounding box is outside the window
  int start_x = rect.x < 0 ? offset : rect.x;
  int start_y = rect.y < 0 ? img.height - offset : img.height - rect.y;

  int end_x = rect.x + rect.width > img.width ? img.width - offset : rect.x + rect.width;
  int end_y = rect.y + rect.height > img.height ? offset : img.height - (rect.y + rect.height);

  int size = 2 * 15;  // x-y pair
  int points_2d[size];
  points_2d[0] = start_x; points_2d[1] = start_y;
  points_2d[2] = end_x;   points_2d[3] = end_y;
  get2DPoints(points_2d);

  pcl::PointCloud<pcl::PointXYZ> points_3d;
  points_3d.height   = 1;
  points_3d.is_dense = false;

  for (size_t i = 0; i < size; i+=2)
  {
    Eigen::Vector3f p;
    bool status = convertMouseCoordsTo3DPC(iren, point_picker, points_2d[i], points_2d[i + 1], p);
    if (status)
    {
      pcl::PointXYZ point(p(0), p(1), p(2));
      points_3d.points.push_back(point);
    }
  }

  // make sure to set the width
  points_3d.width = points_3d.points.size();

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(points_3d, min_pt, max_pt);

  min_3d(0) = min_pt.x;
  max_3d(0) = max_pt.x;

  min_3d(1) = min_pt.y;
  max_3d(1) = max_pt.y;

  // just to be on safe side lets define some offset
  min_3d(2) = min_pt.z - radius;
  max_3d(2) = max_pt.z + radius;

  min_3d(3) = 1;
  max_3d(3) = 1;
}

void SphereDetector::crop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out, Eigen::Vector4f& min,
    Eigen::Vector4f& max) {
  box_filter.setMin(min);
  box_filter.setMax(max);
  box_filter.setInputCloud(in);
  box_filter.filter(*out);
}

bool SphereDetector::validateDetection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  // calculate average RGB values of the input cloud
  int avg_r = 0, avg_g = 0, avg_b = 0;
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    avg_r += (int)cloud->points[i].r;
    avg_g += (int)cloud->points[i].g;
    avg_b += (int)cloud->points[i].b;
  }
  avg_r = avg_r/cloud->points.size();
  avg_g = avg_g/cloud->points.size();
  avg_b = avg_b/cloud->points.size();

  cv::Vec3b avg_hsv = utility::rgb_to_hsv_pixel(avg_r, avg_g, avg_b);

  bool in_between = true;
  for (size_t i = 0; i < 3; i++)
    in_between = in_between &&  between(avg_hsv.val[i], min_hsv_ptr->val[i], max_hsv_ptr->val[i]);

  return in_between;
}

bool SphereDetector::segmentSphere(pcl::visualization::PCLVisualizer* viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud,
    pcl::ModelCoefficients& coefficients) {
  cv::Mat image = utility::getImageFromPCLViewer(viewer);

  //src: https://stackoverflow.com/a/14028277
  cv::Size image_size = image.size();
  //cv::imwrite( "image.jpg", image); //just for checking

  cv::Rect boundary = getBoundingRect(image);
  image.release(); // remove image from memory

  Eigen::Vector4f min_3d, max_3d;
  get3DMinMaxRange(viewer, image_size, boundary, min_3d, max_3d);
  crop(raw_cloud, filtered_cloud, min_3d, max_3d);

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*filtered_cloud, *xyz_cloud);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud(xyz_cloud);
  ne.compute(*cloud_normals);
  seg.setInputCloud(xyz_cloud);
  seg.setInputNormals(cloud_normals);
  seg.segment(*inliers, coefficients);

  // we have detected a sphere
  if (inliers->indices.size() > 0)
  {
      // extract the sphere inliers from the input cloud
   		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
   		extract.setInputCloud(filtered_cloud);
   		extract.setIndices(inliers);
   		extract.setNegative(false);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   		// Get the points associated with the planar surface
   		extract.filter(*sphere_cloud);

      //pcl::io::savePCDFileASCII("sphere_cloud.pcd", *sphere_cloud);

      return validateDetection(sphere_cloud);
  }
  else
    return false;
}
}
