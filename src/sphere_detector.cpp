/**
 * sphere_detector.cpp: class file for detecting sphere
 * Author: Ravi Joshi
 * Date: 2018/02/20
 */

#include <utility.h>
#include <sphere_detector.h>

namespace pcl_project
{
SphereDetector::SphereDetector(float sphere_radius, std::vector<int>* min_hsv,
    std::vector<int>* max_hsv, RansacParams* ransac_params, int overflow_offset)
{
    radius = sphere_radius;

    min_hsv_ptr = new cv::Scalar(min_hsv->at(0), min_hsv->at(1), min_hsv->at(2));
    max_hsv_ptr = new cv::Scalar(max_hsv->at(0), max_hsv->at(1), max_hsv->at(2));

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

void SphereDetector::initSphereDetector()
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
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

bool SphereDetector::getBoundingRect(cv::Mat image, cv::Rect& bound_rect)
{
    // smooth image
    cv::GaussianBlur(image, image, cv::Size(5, 5), 0, 0);

    cv::Mat hsv_image, binary_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV); // convert to hsv

    // remove pixels outside given range
    cv::inRange(hsv_image, *min_hsv_ptr, *max_hsv_ptr, binary_image);
    // cv::imwrite( "binary_image.jpg", binary_image); //just for checking

    hsv_image.release(); // remove hsv_image from memory

    cv::Vec2d largest_contour(0, 0); // index, area

    // cv::imwrite( "binary_image.jpg", binary_image);

    // find contours from binary image
    cv::vector<cv::vector<cv::Point> > contours;
    findContours(binary_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // find contours

    binary_image.release(); // remove binary_image from memory

    // find largest contour
    for (int i = 0; i < contours.size(); i++)
    {
        double area = contourArea(cv::Mat(contours[i]));
        if (area > largest_contour[1])
        {
            largest_contour[1] = area;
            largest_contour[0] = i;
        }
    }

    double largest_contour_area = largest_contour[1];

    /*
    * calculate bouding rectangle only if
    * area associated with largest contour is more than zero
    */
    if (largest_contour[1] > 0)
    {
        bound_rect = boundingRect(cv::Mat(contours[largest_contour[0]]));

        /*
        rectangle( binary_image, bound_rect.tl(), bound_rect.br(), cv::Scalar(255, 255, 255), 2, 8,
        0 );
        cv::imwrite( "rect_binary_image.jpg", binary_image);
        */
        return true;
    }
    else
    {
        return false;
    }
}

bool SphereDetector::convertMouseCoordsTo3DPC(vtkRenderWindowInteractor* iren,
    vtkPointPicker* point_picker, int mouse_x, int mouse_y, Eigen::Vector3f& point)
{
    vtkRenderer* ren = iren->FindPokedRenderer(mouse_x, mouse_y);
    point_picker->Pick(mouse_x, mouse_y, 0.0, ren);

    int id = static_cast<int>(point_picker->GetPointId());
    if (id != -1 && point_picker->GetDataSet() != NULL)
    {
        double p[3];
        point_picker->GetDataSet()->GetPoint(id, p);
        for (size_t i = 0; i < 3; i++)
            point(i) = p[i];
        return true;
    }
    else
        return false;
}

void SphereDetector::get2DPoints(std::vector<cv::Point>& points)
{
    int x_a = points[0].x, y_a = points[0].y;
    int x_c = points[1].x, y_c = points[1].y;

    int x_b = x_c, y_b = y_a;
    int x_d = x_a, y_d = y_c;
    int x_q = mid(x_a, x_c), y_q = mid(y_a, y_c);
    int x_e = mid(x_a, x_d), y_e = mid(y_a, y_d);
    int x_f = mid(x_a, x_b), y_f = mid(y_a, y_b);
    int x_g = mid(x_b, x_c), y_g = mid(y_b, y_c);
    int x_h = mid(x_d, x_c), y_h = mid(y_d, y_c);
    int x_i = mid(x_h, x_e), y_i = mid(y_h, y_e);
    int x_j = mid(x_e, x_f), y_j = mid(y_e, y_f);
    int x_k = mid(x_f, x_g), y_k = mid(y_f, y_g);
    int x_l = mid(x_g, x_h), y_l = mid(y_g, y_h);
    int x_m = mid(x_i, x_l), y_m = mid(y_i, y_l);
    int x_n = mid(x_i, x_j), y_n = mid(y_i, y_j);
    int x_o = mid(x_j, x_k), y_o = mid(y_j, y_k);
    int x_p = mid(x_k, x_l), y_p = mid(y_k, y_l);

    points.push_back(cv::Point(x_a, y_a));
    points.push_back(cv::Point(x_b, y_b));
    points.push_back(cv::Point(x_c, y_c));
    points.push_back(cv::Point(x_d, y_d));
    points.push_back(cv::Point(x_e, y_e));
    points.push_back(cv::Point(x_f, y_f));
    points.push_back(cv::Point(x_g, y_g));
    points.push_back(cv::Point(x_h, y_h));
    points.push_back(cv::Point(x_i, y_i));
    points.push_back(cv::Point(x_j, y_j));
    points.push_back(cv::Point(x_k, y_k));
    points.push_back(cv::Point(x_l, y_l));
    points.push_back(cv::Point(x_m, y_m));
    points.push_back(cv::Point(x_n, y_n));
    points.push_back(cv::Point(x_o, y_o));
    points.push_back(cv::Point(x_p, y_p));
}

void SphereDetector::get3DMinMaxRange(pcl::visualization::PCLVisualizer* viewer, cv::Size img,
    cv::Rect rect, Eigen::Vector4f& min_3d, Eigen::Vector4f& max_3d)
{
    vtkRenderWindowInteractor* iren = viewer->getRenderWindow()->GetInteractor();
    vtkPointPicker* point_picker = vtkPointPicker::SafeDownCast(iren->GetPicker());

    // check if given bounding box is outside the window
    int start_x = rect.x < 0 ? offset : rect.x;
    int start_y = rect.y < 0 ? img.height - offset : img.height - rect.y;

    int end_x = rect.x + rect.width > img.width ? img.width - offset : rect.x + rect.width;
    int end_y = rect.y + rect.height > img.height ? offset : img.height - (rect.y + rect.height);

    std::vector<cv::Point> points_2d;
    points_2d.push_back(cv::Point(start_x, start_y));
    points_2d.push_back(cv::Point(end_x, end_y));
    get2DPoints(points_2d);

    pcl::PointCloud<pcl::PointXYZ> points_3d;
    points_3d.height = 1;
    points_3d.is_dense = false;
    for (size_t i = 0; i < points_2d.size(); i++)
    {
        Eigen::Vector3f p;
        if (convertMouseCoordsTo3DPC(iren, point_picker, points_2d[i].x, points_2d[i].y, p))
        {
            pcl::PointXYZ point(p(0), p(1), p(2));
            points_3d.points.push_back(point);
        }
    }
    // make sure to set the width
    points_3d.width = points_3d.points.size();

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(points_3d, min_pt, max_pt);

    // just to be on safe side lets define some offset
    min_3d(0) = min_pt.x - radius;
    max_3d(0) = max_pt.x + radius;

    min_3d(1) = min_pt.y - radius;
    max_3d(1) = max_pt.y + radius;

    min_3d(2) = min_pt.z - radius;
    max_3d(2) = max_pt.z + radius;

    min_3d(3) = 1;
    max_3d(3) = 1;
}

void SphereDetector::crop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out, Eigen::Vector4f& min, Eigen::Vector4f& max)
{
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
    avg_r = avg_r / cloud->points.size();
    avg_g = avg_g / cloud->points.size();
    avg_b = avg_b / cloud->points.size();

    cv::Vec3b avg_hsv = utility::rgb_to_hsv_pixel(avg_r, avg_g, avg_b);

    bool in_between = true;
    for (size_t i = 0; i < 3; i++)
        in_between
            = in_between && between(avg_hsv.val[i], min_hsv_ptr->val[i], max_hsv_ptr->val[i]);

    return in_between;
}

bool SphereDetector::segmentSphere(pcl::visualization::PCLVisualizer* viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud, pcl::ModelCoefficients& coefficients)
{
    cv::Mat image = utility::getImageFromPCLViewer(viewer);

    // src: https://stackoverflow.com/a/14028277
    cv::Size image_size = image.size();
    // cv::imwrite( "image.jpg", image); //just for checking

    cv::Rect boundary;
    bool status = getBoundingRect(image, boundary);
    image.release(); // remove image from memory

    if (!status)
        return false;

    Eigen::Vector4f min_3d, max_3d;
    get3DMinMaxRange(viewer, image_size, boundary, min_3d, max_3d);
    crop(raw_cloud, filtered_cloud, min_3d, max_3d);

    if (filtered_cloud->points.empty())
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*filtered_cloud, *xyz_cloud);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(xyz_cloud);
    ne.compute(*cloud_normals);
    seg.setInputCloud(xyz_cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, coefficients);
    std::cout << "[####] sphere detector after RANSAC" << std::endl;
    // we have detected a sphere
    if (inliers->indices.size() > 0)
    {
        // extract the sphere inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(filtered_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // get the points associated with sphere
        extract.filter(*sphere_cloud);

        // pcl::io::savePCDFileASCII("sphere_cloud.pcd", *sphere_cloud);
        bool color_comp = validateDetection(sphere_cloud);

        // make sure that sphere radius is in range
        bool radius_range = between(coefficients.values[3], radius - tolerance, radius + tolerance);

        return (color_comp && radius_range);
    }
    else
        return false;
}
}
