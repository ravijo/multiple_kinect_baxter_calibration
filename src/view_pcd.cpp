/**
 * view_pcd.cpp: utility to view point cloud from PCD file
 * Author: Ravi Joshi
 * Date: 2018/02/20
 */

// ros headers
#include <ros/ros.h>
#include <ros/package.h>

// utility header
#include <utility.h>

// pcl headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/crop_box.h>


/*
void keyboardEventOccurred(
  const pcl::visualization::KeyboardEvent& event, void* viewer) {
    if (event.getKeySym() == "q" && event.keyDown()){
        ROS_INFO_STREAM("Key 'q' was pressed. Exiting...");
        pcl::visualization::PCLVisualizer* v
            = static_cast<pcl::visualization::PCLVisualizer*>(viewer);
        v->close(); // doesn't work with PCL 1.7
    }
}
*/

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
      event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    ROS_INFO_STREAM("Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")");

    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    std::string text = "2D (" +  utility::to_string(event.getX()) + ", " + utility::to_string(event.getY()) + ")";
    viewer->addText(text, event.getX(), event.getY(), 20, 1, 1, 0, text);
  }
}

void pointPickingEventOccurred(
  const pcl::visualization::PointPickingEvent& event, void* viewer_void) {
    int index = event.getPointIndex();
  if (index == -1)
    return;

  float x, y, z;
  event.getPoint(x, y, z);
  ROS_INFO_STREAM(
  "Point picking event occurred. Point index="<< index << " coordinate ( " << x << ", "
  << y << ", " << z << ")");

  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  std::string text = "3D (" +  utility::to_string(x) + ", " + utility::to_string(y) + ", " + utility::to_string(z) + ")";

  pcl::PointXYZ point(x, y, z);
  viewer->addText3D(text, point, 0.08, 1, 0, 0, text);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "view_pcd_node", ros::init_options::AnonymousName);

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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
  new pcl::PointCloud<pcl::PointXYZRGB>);

  // load PCD file and check if invalid file is provided
  if (pcl::io::loadPCDFile < pcl::PointXYZRGB > (pcd_file, *cloud) == -1) {
    ROS_ERROR_STREAM("Couldn't read input file " << pcd_file);
    return -1;
  }else{
    ROS_INFO_STREAM("Loading '" << pcd_file << "'");
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

  ROS_INFO_STREAM("Hold down 'SHIFT' key while left-clicking to pick a point.");

  pcl::visualization::PCLVisualizer viewer("Cloud Viewer (Press 'j' to take screenshot)");

  viewer.addPointCloud(cloud, "cloud");

  viewer.initCameraParameters();
  viewer.setCameraParameters(camera);

  viewer.registerPointPickingCallback(pointPickingEventOccurred, (void*) &viewer);
  viewer.registerMouseCallback(mouseEventOccurred, (void*)&viewer);
  //viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

  viewer.spin();

  return 0;
}
