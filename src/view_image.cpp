/**
 * view_image.cpp: utility to view pixel values of image
 * Author: Ravi Joshi
 * Date: 2018/04/13
 */

// ros header
#include <ros/ros.h>

// opencv2 header
#include <opencv2/opencv.hpp>

cv::Mat src_image;
std::string window_text = "View Image (Press 'q' to exit)";

double scale = 0.6;
cv::Scalar color(0, 0, 255, 255);  // BGRA order
int font = cv::FONT_HERSHEY_SIMPLEX;

static void onMouse(int event, int x, int y, int f, void*)
{
  cv::Mat image = src_image.clone();
  cv::Vec3b rgb = image.at<cv::Vec3b>(y, x);
  int B = rgb.val[0];
  int G = rgb.val[1];
  int R = rgb.val[2];

  cv::Mat HSV;
  cv::Mat BGR = image(cv::Rect(x, y, 1, 1));
  cv::cvtColor(BGR, HSV, CV_BGR2HSV);

  cv::Vec3b hsv = HSV.at<cv::Vec3b>(0, 0);
  int H = hsv.val[0];
  int S = hsv.val[1];
  int V = hsv.val[2];

  char text[30];
  sprintf(text, "X=%d", x);
  putText(image, text, cv::Point(10, 40), font, scale, color, 1, 8);

  sprintf(text, "Y=%d", y);
  putText(image, text, cv::Point(10, 70), font, scale, color, 1, 8);

  sprintf(text, "R=%d", R);
  putText(image, text, cv::Point(80, 40), font, scale, color, 1, 8);

  sprintf(text, "G=%d", G);
  putText(image, text, cv::Point(80, 70), font, scale, color, 1, 8);

  sprintf(text, "B=%d", B);
  putText(image, text, cv::Point(80, 100), font, scale, color, 1, 8);

  sprintf(text, "H=%d", H);
  putText(image, text, cv::Point(150, 40), font, scale, color, 1, 8);

  sprintf(text, "S=%d", S);
  putText(image, text, cv::Point(150, 70), font, scale, color, 1, 8);

  sprintf(text, "V=%d", V);
  putText(image, text, cv::Point(150, 100), font, scale, color, 1, 8);

  cv::imshow(window_text, image);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "view_image_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  std::string image_file;
  nh.getParam("file", image_file);

  cv::namedWindow(window_text, CV_WINDOW_AUTOSIZE);
  src_image = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
  if (!src_image.data)  // Check for invalid input
  {
    std::cerr << "Could not open or find the image '" << image_file << "'"
              << std::endl;
    return -1;
  }
  else
  {
    std::cout << "Loading '" << image_file << "'" << std::endl;
  }

  cv::imshow(window_text, src_image);
  cv::setMouseCallback(window_text, onMouse, 0);
  cv::waitKey();
  return 0;
}
