#include <ros/ros.h>
#include "linux_kinect/KinectInterface.hh"
#include "roboenvcv/roboenvcv.hh"
#include "roboenvcv/kinectutils.hh"

// for OCR
#include "linux_kinect/WindowsInterface.hh"
#include "roboenvcv/roboenvcv_extra.hh"

int main(int argc, char **argv)
{
  if (argc != 2) {
    printf("Please enter string to find.\n");
    std::exit(0);
  }

  std::string target_string(argv[1]);

  ros::init(argc, argv, "roboenvcv_kinect");
  ros::NodeHandle nh;

  // create log directory
  std::string dbgfolder = roboenvcv::createLogDir();

  // the sensor
  kinect::interface::KinectInterfacePtr kinect
    (new kinect::interface::KinectInterface(nh));

  // resize for calculation speed up
  float resize_x = 0.25;
  float resize_y = 0.334;

  // ocr
  windows::interface::WindowsInterfacePtr windows
    (new windows::interface::WindowsInterface(nh));


  /// ----------- preperation -----------

  // get kinect point clouds as xtion coordinate
  auto points = kinect->ReadPoints(resize_x, resize_y);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
    (new pcl::PointCloud<pcl::PointXYZRGB>);
  kinect::interface::GetCloud(points, cloud);

  // get kinect image as xtion coordinate
  auto image = kinect->ReadImage();
  cv::Mat img(image.height, image.width, CV_8UC3);
  kinect::interface::GetImage(image, img);

  // in debug mode, DetectObjectnessArea will destroy original image
  cv::Mat res = img.clone();


  /// ----------- the main code -----------

  // detect objects in scene
  auto scene =
    roboenvcv::DetectObjectnessArea(cloud, res, dbgfolder);

  // conduct ocr on objects
  // scene.first is list of objects, scene.second is excluded environment
  // scene.second is null in this example
  std::vector<int> objs =
    roboenvcv::FindTargetWithOcr
    ({target_string}, scene.first, img, windows, dbgfolder);

  // print ocr results
  roboenvcv::printNameProperties(scene.first, dbgfolder);

  // draw found target to results
  roboenvcv::drawBestMatch(scene.first, res, objs);


  /// ----------- draw results -----------

  cv::namedWindow("result", CV_WINDOW_NORMAL);
  cv::resizeWindow("result", 640, 480);
  cv::imshow("result", res);

  // keep alive for debug window
  cv::waitKey(1000000);
}
