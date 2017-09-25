#include <ros/ros.h>
#include "linux_kinect/KinectInterface.hh"
#include "roboenvcv/roboenvcv.hh"
#include "roboenvcv/kinectutils.hh"

int main(int argc, char **argv)
{
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

  // set detection settings
  roboenvcv::detectsettings settings;
  settings.second_clustering = true;
  settings.remove_nan_in_sc = true;
  settings.leap_threshold_in_sc = 0.1;

  // detect objects in scene
  auto scene =
    roboenvcv::DetectObjectnessArea(cloud, res, dbgfolder, settings);

  // from scene, find objects that are green
  // scene.first is list of objects, scene.second is excluded environment
  // scene.second is null in this example
  std::vector<int> greens = roboenvcv::FindTarget("green", scene.first);

  // abort if green objects were not found
  if (greens.size() == 0)
    std::exit(0);

  // filter green objects from original detection list
  std::vector<roboenvcv::objectarea>
    greenobjs = roboenvcv::filter(scene.first, greens);

  // draw found target to results
  roboenvcv::drawBestMatch(scene.first, res, greens);

  // print depth of found object
  printf("depth: %f\n", greenobjs.at(0).center3d.z());

  /// ----------- draw results -----------

  cv::namedWindow("result", CV_WINDOW_NORMAL);
  cv::resizeWindow("result", 640, 480);
  cv::imshow("result", res);

  // keep alive for debug window
  cv::waitKey(1000000);
}
