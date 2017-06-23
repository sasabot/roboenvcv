#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <roboenvcv/RegionOfInterestInfo.h>
#include <roboenvcv/RegionOfInterestInfos.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <mutex>
#include <functional>
#include <cmath>
#include <limits>
#include <vector>
#include <utility>

#include <chrono>

int x_scale_;
int y_scale_;
bool y_up_;
float depth_threshold_;
ros::Publisher face_roi_info_publisher_;
ros::Publisher gray_scale_image_;
cv::CascadeClassifier haar_detector_;

std::mutex dbg_mutex_;
cv::Mat dbg_img_;

float head_height_ = 0.35;
float D_ = 0.25;

void DepthCallback(const sensor_msgs::PointCloud2::ConstPtr &_msg) {
  // auto t1 = std::chrono::high_resolution_clock::now();

  // get point cloud
  pcl::PCLPointCloud2 pcl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
    (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::fromPCLPointCloud2(pcl, *cloud);

  if (cloud->points.size() == 0) { // bad point cloud
    ROS_ERROR("bad point cloud!!!!!!");
    return;
  }

  // compress image for speedup (~320x180 original:240x135)
  int in_height = (cloud->height >> 1);
  int in_width = (cloud->width >> 1);

  // get compressed gray image
  cv::Mat img(in_height, in_width, CV_8U);
  int i = 0;
  int j = 0;
  for (int k = 0; k < cloud->points.size(); ) {
    auto p = cloud->begin() + k;
    if (std::isnan(p->z))
      img.at<char>(i, j++) = 255;
    else if (p->z > depth_threshold_)
      img.at<char>(i, j++) = 255;
    else
      img.at<char>(i, j++) = static_cast<uint8_t>((p->r + p->g + p->b) / 3);
    k += 2; // stride by 2
    if (j >= img.cols) {
      j = 0;
      ++i;
      k = ((i * cloud->width) << 1);
    }
  }

  roboenvcv::RegionOfInterestInfos msg;

  // get roi and 3d position
  std::vector<cv::Rect> heads;
  haar_detector_.detectMultiScale(img, heads);
  for (auto head = heads.begin(); head != heads.end(); ++head) {
    int x = ((head->x + static_cast<int>(head->width * 0.2)) << 1);
    int y = (head->y << 1);
    int half_width = static_cast<int>(head->width * 0.6);
    int half_height = static_cast<int>(head->height * 0.6);

    auto p = cloud->begin() + (y + half_height) * cloud->width + x + half_width; 

    // find y height
    int width = (half_width << 1);
    int height = (half_height << 1);
    int xmax = x + width;
    int ymax = y + height;
    float threshold_z = p->z + D_;
    float minimum_y = std::numeric_limits<float>::max();
    float maximum_y = std::numeric_limits<float>::min();
    for (size_t j = y; j < ymax; ++j) {
      int row_points = 0;
      float average_y = 0.0;
      auto p_ij = cloud->points.begin() + x + j * cloud->width;
      for (size_t i = x; i < xmax; ++i) {
        if (!std::isinf(p_ij->y) && !std::isnan(p_ij->y) && (p_ij->z < threshold_z)) {
          average_y += p_ij->y;
          ++row_points;
        }
        ++p_ij;
      }
      average_y /= row_points;
      if (row_points != 0){
        if (average_y < minimum_y)
          minimum_y = average_y;
        else if (average_y > maximum_y)
          maximum_y = average_y;
      }
    }

    if ((maximum_y - minimum_y) > head_height_) {
      cv::rectangle(img, *head, cv::Scalar(128));
      continue; // likely noise and not head
    }

    roboenvcv::RegionOfInterestInfo info;
    info.roi.x_offset = x * x_scale_;
    info.roi.y_offset = y * y_scale_;
    info.roi.width = width * x_scale_;
    info.roi.height = height * y_scale_;
    info.center3d.x = p->x;
    info.center3d.y = p->y;
    info.center3d.z = p->z;

    msg.infos.push_back(info);

    // for debug
    cv::rectangle(img, *head, cv::Scalar(0));
  }

  if (msg.infos.size() > 0) {
    msg.x_scale = x_scale_;
    msg.y_scale = y_scale_;
    face_roi_info_publisher_.publish(msg);
  }

  // auto t2 = std::chrono::high_resolution_clock::now();

  // ROS_INFO("haar detection: %d",
  //          std::chrono::duration_cast<std::chrono::milliseconds>(t2 -t1).count());

  // copy img to dbg_img_
  dbg_mutex_.lock();
  dbg_img_ = img.clone();
  dbg_mutex_.unlock();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "extract_face_roi_info_from_haar");
  ros::NodeHandle nh("~");

  x_scale_ = 6;
  y_scale_ = 6;
  nh.getParam("image/x_scale", x_scale_);
  nh.getParam("image/y_scale", y_scale_);

  y_up_ = true;
  nh.getParam("y_up", y_up_);

  std::string filename;
  nh.getParam("haarcascade", filename);
  ROS_INFO("loading %s...", filename.c_str());
  if (!haar_detector_.load(filename)) {
    ROS_ERROR("failed to load %s", filename.c_str());
    std::exit(0);
  }

  depth_threshold_ = 2.0;
  nh.getParam("depth_threshold", depth_threshold_);

  face_roi_info_publisher_ =
    nh.advertise<roboenvcv::RegionOfInterestInfos>
    ("/roboenvcv/cropped/boundings", 1);

  cv::namedWindow("debug_window", cv::WINDOW_AUTOSIZE);

  ros::CallbackQueue image_queue;
  ros::SubscribeOptions image_ops =
    ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
        "/camera/low_resolution_rgbd_points",
        1,
        boost::bind(&DepthCallback, _1),
        ros::VoidPtr(),
        &image_queue);
  ros::Subscriber image_sub = nh.subscribe(image_ops);
  ros::AsyncSpinner image_spinner(1, &image_queue);
  image_spinner.start();

  // below for debug
  dbg_mutex_.lock();
  dbg_img_ = cv::Mat(1, 1, CV_8U).clone();
  dbg_mutex_.unlock();
  while (ros::ok()) {
    dbg_mutex_.lock();
    cv::imshow("debug_window", dbg_img_);
    dbg_mutex_.unlock();
    cv::waitKey(3);
  }
}
