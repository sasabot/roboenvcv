// note: code assumes z depth, y up/down in camera coords.

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <jsk_recognition_msgs/PeoplePoseArray.h>
#include <roboenvcv/Int32Stamped.h> // for downstream signal
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
#include <cmath>
#include <limits>
#include <vector>
#include <utility>
#include <algorithm>

std::vector<sensor_msgs::PointCloud2> v_depth_;
std::mutex depth_mutex_;
cv::CascadeClassifier haar_detector_;
int max_faces_;
int x_scale_;
int y_scale_;
ros::Publisher face_roi_info_publisher_;
ros::Publisher downstream_publisher_; // to /local
ros::Publisher full_downstream_publisher_; // to /global/withid
int queue_size_;
double time_thre_;

// parameters
float head_height_ = 0.24;
float D_ = 0.25;

#define __DEBUG__

#ifdef __DEBUG__
std::mutex dbg_mutex_;
cv::Mat dbg_img_;
#endif

void DepthCallback(const sensor_msgs::PointCloud2::ConstPtr &_msg) {
  depth_mutex_.lock();
  if (v_depth_.size() >= queue_size_) // only keep queue_size_ depth images
    v_depth_.erase(v_depth_.begin());
  v_depth_.push_back(*_msg);
  depth_mutex_.unlock();
}

void PeoplePoseCallback
(const jsk_recognition_msgs::PeoplePoseArray::ConstPtr &_msg) {
  // get point cloud
  pcl::PCLPointCloud2 pcl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
    (new pcl::PointCloud<pcl::PointXYZRGB>);

  // find point cloud w/ closest time frame
  double secs = _msg->header.stamp.toSec();
  int found = -1;
  double time_diff = std::numeric_limits<double>::max();
  depth_mutex_.lock();
  for (auto d = v_depth_.begin(); d != v_depth_.end(); ++d) {
    double diff = fabs(secs - d->header.stamp.toSec());
    if (diff < time_diff) {
      time_diff = diff;
      found = static_cast<int>(d - v_depth_.begin());
    }
  }
  if (found >= 0 && time_diff < time_thre_) {
    pcl_conversions::toPCL(v_depth_.at(found), pcl);
    ROS_INFO("found %f == %f", v_depth_.at(found).header.stamp.toSec(), secs);
    v_depth_.erase(v_depth_.begin(), v_depth_.begin() + found);
  } else {
    depth_mutex_.unlock();
    ROS_WARN("depth image w/ close time frame not found! %f ~ %f, looking for %f",
             v_depth_.front().header.stamp.toSec(),
             v_depth_.back().header.stamp.toSec(), secs);
    return;
  }
  depth_mutex_.unlock();

  pcl::fromPCLPointCloud2(pcl, *cloud);

  if (cloud->points.size() == 0) { // bad point cloud
    ROS_ERROR("bad point cloud!!!!!!");
    return;
  }

  if (_msg->poses.size() == 0) {
    // no one found, send person lost signal to downstream
    roboenvcv::Int32Stamped msg;
    msg.header = _msg->header;
    msg.data = 0;
    full_downstream_publisher_.publish(msg);
    return;
  }

  // get openpose results
  std::vector<std::pair<float, std::pair<int, int>> > pose_pixels;
  for (auto obj = _msg->poses.begin(); obj != _msg->poses.end(); ++obj) {
    // find neck position
    auto limb = std::find(obj->limb_names.begin(), obj->limb_names.end(), "Neck");
    if (limb == obj->limb_names.end()) {
      ROS_WARN("could not find neck limb in person!!!!!");
      continue;
    }

    // get value of neck limb
    int limb_id = static_cast<int>(limb - obj->limb_names.begin());
    int pixel_x = static_cast<int>(obj->poses.at(limb_id).position.x);
    int pixel_y = static_cast<int>(obj->poses.at(limb_id).position.y);

    ROS_INFO("----------pixels: %d, %d", pixel_x, pixel_y);

    // reject invalid
    auto p_ref = cloud->points.begin() + pixel_x + pixel_y * cloud->width;
    if (std::isinf(p_ref->z) || std::isnan(p_ref->z)) {
      ROS_WARN("invalid z value detected from neck!");
      continue;
    }

    pose_pixels.push_back({p_ref->z, {pixel_x, pixel_y}});
  }

  // no valid person found, return
  if (pose_pixels.size() == 0) {
    ROS_WARN("no valid person found!!!!!");
    return;
  }

  // get depth threshold using openpose results
  std::sort(pose_pixels.begin(), pose_pixels.end(),
            [](std::pair<float, std::pair<int, int>> _a,
               std::pair<float, std::pair<int, int>> _b) {
              return (_a.first < _b.first);
            });
  if (pose_pixels.size() > max_faces_)
    // exculde faces by depth if more than track number
    pose_pixels.erase(pose_pixels.begin() + max_faces_, pose_pixels.end());
  float depth_threshold = pose_pixels.back().first + D_;

  // inform current tracked number of people to id_mapper
  // note, useful when number of people < max_faces_
  roboenvcv::Int32Stamped nump_msg;
  nump_msg.header = _msg->header;
  nump_msg.data = std::min(static_cast<int>(_msg->poses.size()), max_faces_);
  full_downstream_publisher_.publish(nump_msg);

  // compress image for speedup (~320x180)
  int in_height = cloud->height;
  int in_width = cloud->width;
  // get compressed gray image
  cv::Mat img(in_height, in_width, CV_8U);
  int i = 0;
  int j = 0;
  for (int k = 0; k < cloud->points.size(); ) {
    auto p = cloud->begin() + k;
    if (std::isnan(p->z))
      img.at<char>(i, j++) = 255;
    else if (p->z > depth_threshold)
      img.at<char>(i, j++) = 255;
    else
      img.at<char>(i, j++) = static_cast<uint8_t>((p->r + p->g + p->b) / 3);
    // k += 2; // stride by 1
    ++k;
    if (j >= img.cols) {
      j = 0;
      ++i;
      // k = ((i * cloud->width) << 1);
      k = (i * cloud->width);
    }
  }
  std::vector<cv::Rect> heads;
  haar_detector_.detectMultiScale(img, heads); // get roi

  if (heads.size() < pose_pixels.size()) {
    ROS_WARN("expecting %d heads, only %d heads found from haar",
             static_cast<int>(_msg->poses.size()), static_cast<int>(heads.size()));
    if (heads.size() == 0)
      return; // no result from haar
  } else {
    ROS_INFO("expecting %d heads, found %d heads from haar",
             static_cast<int>(_msg->poses.size()), static_cast<int>(heads.size()));
  }

  roboenvcv::RegionOfInterestInfos msg;

  // use openpose results to remove noise

  // find pair of openpose result and haar result
  std::vector<std::pair<float, std::pair<int, int>> >
    result_pairs(heads.size(), {std::numeric_limits<float>::max(), {-1, -1}});
  for (auto obj = pose_pixels.begin(); obj != pose_pixels.end(); ++obj) {
    auto p_ref = cloud->points.begin() + obj->second.first + obj->second.second * cloud->width;

    // find most closest head from haar
    float pixel_diff = std::numeric_limits<float>::max();
    std::vector<cv::Rect>::iterator head_roi;
    for (auto head = heads.begin(); head != heads.end(); ++head) {
      // float diff_x = abs(((head->x + (head->width >> 1)) << 1) - obj->second.first);
      // float diff_y = fabs(((head->y + head->height) << 1) - obj->second.second);
      float diff_y = abs((head->y + head->height) - obj->second.second);
      float diff_x = abs((head->x + (head->width >> 1)) - obj->second.first);
      if ((diff_x + diff_y) < pixel_diff) {
        pixel_diff = diff_x + diff_y;
        head_roi = head;
      }
    }
    // overwrite previous pair if better match
    int h_idx = static_cast<int>(head_roi - heads.begin());
    auto l = result_pairs.begin() + h_idx;
    if (pixel_diff < l->first) {
      l->first = pixel_diff;
      l->second = {static_cast<int>(p_ref - cloud->points.begin()), h_idx};
    }
  }

#ifdef __DEBUG__
  for (auto head = heads.begin(); head != heads.end(); ++head)
    cv::rectangle(img, *head, cv::Scalar(0));
#endif

  // get final results
  for (auto it = result_pairs.begin(); it != result_pairs.end(); ++it) {
    ROS_INFO("found pairs: %d, %d", it->second.first, it->second.second);
    if (it->second.first < 0 && it->second.second < 0)
      continue;

    auto p_ref = cloud->points.begin() + it->second.first;
    auto head_roi = heads.begin() + it->second.second;

    roboenvcv::RegionOfInterestInfo info;
    // info.roi.x_offset = ((head_roi->x + static_cast<int>(head_roi->width * 0.2)) << 1) * x_scale_;
    // info.roi.y_offset = (head_roi->y << 1) * y_scale_;
    // info.roi.width = static_cast<int>(head_roi->width * 1.2) * x_scale_;
    // info.roi.height = static_cast<int>(head_roi->height * 1.2) * y_scale_;
    info.roi.x_offset = ((head_roi->x + static_cast<int>(head_roi->width * 0.2))) * x_scale_;
    info.roi.y_offset = (head_roi->y) * y_scale_;
    info.roi.width = static_cast<int>(head_roi->width * 0.6) * x_scale_;
    info.roi.height = static_cast<int>(head_roi->height * 0.6) * y_scale_;
    info.center3d.x = p_ref->x; // set x value same as neck
    info.center3d.y = p_ref->y + head_height_;
    info.center3d.z = p_ref->z;

    msg.infos.push_back(info);
  }

  if (msg.infos.size() > 0) {
    msg.x_scale = x_scale_;
    msg.y_scale = y_scale_;
    msg.header = _msg->header;
    face_roi_info_publisher_.publish(msg);
  }

#ifdef __DEBUG__
  dbg_mutex_.lock();
  cv::resize(img, dbg_img_, cv::Size(img.cols >> 1, img.rows >> 1));
  dbg_mutex_.unlock();
#endif
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "extract_face_roi_info_from_openpose");
  ros::NodeHandle nh("~");

  x_scale_ = 2;
  y_scale_ = 2;
  nh.getParam("image/x_scale", x_scale_);
  nh.getParam("image/y_scale", y_scale_);

  std::string filename;
  nh.getParam("haarcascade", filename);
  ROS_INFO("loading %s...", filename.c_str());
  if (!haar_detector_.load(filename)) {
    ROS_ERROR("failed to load %s", filename.c_str());
    std::exit(0);
  }

  max_faces_ = 3;
  nh.getParam("/tracknfaces", max_faces_);

  queue_size_ = 10;
  nh.getParam("queue_size", queue_size_);

  time_thre_ = 0.1;
  nh.getParam("timestamp", time_thre_);

  face_roi_info_publisher_ =
    nh.advertise<roboenvcv::RegionOfInterestInfos>
    ("/roboenvcv/cropped/boundings", 1);

  full_downstream_publisher_ =
    nh.advertise<roboenvcv::Int32Stamped>
    ("/roboenvcv/personcount", 1);

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

  ros::CallbackQueue bb_queue;
  ros::SubscribeOptions bb_ops =
    ros::SubscribeOptions::create<jsk_recognition_msgs::PeoplePoseArray>(
        "/people_pose_estimation_2d/pose",
        1,
        boost::bind(&PeoplePoseCallback, _1),
        ros::VoidPtr(),
        &bb_queue);
  ros::Subscriber bb_sub = nh.subscribe(bb_ops);
  ros::AsyncSpinner bb_spinner(1, &bb_queue);
  bb_spinner.start();

#ifdef __DEBUG__
  cv::namedWindow("debug_window", cv::WINDOW_AUTOSIZE);

  dbg_mutex_.lock();
  dbg_img_ = cv::Mat(1, 1, CV_8U).clone();
  dbg_mutex_.unlock();
  while (ros::ok()) {
    dbg_mutex_.lock();
    cv::imshow("debug_window", dbg_img_);
    dbg_mutex_.unlock();
    cv::waitKey(3);
  }
#else
  ros::spin();
#endif
}
