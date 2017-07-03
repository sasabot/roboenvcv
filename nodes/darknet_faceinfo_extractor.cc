// note: this code is not stable. noise sensitive, sudden freeze.

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <roboenvcv/RegionOfInterestInfo.h>
#include <roboenvcv/RegionOfInterestInfos.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <mutex>
#include <functional>
#include <cmath>
#include <limits>
#include <vector>
#include <utility>

sensor_msgs::PointCloud2 depth_;
std::mutex depth_mutex_;
int x_scale_;
int y_scale_;
bool y_up_;
std::string frame_id_;
ros::Publisher face_roi_info_publisher_;

// parameters
float head_height_ = 0.24;
float D_ = 0.25;

void DepthCallback(const sensor_msgs::PointCloud2::ConstPtr &_msg) {
  depth_mutex_.lock();
  depth_ = *_msg;
  depth_mutex_.unlock();
}

void BoundingsCallback
(const darknet_ros_msgs::BoundingBoxes::ConstPtr &_msg) {
  // get point cloud
  pcl::PCLPointCloud2 pcl;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
    (new pcl::PointCloud<pcl::PointXYZRGB>);
  depth_mutex_.lock();
  pcl_conversions::toPCL(depth_, pcl);
  depth_mutex_.unlock();
  pcl::fromPCLPointCloud2(pcl, *cloud);

  if (cloud->points.size() == 0) { // bad point cloud
    ROS_ERROR("bad point cloud!!!!!!");
    return;
  }

  roboenvcv::RegionOfInterestInfos msg;

  // find person boundings
  for (auto obj = _msg->boundingBoxes.begin();
       obj != _msg->boundingBoxes.end(); ++obj) {
    // look for person only
    if (obj->Class != "person" + frame_id_)
      continue;

    int xmin = obj->xmin;
    int xmax = obj->xmax;
    int ymin = obj->ymin;
    int ymax = obj->ymax;
    int bb_in_width = xmax - xmin;
    int bb_in_height = ymax - ymin;

    // find nearest z
    int y_one_eight = ymin + bb_in_height / 8;
    float nearest_z = std::numeric_limits<float>::max();
    for (size_t j = ymin; j < y_one_eight; ++j) {
      auto p = cloud->points.begin() + xmin + j * cloud->width;
      for (size_t i = 0; i < bb_in_width; ++i) {
        if (!std::isinf(p->z) && !std::isnan(p->z) && p->z < nearest_z)
          nearest_z = p->z;
        ++p;
      }
    }
    // ROS_INFO("----------z: %f", nearest_z);

    // filter point cloud
    float threshold_z = nearest_z + D_;
    for (size_t j = ymin; j < ymax; ++j) {
      auto p = cloud->points.begin() + xmin + j * cloud->width;
      for (size_t i = 0; i < bb_in_width; ++i) {
        if (p->z > threshold_z) {
          p->x = std::numeric_limits<float>::quiet_NaN();
          p->y = std::numeric_limits<float>::quiet_NaN();
          p->z = std::numeric_limits<float>::quiet_NaN();
        }
        ++p;
      }
    }

    // find head y region
    std::function<bool(float, float)> condition;
    if (y_up_)
      condition = [&](float _a, float _b) {return _a < (_b - head_height_);};
    else
      condition = [&](float _a, float _b) {return _a > (_b + head_height_);};
    int bb_out_top_y = -1, bb_out_bottom_y;
    float bb_out_top_y_value;
    float center_y;
    for (size_t j = ymin; j < ymax; ++j) {
      int row_points = 0;
      float average_y = 0.0;
      auto p = cloud->points.begin() + xmin + j * cloud->width;
      for (size_t i = xmin; i < xmax; ++i) {
        if (!std::isinf(p->y) && !std::isnan(p->y)) {
          average_y += p->y;
          ++row_points;
        }
        ++p;
      }
      average_y /= row_points;
      if (row_points != 0){
        if (bb_out_top_y < 0) {
          bb_out_top_y = j;
          bb_out_top_y_value = average_y;
        } else if (condition(average_y, bb_out_top_y_value)) {
          bb_out_bottom_y = j;
          center_y = (bb_out_top_y_value + average_y) * 0.5;
          break;
        }
      }
    }
    if (bb_out_top_y < 0 || bb_out_bottom_y == 0) {
      ROS_ERROR("could not detect head y region!");
      return;
    }
    int bb_out_height = bb_out_bottom_y - bb_out_top_y;

    // find head x region
    std::vector<std::pair<int, float> > average_x_value_in_column;
    for (size_t i = xmin; i < xmax; ++i) {
      int column_points = 0;
      float average_x = 0.0;
      auto p = cloud->points.begin() + i + bb_out_top_y * cloud->width;
      for (size_t j = 0; j < bb_out_height; ++j) {
        if (!std::isinf(p->x) && !std::isnan(p->x)) {
          average_x += p->x;
          ++column_points;
        }
        p += cloud->width;
      }
      if (column_points > 0)
        average_x_value_in_column.push_back({i, average_x /= column_points});
    }
    if (average_x_value_in_column.size() == 0) {
      ROS_ERROR("detected zero points for head!");
      return; // unexpected
    }
    int bb_center_x =
      average_x_value_in_column.at(average_x_value_in_column.size() >> 1).first;
    float center_x =  // get median
      average_x_value_in_column.at(average_x_value_in_column.size() >> 1).second;
    int bb_out_left_x = std::max(0, bb_center_x - (bb_out_height >> 1) - 10);
    int bb_out_right_x =
      std::min(bb_center_x + (bb_out_height >> 1), static_cast<int>(cloud->width) + 10);

    roboenvcv::RegionOfInterestInfo info;
    info.roi.x_offset = bb_out_left_x * x_scale_;
    info.roi.y_offset = bb_out_top_y * y_scale_;
    info.roi.width = (bb_out_right_x - bb_out_left_x) * x_scale_;
    info.roi.height = (bb_out_bottom_y - bb_out_top_y) * y_scale_;
    info.center3d.x = center_x;
    info.center3d.y = center_y;
    info.center3d.z = nearest_z;

    // if (y_up_) // mirror image
    //   info.roi.x_offset = cloud->width - info.roi.x_offset;

    msg.infos.push_back(info);
  }

  if (msg.infos.size() > 0) {
    msg.x_scale = x_scale_;
    msg.y_scale = y_scale_;
    face_roi_info_publisher_.publish(msg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "extract_face_roi_info_from_darknet");
  ros::NodeHandle nh("~");

  x_scale_ = 2;
  y_scale_ = 2;
  nh.getParam("image/x_scale", x_scale_);
  nh.getParam("image/y_scale", y_scale_);

  y_up_ = false;
  nh.getParam("y_up", y_up_);

  bool multigpu = false;
  nh.getParam("multiple_gpu", multigpu);
  frame_id_ = "";
  if (!multigpu) // identify sensor w/ frame_id_ instead of topic
    nh.getParam("camera_link", frame_id_);

  face_roi_info_publisher_ =
    nh.advertise<roboenvcv::RegionOfInterestInfos>
    ("/roboenvcv/cropped/boundings", 1);

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
    ros::SubscribeOptions::create<darknet_ros_msgs::BoundingBoxes>(
        "/darknet_ros/YOLO_BoundingBoxes",
        1,
        boost::bind(&BoundingsCallback, _1),
        ros::VoidPtr(),
        &bb_queue);
  ros::Subscriber bb_sub = nh.subscribe(bb_ops);
  ros::AsyncSpinner bb_spinner(1, &bb_queue);
  bb_spinner.start();

  ros::spin();
}
