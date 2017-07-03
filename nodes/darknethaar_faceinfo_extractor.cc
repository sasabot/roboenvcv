#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <roboenvcv/Person.h>
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

sensor_msgs::PointCloud2 depth_;
std::mutex depth_mutex_;
bool y_up_;
float depth_threshold_;
int sensor_id_;
std::string frame_id_;
ros::Publisher person_publisher_;
cv::CascadeClassifier haar_detector_;

std::mutex dbg_mutex_;
cv::Mat dbg_img_;

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

  // get gray image
  cv::Mat img(cloud->height, cloud->width, CV_8U);
  int i = 0;
  int j = 0;
  for (auto p = cloud->points.begin(); p != cloud->points.end(); ) {
    j = 0;
    for (int x = 0; x < cloud->width; ++x) {
      if (std::isnan(p->z))
        img.at<char>(i, j++) = 255;
      else if (p->z > depth_threshold_)
        img.at<char>(i, j++) = 255;
      else
        img.at<char>(i, j++) = static_cast<uint8_t>((p->r + p->g + p->b) / 3);
       ++p;
    }
    ++i;
  }

  roboenvcv::Person msg;

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
    int in_width = xmax - xmin;
    int in_height = ymax - ymin;

    cv::Mat subimg = img(cv::Rect(xmin, ymin, in_width, in_height));

    // get head orientation and 3d position
    std::vector<cv::Rect> heads;
    haar_detector_.detectMultiScale(subimg, heads);

    roboenvcv::Person msg;
    msg.sensor_id = sensor_id_;
    geometry_msgs::Point position;

    if (heads.size() == 0) { // looking sideways
      // find nearest z
      int y_one_eight = ymin + in_height / 8;
      position.z = std::numeric_limits<float>::max();
      for (size_t j = ymin; j < y_one_eight; ++j) {
        auto p = cloud->points.begin() + xmin + j * cloud->width;
        for (size_t i = 0; i < in_width; ++i) {
          if (!std::isinf(p->z) && !std::isnan(p->z) && p->z < position.z)
            position.z = p->z;
          ++p;
        }
      }
      // ROS_INFO("----------z: %f", position.z);

      // filter point cloud
      float threshold_z = position.z + D_;
      for (size_t j = ymin; j < ymax; ++j) {
        auto p = cloud->points.begin() + xmin + j * cloud->width;
        for (size_t i = 0; i < in_width; ++i) {
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
            position.y = (bb_out_top_y_value + average_y) * 0.5;
            break;
          }
        }
      }
      if (bb_out_top_y < 0 || bb_out_bottom_y == 0) {
        ROS_ERROR("could not detect head y region!");
        continue;
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
        continue; // unexpected
      }

      position.x =  // get median
        average_x_value_in_column.at(average_x_value_in_column.size() >> 1).second;

      msg.position3d = position;
      msg.roll = 0.0;
      msg.pitch = 0.0;
      msg.yaw = -1.57;

      person_publisher_.publish(msg);
      continue;
    }

    // else, frontal face was detected


    if (heads.size() > 1) // has noise, sort by y value
      std::sort(heads.begin(), heads.end(), [&](cv::Rect a, cv::Rect b){return (a.y > b.y);});

    auto subhead = heads.begin();
    cv::Rect head(xmin + subhead->x, ymin + subhead->y, subhead->width, subhead->height);
    // for debug
    cv::rectangle(img, head, cv::Scalar(0));

    auto p =
      cloud->points.begin() + (head.y + (head.height >> 1)) * cloud->width + head.x + (head.width >> 1);
    position.x = p->x;
    position.y = p->y;
    position.z = p->z;
    msg.position3d = position;
    msg.roll = 0.0;
    msg.pitch = 0.0;
    msg.yaw = 0.0;

    person_publisher_.publish(msg);
  }

  // copy img to dbg_img_
  dbg_mutex_.lock();
  cv::resize(img, dbg_img_, cv::Size(img.cols >> 1, img.rows >> 1));
  dbg_mutex_.unlock();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "extract_face_roi_info_from_darknethaar");
  ros::NodeHandle nh("~");

  sensor_id_ = 0;
  nh.getParam("sensor_id", sensor_id_);

  y_up_ = false;
  nh.getParam("y_up", y_up_);

  bool multigpu = false;
  nh.getParam("multiple_gpu", multigpu);
  frame_id_ = "";
  if (!multigpu) // identify sensor w/ frame_id_ instead of topic
    nh.getParam("camera_link", frame_id_);

  std::string filename;
  nh.getParam("haarcascade", filename);
  ROS_INFO("loading %s...", filename.c_str());
  if (!haar_detector_.load(filename)) {
    ROS_ERROR("failed to load %s", filename.c_str());
    std::exit(0);
  }

  depth_threshold_ = 2.0;
  nh.getParam("depth_threshold", depth_threshold_);

  person_publisher_ =
    nh.advertise<roboenvcv::Person>
    ("/roboenvcv/personcoordinate/local", 1);

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

  // below for debug
  dbg_mutex_.lock();
  dbg_img_ = cv::Mat(480, 640, CV_8U).clone();
  dbg_mutex_.unlock();
  while (ros::ok()) {
    dbg_mutex_.lock();
    cv::imshow("debug_window", dbg_img_);
    dbg_mutex_.unlock();
    cv::waitKey(3);
  }
}
