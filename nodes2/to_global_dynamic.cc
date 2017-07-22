#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <roboenvcv/Person.h>
#include <roboenvcv/PersonCoordinate.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// about this code:
// converts head position in camera coordinate to global coordinate
// assumed sensor coordinate: z: depth, y: down

std::vector<geometry_msgs::PoseStamped> v_ps_;

int sensor_id_;
bool y_up_;
ros::Publisher global_head_info_publisher_;
int queue_size_;
double time_thre_;

float min_map_x_;
float min_map_y_;
float max_map_x_;
float max_map_y_;

void calcGlobalHeadInfo(const roboenvcv::Person::ConstPtr& _msg) {
  if (_msg->sensor_id != sensor_id_)
    return;

  // find robot frame w/ closest time frame
  double secs = _msg->header.stamp.toSec();
  int found = -1;
  double time_diff = std::numeric_limits<double>::max();
  for (auto p = v_ps_.begin(); p != v_ps_.end(); ++p) {
    double diff = fabs(secs - p->header.stamp.toSec());
    if (diff < time_diff) {
      time_diff = diff;
      found = static_cast<int>(p - v_ps_.begin());
    }
  }
  std::vector<geometry_msgs::PoseStamped>::iterator ps;
  if (found > 0 && time_diff < time_thre_) {
    ROS_INFO("found %f == %f", v_ps_.at(found).header.stamp.toSec(), secs);
    ps = v_ps_.erase(v_ps_.begin(), v_ps_.begin() + found);
  } else {
    ROS_WARN("robot and person data frame cannot be aligned! %f > %f", time_diff, time_thre_);
    return;
  }

  Eigen::Vector3f pos_global
    (ps->pose.position.x, ps->pose.position.y, ps->pose.position.z);
  Eigen::Quaternionf rot_global
    (ps->pose.orientation.x, ps->pose.orientation.y,
     ps->pose.orientation.z, ps->pose.orientation.w);

  roboenvcv::PersonCoordinate msg;

  Eigen::Vector3f position3d_map =
    rot_global * Eigen::Vector3f(_msg->position3d.x,
                                 _msg->position3d.y,
                                 _msg->position3d.z) + pos_global;

  // don't send if person is out of bounds
  if (position3d_map.x() < min_map_x_ || position3d_map.x() > max_map_x_
      || position3d_map.y() < min_map_y_ || position3d_map.y() > max_map_y_)    
    return;

  Eigen::Matrix3f mat;
  Eigen::Quaternionf q;
  if (y_up_) { // Kinect from Windows, image reversed
    Eigen::AngleAxisf roll_angle(_msg->pitch, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yaw_angle(_msg->roll, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitch_angle(_msg->yaw, Eigen::Vector3f::UnitY());
    q = yaw_angle * pitch_angle * roll_angle;
    mat << 0, -1, 0, 0, 0, 1, -1, 0, 0;
  } else { // xtion, Kinect from Linux, etc.
    Eigen::AngleAxisf roll_angle(-_msg->pitch, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yaw_angle(_msg->roll, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitch_angle(_msg->yaw, Eigen::Vector3f::UnitY());
    q = yaw_angle * pitch_angle * roll_angle;
    mat << 0, 1, 0, 0, 0, 1, 1, 0, 0;
  }
  auto pose3d = q * Eigen::Quaternionf(mat);

  msg.position3d_camera.x = _msg->position3d.x;
  msg.position3d_camera.y = _msg->position3d.y;
  msg.position3d_camera.z = _msg->position3d.z;

  msg.map_to_camera.position.x = pos_global.x();
  msg.map_to_camera.position.y = pos_global.y();
  msg.map_to_camera.position.z = pos_global.z();

  msg.map_to_camera.orientation.x = rot_global.w();
  msg.map_to_camera.orientation.y = rot_global.x();
  msg.map_to_camera.orientation.z = rot_global.y();
  msg.map_to_camera.orientation.w = rot_global.z();

  msg.position3d_map.x = position3d_map.x();
  msg.position3d_map.y = position3d_map.y();
  msg.position3d_map.z = position3d_map.z();

  msg.pose3d_camera.x = pose3d.w();
  msg.pose3d_camera.y = pose3d.x();
  msg.pose3d_camera.z = pose3d.y();
  msg.pose3d_camera.w = pose3d.z();

  global_head_info_publisher_.publish(msg);
}

void SensorPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg) {
  if (v_ps_.size() >= queue_size_)
    v_ps_.erase(v_ps_.begin());
  v_ps_.push_back(*_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "convert_person_info_to_global");
  ros::NodeHandle nh("~");

  sensor_id_ = 0;
  nh.getParam("sensor_id", sensor_id_);

  y_up_ = false;
  nh.getParam("y_up", y_up_);

  queue_size_ = 10;
  nh.getParam("queue_size", queue_size_);

  time_thre_ = 0.1;
  nh.getParam("timestamp", time_thre_);

  min_map_x_ = -20.0;
  nh.getParam("min_map_x", min_map_x_);

  min_map_y_ = -20.0;
  nh.getParam("min_map_y", min_map_y_);

  max_map_x_ = 20.0;
  nh.getParam("max_map_x", max_map_x_);

  max_map_y_ = 20.0;
  nh.getParam("max_map_y", max_map_y_);

  global_head_info_publisher_ =
    nh.advertise<roboenvcv::PersonCoordinate>
    ("/roboenvcv/personcoordinate/global/withoutid", 1);

  ros::Subscriber camera_head_info_subscriber =
    nh.subscribe("/roboenvcv/personcoordinate/local", 100, calcGlobalHeadInfo);
  ros::Subscriber sensor_pose_subscriber =
    nh.subscribe("/tf_msg/sensor", 100, SensorPoseCallback);

  ros::Rate r(30);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}

