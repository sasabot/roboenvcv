#include <ros/ros.h>
#include <roboenvcv/Person.h>
#include <roboenvcv/PersonCoordinate.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// about this code:
// converts head position in camera coordinate to global coordinate
// assumed sensor coordinate: z: depth, y: down

Eigen::Vector3f pos_global_;
Eigen::Quaternionf rot_global_;

int sensor_id_;
bool y_up_;
ros::Publisher global_head_info_publisher_;

void calcGlobalHeadInfo(const roboenvcv::Person::ConstPtr& _msg) {
  if (_msg->sensor_id != sensor_id_)
    return;

  roboenvcv::PersonCoordinate msg;

  auto position3d_map =
    rot_global_ * Eigen::Vector3f(_msg->position3d.x,
                                  _msg->position3d.y,
                                  _msg->position3d.z) + pos_global_;

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

  msg.map_to_camera.position.x = pos_global_.x();
  msg.map_to_camera.position.y = pos_global_.y();
  msg.map_to_camera.position.z = pos_global_.z();

  msg.map_to_camera.orientation.x = rot_global_.w();
  msg.map_to_camera.orientation.y = rot_global_.x();
  msg.map_to_camera.orientation.z = rot_global_.y();
  msg.map_to_camera.orientation.w = rot_global_.z();

  msg.position3d_map.x = position3d_map.x();
  msg.position3d_map.y = position3d_map.y();
  msg.position3d_map.z = position3d_map.z();

  msg.pose3d_camera.x = pose3d.w();
  msg.pose3d_camera.y = pose3d.x();
  msg.pose3d_camera.z = pose3d.y();
  msg.pose3d_camera.w = pose3d.z();

  global_head_info_publisher_.publish(msg);
}

void SensorPoseCallback(const geometry_msgs::Pose::ConstPtr &_msg) {
  pos_global_ =
    Eigen::Vector3f(_msg->position.x, _msg->position.y, _msg->position.z);
  rot_global_ =
    Eigen::Quaternionf(_msg->orientation.x, _msg->orientation.y,
                       _msg->orientation.z, _msg->orientation.w);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "convert_person_info_to_global");
  ros::NodeHandle nh("~");

  sensor_id_ = 0;
  nh.getParam("sensor_id", sensor_id_);

  y_up_ = false;
  nh.getParam("y_up", y_up_);

  global_head_info_publisher_ =
    nh.advertise<roboenvcv::PersonCoordinate>
    ("/roboenvcv/personcoordinate/global/withoutid", 1);

  ros::Subscriber camera_head_info_subscriber =
    nh.subscribe("/roboenvcv/personcoordinate/local", 100, calcGlobalHeadInfo);
  ros::Subscriber sensor_pose_subscriber =
    nh.subscribe("/tf_msg/sensor", 100, SensorPoseCallback);

  pos_global_ = Eigen::Vector3f(0, 0, 0);
  rot_global_ = Eigen::Quaternionf(1, 0, 0, 0);

  ros::Rate r(30);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}

