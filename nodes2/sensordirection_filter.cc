#include <ros/ros.h>
#include <roboenvcv/BoolStamped.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub_;
int queue_size_;
double time_thre_;
float ref_x_;
float ref_y_;
float theta_;

std::vector<geometry_msgs::PoseStamped> v_base_position_;

void BasePositionCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg) {
  if (v_base_position_.size() >= queue_size_)
    v_base_position_.erase(v_base_position_.begin());
  v_base_position_.push_back(*_msg);
}

void SensorPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg) {
  // find most near matched time
  double secs = _msg->header.stamp.toSec();
  int found = -1;
  double time_diff = std::numeric_limits<double>::max();
  for (auto d = v_base_position_.begin(); d != v_base_position_.end(); ++d) {
    double diff = fabs(secs - d->header.stamp.toSec());
    if (diff < time_diff) {
      time_diff = diff;
      found = static_cast<int>(d - v_base_position_.begin());
    }
  }
  if (found >= 0 && time_diff < time_thre_) {
    ROS_INFO("found %f == %f", v_base_position_.at(found).header.stamp.toSec(), secs);
    v_base_position_.erase(v_base_position_.begin(), v_base_position_.begin() + found);
  } else {
    ROS_WARN("base position w/ close time frame not found! %f ~ %f, looking for %f",
             v_base_position_.front().header.stamp.toSec(),
             v_base_position_.back().header.stamp.toSec(), secs);
    return;
  }
  float x = _msg->pose.position.x - v_base_position_.begin()->pose.position.x;
  float y = _msg->pose.position.y - v_base_position_.begin()->pose.position.y;
  float theta = acos(ref_x_ * x + ref_y_ * y) * 2;
  roboenvcv::BoolStamped msg;
  msg.header = _msg->header;
  if (theta > theta_ || theta < -theta_)
    msg.data = false;
  else
    msg.data = true;
  pub_.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensordirection_filter");
  ros::NodeHandle nh("~");

  std::string ns("kinect");
  nh.getParam("ns", ns);

  queue_size_ = 10;
  nh.getParam("queue_size", queue_size_);

  time_thre_ = 0.1;
  nh.getParam("timestamp", time_thre_);

  ref_x_ = 0.0;
  nh.getParam("x", ref_x_);

  ref_y_ = 1.0;
  nh.getParam("y", ref_y_);

  theta_ = 1.57;
  nh.getParam("theta", theta_);

  pub_ = nh.advertise<roboenvcv::BoolStamped>("/" + ns + "/global/sensordirection/filter", 1);

  ros::Subscriber base_subscriber =
    nh.subscribe("/tf_msg/base", 10, BasePositionCallback);

  ros::Subscriber sensor_subscriber =
    nh.subscribe("/tf_msg/sensor", 10, SensorPositionCallback);

  ros::Rate r(30);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}
