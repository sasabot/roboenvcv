#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensordirection_filter");
  ros::NodeHandle nh("~");

  std::string ns("kinect");
  nh.getParam("ns", ns);

  geometry_msgs::PoseStamped msg;
  nh.getParam("x", msg.pose.position.x);
  nh.getParam("y", msg.pose.position.y);
  msg.pose.position.z = 0.0;

  ros::Publisher pub =
    nh.advertise<geometry_msgs::PoseStamped>
    ("/" + ns + "/global/sensordirection/inputaxis", 10);

  ros::Rate r(10);
  while (ros::ok()) {
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
    r.sleep();
  }
}
