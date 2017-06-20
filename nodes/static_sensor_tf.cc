#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "static_sensor_tf_publisher");

  ros::NodeHandle nh;

  ros::Publisher tf_publisher =
    nh.advertise<geometry_msgs::Pose>("/tf_msg/static_sensor", 10);

  tf::StampedTransform transform;
  transform.setOrigin(tf::Vector3(atof(argv[1]), atof(argv[2]), atof(argv[3])));
  transform.setRotation(tf::Quaternion(atof(argv[4]), atof(argv[5]), atof(argv[6])));

  geometry_msgs::Pose msg;
  msg.position.x = transform.getOrigin().x();
  msg.position.y = transform.getOrigin().y();
  msg.position.z = transform.getOrigin().z();
  // for some reason, setting from rpy results to very strange quaternion
  msg.orientation.x = transform.getRotation().w();
  msg.orientation.y = transform.getRotation().z();
  msg.orientation.z = transform.getRotation().y();
  msg.orientation.w = transform.getRotation().x();

  ros::Rate rate(10.0);
  while (nh.ok()) {
    tf_publisher.publish(msg);
    rate.sleep();
  }

  return 0;
};
