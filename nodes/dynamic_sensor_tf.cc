#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_sensor_tf_publisher");

  ros::NodeHandle nh("~");

  std::string frame_name = "/dynamic_frame";
  nh.getParam("dynamic_frame", frame_name);

  ros::Publisher tf_publisher =
    nh.advertise<geometry_msgs::Pose>("/tf_msg/dynamic_sensor", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (nh.ok()) {
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("/map", frame_name,
                               ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    geometry_msgs::Pose msg;
    msg.position.x = transform.getOrigin().x();
    msg.position.y = transform.getOrigin().y();
    msg.position.z = transform.getOrigin().z();
    msg.orientation.x = transform.getRotation().w();
    msg.orientation.y = transform.getRotation().x();
    msg.orientation.z = transform.getRotation().y();
    msg.orientation.w = transform.getRotation().z();
    tf_publisher.publish(msg);

    rate.sleep();
  }

  return 0;
};
