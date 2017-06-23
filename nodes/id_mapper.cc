#include <ros/ros.h>
#include <roboenvcv/PersonCoordinate.h>
#include <vector>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

// about this code:
// tracks id of found face with 3d position likeliness

ros::Publisher global_head_id_publisher_;
int max_faces_;
int next_reserved_face_id_;

struct FaceLog {
  int id;
  Eigen::Vector3d position;
  bool is_tracked;
  std::chrono::high_resolution_clock::time_point last_track_point;

  FaceLog(int _id) {
    id = _id;
    position = Eigen::Vector3d(0, 0, 0);
    is_tracked = false;
    last_track_point = std::chrono::high_resolution_clock::now();
  }

  void Update(Eigen::Vector3d _position) {
    position = _position;
    is_tracked = true;
    last_track_point = std::chrono::high_resolution_clock::now();
  }

  void Free(int _renew_id) {
    id = _renew_id;
    position = Eigen::Vector3d(0, 0, 0);
    is_tracked = false;
    last_track_point = std::chrono::high_resolution_clock::now();
  }
};

std::vector<FaceLog> face_log_;

void PersonCoordinateCallback
(const roboenvcv::PersonCoordinate::ConstPtr& _msg) {
  // check if any old log exists
  for (auto log = face_log_.begin(); log != face_log_.end(); ++log)
    if (std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::high_resolution_clock::now() - log->last_track_point)
        .count() > 10000) { // person not found for 10 seconds
      log->Free(next_reserved_face_id_++);
    }

  auto position =
    Eigen::Vector3d(_msg->position3d_map.x,
                    _msg->position3d_map.y,
                    _msg->position3d_map.z);

  // find matching log
  int likely_enum = -1;
  float likeliness = std::numeric_limits<float>::max();
  for (size_t i = 0; i < face_log_.size(); ++i) {
    if (!face_log_.at(i).is_tracked)
      continue;

    ROS_INFO("id %d is being tracked", static_cast<int>(i));

    Eigen::Vector3d diff = face_log_.at(i).position - position;
    float dist = diff.norm();
    float z_dist = fabs(diff.z());
    // face within 50cm of previous frame
    if (dist < 0.5 && dist < likeliness) {
      likely_enum = i;
      likeliness = dist;
    }
  }

  // no likely log, register new
  if (likely_enum < 0)
    for (size_t i = 0; i < face_log_.size(); ++i)
      if (!face_log_.at(i).is_tracked) {
        likely_enum = i;
        break;
      }

  if (likely_enum < 0) {
    ROS_WARN("exceeded trackable number in id_mapper!!!!!!");
    return; // trackable number of faces occupied
  }

  face_log_.at(likely_enum).Update(position);

  roboenvcv::PersonCoordinate msg;
  msg = *_msg;
  msg.id = std::to_string(face_log_.at(likely_enum).id);
  global_head_id_publisher_.publish(msg);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "id_mapper");
  ros::NodeHandle nh;

  max_faces_ = 3;
  nh.getParam("/tracknfaces", max_faces_);

  face_log_.reserve(max_faces_);
  for (size_t i = 0; i < max_faces_; ++i)
    face_log_.push_back(FaceLog(i));
  next_reserved_face_id_ = max_faces_;

  global_head_id_publisher_ =
    nh.advertise<roboenvcv::PersonCoordinate>
    ("/roboenvcv/personcoordinate/global/withid", 1);

  ros::Subscriber global_head_info_subscriber =
    nh.subscribe("/roboenvcv/personcoordinate/global/withoutid", 100,
                 PersonCoordinateCallback);

  ros::Rate r(30);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}
