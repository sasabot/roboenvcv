#include <ros/ros.h>
#include <roboenvcv/Int32Stamped.h>
#include <roboenvcv/PersonCoordinate.h>
#include <vector>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

// about this code:
// tracks id of found face with 3d position likeliness

std::vector<roboenvcv::Int32Stamped> v_nump_;
ros::Publisher global_head_id_publisher_;
int max_faces_;
int next_reserved_face_id_;
int previous_count_;

int queue_size_;
double time_thre_;

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

struct RemoveTimer {
  bool active;
  int remove_count;
  std::chrono::high_resolution_clock::time_point refer_time;
};

std::vector<FaceLog> face_log_;
RemoveTimer remove_timer_;

void PersonCoordinateCallback
(const roboenvcv::PersonCoordinate::ConstPtr& _msg) {
  // find person count w/ closest time frame
  double secs = _msg->header.stamp.toSec();
  int found = -1;
  double time_diff = std::numeric_limits<double>::max();
  for (auto p = v_nump_.begin(); p != v_nump_.end(); ++p) {
    double diff = fabs(secs - p->header.stamp.toSec());
    if (diff < time_diff) {
      time_diff = diff;
      found = static_cast<int>(p - v_nump_.begin());
    }
  }
  int count;
  if (found >= 0 && time_diff < time_thre_) {
    ROS_INFO("found %f == %f, %d persons", v_nump_.at(found).header.stamp.toSec(), secs, count);
    count = v_nump_.erase(v_nump_.begin(), v_nump_.begin() + found)->data;
  } else {
    ROS_WARN("person count and data frame cannot be aligned! %f > %f", time_diff, time_thre_);
    return;
  }

  // person all removed signal from upstream
  if (count == 0) {
    previous_count_ = 0;
    remove_timer_.active = false;
    remove_timer_.remove_count = 0;
    for (auto log = face_log_.begin(); log != face_log_.end(); ++log)
      log->Free(next_reserved_face_id_++);
    roboenvcv::PersonCoordinate msg;
    msg.id = "---remove-all";
    return;
  }

  // if count is smaller than previous count, setup a remove timer
  if (count < previous_count_) {
    remove_timer_.active = true;
    ++remove_timer_.remove_count;
    if (!remove_timer_.active)
      remove_timer_.refer_time = std::chrono::high_resolution_clock::now();
  }

  previous_count_ = count; // update count, not used from here

  // when a remove timer is activated, remove oldest log
  if (remove_timer_.active &&
      std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::high_resolution_clock::now() - remove_timer_.refer_time)
      .count() > 2000) { // wait two seconds before start removing
    int tracked_count = 0;
    std::vector<FaceLog>::iterator to_remove;
    double timepassed = std::numeric_limits<double>::max();
    for (auto log = face_log_.begin(); log != face_log_.end(); ++log) {
      if (log->is_tracked)
        ++tracked_count;
      auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::high_resolution_clock::now() - log->last_track_point).count();
      if (duration < timepassed) {
        to_remove = log;
        timepassed = duration;
      }
    }
    if (tracked_count > count) // make sure we're not removing too much
      to_remove->Free(next_reserved_face_id_++); // remove one log per callback
    else
      remove_timer_.remove_count == 0; // don't remove and stop removing

    if  (remove_timer_.remove_count == 0)
      remove_timer_.active = false;
    else
      --remove_timer_.remove_count;
  }

  auto position =
    Eigen::Vector3d(_msg->position3d_map.x,
                    _msg->position3d_map.y,
                    _msg->position3d_map.z);

  // find matching log
  int tracked_count = 0;
  int likely_enum = -1;
  int uncertain_likely_enum = -1;
  float likeliness = std::numeric_limits<float>::max();
  for (size_t i = 0; i < face_log_.size(); ++i) {
    if (!face_log_.at(i).is_tracked)
      continue;

    ++tracked_count;
    ROS_INFO("id %d is being tracked", static_cast<int>(i));

    Eigen::Vector3d diff = face_log_.at(i).position - position;
    float z_dist = fabs(diff.z());
    diff[2] = 0.0; // exclude height: people sometimes bend toward
    float dist = diff.norm();
    if (dist < likeliness) {
      // face within 50cm of previous frame
      if (dist < 0.5) {
        likely_enum = i;
      }
      uncertain_likely_enum = i;
      likeliness = dist;
    }
  }

  // no likely log, check count number
  if (likely_enum < 0 && tracked_count < count) {
    // register new    
    for (size_t i = 0; i < face_log_.size(); ++i)
      if (!face_log_.at(i).is_tracked) {
        likely_enum = i;
        break;
      }
  } else {
    likely_enum = uncertain_likely_enum; // don't register new
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

void PersonCountCallback(const roboenvcv::Int32Stamped::ConstPtr &_msg) {
  if (v_nump_.size() >= queue_size_)
    v_nump_.erase(v_nump_.begin());
  v_nump_.push_back(*_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "id_mapper");
  ros::NodeHandle nh;

  max_faces_ = 3;
  nh.getParam("/tracknfaces", max_faces_);

  queue_size_ = 10;
  nh.getParam("queue_size", queue_size_);

  time_thre_ = 0.1;
  nh.getParam("timestamp", time_thre_);

  face_log_.reserve(max_faces_);
  for (size_t i = 0; i < max_faces_; ++i)
    face_log_.push_back(FaceLog(i));
  next_reserved_face_id_ = max_faces_;

  remove_timer_ = {false, 0, std::chrono::high_resolution_clock::now()};
  previous_count_ = 0;

  global_head_id_publisher_ =
    nh.advertise<roboenvcv::PersonCoordinate>
    ("/roboenvcv/personcoordinate/global/withid", 1);

  ros::Subscriber global_head_info_subscriber =
    nh.subscribe("/roboenvcv/personcoordinate/global/withoutid", 100,
                 PersonCoordinateCallback);
  ros::Subscriber personcount_callback =
    nh.subscribe("/roboenvcv/personcount", 100, PersonCountCallback);

  ros::Rate r(30);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}
