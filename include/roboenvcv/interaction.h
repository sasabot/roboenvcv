#ifndef _ROBOENVCV_INTERACTION_
#define _ROBOENVCV_INTERACTION_

#include "roboenvcv/types_light.h"

namespace roboenvcv
{
  /// @brief From human head posture, check if person is looking at point.
  /// @param _person Info of human head position/posture in camera coordinate.
  /// @param _point_base Target point location in base coordinate.
  /// @param _dist_thre Angle between sight ray and object in radian to define as looking.
  /// @param _project_points Evaluate on xy plane (no z) if true.
  /// @return Looking or not looking.
  bool SharedAttention
    (PersonCameraCoords _person, Eigen::Vector3f _point_base, float _dist_thre,
     bool _project_points=false)
  {
    Eigen::Vector3f
      sight_direction_camera = _person.pose3d * Eigen::Vector3f(1.0, 0, 0);
    Eigen::Vector3f
      sight_direction_base = _person.mat_base_to_camera * sight_direction_camera;
    Eigen::Vector3f
      point_to_person_base = _point_base -
      (_person.mat_base_to_camera * _person.position3d + _person.p_base_to_camera);

    if (_project_points) {
      sight_direction_base.z() = 0.0;
      point_to_person_base.z() = 0.0;
    }

    float distance =
      acos(sight_direction_base.dot(point_to_person_base)
           / (sight_direction_camera.norm() * point_to_person_base.norm()));

    if (distance < _dist_thre) return true;
    else return false;
  };

  /// @brief From human head posture, check whether person is looking at plane.
  /// @param _person Info of human head position/posture in camera coordinate.
  /// @param _plane_normal Normal of plane in base coordinate or global coordinate.
  /// @param _plane_center Position of plane in base coordinate or global coordinate
  /// @param _inner_threshold Distance range in meters to define as absolutely looking at plane.
  /// @param _outer_threshold Distance range in meters to define as likely looking at plane.
  /// @param _mat_global_to_base When applied, _plane_normal will be parsed as global coordinate.
  /// @param _p_global_to_base When applied, _plane_normal will be parsed as global coordinate.
  /// @return Score of looking or not.
  float SharedAttention
    (PersonCameraCoords _person,
     Eigen::Vector3f _plane_normal, Eigen::Vector3f _plane_center,
     float _inner_threshold, float _outer_threshold,
     Eigen::Quaternionf _mat_global_to_base=Eigen::Quaternionf(0, 0, 0, 0),
     Eigen::Vector3f _p_global_to_base=Eigen::Vector3f(0, 0, 0))
  {
    Eigen::Vector3f plane_normal;
    Eigen::Vector3f plane_center;

    // check global or base coordinate
    if (_mat_global_to_base.norm() < 0.00001
        && _p_global_to_base.norm() < 0.00001) {
      plane_normal = _plane_normal;
      plane_center = _plane_center;
    } else {
      plane_normal = _mat_global_to_base.inverse() * _plane_normal;
      plane_center =
        _mat_global_to_base.inverse() * (_plane_center - _p_global_to_base);
    }

    Eigen::Vector3f
      sight_direction_camera = _person.pose3d * Eigen::Vector3f(1.0, 0, 0);
    Eigen::Vector3f
      sight_direction_base = _person.mat_base_to_camera * sight_direction_camera;
    Eigen::Vector3f
      person_position3d_base =
      _person.mat_base_to_camera * _person.position3d + _person.p_base_to_camera;

    float t = plane_normal.dot(person_position3d_base - plane_center)
      / plane_normal.dot(sight_direction_base);

    // t cannot be negative as sight must be looking 'forward'
    Eigen::Vector3f
      intersection = person_position3d_base + fabs(t) * sight_direction_base;
    float distance = (intersection - plane_center).norm();

    if (distance < _inner_threshold)
      return 1.0;
    else if (distance < _outer_threshold)
      return 1.0 - (distance - _inner_threshold)
        / (_outer_threshold - _inner_threshold);

    return 0.0;
  };
}

#endif
