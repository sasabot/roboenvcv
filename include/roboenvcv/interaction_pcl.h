#ifndef _ROBOENVCV_INTERACTION_PCL_
#define _ROBOENVCV_INTERACTION_PCL_

#include "roboenvcv/types_light.h"
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>

namespace roboenvcv
{
  /// @brief From human head posture, find cloud region the human is looking at.
  /// @param _person Info of human head position/posture in camera coordinate.
  /// @param _points_camera Point cloud to be edited. Points not in shared attention will be removed.
  /// @param _mat_base_to_camera Camera orientation where point cloud was captured.
  /// @param _p_base_to_camera Camera position where point cloud was captured.
  /// @param _threshold Point to sight ray distance in meters to include as looking.
  /// @param[in] _transform_points If true, returns points in base coordinate. Else, camera coordinate. 
  inline void SharedAttention
    (PersonCameraCoords _person,
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr _points_camera,
     Eigen::Quaternionf _mat_base_to_camera, Eigen::Vector3f _p_base_to_camera,
     float _threshold, bool _transform_points=false)
  {
    Eigen::Vector3f
      sight_direction_camera = _person.pose3d * Eigen::Vector3f(1.0, 0, 0);
    Eigen::Vector3f
      sight_direction_base = _person.mat_base_to_camera * sight_direction_camera;

    for (auto it = _points_camera->points.begin();
         it != _points_camera->points.end(); ) {

      if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z)) {
        it = _points_camera->points.erase(it);
        continue;
      }

      Eigen::Vector3f point_base =
        _mat_base_to_camera * (Eigen::Vector3f(it->x, it->y, it->z))
        + _p_base_to_camera;

      float distance =
        (sight_direction_base.cross(_person.mat_base_to_camera * _person.position3d
                                    + _person.p_base_to_camera - point_base)).norm()
        / sight_direction_camera.norm();

      if (distance > _threshold) {
        it = _points_camera->points.erase(it);
        continue;
      }

      if (_transform_points) {
        it->x = point_base.x();
        it->y = point_base.y();
        it->z = point_base.z();
      }

      ++it;
    }

    _points_camera->width = _points_camera->points.size();
    _points_camera->height = 1;
  };
}

#endif
