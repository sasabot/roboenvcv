#ifndef _ROBOENVCV_UTILS_
#define _ROBOENVCV_UTILS_

#include <cmath>
#include "roboenvcv/types.h"

namespace roboenvcv
{
  /// @brief Filters list of objects with given indices.
  /// @param[in] _objects List of objects to filter.
  /// @param[in] _indices The indices of objects to retrieve.
  /// @return List of objects from applied indices.
  std::vector<objectarea> filter
    (std::vector<objectarea> _objects, std::vector<int> _indices)
  {
    std::vector<objectarea> result;
    for (auto it = _indices.begin(); it != _indices.end(); ++it)
      if (*it >= 0 && *it < _objects.size())
        result.push_back(_objects.at(*it));
    return result;
  }

  /// @brief Calculates height of object from environment.
  /// @param[in] _obj Target object.
  /// @param[in] _env Target environment (should be plane).
  ///    Make sure _env and _obj has values in the same coordinate.
  /// @return Height of object from environment..
  float getHeight
    (objectarea _obj, objectarea _env)
  {
    Eigen::Vector3f w = _obj.center3d - _env.center3d;
    return fabs(_env.normal3d.dot(w)) * 2;
  }
}

#endif
