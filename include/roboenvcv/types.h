#ifndef _ROBOENVCV_TYPES_
#define _ROBOENVCV_TYPES_

#include "roboenvcv/types_light.h"
#include <opencv2/core/types.hpp>

namespace roboenvcv
{
  /// @brief Information of detected object.
  struct objectarea {
    std::vector<int> indices3d; // compressed indices
    cv::Rect bounds2d;
    bool visible3d; // center3d, normal3d is null for such objects
    Eigen::Vector3f center3d; // used in post-process
    Eigen::Vector3f normal3d; // pre-calculated for post-process
    objectproperties properties; // only colors property is filled w/ detection
  };
}

#endif
