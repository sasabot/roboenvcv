#ifndef _ROBOENVCV_ROBOENVCV_EXTRA_
#define _ROBOENVCV_ROBOENVCV_EXTRA_

#include "roboenvcv/types.h"
#include "linux_kinect/WindowsInterface.hh"

namespace roboenvcv
{
  std::vector<int> FindTargetWithOcr
  (std::vector<std::string> _target_name, std::vector<objectarea> &_scene,
   cv::Mat &_img, windows::interface::WindowsInterfacePtr _windows,
   std::string _debug_folder="");
}

#endif
