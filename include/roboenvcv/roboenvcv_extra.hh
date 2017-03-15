#ifndef _ROBOENVCV_ROBOENVCV_EXTRA_
#define _ROBOENVCV_ROBOENVCV_EXTRA_

#include "roboenvcv/types.h"
#include "linux_kinect/WindowsInterface.hh"

namespace roboenvcv
{
  void PatchBoundsForOcr
  (roboenvcv::objectarea &_it, cv::Mat &_img, float _margin_top,
   float _margin_right, float _margin_bottom, float _margin_left);

  std::vector<int> FindTargetWithOcr
  (std::vector<std::string> _target_name, std::vector<objectarea> &_scene,
   cv::Mat &_img, windows::interface::WindowsInterfacePtr _windows,
   std::string _debug_folder="");
}

#endif
