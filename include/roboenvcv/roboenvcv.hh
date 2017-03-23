#ifndef _ROBOENVCV_ROBOENVCV_
#define _ROBOENVCV_ROBOENVCV_

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>

#include <chrono>

#include "roboenvcv/colors.h"
#include "roboenvcv/proc2d.h"

#include "roboenvcv/types.h"
#include "roboenvcv/utils.h"

namespace roboenvcv
{
  /// @brief Detect area where objects likely exist with 3d region growing.
  ///   First, region growing segmentation is conducted.
  ///   Second, 2d clustering for non-detected area is conducted.
  ///   Third, 2d compression segmentation is conducted for large non-detected area.
  /// @param[in] _cloud Point cloud from sensor (low resolution for performance).
  /// @param[in] _img Image from sensor (for resolution ratio and drawing).
  /// @param[in] _env_color Color of environment to remove from object list.
  /// @param[in] _debug_folder Folder to save process images. No save if empty.
  /// @param[in] _settings Optional settings for detection.
  /// @return {List of detected objects, List of detected environment}
  std::pair<std::vector<objectarea>, std::vector<objectarea> > DetectObjectnessArea
  (pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud,
   cv::Mat &_img, cv::Vec3b _env_color, std::string _debug_folder="",
   detectsettings _settings=detectsettings());

  /// @brief DetectObjectnessArea without background color subtraction.
  /// @param[in] _cloud Point cloud from sensor (low resolution for performance).
  /// @param[in] _img Image from sensor (for resolution ratio and drawing).
  /// @param[in] _debug_folder Folder to save process images. No save if empty.
  /// @param[in] _settings Optional settings for detection.
  /// @return {List of detected objects, }
  std::pair<std::vector<objectarea>, std::vector<objectarea> > DetectObjectnessArea
  (pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud,
   cv::Mat &_img, std::string _debug_folder="", detectsettings _settings=detectsettings());

  /// @brief DetectObjectnessArea with threshold arguments. (beta-type function)
  /// @param[in] _cloud Point cloud from sensor (low resolution for performance).
  /// @param[in] _img Image from sensor (for resolution ratio and drawing).
  /// @param[in] _env_color Color of environment to remove from object list.
  /// @param[in] _color_thre Color distance threshold to label as background color.
  /// @param[in] _dist_thre Distance threshold to eliminate from ROI.
  /// @param[in] _debug_folder Folder to save process images. No save if empty.
  /// @return {List of detected objects, List of detected environment}
  std::pair<std::vector<objectarea>, std::vector<objectarea> > DetectObjectnessArea
  (pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud,
   cv::Mat &_img, cv::Vec3b _env_color, float _color_thre=20, float _dist_thre=1.2,
   std::string _debug_folder="");

  /// @brief Sorts by color percentage and distance (best match comes first).
  ///   e.g. 100% blue is better match than 50% blue
  ///   e.g. 100% blue nearby is better match than 100% blue far away
  ///   e.g. 100% blue far away is better match than 50% blue nearby
  /// @param[in] _target_color Named color to find. cf. roboenvcv::colorMap9
  /// @param[in] _scene Lists of objects to find from.
  /// @return Indices (in vector _scene) of matched objects sorted by likeliness.
  std::vector<int> FindTarget
  (std::string _target_color, std::vector<objectarea> &_scene);

  /// @brief Patch object region with surrounding depth points.
  /// @param[in] _target Target object to patch.
  /// @param[in] _search_rows Number of surrounding rows to search.
  ///    Searches above for positive rows and searches below for negative.
  /// @param[in] _include_depths Depth threshold to include in patched region.
  ///    Adds points deeper than target if positive, and points shallow if negative.
  /// @param[in] _cloud Point cloud from sensor (should be same as target).
  /// @param[in] _img Image from sensor (for resolution ratio).
  /// @return Patched object region.
  objectarea patchGrowth
  (objectarea _target, int _search_rows, float _include_depths,
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat _img);

  /// @brief Draw box around best match using sorted indices.
  /// @param[in] _objects List of detected objects.
  /// @param[in] _img Image to draw result on (same as image used for detecting).
  /// @param[in] _sorted_idx Sorted index in best match order.
  /// @param[in] _color Color of box around best match.
  void drawBestMatch
  (std::vector<objectarea> &_objects, cv::Mat &_img, std::vector<int> &_sorted_idx,
   cv::Scalar _color=cv::Scalar(0, 0, 255));

  /// @brief Detect object information in relation to environment for grasping.
  /// @param[in] _target Id of target in vector _scene.
  /// @param[in] _scene List of objects to consider when analyzing.
  /// @param[in] _cloud Point cloud from sensor (used in DetectObjectnessArea).
  /// @param[in] _img Image from sensor (used in DetecObjectnessArea)
  /// @param[in] _debug_folder Folder to save process images. No save if empty.
  /// @return Likely required grasp strategy and detected object relations.
  graspconfig ConfigurationFromLocal1DState
  (int _target, std::vector<objectarea> &_scene,
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat &_img,
   std::string _debug_folder="");
}

#endif
