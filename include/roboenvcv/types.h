#ifndef _ROBOENVCV_TYPES_
#define _ROBOENVCV_TYPES_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/types.hpp>

namespace roboenvcv
{
  /// @brief Detection settings.
  struct detectsettings {
    detectsettings() {
      color_thre = 20;
      dist_thre = 1.2;
      rg_min_cluster_size = 100;
      ec_min_cluster_size = 50;
      second_clustering = true;
    };

    float color_thre;
    float dist_thre;
    float rg_min_cluster_size;
    float ec_min_cluster_size;
    bool second_clustering;
  };

  /// @brief Identified information of object.
  struct objectproperties {
    std::vector<std::string> name;
    /// @brief List of named colors with percentage information.
    std::vector<std::pair<std::string, float> > colors;
  };

  /// @brief Information of detected object.
  struct objectarea {
    std::vector<int> indices3d; // compressed indices
    cv::Rect bounds2d;
    bool visible3d; // center3d, normal3d is null for such objects
    Eigen::Vector3f center3d; // used in post-process
    Eigen::Vector3f normal3d; // pre-calculated for post-process
    objectproperties properties; // only colors property is filled w/ detection
  };

  /// @brief Information of object in relation to environment.
  struct graspconfig {
    int sparse; // -1: detected facet, 0: detected nearby obstacle, 1: clear
    int contact; // -1: cannot grasp, 0: top grasp-able, 1: contact grasp
    std::vector<int> facets; // id of detected facets (for futher analysis)
  };

  /// @brief Head pose information in detected camera coordinate.
  struct PersonCameraCoords {
    Eigen::Vector3f position3d;
    // head coordinate is x axis forward, z axis up
    // pose3d converts camera coords to head coords 
    Eigen::Quaternionf pose3d;
    // information of camera pose when face was detected
    Eigen::Quaternionf mat_base_to_camera;
    Eigen::Vector3f p_base_to_camera;
  };
}

#endif
