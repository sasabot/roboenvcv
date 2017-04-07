#ifndef _ROBOENVCV_TYPES_LIGHT_
#define _ROBOENVCV_TYPES_LIGHT_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace roboenvcv
{
  /// @brief Detection settings.
  struct detectsettings {
    detectsettings() {
      color_thre = 20;
      dist_thre = 1.2;
      rg_min_cluster_size = 100;
      ec_min_cluster_size = 50;
      leap_threshold_in_sc = 0.2;
      second_clustering = true;
      remove_nan_in_sc = true;
    };

    float color_thre;
    float dist_thre;
    float rg_min_cluster_size;
    float ec_min_cluster_size;
    float leap_threshold_in_sc;
    bool second_clustering;
    bool remove_nan_in_sc;
  };

  /// @brief Identified information of object.
  struct objectproperties {
    std::vector<std::string> name;
    /// @brief List of named colors with percentage information.
    std::vector<std::pair<std::string, float> > colors;
    /// @brief Variable to save likeliness score of object.
    float likeliness;
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
