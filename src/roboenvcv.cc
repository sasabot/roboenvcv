#include "roboenvcv/roboenvcv.hh"

//////////////////////////////////////////////////
void Get3DdataFrom2DBounds(roboenvcv::objectarea &obj,
                           int x, int y, int width, int height, int k,
                           cv::Mat &labeled_image,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud,
                           pcl::PointCloud<pcl::Normal>::Ptr normals,
                           float leap_threshold) {
  obj.indices3d.reserve(width * height);
  obj.visible3d = false;

  std::vector<std::pair<int, float> > centers;
  for (unsigned int i = 0; i < height; ++i)
    for (unsigned int j = 0; j < width; ++j)
      if (labeled_image.at<int>(y + i, x + j) == k) {
        int index = static_cast<int>((y + i) * labeled_image.cols + x + j);
        // add depth point candidate if valid
        if (!std::isnan(_cloud->points[index].x) && !std::isnan(_cloud->points[index].y)
            && !std::isnan(_cloud->points[index].z))
          centers.push_back({index, _cloud->points[index].z});
      }

  if (centers.size() > 0) { // we found some points with depth
    // sort centers in distance order
    std::sort(centers.begin(), centers.end(),
              [&](std::pair<int, float> a, std::pair<int, float> b){
                return a.second < b.second;
              });

    // make sure we are getting depth of the same object
    Eigen::Vector3f center = {0.0, 0.0, 0.0};
    Eigen::Vector3f normal = {0.0, 0.0, 0.0};
    int center_count = 0;
    int normal_count = 0;
    for (auto it = centers.begin() + 1; it != centers.end(); ++it) {
      if ((it->second - (it-1)->second) > leap_threshold)
        break; // there was a jump in depth, likely not the same object
      int index = (it-1)->first;
      obj.indices3d.push_back(index);
      center +=
        Eigen::Vector3f(_cloud->points[index].x, _cloud->points[index].y,
                        _cloud->points[index].z);
      ++center_count;
      if (!std::isnan(normals->points[index].normal_x) &&
          !std::isnan(normals->points[index].normal_y) &&
          !std::isnan(normals->points[index].normal_z)) {
        normal +=
          Eigen::Vector3f(normals->points[index].normal_x,
                          normals->points[index].normal_y,
                          normals->points[index].normal_z);
        ++normal_count;
        obj.visible3d = true;
      }
    }
    if (center_count > 0) center /= center_count;
    if (normal_count > 0) normal.normalize();
    obj.center3d = center;
    obj.normal3d = normal;

    if (obj.indices3d.size() < 10) { // likely noise + cannot get color
      obj.visible3d = false;
      return;
    }

    // get color data
    std::vector<cv::Vec3b> colors1d(obj.indices3d.size());
    int color1d_idx = 0;
    for (auto pit = obj.indices3d.begin(); pit != obj.indices3d.end(); ++pit) {
      uint32_t rgb = *reinterpret_cast<int*>(&_cloud->points[*pit].rgb);
      colors1d[color1d_idx++] = cv::Vec3b
        ((rgb) & 0x0000ff, (rgb >> 8) & 0x0000ff, (rgb >> 16) & 0x0000ff);
    }
    std::vector<cv::Vec3b> dominant_colors(4); // get four colors
    roboenvcv::medianCut(colors1d, 0, 0, dominant_colors);
    roboenvcv::getColorNames
      (dominant_colors, obj.properties.colors, roboenvcv::colorMap9);
  }
};

//////////////////////////////////////////////////
std::pair<std::vector<roboenvcv::objectarea>,
          std::vector<roboenvcv::objectarea> > roboenvcv::DetectObjectnessArea
(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat &_img,
 std::string _debug_folder, roboenvcv::detectsettings _settings)
{
  auto res = roboenvcv::DetectObjectnessArea
    (_cloud, _img, cv::Vec3b(0, 0, 0), _debug_folder, _settings);
  res.first.insert(res.first.end(), res.second.begin(), res.second.end());
  res.second.clear();
  return res;
}

//////////////////////////////////////////////////
std::pair<std::vector<roboenvcv::objectarea>,
          std::vector<roboenvcv::objectarea> > roboenvcv::DetectObjectnessArea
(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat &_img,
 cv::Vec3b _env_color, float _color_thre, float _dist_thre, std::string _debug_folder)
{
  roboenvcv::detectsettings settings;
  settings.color_thre = _color_thre;
  settings.dist_thre = _dist_thre;
  return roboenvcv::DetectObjectnessArea
    (_cloud, _img, _env_color, _debug_folder, settings);
}

//////////////////////////////////////////////////
std::pair<std::vector<roboenvcv::objectarea>,
          std::vector<roboenvcv::objectarea> > roboenvcv::DetectObjectnessArea
(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat &_img,
 cv::Vec3b _env_color, std::string _debug_folder, roboenvcv::detectsettings _settings)
{
  auto begin = std::chrono::high_resolution_clock::now();

  std::vector<roboenvcv::objectarea> scene;
  std::vector<roboenvcv::objectarea> env;
  int w_scale = _img.cols / _cloud->width;
  int h_scale = _img.rows / _cloud->height;

  // get normal
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(_cloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree
    (new pcl::search::KdTree<pcl::PointXYZRGB>);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.03);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  // cluster with region growing
  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize(_settings.rg_min_cluster_size);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(10);
  reg.setInputCloud(_cloud);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);
  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  // euclidean cluster each cluster
  // region growing sometimes fails clustering
  // the reason behind: RGS uses neighbors but not with distance
  int num_rgs_clusters = clusters.size();
  for (auto c = clusters.begin(); c != clusters.end(); ) {
    // when all initial clusters were checked, return
    if (--num_rgs_clusters == 0) break;

    // create point cloud from detected cluster indices
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster
      (new pcl::PointCloud<pcl::PointXYZ>);
    cluster->width = c->indices.size();
    cluster->height = 1;
    cluster->points.resize(cluster->width * cluster->height);
    int index = 0;
    for (auto it = c->indices.begin(); it != c->indices.end(); ++it) {
      cluster->points[index].x = _cloud->points[*it].x;
      cluster->points[index].y = _cloud->points[*it].y;
      cluster->points[index].z = _cloud->points[*it].z;
      ++index;
    }

    // euclidean clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree
      (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cluster);
    std::vector<pcl::PointIndices> ec_clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(_settings.ec_min_cluster_size);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cluster);
    ec.extract(ec_clusters);

    if (ec_clusters.size() == 0) {
      c = clusters.erase(c);
      continue;
    }

    if (ec_clusters.size() > 1 || // cluster can be divided
        (ec_clusters.size() == 1 && // noises found in cluster
         ec_clusters.at(0).indices.size() != c->indices.size())) {
      // add newly seperated clusters
      for (auto nc = ec_clusters.begin(); nc != ec_clusters.end(); ++nc) {
        pcl::PointIndices pi;
        pi.indices.resize(nc->indices.size());
        // indices in masked point cloud -> indices in original point cloud
        for (unsigned int i = 0; i < nc->indices.size(); ++i)
          pi.indices[i] = c->indices.at(nc->indices.at(i));
        clusters.push_back(pi);
      }

      c = clusters.erase(c);
      continue;
    }

    ++c;
  }

  // evaluate whether cluster is within expected range
  // this operation removes floors and walls
  for (auto it = clusters.begin(); it != clusters.end(); ) {
    Eigen::Vector3f center = {0.0, 0.0, 0.0};
    Eigen::Vector3f normal = {0.0, 0.0, 0.0};

    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      center +=
        Eigen::Vector3f(_cloud->points[*pit].x, _cloud->points[*pit].y,
                        _cloud->points[*pit].z);
      if (!std::isnan(normals->points[*pit].normal_x) &&
          !std::isnan(normals->points[*pit].normal_y) &&
          !std::isnan(normals->points[*pit].normal_z))
        normal +=
          Eigen::Vector3f(normals->points[*pit].normal_x,
                          normals->points[*pit].normal_y,
                          normals->points[*pit].normal_z);
    }

    center /= it->indices.size();
    normal /= it->indices.size();
    if (center.norm() > _settings.dist_thre) {
      it = clusters.erase(it);
      continue;
    }

    // add object to scene
    roboenvcv::objectarea obj;
    obj.indices3d.assign(it->indices.begin(), it->indices.end());
    obj.visible3d = true;
    obj.center3d = center;
    normal.normalize();
    obj.normal3d = normal;
    scene.push_back(obj);
    ++it;
  }

  // find indices without a cluster label
  cv::Mat binary_img = cv::Mat::zeros(_cloud->height, _cloud->width, CV_8U);
  cv::Mat outmost(_cloud->height, _cloud->width, CV_8U);
  int outmost_label = -1;
  if (_settings.second_clustering) {
    std::vector<uchar> indices_without_label(_cloud->points.size(), 255);
    for (auto it = clusters.begin(); it != clusters.end(); ++it)
      for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        indices_without_label[*pit] = 0;

    // depth cut
    for (auto it = _cloud->points.begin(); it != _cloud->points.end(); ++it)
      if (_settings.remove_nan_in_sc &&
          std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
        indices_without_label[static_cast<int>(it - _cloud->points.begin())] = 0;
      else if (it->z > _settings.dist_thre)
        indices_without_label[static_cast<int>(it - _cloud->points.begin())] = 0;

    // binary cluster non-labeled regions
    int bottom_row_cut = 5; // cut bottom of image as, usually NaN, and not reachable
    // cv::Mat binary_img = cv::Mat::zeros(_cloud->height, _cloud->width, CV_8U);
    int at = 0;
    for (unsigned int i = 0; i < binary_img.rows - bottom_row_cut; ++i)
      for (unsigned int j = 0; j < binary_img.cols; ++j)
        binary_img.at<uchar>(i, j) = indices_without_label[at++];
    cv::Mat labeled_image;
    cv::Mat stats;
    cv::Mat centroids;
    int n_labels =
      cv::connectedComponentsWithStats(binary_img, labeled_image, stats, centroids);

    // analyze non-labeled regions
    int noise_threshold = 50; // pixels
    // int outmost_label = -1;
    int max_outmost_area = 0;
    for (int k = 1; k < n_labels; ++k) {
      int *param = stats.ptr<int>(k);
      int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
      int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
      int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
      int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

      // remove noise
      if (width * height < noise_threshold) continue;
      // remove region adjacent to the edge of image
      // this removes edge NaN noises all together
      // but will also remove some connected regions as well
      // note, often the NaN edges do connect to objects through object NaN borders
      if (x == 0 || (x + width) == _cloud->width ||
          y == 0 || (y + height) == _cloud->height) {
        if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA] > max_outmost_area) {
          outmost_label = k; // keep track of largest edge region
          max_outmost_area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];
        }
        continue;
      }

      // no compression when NaN values are removed
      // region with missing points will be eliminated if so
      if (_settings.remove_nan_in_sc) {
        roboenvcv::objectarea obj;
        // get 3d data and color data
        Get3DdataFrom2DBounds(obj, x, y, width, height, k,
                              labeled_image, _cloud, normals,
                              _settings.leap_threshold_in_sc);
        obj.bounds2d =
          cv::Rect(x*w_scale, y*h_scale, width*w_scale, height*h_scale);
        scene.push_back(obj);
        continue;
      }

      // given region, conduct border suppression

      // get roi
      cv::Mat roi = binary_img(cv::Rect(x, y, width, height));

      // find contours destroyes image, so clone
      std::vector<std::vector<cv::Point> > contours;
      cv::findContours(roi.clone(), contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

      // break region apart by suppressing contours
      int suppress_margin = 6;
      for (auto contour = contours.begin(); contour != contours.end(); ++contour)
        cv::polylines(roi, *contour, true, cv::Scalar(0), suppress_margin);

      // get new clusters
      cv::Mat labeled_image;
      cv::Mat stats;
      cv::Mat centroids;
      int n_labels =
        cv::connectedComponentsWithStats(roi, labeled_image, stats, centroids, 4);

      // add object to scene
      int lower_noise_threshold = 10;
      for (int k = 1; k < n_labels; ++k) {
        int *param = stats.ptr<int>(k);
        int x_k = x + param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
        int y_k = y + param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
        int height_k = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
        int width_k = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

        // remove noise
        if (width_k * height_k < lower_noise_threshold) continue;

        roboenvcv::objectarea obj;
        // get 3d data and color data
        Get3DdataFrom2DBounds(obj, x_k, y_k, width_k, height_k, k,
                              labeled_image, _cloud, normals,
                              _settings.leap_threshold_in_sc);
        obj.bounds2d =
          cv::Rect(x_k*w_scale, y_k*h_scale, width_k*w_scale, height_k*h_scale);
        scene.push_back(obj);
      }
    }

    // largest edge region usually has a complex over connection
    // break region apart by suppressing countour regions
    // why this works: connections are usually due to undefined object edges
    // the suppression eliminates such undefined edges
    if (outmost_label > 0 && !_settings.remove_nan_in_sc) {
      int at = 0;
      for (unsigned int i = 0; i < labeled_image.rows; ++i)
        for (unsigned int j = 0; j < labeled_image.cols; ++j)
          if (labeled_image.at<int>(i, j) == outmost_label)
            outmost.at<uchar>(i, j) = 255;
          else
            outmost.at<uchar>(i, j) = 0;

      // find contours destroyes image, so clone
      std::vector<std::vector<cv::Point> > contours;
      cv::findContours(outmost.clone(), contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

      // break region apart by suppressing contours
      int suppress_margin = 6;
      for (auto contour = contours.begin(); contour != contours.end(); ++contour)
        cv::polylines(outmost, *contour, true, cv::Scalar(0), suppress_margin);

      // get new clusters
      cv::Mat labeled_image;
      cv::Mat stats;
      cv::Mat centroids;
      int n_labels =
        cv::connectedComponentsWithStats(outmost, labeled_image, stats, centroids, 4);

      // add clusters to list if not too small
      // because, clusters were suppressed, use a lower noise threshold
      // the upper threshold will remove backgrounds
      int lower_noise_threshold = 30;
      int upper_noise_threshold = 300;
      for (int k = 1; k < n_labels; ++k) {
        int *param = stats.ptr<int>(k);
        int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
        int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
        int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
        int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

        // remove noise
        if (width * height < lower_noise_threshold) continue;

        // big clusters nearby camera are likely to be objects, so do not remove
        // when the bound of background clusters covers the whole image,
        // the cluster center is the center of image
        // the distance threshold is set so that such clusters are removed
        if (y + 0.5 * height <= 0.5 * _cloud->height + 1)
          if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA]
              > upper_noise_threshold)
            continue;

        roboenvcv::objectarea obj;
        // get current cluster and 3d data + color data
        Get3DdataFrom2DBounds(obj, x, y, width, height, k,
                              labeled_image, _cloud, normals,
                              _settings.leap_threshold_in_sc);
        obj.bounds2d =
          cv::Rect(x*w_scale, y*h_scale, width*w_scale, height*h_scale);
        scene.push_back(obj);
      }
    }
  }

  // analyze bounds, corners, color properties of RGS regions
  for (auto it = clusters.begin(); it != clusters.end(); ) {
    auto obj = scene.begin() + static_cast<int>(it - clusters.begin());

    std::vector<uchar> cluster1d(_cloud->points.size(), 0);
    std::vector<cv::Vec3b> colors1d(it->indices.size());
    int color1d_idx = 0;
    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      cluster1d[*pit] = 255;
      // in the meantime, get color as well
      uint32_t rgb = *reinterpret_cast<int*>(&_cloud->points[*pit].rgb);
      colors1d[color1d_idx++] = cv::Vec3b
        ((rgb) & 0x0000ff, (rgb >> 8) & 0x0000ff, (rgb >> 16) & 0x0000ff);
    }

    // create cv::Mat from cluster1d
    cv::Mat cluster(_cloud->height, _cloud->width, CV_8U);
    int at = 0;
    for (unsigned int i = 0; i < cluster.rows; ++i)
      for (unsigned int j = 0; j < cluster.cols; ++j)
        cluster.at<uchar>(i, j) = cluster1d[at++];

    // get bounding box of cluster
    auto bb = cv::boundingRect(cluster);
    obj->bounds2d =
      cv::Rect(bb.x*w_scale, bb.y*h_scale, bb.width*w_scale, bb.height*h_scale);

    // get color information of cluster
    int quantize_amount = 4;
    std::vector<cv::Vec3b> dominant_colors(quantize_amount);
    roboenvcv::medianCut(colors1d, 0, 0, dominant_colors);
    // check how many colors match environment color information
    int expected_matches = 3;
    int matches = 0;
    for (auto c = dominant_colors.begin(); c != dominant_colors.end(); ++c)
      if (roboenvcv::distance(roboenvcv::rgb2lab(*c),
                                 roboenvcv::rgb2lab(_env_color))
          < _settings.color_thre)
        ++matches;
    if (matches >= expected_matches) { // if likely environment
      if (_debug_folder != "")
        roboenvcv::drawPalette
          (dominant_colors, _env_color, _debug_folder + "objectness_"
           + std::to_string(obj->bounds2d.width) + "x"
           + std::to_string(obj->bounds2d.height)); // give identical name
      env.push_back(*obj);
      it = clusters.erase(it);
      scene.erase(obj); // remove environment object(e.g. table) from scene
      continue;
    }

    // from color palette, get best matched color names
    roboenvcv::getColorNames
      (dominant_colors, obj->properties.colors, roboenvcv::colorMap9);

    ++it;
  }

  auto end = std::chrono::high_resolution_clock::now();

  std::cout << "detection time "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count()
            << std::endl;

  // view results if debug mode is true

  if (_debug_folder != "") {
    // draw bounds2d

    for (auto it = scene.begin(); it != scene.end(); ++it) {
      if (it < scene.begin() + clusters.size())
        cv::rectangle(_img, it->bounds2d, cv::Scalar(0, 255, 0), 2);
      else
        cv::rectangle(_img, it->bounds2d, cv::Scalar(255, 0, 0), 2);

      if (!it->visible3d)
        continue;

      // write color properties info
      std::string text = "";
      for (auto c = it->properties.colors.begin();
           c != it->properties.colors.end(); ++c) {
        std::ostringstream val;
        val << std::setprecision(2) << c->second;
        text += c->first + "(" + val.str() + ") ";
      }
      cv::putText(_img, text,
                  cv::Point(it->bounds2d.x, it->bounds2d.y + it->bounds2d.height),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1.0);

      // draw norm
      float s = 10; // scale
      cv::Point center(it->bounds2d.x + 0.5 * it->bounds2d.width,
                       it->bounds2d.y + 0.5 * it->bounds2d.height);
      // note: normal z value is likely negative
      cv::Point normal(
          s * static_cast<float>(it->normal3d.x()) / fabs(it->normal3d.z()),
          s * static_cast<float>(it->normal3d.y()) / fabs(it->normal3d.z()));
      cv::line(_img, center, cv::Point(center.x + normal.x, center.y + normal.y),
               cv::Scalar(255, 255, 0));
      cv::circle(_img, cv::Point(center.x + normal.x, center.y + normal.y),
                 2, cv::Scalar(255, 255, 0), 2);
    }

    // show results

    if (_settings.second_clustering) {
      cv::imwrite(_debug_folder + "objectness_mid1.jpg", binary_img);
      if (outmost_label > 0)
        cv::imwrite(_debug_folder + "objectness_mid2.jpg", outmost);
    }
    cv::imwrite(_debug_folder + "objectness.jpg", _img);
  }

  return {scene, env};
}

//////////////////////////////////////////////////
std::vector<int> roboenvcv::FindTarget
(std::string _target_color, std::vector<roboenvcv::objectarea> &_scene,
 float _thre)
{
  if (_target_color == "any") {
    std::vector<int> result(_scene.size());
    std::iota(std::begin(result), std::end(result), 0);
    return result;
  }

  std::vector<std::pair<int, float> > candidates;

  for (auto obj = _scene.begin(); obj != _scene.end(); ++obj) {
    if (!obj->visible3d) continue;
    for (auto c = obj->properties.colors.begin();
         c != obj->properties.colors.end(); ++c)
      if (c->first == _target_color) {
        candidates.push_back({static_cast<int>(obj - _scene.begin()), c->second});
        break;
      }
  }

  if (candidates.size() == 0)
    return std::vector<int> {};

  // order candidates such that best match comes first
  std::sort(candidates.begin(), candidates.end(),
            [](std::pair<int, float> x, std::pair<int, float> y) -> bool {
              return (x.second > y.second);
            });

  // divide candidates into fields depending on color score
  std::vector<std::vector<std::pair<int, float> > > ordered_candidates(1);
  auto oc = ordered_candidates.begin();
  float best_score = candidates.begin()->second;
  for (auto obj = candidates.begin(); obj != candidates.end(); ++obj) {
    auto s = _scene.begin() + obj->first;
    if (fabs(obj->second - best_score) < _thre) {
      oc->push_back({obj->first, s->center3d.norm()});
    } else { // any score with more than _thre difference, add to next field
      best_score = obj->second;
      ordered_candidates.push_back({{obj->first, s->center3d.norm()}});
      oc = ordered_candidates.end() - 1;
    }
  }

  // sort matches by distance and add to result
  std::vector<int> result(candidates.size());
  auto it = result.begin();
  for (auto c = ordered_candidates.begin(); c != ordered_candidates.end(); ++c) {
    std::sort(c->begin(), c->end(),
              [](std::pair<int, float> x, std::pair<int, float> y) -> bool {
                return (x.second < y.second);
              });
    for (auto obj = c->begin(); obj != c->end(); ++obj)
      *it++ = obj->first;
  }

  return result;
}

//////////////////////////////////////////////////
roboenvcv::objectarea patchGrowth
(roboenvcv::objectarea _target, int _search_rows, float _include_depths,
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat _img)
{
  std::vector<int> indices(_target.indices3d.begin(), _target.indices3d.end());
  std::sort(indices.begin(), indices.end());
  int min_index = indices.at(0);
  int max_index = indices.at(indices.size() - 1);

  int target_left_border = _target.bounds2d.x * _cloud->width / _img.cols;
  int target_right_border =
    (_target.bounds2d.x + _target.bounds2d.width) * _cloud->width / _img.cols;

  int upper_border, lower_border;
  if (_search_rows < 0) {
    int target_lower_border =
      (_target.bounds2d.y + _target.bounds2d.height) * _cloud->height / _img.rows;
    upper_border = target_lower_border + 1;
    lower_border = std::min(static_cast<int>(_cloud->height) - 1,
                            target_lower_border - _search_rows);
  } else {
    int target_upper_border = _target.bounds2d.y * _cloud->height / _img.rows;
    upper_border = std::max(0, target_upper_border - _search_rows);
    lower_border = target_upper_border - 1;
  }

  roboenvcv::objectarea res;

  for (int j = lower_border; j >= upper_border; --j) {
    for (int i = target_left_border; i <= target_right_border; ++i) {
      int idx = j * _cloud->width + i;

      if (std::isnan(_cloud->points.at(idx).x)
          || std::isnan(_cloud->points.at(idx).y)
          || std::isnan(_cloud->points.at(idx).z))
        continue;

      if (_search_rows < 0) {
        if (idx <= max_index) // already in indices
          continue;
      } else {
        if (idx >= min_index) // already in indices
          continue;
      }

      float diff;
      if (_include_depths < 0)
        diff = _target.center3d.z() - _cloud->points.at(idx).z;
      else
        diff = _cloud->points.at(idx).z - _target.center3d.z();

      if (diff < fabs(_include_depths) && diff > 0) {
        res.indices3d.push_back(idx);
        res.center3d +=
          Eigen::Vector3f(_cloud->points.at(idx).x,
                          _cloud->points.at(idx).y,
                          _cloud->points.at(idx).z);
      }
    }
  }

  // get 2d bounds
  cv::Mat binary_img = cv::Mat::zeros(_cloud->height, _cloud->width, CV_8U);
  int at = 0;
  for (auto p = res.indices3d.begin(); p != res.indices3d.end(); ++p) {
    int j = *p / _cloud->width;
    int i = *p - j * _cloud->width;
    binary_img.at<uchar>(j, i) = 255;
  }
  cv::Mat labeled_image;
  cv::Mat stats;
  cv::Mat centroids;
  int n_labels =
    cv::connectedComponentsWithStats(binary_img, labeled_image, stats, centroids);

  int max_area = std::numeric_limits<int>::min();
  float w_scale = _img.cols / _cloud->width;
  float h_scale = _img.rows / _cloud->height;
  for (int k = 1; k < n_labels; ++k) {
    int *param = stats.ptr<int>(k);
    int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
    int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
    int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
    int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

    if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA] > max_area) {
      res.bounds2d =
        cv::Rect(x*w_scale, y*h_scale, width*w_scale, height*h_scale);
      max_area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];
    }
  }

  if (res.indices3d.size() > 0)
    res.center3d /= res.indices3d.size();

  return res;
}

//////////////////////////////////////////////////
void roboenvcv::drawBestMatch
(std::vector<roboenvcv::objectarea> &_objects, cv::Mat &_img,
 std::vector<int> &_sorted_idx, cv::Scalar _color)
{
  for (auto it = _sorted_idx.begin(); it != _sorted_idx.end(); ++it) {
    auto obj = _objects.begin() + *it;
    int k = static_cast<int>(it - _sorted_idx.begin()) * 2;
    cv::rectangle(_img, obj->bounds2d, _color, std::max(2, 10 - k));
  }
}

//////////////////////////////////////////////////
roboenvcv::graspconfig roboenvcv::ConfigurationFromLocal1DState
(int _target, std::vector<roboenvcv::objectarea> &_scene,
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat &_img,
 std::string _debug_folder)
{
  roboenvcv::graspconfig result;
  auto target = _scene.begin() + _target;

  // check sparsity of surroundings
  // here, objects with more than two detected facets will not be sparse
  // only objects with one detected facet and are far apart will be sparse
  float distance_threshold = 0.15; // m
  std::vector<std::vector<roboenvcv::objectarea>::iterator > nearby_objects;
  for (auto obj = _scene.begin(); obj != _scene.end(); ++obj) {
    if (!obj->visible3d) continue; // currently not supported
    if (obj == target) continue; // obviously, don't include self
    if ((obj->center3d - target->center3d).norm() < distance_threshold)
      nearby_objects.push_back(obj);
  }

  if (nearby_objects.size() == 0) {
    std::cout << "sparse top-grasp-able!\n";
    result.sparse = 1;
    result.contact = 0;
    return result;
  }

  // get indices bounds of target object
  // note, bounds2d is image coords so cannot be used as comparison
  int bound_y = std::numeric_limits<int>::max();
  int bound_left = std::numeric_limits<int>::max();
  int bound_right = -1;
  for (auto p = target->indices3d.begin(); p != target->indices3d.end(); ++p) {
    int y = *p / _cloud->width;
    int x = *p - y * _cloud->width;
    if (y < bound_y) bound_y = y;
    if (x < bound_left) bound_left = x;
    if (x > bound_right) bound_right = x;
  }

  // check top grasp-ability from environment
  // check depth of points behind target
  float depth_threshold = 0.02; // m
  std::vector<std::vector<roboenvcv::objectarea>::iterator > objects_behind;
  for (auto it = nearby_objects.begin(); it != nearby_objects.end(); ++it) {
    auto obj = *it;

    for (auto p = obj->indices3d.begin(); p != obj->indices3d.end(); ++p) {
      int y = *p / _cloud->width;
      int x = *p - y * _cloud->width;
      // check if point is behind object
      if (y < bound_y && bound_left < x && x < bound_right)
        // check if point is deeper than target
        if (_cloud->points[*p].z < target->center3d.z() + depth_threshold) {
          objects_behind.push_back(obj);
          break;
        }
    }
  }

  // if target is not buried, target is likely top-grasp-able
  if (objects_behind.size() == 0) {
    std::cout << "not buried top-grasp-able!\n";
    result.sparse = 0;
    result.contact = 0;
    return result;
  }

  // check norm of objects behind and check for facets
  bool detected_parallel = false;
  std::vector<std::vector<roboenvcv::objectarea>::iterator > facets;
  for (auto it = objects_behind.begin(); it != objects_behind.end(); ++it) {
    auto obj = *it;

    // find objects right behind considering orientation
    // the following projected dot product can calculate this
    Eigen::Vector3f v = target->center3d - obj->center3d;
    float c_norm = std::sqrt(std::pow(v.x(), 2) + std::pow(v.y(), 2));
    float n_norm =
      std::sqrt(std::pow(target->normal3d.x(), 2)
                + std::pow(target->normal3d.y(), 2));
    if (fabs(v.x() * target->normal3d.x() + v.y() * target->normal3d.y())
        < 0.7 * c_norm * n_norm)
      continue;

    // check clockwise-ness of 3d dot product
    // parallel > front facing > slightly bigger object behind (contact-grasp-able)
    // c clockwise > front facing > detected facet behind (further analyze)
    // clockwise > top facing > detected object preventing grasp (not grasp-able)
    float dot3d = target->normal3d.dot(obj->normal3d);

    if (0.7 < dot3d) { // likely parallel
      detected_parallel = true;
      continue;
    }

    if (fabs(dot3d) < 0.3) { // likely perpendicular
      Eigen::Vector3f crossv = target->normal3d.cross(obj->normal3d);
      crossv.normalize();
      float det3d = Eigen::Vector3f(-1, 0, 0).dot(crossv);
      if (det3d < 0) { // likely clockwise (higher priority than parallel)
        std::cout << "buried not grasp-able!\n";
        result.sparse = 0;
        result.contact = -1;
        // report what object may be preventing grasp
        result.facets.push_back(static_cast<int>(obj - _scene.begin()));
        return result;
      }
      // likely counter clockwise
      facets.push_back(obj);
    }
  }

  // at least not buried
  if (detected_parallel) {
    std::cout << "large object behind, insist contact grasp!\n";
    result.sparse = 0;
    result.contact = 1;
    return result;
  }

  // if no objects behind are preventing grasp
  if (facets.size() == 0) {
    std::cout << "no direct behind objects found, likely top-grasp-able!\n";
    result.sparse = 0;
    result.contact = 0;
    return result;
  }

  // analyze whether facet is pile of objects or single facets
  for (auto it = facets.begin(); it != facets.end(); ++it) {
    auto obj = *it;

    // get facet image as gray
    cv::Mat gray;
    cv::cvtColor(_img(obj->bounds2d), gray, CV_RGB2GRAY);
    cv::Mat stats; // information of each package cluster if found
    if (roboenvcv::findPackagePattern(gray, stats, _debug_folder)) {
      std::cout << "detected pile, likely contact grasp-able!\n";
      result.sparse = 0;
      result.contact = 1;
      return result;
    }

    // keep track of facet id
    result.facets.push_back(static_cast<int>(obj - _scene.begin()));
  }

  // code exits here
  // recursively call ConfigurationFromLocal1DState when facet is detected
  std::cout << "detected facet, state could not be determined!\n";
  result.sparse = -1;
  result.contact = -1;
  return result;
}
