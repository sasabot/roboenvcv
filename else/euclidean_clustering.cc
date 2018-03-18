#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// assumption: set parameters on cluster size
// return: nearest cluster in euclidean distance

class EuclideanClustering
{
public: EuclideanClustering(ros::NodeHandle _nh) : nh_(_nh) {
    nh_.param<float>("tolerance", tolerance_, 0.2); // 10cm
    nh_.param<int>("min_size", min_size_, 2000);
    nh_.param<int>("max_size", max_size_, 100000);
    nh_.param<float>("resize_x", resize_x_, 0.25);
    nh_.param<float>("resize_y", resize_y_, 0.25);

    nearest_cluster_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/euclidean_nearest_cluster", 1);
    on_off_ =
      nh_.advertiseService("/get_euclidean_cluster", &EuclideanClustering::OnOff, this);
  };

public: ~EuclideanClustering() {
  };

private: bool OnOff(std_srvs::SetBool::Request &_req, std_srvs::SetBool::Response &_res) {
    if (_req.data)
      point_cloud_listener_ =
        nh_.subscribe("/sensor_topic", 1,
                      &EuclideanClustering::SubscribePoints, this);
    else
      point_cloud_listener_.shutdown();
    _res.success = true;
    return true;
  };

private: void resize(const sensor_msgs::PointCloud2::ConstPtr& depth,
                     sensor_msgs::PointCloud2& res) {
    res.header.frame_id = depth->header.frame_id;
    res.header.stamp = depth->header.stamp;
    res.fields.assign(depth->fields.begin(), depth->fields.end());
    res.height = static_cast<int>(depth->height * resize_y_);
    res.width = static_cast<int>(depth->width * resize_x_);
    res.point_step = depth->point_step;
    res.row_step = res.point_step * res.width;
    res.is_dense = depth->is_dense;
    res.is_bigendian = depth->is_bigendian;

    int rgb_offset = 0;
    for (auto f = res.fields.begin(); f != res.fields.end(); ++f)
      if (f->name == "rgb") {
        rgb_offset = f->offset - 12;
        break;
      }

    float stride_x;
    float stride_y;

    if (resize_x_ > 0.5 || resize_y_ > 0.5) {
      ROS_WARN("scale larger than 0.5 may cause some errors.");
      stride_x = 1.0 / resize_x_;
      stride_y = 1.0 / resize_y_;
    } else {
      // 1.01 for scale 0.334, note, points should not be compressed with scale ~0.01
      stride_x = static_cast<int>(1.01 / resize_x_);
      stride_y = static_cast<int>(1.01 / resize_y_);
    }

    res.data.resize(res.height * res.width * res.point_step);
    float row = 0.0f;
    float at = 0.0f;
    int head = 0;
    int j = 0;
    while (j < res.data.size()) {
      int tl = static_cast<int>(head + at) * depth->point_step; // top left
      int tr = static_cast<int>(head + at + stride_x - 1) * depth->point_step; // top right
      int bl = static_cast<int>(row + stride_y - 1) * depth->row_step
        + static_cast<int>(at) * depth->point_step; // bottom left
      int br = static_cast<int>(row + stride_y - 1) * depth->row_step
        + static_cast<int>(at + stride_x - 1) * depth->point_step; // bottom right

      // get xyz
      float val[3] = {std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::quiet_NaN(),
                      std::numeric_limits<float>::quiet_NaN()};
      for (int i = 0; i < 3; ++i) {
        uint8_t tl_bytes[4] =
          {depth->data[tl++], depth->data[tl++], depth->data[tl++], depth->data[tl++]};
        float tl_float;
        std::memcpy(&tl_float, &tl_bytes, 4);
        if (std::isnan(tl_float)) {
          tr += 4; bl += 4; br += 4;
          continue;
        }

        uint8_t tr_bytes[4] =
          {depth->data[tr++], depth->data[tr++], depth->data[tr++], depth->data[tr++]};
        float tr_float;
        std::memcpy(&tr_float, &tr_bytes, 4);
        if (std::isnan(tr_float)) {
          bl += 4; br += 4;
          continue;
        }

        uint8_t bl_bytes[4] =
          {depth->data[bl++], depth->data[bl++], depth->data[bl++], depth->data[bl++]};
        float bl_float;
        std::memcpy(&bl_float, &bl_bytes, 4);
        if (std::isnan(bl_float)) {
          br += 4;
          continue;
        }

        uint8_t br_bytes[4] =
          {depth->data[br++], depth->data[br++], depth->data[br++], depth->data[br++]};
        float br_float;
        std::memcpy(&br_float, &br_bytes, 4);
        if (std::isnan(br_float))
          continue;

        val[i] = (tl_float + tr_float + bl_float + br_float) * 0.25;
      }

      auto x = reinterpret_cast<uint8_t*>(&val[0]);
      auto y = reinterpret_cast<uint8_t*>(&val[1]);
      auto z = reinterpret_cast<uint8_t*>(&val[2]);

      res.data[j++] = x[0]; res.data[j++] = x[1]; res.data[j++] = x[2]; res.data[j++] = x[3];
      res.data[j++] = y[0]; res.data[j++] = y[1]; res.data[j++] = y[2]; res.data[j++] = y[3];
      res.data[j++] = z[0]; res.data[j++] = z[1]; res.data[j++] = z[2]; res.data[j++] = z[3];

      // get rgb
      int rgb[3] = {0, 0, 0};
      tl += rgb_offset;
      tr += rgb_offset;
      bl += rgb_offset;
      br += rgb_offset;
      for (int i = 0; i < 3; ++i)
        rgb[i] = static_cast<int>((static_cast<int>(depth->data[tl++])
                                   + static_cast<int>(depth->data[tr++])
                                   + static_cast<int>(depth->data[bl++])
                                   + static_cast<int>(depth->data[br++])) * 0.25);

      j += rgb_offset;
      res.data[j++] = reinterpret_cast<uint8_t*>(&rgb[0])[0];
      res.data[j++] = reinterpret_cast<uint8_t*>(&rgb[1])[0];
      res.data[j++] = reinterpret_cast<uint8_t*>(&rgb[2])[0];
      res.data[j++] = 0;
      j += res.point_step - rgb_offset - 16;

      at += stride_x;
      if (at > depth->width - stride_x + 0.01) { // +0.01 to avoid float error
        row += stride_y;
        head = static_cast<int>(row) * depth->width;
        at = 0.0f;
      }
    }
  };

private: void SubscribePoints(const sensor_msgs::PointCloud2::ConstPtr& _msg) {
    ROS_INFO("[euclidean_clustering] subscibed once");

    // resize msg for speed up
    auto points = sensor_msgs::PointCloud2();
    resize(_msg, points);

    // ros msg -> PCLPointCloud2
    pcl::PCLPointCloud2 pcl;
    pcl_conversions::toPCL(points, pcl);

    // get point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl, *cloud);

    ROS_INFO("[euclidean_clustering] got cloud %d x %d",
             cloud->width, cloud->height);    

    // creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    // euclidean cluster
    std::vector<pcl::PointIndices> cluster_indices;;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(tolerance_);
    ec.setMinClusterSize(min_size_);
    ec.setMaxClusterSize(max_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // find nearest cluster
    int j = 0;
    int nearest_cluster = -1;
    float z_min = 10000; // large number
    for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
         it != cluster_indices.end(); ++it) {
      float z = 0.0;
      for (std::vector<int>::const_iterator pit = it->indices.begin();
           pit != it->indices.end(); ++pit)
        z += cloud->points[*pit].z;
      z /= it->indices.size();
      if (z < z_min) {
        z_min = z;
        nearest_cluster = j;
      }
      ++j;
    }

    ROS_INFO("[euclidean_clustering] nearest cluster %d points",
             static_cast<int>(
                 (cluster_indices.begin() + nearest_cluster)->indices.size()));

    // create point cloud of nearest cluster
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointIndices>::const_iterator it =
        cluster_indices.begin() + nearest_cluster;
    cloud_cluster->reserve(it->indices.size());
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) 
      cloud_cluster->points.push_back(cloud->points[*pit]);

    // publish cluster
    sensor_msgs::PointCloud2 msg;
    pcl::PCLPointCloud2 cloud_out;
    pcl::toPCLPointCloud2(*cloud_cluster, cloud_out);
    pcl_conversions::fromPCL(cloud_out, msg);
    msg.header.frame_id = _msg->header.frame_id;
    msg.header.stamp = _msg->header.stamp;
    nearest_cluster_pub_.publish(msg);

    ROS_INFO("[euclidean_clustering] published");
  };

private: ros::NodeHandle nh_;

private: ros::Subscriber point_cloud_listener_;

private: ros::Publisher nearest_cluster_pub_;

private: ros::ServiceServer on_off_;

private: float tolerance_;

private: int min_size_;

private: int max_size_;

private: float resize_x_;

private: float resize_y_;
};

typedef boost::shared_ptr<EuclideanClustering> EuclideanClusteringPtr;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "euclidean_cluster");
  ros::NodeHandle nh;

  EuclideanClusteringPtr sensor(new EuclideanClustering(nh));

  ros::spin();

  return 0;
}
