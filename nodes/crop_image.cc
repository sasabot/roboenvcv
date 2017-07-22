#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <roboenvcv/UInt8ArrayStamped.h>
#include <sensor_msgs/Image.h>
#include <roboenvcv/RegionOfInterestInfos.h>

#include <mutex>

//#define __DEBUG__

sensor_msgs::Image image_;
std::mutex image_mutex_;
int sensor_id_;
ros::Publisher crop_publisher_;

#ifdef __DEBUG__
ros::Publisher debug_image_publisher_;
#endif

void ImageCallback(const sensor_msgs::Image::ConstPtr &_msg) {
  image_mutex_.lock();
  image_ = *_msg;
  image_mutex_.unlock();
}

void BoundingsCallback
(const roboenvcv::RegionOfInterestInfos::ConstPtr &_msg) {
  roboenvcv::UInt8ArrayStamped msg;
  // first 1 byte is number of faces
  msg.data.push_back(static_cast<uint8_t>(_msg->infos.size()));

  int at = 1;
  image_mutex_.lock();
  if (image_.data.size() == 0) { // bad image
    image_mutex_.unlock();
    return;
  }
  int stride = image_.step / image_.width;
#ifdef __DEBUG__
  sensor_msgs::Image dbg;
  dbg.header = image_.header;
  dbg.encoding = "bgr8";
  dbg.is_bigendian = image_.is_bigendian;
#endif
  for (auto info = _msg->infos.begin(); info != _msg->infos.end(); ++info) {
    int width = info->roi.width;
    int height = info->roi.height;
    msg.data.resize(msg.data.size() + width * height * 3 + 20);

    // 2 bytes width
    uint8_t *width_bytes;
    width_bytes = reinterpret_cast<uint8_t*>(&width);
    msg.data[at++] = width_bytes[0];
    msg.data[at++] = width_bytes[1];
    // 2 bytes height
    uint8_t *height_bytes;
    height_bytes = reinterpret_cast<uint8_t*>(&height);
    msg.data[at++] = height_bytes[0];
    msg.data[at++] = height_bytes[1];
    // 4 bytes x
    uint8_t *x_bytes;
    float x = static_cast<float>(info->center3d.x);
    x_bytes = reinterpret_cast<uint8_t*>(&x);
    msg.data[at++] = x_bytes[0];
    msg.data[at++] = x_bytes[1];
    msg.data[at++] = x_bytes[2];
    msg.data[at++] = x_bytes[3];
    // 4 bytes y
    uint8_t *y_bytes;
    float y = static_cast<float>(info->center3d.y);
    y_bytes = reinterpret_cast<uint8_t*>(&y);
    msg.data[at++] = y_bytes[0];
    msg.data[at++] = y_bytes[1];
    msg.data[at++] = y_bytes[2];
    msg.data[at++] = y_bytes[3];
    // 4 bytes z
    uint8_t *z_bytes;
    float z = static_cast<float>(info->center3d.z);
    z_bytes = reinterpret_cast<uint8_t*>(&z);
    msg.data[at++] = z_bytes[0];
    msg.data[at++] = z_bytes[1];
    msg.data[at++] = z_bytes[2];
    msg.data[at++] = z_bytes[3];
    // 4 bytes sensor_id
    uint8_t *id_bytes;
    id_bytes = reinterpret_cast<uint8_t*>(&sensor_id_);
    msg.data[at++] = id_bytes[0];
    msg.data[at++] = id_bytes[1];
    msg.data[at++] = id_bytes[2];
    msg.data[at++] = id_bytes[3];

    // rest, copy image
    int x_offset = info->roi.x_offset;
    int y_offset = info->roi.y_offset;
    if (stride == 3) {
      for (int y = y_offset; y < (y_offset + height); ++y) {
        std::copy(image_.data.begin() + y*image_.step + x_offset*stride,
                  image_.data.begin() + y*image_.step + (x_offset + width)*stride,
                  msg.data.begin() + at);
        at += width * stride;
      }
    } else { // image is bgra, exclude 'a'
      for (int y = y_offset; y < (y_offset + height); ++y)
        for (int x = x_offset; x < (x_offset + width); ++x) {
          auto dat = image_.data.begin() + y*image_.step + x*stride; 
          std::copy(dat, dat + 3, msg.data.begin() + at);
          at += stride;
        }
    }

#ifdef __DEBUG__
    dbg.width = width;
    dbg.height = height;
    dbg.step = 3 * dbg.width;
    dbg.data.resize(width * height * 3);
    std::copy(msg.data.begin() + at - dbg.data.size(), msg.data.begin() + at,
              dbg.data.begin());
#endif
  }
  image_mutex_.unlock();

  crop_publisher_.publish(msg);
#ifdef __DEBUG__
  debug_image_publisher_.publish(dbg);
#endif
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "crop_raw_images_from_bounds");
  ros::NodeHandle nh("~");

  sensor_id_ = 0;
  nh.getParam("sensor_id", sensor_id_);

  crop_publisher_ =
    nh.advertise<roboenvcv::UInt8ArrayStamped>
    ("/roboenvcv/cropped/images", 100);

#ifdef __DEBUG__
  debug_image_publisher_ =
    nh.advertise<sensor_msgs::Image>
    ("/roboenvcv/cropped/image/debug/tail", 1);
#endif

  ros::CallbackQueue image_queue;
  ros::SubscribeOptions image_ops =
    ros::SubscribeOptions::create<sensor_msgs::Image>(
        "/camera/high_resolution_image",
        10,
        boost::bind(&ImageCallback, _1),
        ros::VoidPtr(),
        &image_queue);
  ros::Subscriber image_sub = nh.subscribe(image_ops);
  ros::AsyncSpinner image_spinner(1, &image_queue);
  image_spinner.start();

  ros::CallbackQueue bb_queue;
  ros::SubscribeOptions bb_ops =
    ros::SubscribeOptions::create<roboenvcv::RegionOfInterestInfos>(
        "/roboenvcv/cropped/boundings",
        10,
        boost::bind(&BoundingsCallback, _1),
        ros::VoidPtr(),
        &bb_queue);
  ros::Subscriber bb_sub = nh.subscribe(bb_ops);
  ros::AsyncSpinner bb_spinner(1, &bb_queue);
  bb_spinner.start();

  ros::spin();
}
