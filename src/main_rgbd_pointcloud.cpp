#include <iostream>
#include <mutex>
#include <memory>
#include <map>
#include <cstring>
#include <cmath>
#include <limits>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <xvsdk/xv-sdk.h>
#include <xvsdk/xv-types.h>

std::mutex rgbd_mutex;

class RGBDPointCloudNode : public rclcpp::Node
{
public:
  RGBDPointCloudNode() : rclcpp::Node("RGBDPointCloudNode"), rgbd_id_(-1)
  {
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("rgbd_rgb", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("rgbd_depth", 10);
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rgbd_points", 10);

    auto devices = xv::getDevices(5.0);

    if (devices.empty())
    {
      std::cerr << "No XVisio device detected." << std::endl;
      rclcpp::shutdown();
      return;
    }

    device_ = devices.begin()->second;

    if (!device_ || !device_->tofCamera() || !device_->colorCamera())
    {
      std::cerr << "RGB-D point cloud requires both ToF and RGB camera." << std::endl;
      rclcpp::shutdown();
      return;
    }
  }

  void init()
  {
    auto tof = device_->tofCamera();
    auto color = device_->colorCamera();

    color->setResolution(xv::ColorCamera::Resolution::RGB_640x480);
    color->start();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    tof->setStreamMode(xv::TofCamera::StreamMode::DepthOnly);
    tof->start();

    rgbd_id_ = tof->registerColorDepthImageCallback(
        std::bind(&RGBDPointCloudNode::rgbd_callback, this, std::placeholders::_1));

    std::cout << "XVisio RGB-D colored point cloud node started." << std::endl;

    printCalibration("RGB", device_->colorCamera()->calibration());
    printCalibration("TOF", device_->tofCamera()->calibration());
  }

  ~RGBDPointCloudNode()
  {
    try
    {
      if (device_ && device_->tofCamera())
      {
        if (rgbd_id_ >= 0)
        {
          device_->tofCamera()->unregisterColorDepthImageCallback(rgbd_id_);
        }
        device_->tofCamera()->stop();
      }

      if (device_ && device_->colorCamera())
      {
        device_->colorCamera()->stop();
      }
    }
    catch (...)
    {
      std::cerr << "Warning: exception while stopping RGB-D point cloud node." << std::endl;
    }
  }

private:
  void rgbd_callback(const xv::DepthColorImage &rgbd)
  {
    std::lock_guard<std::mutex> lock(rgbd_mutex);

    if (!rgbd.data || rgbd.width == 0 || rgbd.height == 0)
    {
      std::cerr << "Invalid RGB-D frame." << std::endl;
      return;
    }

    const size_t width = rgbd.width;
    const size_t height = rgbd.height;
    const size_t pixel_count = width * height;
    const uint8_t *raw = rgbd.data.get();

    auto stamp = this->now();

    sensor_msgs::msg::Image rgb_msg;
    rgb_msg.header.stamp = stamp;
    rgb_msg.header.frame_id = "xvisio_rgbd_frame";
    rgb_msg.height = height;
    rgb_msg.width = width;
    rgb_msg.encoding = "rgb8";
    rgb_msg.is_bigendian = false;
    rgb_msg.step = width * 3;
    rgb_msg.data.resize(height * rgb_msg.step);

    sensor_msgs::msg::Image depth_msg;
    depth_msg.header.stamp = stamp;
    depth_msg.header.frame_id = "xvisio_rgbd_frame";
    depth_msg.height = height;
    depth_msg.width = width;
    depth_msg.encoding = "32FC1";
    depth_msg.is_bigendian = false;
    depth_msg.step = width * sizeof(float);
    depth_msg.data.resize(height * depth_msg.step);

    for (size_t i = 0; i < pixel_count; ++i)
    {
      const size_t src_offset = i * 7;
      const size_t rgb_offset = i * 3;
      const size_t depth_offset = i * sizeof(float);

      rgb_msg.data[rgb_offset + 0] = raw[src_offset + 0];
      rgb_msg.data[rgb_offset + 1] = raw[src_offset + 1];
      rgb_msg.data[rgb_offset + 2] = raw[src_offset + 2];

      std::memcpy(
          depth_msg.data.data() + depth_offset,
          raw + src_offset + 3,
          sizeof(float));
    }

    rgb_pub_->publish(rgb_msg);
    depth_pub_->publish(depth_msg);

    publish_pointcloud(rgb_msg, depth_msg);
  }
  static void printCalibration(
      const std::string& name,
      const std::vector<xv::Calibration>& calibs)
  {
      std::cout << "\n===== " << name << " Calibration =====" << std::endl;

      for (size_t i = 0; i < calibs.size(); ++i) {
          const auto& calib = calibs[i];

          std::cout << "--- calibration[" << i << "] ---" << std::endl;
          std::cout << "PDCM count: " << calib.pdcm.size() << std::endl;

          for (size_t j = 0; j < calib.pdcm.size(); ++j) {
              const auto& cam = calib.pdcm[j];

              std::cout << "PDCM[" << j << "]" << std::endl;
              std::cout << "width  = " << cam.w << std::endl;
              std::cout << "height = " << cam.h << std::endl;
              std::cout << "fx     = " << cam.fx << std::endl;
              std::cout << "fy     = " << cam.fy << std::endl;
              std::cout << "cx/u0  = " << cam.u0 << std::endl;
              std::cout << "cy/v0  = " << cam.v0 << std::endl;
          }
      }
  }

  void publish_pointcloud(
      const sensor_msgs::msg::Image &rgb_msg,
      const sensor_msgs::msg::Image &depth_msg)
  {
    const size_t width = rgb_msg.width;
    const size_t height = rgb_msg.height;
    const size_t pixel_count = width * height;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header = rgb_msg.header;
    cloud_msg.height = height;
    cloud_msg.width = width;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(pixel_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

    // Temporary intrinsics for 640x480.
    // Replace these with XVisio RGB-D calibration values later for accurate geometry.
    const float fx = 525.0f;
    const float fy = 525.0f;
    const float cx = static_cast<float>(width) / 2.0f;
    const float cy = static_cast<float>(height) / 2.0f;

    for (size_t v = 0; v < height; ++v)
    {
      for (size_t u = 0; u < width; ++u)
      {
        const size_t i = v * width + u;

        float z = 0.0f;
        std::memcpy(
            &z,
            depth_msg.data.data() + i * sizeof(float),
            sizeof(float));

        // If depth looks too large or too small, check whether SDK depth is meter or millimeter.
        if (z <= 0.0f || std::isnan(z) || std::isinf(z))
        {
          *iter_x = std::numeric_limits<float>::quiet_NaN();
          *iter_y = std::numeric_limits<float>::quiet_NaN();
          *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
          *iter_x = (static_cast<float>(u) - cx) * z / fx;
          *iter_y = (static_cast<float>(v) - cy) * z / fy;
          *iter_z = z;
        }

        *iter_r = rgb_msg.data[i * 3 + 0];
        *iter_g = rgb_msg.data[i * 3 + 1];
        *iter_b = rgb_msg.data[i * 3 + 2];

        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
      }
    }

    cloud_pub_->publish(cloud_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::shared_ptr<xv::Device> device_;
  int rgbd_id_;
};

int main(int argc, char **argv)
{
  rclcpp::InitOptions init_options;
  init_options.auto_initialize_logging(false);

  rclcpp::init(argc, argv, init_options);

  auto node = std::make_shared<RGBDPointCloudNode>();

  if (rclcpp::ok())
  {
    node->init();
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}
