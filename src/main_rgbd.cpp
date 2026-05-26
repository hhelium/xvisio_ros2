#include <iostream>
#include <mutex>
#include <memory>
#include <map>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <xvsdk/xv-sdk.h>
#include <xvsdk/xv-types.h>

std::mutex rgbd_mutex;

class RGBDNode : public rclcpp::Node
{
public:
  RGBDNode() : rclcpp::Node("RGBDNode"), rgbd_id_(-1)
  {
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("rgbd_rgb", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("rgbd_depth", 10);

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
      std::cerr << "RGB-D requires both ToF camera and RGB camera." << std::endl;
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
      std::bind(&RGBDNode::rgbd_callback, this, std::placeholders::_1)
    );

    std::cout << "XVisio RGB-D aligned stream started." << std::endl;
  }

  ~RGBDNode()
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
      std::cerr << "Warning: exception while stopping RGB-D node." << std::endl;
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

    sensor_msgs::msg::Image rgb_msg;
    sensor_msgs::msg::Image depth_msg;

    auto stamp = this->now();

    rgb_msg.header.stamp = stamp;
    rgb_msg.header.frame_id = "xvisio_rgbd_frame";
    rgb_msg.height = height;
    rgb_msg.width = width;
    rgb_msg.encoding = "rgb8";
    rgb_msg.is_bigendian = false;
    rgb_msg.step = width * 3;
    rgb_msg.data.resize(height * rgb_msg.step);

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
        sizeof(float)
      );
    }

    rgb_pub_->publish(rgb_msg);
    depth_pub_->publish(depth_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

  std::shared_ptr<xv::Device> device_;
  int rgbd_id_;
};

int main(int argc, char **argv)
{
  rclcpp::InitOptions init_options;
  init_options.auto_initialize_logging(false);

  rclcpp::init(argc, argv, init_options);

  auto node = std::make_shared<RGBDNode>();

  if (rclcpp::ok())
  {
    node->init();
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}
