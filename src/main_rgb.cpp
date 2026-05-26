#include <mutex>
#include <iostream>
#include <cstring>
#include <thread>
#include <memory>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <xvsdk/xv-sdk.h>
#include <xvsdk/xv-types.h>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

std::mutex rgb_mutex;

class RGBNode : public rclcpp::Node
{
public:
  RGBNode() : rclcpp::Node("RGBNode"), color_id_(-1)
  {
    rgb_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("rgb_image", 10);

    std::map<std::string, std::shared_ptr<xv::Device>> devices =
        xv::getDevices(5.0);

    if (devices.empty())
    {
      std::cerr << "No XVisio device detected." << std::endl;
      rclcpp::shutdown();
      return;
    }

    device = devices.begin()->second;

    if (!device)
    {
      std::cerr << "XVisio device pointer is null." << std::endl;
      rclcpp::shutdown();
      return;
    }
  }

  void init()
  {
    image_transport::ImageTransport it(shared_from_this());
    pub_ = it.advertise("image", 1);

    if (!device)
    {
      std::cerr << "Cannot start RGB camera because device is null." << std::endl;
      return;
    }

    if (!device->colorCamera())
    {
      std::cerr << "No color camera found on XVisio device." << std::endl;
      return;
    }

    color_id_ = device->colorCamera()->registerCallback(
        std::bind(&RGBNode::rgb_callback, this, std::placeholders::_1));

    device->colorCamera()->start();

    std::cout << "XVisio RGB camera started successfully." << std::endl;
  }

  ~RGBNode()
  {
    try
    {
      if (device && device->colorCamera() && color_id_ >= 0)
      {
        device->colorCamera()->unregisterCallback(color_id_);
        device->colorCamera()->stop();
      }
    }
    catch (...)
    {
      std::cerr << "Warning: exception while stopping RGB camera." << std::endl;
    }
  }

  void rgb_callback(xv::ColorImage const &color_img)
  {
    std::lock_guard<std::mutex> lock(rgb_mutex);

    std::shared_ptr<xv::RgbImage> rgb_image =
        std::make_shared<xv::RgbImage>(color_img.toRgb());

    if (!rgb_image || !rgb_image->data)
    {
      std::cerr << "Received invalid RGB image." << std::endl;
      return;
    }

    sensor_msgs::msg::Image rgb_msg;
    rgb_msg.header.frame_id = "xvisio_rgb_frame";
    rgb_msg.header.stamp = this->now();
    rgb_msg.height = rgb_image->height;
    rgb_msg.width = rgb_image->width;
    rgb_msg.encoding = "rgb8";
    rgb_msg.step = rgb_msg.width * 3;

    size_t image_size = rgb_msg.height * rgb_msg.step;
    rgb_msg.data.resize(image_size);

    std::memcpy(rgb_msg.data.data(), rgb_image->data.get(), image_size);

    cv::Mat src = cv_bridge::toCvCopy(rgb_msg, "rgb8")->image;
    cv::Mat dst;

    cv::resize(src, dst, cv::Size(), 0.25, 0.25, cv::INTER_AREA);

    std::shared_ptr<sensor_msgs::msg::Image> out_msg =
        cv_bridge::CvImage(rgb_msg.header, "rgb8", dst).toImageMsg();

    pub_.publish(out_msg);

    // Also publish full-size raw RGB image
    rgb_publisher_->publish(rgb_msg);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
  std::shared_ptr<xv::Device> device;
  image_transport::Publisher pub_;
  int color_id_;
};

int main(int argc, char **argv)
{
  rclcpp::InitOptions init_options;
  init_options.auto_initialize_logging(false);

  rclcpp::init(argc, argv, init_options);

  auto node = std::make_shared<RGBNode>();

  if (rclcpp::ok())
  {
    node->init();
    rclcpp::spin(node);
  }

  rclcpp::shutdown();
  return 0;
}