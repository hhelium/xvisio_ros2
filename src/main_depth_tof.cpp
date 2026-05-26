#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <xvsdk/xv-sdk.h>
#include <xvsdk/xv-types.h>
// #include <sensor_msgs/msg/point_cloud2.hpp>
#include <cstring> // For std::memcpy
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <thread>
#include <iostream>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>

using namespace std::chrono_literals;
std::mutex tof_mutex;

std::mutex rgb_mutex;

std::mutex slam_mutex;

// std::mutex pointcloud_mutex;
// std::shared_ptr<const xv::PointCloud> s_pointcloud = nullptr;
static std::vector<std::vector<unsigned char>> colors = {
    {0, 0, 0},     {255, 4, 0},   {255, 8, 0},   {255, 12, 0},  {255, 17, 0},
    {255, 21, 0},  {255, 25, 0},  {255, 29, 0},  {255, 34, 0},  {255, 38, 0},
    {255, 42, 0},  {255, 46, 0},  {255, 51, 0},  {255, 55, 0},  {255, 59, 0},
    {255, 64, 0},  {255, 68, 0},  {255, 72, 0},  {255, 76, 0},  {255, 81, 0},
    {255, 85, 0},  {255, 89, 0},  {255, 93, 0},  {255, 98, 0},  {255, 102, 0},
    {255, 106, 0}, {255, 110, 0}, {255, 115, 0}, {255, 119, 0}, {255, 123, 0},
    {255, 128, 0}, {255, 132, 0}, {255, 136, 0}, {255, 140, 0}, {255, 145, 0},
    {255, 149, 0}, {255, 153, 0}, {255, 157, 0}, {255, 162, 0}, {255, 166, 0},
    {255, 170, 0}, {255, 174, 0}, {255, 179, 0}, {255, 183, 0}, {255, 187, 0},
    {255, 191, 0}, {255, 196, 0}, {255, 200, 0}, {255, 204, 0}, {255, 209, 0},
    {255, 213, 0}, {255, 217, 0}, {255, 221, 0}, {255, 226, 0}, {255, 230, 0},
    {255, 234, 0}, {255, 238, 0}, {255, 243, 0}, {255, 247, 0}, {255, 251, 0},
    {255, 255, 0}, {251, 255, 0}, {247, 255, 0}, {243, 255, 0}, {238, 255, 0},
    {234, 255, 0}, {230, 255, 0}, {226, 255, 0}, {221, 255, 0}, {217, 255, 0},
    {213, 255, 0}, {209, 255, 0}, {204, 255, 0}, {200, 255, 0}, {196, 255, 0},
    {191, 255, 0}, {187, 255, 0}, {183, 255, 0}, {179, 255, 0}, {174, 255, 0},
    {170, 255, 0}, {166, 255, 0}, {162, 255, 0}, {157, 255, 0}, {153, 255, 0},
    {149, 255, 0}, {145, 255, 0}, {140, 255, 0}, {136, 255, 0}, {132, 255, 0},
    {128, 255, 0}, {123, 255, 0}, {119, 255, 0}, {115, 255, 0}, {110, 255, 0},
    {106, 255, 0}, {102, 255, 0}, {98, 255, 0},  {93, 255, 0},  {89, 255, 0},
    {85, 255, 0},  {81, 255, 0},  {76, 255, 0},  {72, 255, 0},  {68, 255, 0},
    {64, 255, 0},  {59, 255, 0},  {55, 255, 0},  {51, 255, 0},  {46, 255, 0},
    {42, 255, 0},  {38, 255, 0},  {34, 255, 0},  {29, 255, 0},  {25, 255, 0},
    {21, 255, 0},  {17, 255, 0},  {12, 255, 0},  {8, 255, 0},   {4, 255, 0},
    {0, 255, 0},   {0, 255, 4},   {0, 255, 8},   {0, 255, 12},  {0, 255, 17},
    {0, 255, 21},  {0, 255, 25},  {0, 255, 29},  {0, 255, 34},  {0, 255, 38},
    {0, 255, 42},  {0, 255, 46},  {0, 255, 51},  {0, 255, 55},  {0, 255, 59},
    {0, 255, 64},  {0, 255, 68},  {0, 255, 72},  {0, 255, 76},  {0, 255, 81},
    {0, 255, 85},  {0, 255, 89},  {0, 255, 93},  {0, 255, 98},  {0, 255, 102},
    {0, 255, 106}, {0, 255, 110}, {0, 255, 115}, {0, 255, 119}, {0, 255, 123},
    {0, 255, 128}, {0, 255, 132}, {0, 255, 136}, {0, 255, 140}, {0, 255, 145},
    {0, 255, 149}, {0, 255, 153}, {0, 255, 157}, {0, 255, 162}, {0, 255, 166},
    {0, 255, 170}, {0, 255, 174}, {0, 255, 179}, {0, 255, 183}, {0, 255, 187},
    {0, 255, 191}, {0, 255, 196}, {0, 255, 200}, {0, 255, 204}, {0, 255, 209},
    {0, 255, 213}, {0, 255, 217}, {0, 255, 221}, {0, 255, 226}, {0, 255, 230},
    {0, 255, 234}, {0, 255, 238}, {0, 255, 243}, {0, 255, 247}, {0, 255, 251},
    {0, 255, 255}, {0, 251, 255}, {0, 247, 255}, {0, 243, 255}, {0, 238, 255},
    {0, 234, 255}, {0, 230, 255}, {0, 226, 255}, {0, 221, 255}, {0, 217, 255},
    {0, 213, 255}, {0, 209, 255}, {0, 204, 255}, {0, 200, 255}, {0, 196, 255},
    {0, 191, 255}, {0, 187, 255}, {0, 183, 255}, {0, 179, 255}, {0, 174, 255},
    {0, 170, 255}, {0, 166, 255}, {0, 162, 255}, {0, 157, 255}, {0, 153, 255},
    {0, 149, 255}, {0, 145, 255}, {0, 140, 255}, {0, 136, 255}, {0, 132, 255},
    {0, 128, 255}, {0, 123, 255}, {0, 119, 255}, {0, 115, 255}, {0, 110, 255},
    {0, 106, 255}, {0, 102, 255}, {0, 98, 255},  {0, 93, 255},  {0, 89, 255},
    {0, 85, 255},  {0, 81, 255},  {0, 76, 255},  {0, 72, 255},  {0, 68, 255},
    {0, 64, 255},  {0, 59, 255},  {0, 55, 255},  {0, 51, 255},  {0, 46, 255},
    {0, 42, 255},  {0, 38, 255},  {0, 34, 255},  {0, 29, 255},  {0, 25, 255},
    {0, 21, 255},  {0, 17, 255},  {0, 12, 255},  {0, 8, 255},   {0, 4, 255},
    {0, 0, 255},   {4, 0, 255},   {8, 0, 255},   {12, 0, 255},  {17, 0, 255},
    {21, 0, 255},  {25, 0, 255},  {29, 0, 255},  {34, 0, 255},  {38, 0, 255},
    {42, 0, 255},  {46, 0, 255},  {51, 0, 255},  {55, 0, 255},  {59, 0, 255},
    {64, 0, 255}};
cv::Mat raw_to_opencv(std::shared_ptr<const xv::DepthImage> tof)
{
  cv::Mat out;
  if (tof->height > 0 && tof->width > 0)
  {
    out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
    if (tof->type == xv::DepthImage::Type::Depth_32)
    {
      float dmax = 7.5;
      const auto tmp_d = reinterpret_cast<float const *>(tof->data.get());
      for (unsigned int i = 0; i < tof->height * tof->width; i++)
      {
        const auto &d = tmp_d[i];
        if (d < 0.01 || d > 9.9)
        {
          out.at<cv::Vec3b>(i / tof->width, i % tof->width) = 0;
        }
        else
        {
          unsigned int u = static_cast<unsigned int>(
              std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
          const auto &cc = colors.at(u);
          out.at<cv::Vec3b>(i / tof->width, i % tof->width) =
              cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
        }
      }
    }
    else if (tof->type == xv::DepthImage::Type::Depth_16)
    {
      // #define DEPTH_RANGE_20M_SF       7494  //7.494m
      // #define DEPTH_RANGE_60M_SF       2494  //2.494m
      // #define DEPTH_RANGE_100M_SF      1498  //1.498m
      // #define DEPTH_RANGE_120M_SF      1249  //1.249m
      float dmax =
          2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
      const auto tmp_d = reinterpret_cast<int16_t const *>(tof->data.get());
      for (unsigned int i = 0; i < tof->height * tof->width; i++)
      {
        const auto &d = tmp_d[i];
        unsigned int u = static_cast<unsigned int>(
            std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
        const auto &cc = colors.at(u);
        out.at<cv::Vec3b>(i / tof->width, i % tof->width) =
            cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
      }
    }
    else if (tof->type == xv::DepthImage::Type::IR)
    {
      out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
      float dmax =
          2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
      auto tmp_d = reinterpret_cast<unsigned short const *>(tof->data.get());
      for (unsigned int i = 0; i < tof->height * tof->width; i++)
      {
        unsigned short d = tmp_d[i];
        unsigned int u = static_cast<unsigned int>(
            std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
        if (u < 15)
          u = 0;
        const auto &cc = colors.at(u);
        out.at<cv::Vec3b>(i / tof->width, i % tof->width) =
            cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
      }
    }
  }
  return out;
}

class DepthNode : public rclcpp::Node
{
  public:
  DepthNode() : rclcpp::Node("DepthNode"), tof_data_(nullptr), tof_id_(-1)
  {                                                                     //
    depth_publisher_ = this->create_publisher<sensor_msgs::msg::Image>( //
        "depth_image",                                                  //
        10                                                              //
    );

    std::map<std::string, std::shared_ptr<xv::Device>> devices =
        xv::getDevices(5.0);

    device = devices.begin()->second;
    // imu_id_ = device->orientationStream()->registerCallback( //
    //     [](xv::Orientation const &orien) {}                  //
    // );
    // if (device->orientationStream())
    // {
    //   device->orientationStream()->unregisterCallback(imu_id_);
    // }
    /*
    - Old:
        Edge mode: ON
        Mixed mode: ON
        Stereo: ON
        RGB: ON
        ToF: ON
        IA: OFF
        SGBM: ON
        eyetracking: OFF
        faceID: OFF
    */
    // device->sgbmCamera()->stop(); // SGBM: OFF
    if (device->tofCamera())
    {
      // - Old syntax, not reliable
      // device->tofCamera()->setLibWorkMode(
      //     static_cast<xv::TofCamera::SonyTofLibMode>(1));
      // tof_id_ = device->tofCamera()->registerCallback(
      //     std::bind(&DepthNode::tof_callback, this, std::placeholders::_1));
      // device->tofCamera()->start();

      // - new syntax
      device->tofCamera()->start();
      // device->tofCamera()->setDistanceMode(xv::TofCamera::DistanceMode::Short); // No need for Sony
      // device->tofCamera()->setFramerate(5.); // no need for sony
      device->tofCamera()->setSonyTofSetting(      //
          xv::TofCamera::SonyTofLibMode::IQMIX_DF, //
          // xv::TofCamera::Resolution::VGA,          //
          xv::TofCamera::Resolution::Unknown, //
          xv::TofCamera::Framerate::FPS_5     //
      );
      tof_id_ = device->tofCamera()->registerCallback(
          std::bind(&DepthNode::tof_callback, this, std::placeholders::_1));
    }
    // ? if (device->tofCamera()) {
    // ?     // +  device->tofCamera()->setSonyTofSetting(      //
    // ?     // +      xv::TofCamera::SonyTofLibMode::IQMIX_SF, //
    // ?     // +      xv::TofCamera::Resolution::VGA,          //
    // ?     // +      xv::TofCamera::Framerate::FPS_30         //
    // ?     // +  );
    // ?     device->tofCamera()->setSonyTofSetting(      //
    // ?         xv::TofCamera::SonyTofLibMode::IQMIX_SF, //
    // ?         xv::TofCamera::Resolution::QVGA,         //
    // ?         xv::TofCamera::Framerate::FPS_30         //
    // ?     );
    // ?
    // ?     // ? device->tofCamera()->setSonyTofSetting( //
    // ?     // ?     xv::TofCamera::SonyTofLibMode::
    // ?     // ?         IQMIX_SF, // xv::TofCamera::SonyTofLibMode::IQMIX_DF,
    // ?     // ?                 // //
    // ?     // ?     -1,           //   xv::TofCamera::Resolution::VGA,
    // ?     // ?     xv::TofCamera::Framerate::FPS_15 //
    // ?     // xv::TofCamera::Framerate::FPS_30 ? // // ? );
    // ?
    // ?     tof_id_ = device->tofCamera()->registerCallback(
    // ?         std::bind(&DepthNode::tof_callback, this, std::placeholders::_1));
    // ?     device->tofCamera()->start();
    // ? }

    // ? if (device->colorCamera()) {
    // ?
    // ?     device->colorCamera()->setResolution(
    // ?         xv::ColorCamera::Resolution::RGB_640x480);
    // ?     // device->colorCamera()->setRGBFocalDistance(1);
    // ?
    // ?     color_id = device->colorCamera()->registerCallback(
    // ?         std::bind(&DepthNode::rgb_callback, this, std::placeholders::_1)
    // // ?     );
    // ?
    // ?     device->colorCamera()->start();
    // ? }

    /* if (device->slam()) {
      imu_id = device->orientationStream()->registerCallback(
          [](xv::Orientation const &orien) {});
      device->orientationStream()->unregisterCallback(imu_id);

      slam_id = device->slam()->registerCallback(
          std::bind(&DepthNode::slam_callback, this, std::placeholders::_1)
      );

      device->slam()->start(xv::Slam::Mode::Mixed);
    } */
  }

  public:
  ~DepthNode()
  {
    if (device->tofCamera())
    {
      device->tofCamera()->unregisterCallback(tof_id_);
      device->tofCamera()->stop();
    }
    // ? if (device->colorCamera()) {
    // ?     device->colorCamera()->unregisterCallback(color_id);
    // ?     device->colorCamera()->stop();
    // ? }
    // - if (device->slam()) {
    // -   device->slam()->unregisterCallback(slam_id);
    // -   device->slam()->stop();
    // - }
  }

  public:
  void tof_callback(xv::DepthImage const &depth_image)
  {
    if (depth_image.type == xv::DepthImage::Type::Depth_16 ||
        depth_image.type == xv::DepthImage::Type::Depth_32)
    {
      tof_mutex.lock();
      // - RCLCPP_INFO(this->get_logger(), "Publish message");
      // - RCLCPP_INFO_STREAM(this->get_logger(),
      // -                     "Depth image received: "        //
      // -                         << depth_image.width << "x" //
      // -                         << depth_image.height       //
      // - );
      // - RCLCPP_INFO_STREAM(
      // -     this->get_logger(),
      // -     "Depth type: "                                              //
      // -         << (depth_image.type == xv::DepthImage::Type::Depth_32) //
      // -
      // - );

      // std::shared_ptr<xv::PointCloud> pointcloud =
      //     device->tofCamera()->depthImageToPointCloud(depth_image);
      std::shared_ptr<xv::DepthImage> tof_image =
          std::make_shared<xv::DepthImage>(depth_image);
      cv::Mat img = raw_to_opencv(tof_image);
      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = "map";
      std::string encoding = sensor_msgs::image_encodings::BGR8;
      cv_bridge::CvImage cv_image(header, encoding, img);
      sensor_msgs::msg::Image::SharedPtr msg = cv_image.toImageMsg();

      // depth_publisher_->publish(*msg);

      // * Rescale:
      cv::Mat src = cv_bridge::toCvCopy(*msg, "rgb8")->image;
      cv::Mat dst;
      cv::resize(src, dst, cv::Size(), 0.25, 0.25, cv::INTER_AREA);
      std::shared_ptr<sensor_msgs::msg::Image> out_msg =
          cv_bridge::CvImage(msg->header, "rgb8", dst).toImageMsg();

      depth_publisher_->publish(*out_msg);

      // - old way
      // sensor_msgs::msg::Image tof_msg;
      // tof_msg.header.frame_id = "camera_link";
      // tof_msg.header.stamp = this->now();
      // tof_msg.height = depth_image.height;
      // tof_msg.width = depth_image.width;
      // tof_msg.encoding = (depth_image.type == xv::DepthImage::Type::Depth_16)
      //                        ? "16UC1"
      //                        : "32FC1"; // Float32
      // tof_msg.step = depth_image.width * sizeof(uint16_t);
      // tof_msg.data.resize(tof_msg.height * tof_msg.step);
      // std::memcpy(tof_msg.data.data(), depth_image.data.get(),
      //             tof_msg.height * tof_msg.step);
      // depth_publisher_->publish(tof_msg);

      // ? // Pointcloud
      // ? sensor_msgs::msg::PointCloud pointcloud_msg;
      // ? pointcloud_msg.header.frame_id = "odom";
      // ? pointcloud_msg.header.stamp = this->now();
      // ? pointcloud_msg.points.resize(pointcloud->points.size());
      // ? for (size_t i = 0; i < pointcloud->points.size(); ++i)
      // ? {
      // ?   geometry_msgs::msg::Point32 point;
      // ?   point.x = pointcloud->points[i][0] / 1000.0; // Convert mm to m
      // ?   point.y = pointcloud->points[i][1] / 1000.0; // Convert mm to m
      // ?   point.z = pointcloud->points[i][2] / 1000.0; // Convert mm to m
      // ?   pointcloud_msg.points.push_back(point);
      // ?   // RCLCPP_INFO_STREAM(this->get_logger(),
      // ?   //                    "Point " << i << ": " <<
      // ?   //                    pointcloud->points[i][0]
      // ?   //                             << ", " << pointcloud->points[i][1] <<
      // ?   //                             ",
      // ?   //                             "
      // ?   //                             << pointcloud->points[i][2]);
      // ? }
      // ?
      // ? pointcloud_publisher_->publish(pointcloud_msg);
      tof_mutex.unlock();
    }
  }

  public:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;

  public:
  std::shared_ptr<xv::Device> device;

  public:
  std::shared_ptr<const xv::DepthImage> tof_data_;

  public:
  int tof_id_;

  public:
  int imu_id_;
};

int main(int argc, char **argv)
{
  rclcpp::InitOptions init_options;
  init_options.auto_initialize_logging(false);
  rclcpp::init(argc, argv, init_options);
  
  
  auto node = std::make_shared<DepthNode>();
  // node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
