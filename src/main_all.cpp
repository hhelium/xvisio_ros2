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
// #include <nav_msgs/msg/odometry.hpp>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Quaternion.h>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>

using namespace std::chrono_literals;

std::mutex rgb_mutex;
std::mutex stereo_mutex;
std::mutex tof_mutex;

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

class AllNode : public rclcpp::Node
{
  public:
  AllNode() : rclcpp::Node("AllNode")
  { //
    // RCLCPP_INFO_STREAM(this->get_logger(), "GREATTTTTTTT");

    depth_publisher_ = this->create_publisher<sensor_msgs::msg::Image>( //
        "depth_image",                                                  //
        10                                                              //
    );

    stereo_publisher_ = this->create_publisher<sensor_msgs::msg::Image>( //
        "stereo_img",                                                    //
        10                                                               //
    );

    rgb_publisher_ =                                     //
        this->create_publisher<sensor_msgs::msg::Image>( //
            "rgb_image",                                 //
            10                                           //
        );

    std::map<std::string, std::shared_ptr<xv::Device>> devices =
        xv::getDevices(5.0);

    device = devices.begin()->second;
    // imu_id_ = device->orientationStream()->registerCallback( //
    //     [](xv::Orientation const &) {}                       //
    // );
    // if (device->orientationStream())
    // {
    //   device->orientationStream()->unregisterCallback(imu_id_);
    // }

    // ? device->sgbmCamera()->registerCallback(
    // ?     std::bind(&AllNode::stereo_callback, this, std::placeholders::_1));
    // ? xv::sgbm_config global_config = {
    // ?     0,    //enable_dewarp
    // ?     1.0,  //dewarp_zoom_factor
    // ?     0,    //enable_disparity
    // ?     1,    //enable_depth
    // ?     0,    //enable_point_cloud
    // ?     0.08, //baseline
    // ?     96,   //fov
    // ?     255,  //disparity_confidence_threshold
    // ?     {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, //homography
    // ?     1,                                             //enable_gamma
    // ?     2.2,                                           //gamma_value
    // ?     0,                                             //enable_gaussian
    // ?     0,                                             //mode
    // ?     8000,                                          //max_distance
    // ?     100,                                           //min_distance
    // ? };
    // ? device->sgbmCamera()->start(global_config);

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
          // xv::TofCamera::Framerate::FPS_5     //
          xv::TofCamera::Framerate::FPS_30);
      tof_id_ = device->tofCamera()->registerCallback(
          std::bind(&AllNode::tof_callback, this, std::placeholders::_1));
    }

    if (device->colorCamera())
    {
      color_id_ = device->colorCamera()->registerCallback(
          std::bind(&AllNode::rgb_callback, this, std::placeholders::_1));
      device->colorCamera()->start();
    }
  }

  public:
  ~AllNode() {}

  public:
  std::tuple<int, int, int> color(double distance, double distance_min,
                                  double distance_max, double threshold)
  {
    double d = std::max(distance_min, std::min(distance, distance_max));
    d = (d - distance_min) / (distance_max - distance_min);
    // std::cout<<"color max"<<distance_max<<"color min"<<distance_min<<"color"<<distance<<std::endl;
    if (distance <= threshold || distance > distance_max)
    {
      return std::tuple<int, int, int>(0, 0, 0);
    }
    int b = static_cast<int>(
        255.0 *
        std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * (d - 0.5))), 1.0));
    int g = static_cast<int>(
        255.0 *
        std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * (d - 0.25))), 1.0));
    int r = static_cast<int>(
        255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * d)), 1.0));
    return std::tuple<int, int, int>(r, g, b);
  }

  cv::Mat convdispToMat(std::shared_ptr<const xv::SgbmImage> sbgm_image,
                        bool col_map)
  {
    cv::Mat im_gray;

    im_gray = cv::Mat(cv::Size(sbgm_image->width, sbgm_image->height), CV_8UC1,
                      const_cast<uint8_t *>(sbgm_image->data.get()));

    if (stretch_disparity)
    {
      //for better visualization
      stretchDisparityRange(im_gray, 96);
    }

    if (col_map)
    {
      cv::Mat im_col(cv::Size(sbgm_image->width, sbgm_image->height), CV_8UC3);
      applyColorMap(im_gray, im_col, cv::COLORMAP_JET);
      return im_col;
    }
    else
    {
      return im_gray;
    }
  }
  std::shared_ptr<unsigned char>
  depthImage(uint16_t *data, unsigned int width, unsigned int height,
             double min_distance_m, double max_distance_m, bool colorize)
  {
    std::shared_ptr<unsigned char> out;
    if (colorize)
    {
      out = std::shared_ptr<unsigned char>(
          new unsigned char[width * height * 3],
          std::default_delete<unsigned char[]>());
    }
    else
    {
      out = std::shared_ptr<unsigned char>(
          new unsigned char[width * height],
          std::default_delete<unsigned char[]>());
    }

    for (unsigned int i = 0; i < width * height; i++)
    {
      double distance_mm = data[i];
      if (colorize)
      {
        double distance_m = distance_mm / 1000.;

        auto c =
            color(distance_m, min_distance_m, max_distance_m, min_distance_m);
        out.get()[i * 3 + 0] = static_cast<unsigned char>(std::get<2>(c));
        out.get()[i * 3 + 1] = static_cast<unsigned char>(std::get<1>(c));
        out.get()[i * 3 + 2] = static_cast<unsigned char>(std::get<0>(c));
      }
      else
      {
        double max_distance_mm = max_distance_m * 1000.;
        double min_distance_mm = min_distance_m * 1000.;
        distance_mm = std::min(max_distance_mm, distance_mm);
        distance_mm = std::max(distance_mm, min_distance_mm);

        double norm = (distance_mm - min_distance_mm) /
                      (max_distance_mm - min_distance_mm);
        auto c = 255. * norm;
        out.get()[i] = static_cast<unsigned char>(c);
      }
    }

    return out;
  }

  cv::Mat convDepthToMat(std::shared_ptr<const xv::SgbmImage> sgbm_image,
                         bool _colorize_depth)
  {
    uint16_t *p16 = (uint16_t *)sgbm_image->data.get();

    // cv::Mat mask;
    // cv::Mat im_gray_d = cv::Mat(cv::Size(sgbm_image->width, sgbm_image->height),  CV_16UC1, p16); //18
    // cv::inRange(im_gray_d, cv::Scalar(1), cv::Scalar(65535), mask);
    // p16 = (uint16_t *)im_gray_d.data;

    double focal_length =
        sgbm_image->width / (2.f * tan(
                                       /*global_config.fov*/
                                       69 / 2 / 180.f * M_PI));
    double max_distance_m = (focal_length *
                             /*global_config.baseline*/
                             0.11285 / 1);
    double min_distance_m =
        0; //0 is considered invalid distance (0 disparity == unknown)
    max_distance_m = std::min(max_distance_m, depth_max_distance_m);
    min_distance_m = depth_min_distance_m;
    assert(max_distance_m > min_distance_m);

    static std::shared_ptr<unsigned char> tmp;
    tmp = depthImage(p16, sgbm_image->width, sgbm_image->height, min_distance_m,
                     max_distance_m, !!_colorize_depth);
    if (_colorize_depth)
    {
      cv::Mat im_col(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC3,
                     tmp.get());
      // cv::Mat roi = cv::Mat::zeros(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC3);
      // im_col.copyTo(roi,mask);
      return im_col;
    }
    else
    {
      cv::Mat im_col(cv::Size(sgbm_image->width, sgbm_image->height), CV_8UC1,
                     tmp.get());
      return im_col;
    }
  }

  void stretchDisparityRange(cv::Mat &frame, int disp)
  {
    const int scaleFactorLowerD = 2;
    const int scaleFactorUpperD = 1;

    int halfIntervalD = disp / 2;

    for (int row = 0; row < frame.rows; row++)
      for (size_t col = 0; col < frame.step; col++)
      {
        // if disparity value = [0, D/2), multiply it with 4
        if (frame.data[col + row * frame.step] < halfIntervalD)
          frame.data[col + row * frame.step] *= scaleFactorLowerD;
        else
          // if disparity value = [D/2, D), multiply it with 2
          //frame.data[col + row * frame.step] *= scaleFactorUpperD;
          frame.data[col + row * frame.step] =
              frame.data[col + row * frame.step] * scaleFactorUpperD +
              scaleFactorLowerD * halfIntervalD;
      }
  }

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
  cv::Mat raw_to_opencv(std::shared_ptr<const xv::SgbmImage> sbgm_image)
  {
    if (xv::SgbmImage::Type::Depth == sbgm_image->type)
    {
      cv::Mat mat = convDepthToMat(sbgm_image, colorize_depth);
      // cvtColor(mat, mat, cv::COLOR_GRAY2RGB);
      // cv::Mat mask;
      // cv::medianBlur(mat, mat, 5);
      // cv::inRange(mat, cv::Scalar(1), cv::Scalar(255), mask);
      return mat;
    }
    else if (xv::SgbmImage::Type::Disparity == sbgm_image->type)
    {
      return convdispToMat(sbgm_image, colorize_disparity);
    }
    return cv::Mat();
  }

  public:
  void rgb_callback(xv::ColorImage const &color_img)
  {
    rgb_mutex.lock();
    std::shared_ptr<xv::RgbImage> rgb_image =
        std::make_shared<xv::RgbImage>(color_img.toRgb());
    sensor_msgs::msg::Image rgb_msg;
    rgb_msg.header.frame_id = "base_link";
    rgb_msg.header.stamp = this->now();
    rgb_msg.height = rgb_image->height;
    rgb_msg.width = rgb_image->width;
    rgb_msg.encoding = "rgb8";
    rgb_msg.step = rgb_msg.width * 3; // 3 bytes per pixel (BGR)
    size_t image_size = rgb_msg.height * rgb_msg.step;
    rgb_msg.data.resize(image_size);
    std::memcpy(rgb_msg.data.data(), rgb_image->data.get(), image_size);

    // * Rescale:
    cv::Mat src = cv_bridge::toCvCopy(rgb_msg, "rgb8")->image;
    cv::Mat dst;
    cv::resize(src, dst, cv::Size(), 0.25, 0.25, cv::INTER_AREA);
    std::shared_ptr<sensor_msgs::msg::Image> out_msg =
        cv_bridge::CvImage(rgb_msg.header, "rgb8", dst).toImageMsg();

    // pub_.publish(rgb_msg);
    // - RCLCPP_INFO_STREAM(this->get_logger(), "size HxW: " << out_msg->height
    // -                                                     << "X"
    // -                                                     << out_msg->width);
    // pub_.publish(std::make_shared<sensor_msgs::msg::Image>(rgb_msg));
    // * for compressed
    rgb_publisher_->publish(*out_msg);

    // * for non-compressed
    // rgb_publisher_->publish(*out_msg);
    rgb_mutex.unlock();
  }

  public:
  void stereo_callback(xv::SgbmImage const &stereo_image)
  {
    stereo_mutex.lock();
    // RCLCPP_INFO_STREAM(this->get_logger(), "GREAT");
    if (stereo_image.type == xv::SgbmImage::Type::Depth)
    {
      stereo_img = std::make_shared<xv::SgbmImage>(stereo_image);

      int hole = 1;
      // uint16_t d1 = 8 * 1000;
      // uint16_t d2 = 0.1 * 1000;
      xv::SgbmImage img_ = fillHoles(
          *stereo_img.get(),
          [](uint16_t d) { return d < 8 * 1000 || d > 0.1 * 1000; }, hole);

      cv::Mat img = raw_to_opencv(std::shared_ptr<const xv::SgbmImage>(
          std::make_shared<xv::SgbmImage>(img_)));
      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = "map";
      std::string encoding = sensor_msgs::image_encodings::BGR8;
      cv_bridge::CvImage cv_image(header, encoding, img);
      sensor_msgs::msg::Image::SharedPtr msg = cv_image.toImageMsg();
      stereo_publisher_->publish(*msg);

      /* 
        RCLCPP_INFO_STREAM(this->get_logger(), "Depth");
        stereo_img = std::make_shared<xv::SgbmImage>(stereo_image);

        cv::Mat img = raw_to_opencv(stereo_img);
        char text[256];
        uint16_t *p16 = (uint16_t *)stereo_img->data.get();

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "map";

        std::string encoding = sensor_msgs::image_encodings::BGR8;

        cv_bridge::CvImage cv_image(header, encoding, img);

        sensor_msgs::msg::Image::SharedPtr msg = cv_image.toImageMsg();
        stereo_publisher_->publish(*msg); 
      */
    }
    else
    {
      // RCLCPP_INFO_STREAM(this->get_logger(), "Not depth");
    }
    stereo_mutex.unlock();
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
  xv::SgbmImage fillHoles(const xv::SgbmImage &src,
                          std::function<bool(uint16_t)> isHole, int holeRadius)
  {
    // Only operate on depth-type SGBM images; otherwise return copy
    // (Adjust the Type enum check to match your SgbmImage definition.)
    if (src.data == nullptr)
      return src;
#ifdef SOME_SGBM_DEPTH_CHECK
    if (src.type != xv::SgbmImage::Type::Depth)
      return src;
#endif

    int width = src.width;
    int height = src.height;
    if (width <= 0 || height <= 0)
      return src;

    const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);
    const size_t bytes = n * sizeof(uint16_t);

    // Read source buffer into a working vector of uint16_t
    std::vector<uint16_t> cur(n);
    std::memcpy(cur.data(), src.data.get(), bytes);

    std::vector<uint16_t> next = cur;

    // neighbors offsets (8-neighborhood)
    const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    int iterations = std::max(
        1, holeRadius); // ensure at least one iteration if holeRadius > 0

    for (int iter = 0; iter < iterations; ++iter)
    {
      bool anyFilled = false;

      // For each pixel
      for (int y = 0; y < height; ++y)
      {
        for (int x = 0; x < width; ++x)
        {
          size_t idx = static_cast<size_t>(y) * width + x;
          uint16_t val = cur[idx];

          if (!isHole(val))
          {
            // not a hole; copy as-is
            next[idx] = val;
            continue;
          }

          // collect valid neighbor values
          uint32_t sum = 0;
          int count = 0;
          for (int k = 0; k < 8; ++k)
          {
            int nx = x + dx[k];
            int ny = y + dy[k];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height)
              continue;
            size_t nidx = static_cast<size_t>(ny) * width + nx;
            uint16_t nval = cur[nidx];
            if (!isHole(nval))
            {
              sum += nval;
              ++count;
            }
          }

          if (count > 0)
          {
            // fill with average of neighbors (clamped to uint16_t)
            uint16_t filled =
                static_cast<uint16_t>(std::min<uint32_t>(sum / count, 0xFFFFu));
            next[idx] = filled;
            anyFilled = true;
          }
          else
          {
            // no valid neighbor yet; keep as hole for this iteration
            next[idx] = cur[idx];
          }
        }
      }

      // Swap buffers for next iteration
      cur.swap(next);

      if (!anyFilled)
        break; // nothing changed, stop early
    }

    // Build output SgbmImage (copy other metadata, but replace data)
    xv::SgbmImage out = src; // copy structure members
    // allocate a new contiguous byte buffer and copy cur into it
    std::shared_ptr<uint8_t> outData(new uint8_t[bytes],
                                     std::default_delete<uint8_t[]>());
    std::memcpy(outData.get(), cur.data(), bytes);
    out.data = outData;

    return out;
  }

  public:
  std::shared_ptr<xv::Device> device;

  public:
  std::shared_ptr<xv::SgbmImage> stereo_img_;

  public:
  std::shared_ptr<const xv::DepthImage> tof_data_;

  public:
  int tof_id_;

  public:
  int imu_id_;

  public:
  int color_id_;

  public:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;

  public:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_publisher_;

  public:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;

  public:
  bool colorize_disparity = true;
  bool colorize_depth = true;
  double depth_max_distance_m = 5;
  double depth_min_distance_m = 0.1;
  bool stretch_disparity = true;

  std::shared_ptr<xv::SgbmImage> stereo_img;
};

int main(int argc, char **argv)
{
  rclcpp::InitOptions init_options;
  init_options.auto_initialize_logging(false);
  rclcpp::init(argc, argv, init_options);
  rclcpp::spin(std::make_shared<AllNode>());
  rclcpp::shutdown();
}
