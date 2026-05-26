# XVisio ROS 2 Jazzy RGB-D and Colored Point Cloud

This repository provides a working ROS 2 Jazzy integration for the XVisio vSLAM sensor on Ubuntu 24.04. It supports RGB streaming, ToF depth streaming, RGB-D aligned images, and colored point cloud visualization in RViz2.

The setup was tested with an XVisio vSLAM device and XVisio SDK 3.2.0 on Ubuntu 24.04.

---

## Demo

### RGB-D Colored Point Cloud in RViz2

![RGB-D colored point cloud demo](assets/rgbd_colored_pointcloud_demo.gif)

### ToF Depth Image Stream

![ToF depth image demo](assets/tof_depth_demo.gif)

---

## Features

| Feature | Status |
|---|---|
| RGB image stream | Working |
| ToF depth stream | Working |
| RGB-D aligned stream | Working |
| Colored RGB-D point cloud | Working |
| RViz2 visualization | Working |
| `rqt_image_view` support | Working |
| CycloneDDS support for ROS 2 Jazzy | Working |

---

## Tested System

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| XVisio SDK | 3.2.0 |
| Sensor | XVisio vSLAM / DS80-style RGB-D device |
| Platform | Intel x86_64 |

---

## Published Topics

Depending on the node you run, the package can publish:

```text
/image
/image/compressed
/rgb_image
/depth_image
/rgbd_rgb
/rgbd_depth
/rgbd_points
```

The RGB-D point cloud node publishes:

```text
/rgbd_rgb
/rgbd_depth
/rgbd_points
```

The point cloud frame is:

```text
xvisio_rgbd_frame
```

---

## Repository Structure

```text
xvisio_ros2/
├── src/
│   ├── main_rgb.cpp
│   ├── main_depth_tof.cpp
│   ├── main_rgbd.cpp
│   └── main_rgbd_pointcloud.cpp
├── assets/
│   ├── rgbd_colored_pointcloud_demo.gif
│   └── tof_depth_demo.gif
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 1. Install Dependencies

Install ROS 2 Jazzy first, then install the required packages:

```bash
sudo apt update

sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-rqt-image-view \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-rcl-logging-noop \
  libopencv-dev \
  libceres-dev
```

Make sure the XVisio SDK is already installed on your system. The package expects the XVisio libraries and headers to be available from the SDK installation.

---

## 2. Clone the Repository

```bash
cd ~

git clone <YOUR_REPOSITORY_URL> xvisio_ros2

cd xvisio_ros2
```

Replace `<YOUR_REPOSITORY_URL>` with your GitHub repository URL.

---

## 3. Important ROS 2 Jazzy Notes

### Disable Automatic ROS Logging Initialization

The XVisio SDK may conflict with ROS 2 Jazzy logging initialization through `spdlog`. To avoid startup crashes, the executables should initialize ROS 2 with automatic logging disabled:

```cpp
rclcpp::InitOptions init_options;
init_options.auto_initialize_logging(false);

rclcpp::init(argc, argv, init_options);
```

### Use CycloneDDS

FastDDS/FastCDR may cause symbol conflicts with the XVisio SDK on Ubuntu 24.04. Use CycloneDDS:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

To make this permanent:

```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

---

## 4. Build the Workspace

```bash
cd ~/xvisio_ros2

rm -rf build install log

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/setup.bash
```

If the build completes successfully, the ROS 2 executables will be available through `ros2 run`.

---

## 5. Run the RGB Camera Node

```bash
cd ~/xvisio_ros2

env -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u LD_LIBRARY_PATH \
    -u PYTHONPATH \
    bash -c '
source /opt/ros/jazzy/setup.bash
source ~/xvisio_ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run xvisio_ros2 main_rgb
'
```

Expected topics:

```text
/image
/image/compressed
/rgb_image
```

---

## 6. Run the ToF Depth Node

```bash
cd ~/xvisio_ros2

env -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u LD_LIBRARY_PATH \
    -u PYTHONPATH \
    bash -c '
source /opt/ros/jazzy/setup.bash
source ~/xvisio_ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run xvisio_ros2 main_depth_tof
'
```

Expected topic:

```text
/depth_image
```

---

## 7. Run the RGB-D Aligned Stream Node

This node publishes RGB and depth images where each RGB pixel corresponds to the same depth pixel.

```bash
cd ~/xvisio_ros2

env -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u LD_LIBRARY_PATH \
    -u PYTHONPATH \
    bash -c '
source /opt/ros/jazzy/setup.bash
source ~/xvisio_ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run xvisio_ros2 main_rgbd
'
```

Expected topics:

```text
/rgbd_rgb
/rgbd_depth
```

---

## 8. Run the Colored RGB-D Point Cloud Node

```bash
cd ~/xvisio_ros2

env -u AMENT_PREFIX_PATH \
    -u CMAKE_PREFIX_PATH \
    -u COLCON_PREFIX_PATH \
    -u LD_LIBRARY_PATH \
    -u PYTHONPATH \
    bash -c '
source /opt/ros/jazzy/setup.bash
source ~/xvisio_ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run xvisio_ros2 main_rgbd_pointcloud
'
```

Expected topics:

```text
/rgbd_rgb
/rgbd_depth
/rgbd_points
```

---

## 9. Visualize RGB and Depth Images

Run:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run rqt_image_view rqt_image_view
```

Useful image topics:

```text
/rgb_image
/depth_image
/rgbd_rgb
/rgbd_depth
```

---

## 10. Visualize Colored Point Cloud in RViz2

Start RViz2:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

rviz2
```

Set the fixed frame:

```text
Fixed Frame: xvisio_rgbd_frame
```

Add the colored point cloud:

```text
Add → PointCloud2
Topic: /rgbd_points
Color Transformer: RGB8
Style: Points
```

Optional image displays:

```text
Add → Image
Topic: /rgbd_rgb

Add → Image
Topic: /rgbd_depth
```

---

## 11. Check ROS Topics

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 topic list
```

Example output:

```text
/rgbd_rgb
/rgbd_depth
/rgbd_points
/parameter_events
/rosout
```

Check topic frequency:

```bash
ros2 topic hz /rgbd_rgb
ros2 topic hz /rgbd_depth
ros2 topic hz /rgbd_points
```

---

## 12. Record a ROS Bag

```bash
ros2 bag record \
  /rgb_image \
  /depth_image \
  /rgbd_rgb \
  /rgbd_depth \
  /rgbd_points
```

---

## 13. Camera Calibration Notes

The XVisio SDK provides RGB and ToF calibration through:

```cpp
device_->colorCamera()->calibration()
device_->tofCamera()->calibration()
```

Example calibration values observed from the device:

```text
RGB 640x480:
fx = 494.306
fy = 494.306
cx = 317.954
cy = 228.748

ToF 640x480:
fx = 516.418
fy = 516.418
cx = 324.052
cy = 238.431
```

The point cloud node may use approximate intrinsics such as:

```cpp
fx = 525.0f;
fy = 525.0f;
cx = width / 2.0f;
cy = height / 2.0f;
```

For more accurate metric reconstruction, replace these values with the calibration values from your own XVisio device.

---

## 14. Common Issues

### Segmentation Fault at Startup

Cause: ROS 2 Jazzy logging conflict with the XVisio SDK.

Fix: Disable automatic ROS logging initialization:

```cpp
rclcpp::InitOptions init_options;
init_options.auto_initialize_logging(false);

rclcpp::init(argc, argv, init_options);
```

---

### FastDDS or FastCDR Symbol Lookup Error

Cause: DDS middleware conflict on Ubuntu 24.04.

Fix: Use CycloneDDS:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

### `/rgbd_points` Is Not Visible in RViz2

Check that RViz2 uses:

```text
Fixed Frame: xvisio_rgbd_frame
```

Then confirm the topic exists:

```bash
ros2 topic list | grep rgbd
```

---

### `rqt_image_view` Not Found

Install it:

```bash
sudo apt install ros-jazzy-rqt-image-view
```

Run it:

```bash
ros2 run rqt_image_view rqt_image_view
```

---

### Broken apt Repository

If `sudo apt update` fails because of a broken PPA, remove the problematic repository. Example:

```bash
sudo add-apt-repository --remove ppa:lyx-devel/release
sudo apt update
```

---

## 15. Useful Commands

List USB devices:

```bash
lsusb
```

Check for the XVisio device:

```bash
lsusb | grep -i xvisio
```

List ROS topics:

```bash
ros2 topic list
```

Echo depth topic:

```bash
ros2 topic echo /rgbd_depth
```

Check point cloud frequency:

```bash
ros2 topic hz /rgbd_points
```

---

## 16. Notes

The XVisio SDK dynamically links several internal libraries, including:

```text
libxvsdk
libxvslam
libhandskeleton_wrapper
libceres
```

Even if hand tracking is not used, some of these dependencies may still be loaded internally by the SDK.

---

## Acknowledgement

The colored point cloud component was developed by **Akhlak Uz Zaman**, PhD Candidate, Department of Computer Science, The University of Alabama. This implementation is based on XVisio sample code for ToF depth and RGB image acquisition, with both streams registered together for RGB-D point cloud generation.

**Principal Investigator: Dr. Hongsheng He**, Associate Professor, Department of Computer Science, The University of Alabama.

---

## License

MIT License
