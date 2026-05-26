cmake_minimum_required(VERSION 3.8)
project(xvisio_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(xvsdk REQUIRED)
find_package(Ceres REQUIRED)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined")
set(CERES_LIB /usr/lib/libceres.so)

set(xvsdk_INCLUDE ${xvsdk_INCLUDE_DIRS}/xvsdk)

include_directories(
  include
  ${xvsdk_INCLUDE}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}

)

add_executable(main_depth_tof src/main_depth_tof.cpp)
ament_target_dependencies(main_depth_tof
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
)

target_link_libraries(main_depth_tof
  ${xvsdk_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIB}
)


add_executable(main_rgb src/main_rgb.cpp)
ament_target_dependencies(main_rgb
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

target_link_libraries(main_rgb
  ${xvsdk_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIB}
)

add_executable(main_stereo src/main_depth_sgbm_only.cpp)
ament_target_dependencies(main_stereo
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)
target_link_libraries(main_stereo
  ${xvsdk_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIB}
)

add_executable(main_all src/main_all.cpp)
ament_target_dependencies(main_all
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
)
target_link_libraries(main_all
  ${xvsdk_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIB}
)

add_executable(main_rgbd src/main_rgbd.cpp)

ament_target_dependencies(main_rgbd
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

target_link_libraries(main_rgbd
  ${xvsdk_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIB}
)

add_executable(main_rgbd_pointcloud src/main_rgbd_pointcloud.cpp)

ament_target_dependencies(main_rgbd_pointcloud
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

target_link_libraries(main_rgbd_pointcloud
  ${xvsdk_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIB}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    main_depth_tof
    main_rgb
    main_stereo
    main_all
    main_rgbd
    main_rgbd_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

# Only install launch folder if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()