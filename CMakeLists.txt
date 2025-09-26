cmake_minimum_required(VERSION 3.5)
project(odin_ros_driver)

# =========================
# 0) 平台与 ROS 版本检测
# =========================

# 优先使用外部指示
if(DEFINED BUILD_SYSTEM)
  set(ROS_VERSION ${BUILD_SYSTEM})
  message(STATUS "ROS_VERSION: ${ROS_VERSION}")
elseif(DEFINED ENV{ROS_DISTRO})
  if("$ENV{ROS_DISTRO}" MATCHES "foxy|galactic|humble|iron|rolling")
    set(ROS_VERSION "ROS2")
  else()
    set(ROS_VERSION "ROS1")
  endif()
elseif(DEFINED ENV{ROS_VERSION})
  if("$ENV{ROS_VERSION}" STREQUAL "2")
    set(ROS_VERSION "ROS2")
  else()
    set(ROS_VERSION "ROS1")
  endif()
else()
  # 无环境变量时给出一个合理默认（多数情况是 ROS2）
  set(ROS_VERSION "ROS2")
  message(WARNING "Unable to determine ROS version, defaulting to ROS2")
endif()

# 宏定义
if(ROS_VERSION STREQUAL "ROS2")
  add_definitions(-DROS2)
  message(STATUS "Defining ROS2")
else()
  add_definitions(-DROS1)
  message(STATUS "Defining ROS1")
endif()
message(STATUS "Build system: ${ROS_VERSION}")

# 架构检测（用于选择预编译库）
execute_process(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(ARCH STREQUAL "x86_64")
  set(TARGET_PLATFORM "x86")
  message(STATUS "Detected x86_64 architecture")
elseif(ARCH MATCHES "arm|aarch64")
  set(TARGET_PLATFORM "arm")
  message(STATUS "Detected ARM architecture: ${ARCH}")
else()
  set(TARGET_PLATFORM "unknown")
  message(WARNING "Unsupported architecture: ${ARCH}. Using default settings")
endif()

# 预编译私有库（lydHostApi）定位
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib")
message(STATUS "Library directory: ${LIB_DIR}")
if(TARGET_PLATFORM STREQUAL "arm")
  set(LYD_HOST_API_LIB_NAME "lydHostApi_arm")
else()
  set(LYD_HOST_API_LIB_NAME "lydHostApi_amd")
endif()

find_library(LYD_HOST_API_LIB
  NAMES
    ${LYD_HOST_API_LIB_NAME}
    lib${LYD_HOST_API_LIB_NAME}.a
    lib${LYD_HOST_API_LIB_NAME}.so
  PATHS ${LIB_DIR}
  NO_DEFAULT_PATH
)
if(LYD_HOST_API_LIB)
  message(STATUS "Found lydHostApi library: ${LYD_HOST_API_LIB}")
else()
  file(GLOB LIB_FILES "${LIB_DIR}/lib${LYD_HOST_API_LIB_NAME}.*")
  if(LIB_FILES)
    list(GET LIB_FILES 0 LYD_HOST_API_LIB)
    message(STATUS "Found library files: ${LIB_FILES}")
  else()
    message(FATAL_ERROR "Could not find precompiled lydHostApi library in ${LIB_DIR}")
  endif()
endif()

# =========================
# 1) 通用编译与依赖
# =========================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# PCL 所需模块（与你源码中使用的模块对应）
find_package(PCL REQUIRED COMPONENTS common io filters registration)

# yaml-cpp（现代 CMake 一般提供目标 yaml-cpp）
find_package(yaml-cpp REQUIRED)

# OpenSSL（如未显式使用，可保留）
find_package(OpenSSL REQUIRED)

# libusb
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# 头文件路径（公共）
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 公共库清单（供多个目标链接）
set(COMMON_LIBS
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp
  ${OPENSSL_LIBRARIES}
  ${LIBUSB_LIBRARIES}
  pthread
  rt
  ${CMAKE_DL_LIBS}
  ${LYD_HOST_API_LIB}
)

# =========================
# 2) ROS1 配置
# =========================
if(ROS_VERSION STREQUAL "ROS1")
  message(STATUS "Configuring for ROS1 build")

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    cv_bridge
    image_transport
  )
  include_directories(${catkin_INCLUDE_DIRS})

  catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs cv_bridge image_transport
    INCLUDE_DIRS include
  )

  # 输出目录
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)

  # 可执行文件（示例主程序）
  add_executable(host_sdk_sample
    src/host_sdk_sample.cpp
    src/yaml_parser.cpp
    src/rawCloudRender.cpp
  )
  target_link_libraries(host_sdk_sample
    ${catkin_LIBRARIES}
    ${COMMON_LIBS}
    usb-1.0
  )

  # 库与节点
  add_library(pointcloud_depth_converter src/pointcloud_depth_converter.cpp)
  target_link_libraries(pointcloud_depth_converter
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )

  add_library(depth_image_ros_node src/depth_image_ros_node.cpp)
  target_link_libraries(depth_image_ros_node
    pointcloud_depth_converter
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )

  add_executable(pcd2depth_node src/pcd2depth_ros.cpp)
  target_link_libraries(pcd2depth_node
    depth_image_ros_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )

  # 安装
  install(TARGETS host_sdk_sample
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
  )

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
    install(DIRECTORY config/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )
  endif()

# =========================
# 3) ROS2 配置
# =========================
elseif(ROS_VERSION STREQUAL "ROS2")
  message(STATUS "Configuring for ROS2 build")

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(std_srvs REQUIRED)

  # ---- host_sdk_sample 可执行文件 ----
  add_executable(host_sdk_sample
    src/host_sdk_sample.cpp
    src/yaml_parser.cpp
    src/rawCloudRender.cpp
  )
  target_link_libraries(host_sdk_sample
    ${COMMON_LIBS}
    usb-1.0
  )
  ament_target_dependencies(host_sdk_sample
    rclcpp std_msgs sensor_msgs nav_msgs
    cv_bridge image_transport
    pcl_conversions message_filters
  )

  # ---- 点云转深度：库（ROS2） ----
  add_library(pointcloud_depth_converter_ros2 SHARED src/pointcloud_depth_converter.cpp)
  set_target_properties(pointcloud_depth_converter_ros2 PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(pointcloud_depth_converter_ros2
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )

  add_library(depth_image_ros2_node_lib SHARED src/depth_image_ros2_node.cpp)
  set_target_properties(depth_image_ros2_node_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(depth_image_ros2_node_lib
    pointcloud_depth_converter_ros2
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )
  ament_target_dependencies(depth_image_ros2_node_lib
    rclcpp sensor_msgs std_msgs
    cv_bridge image_transport
    pcl_conversions message_filters
  )

  add_executable(pcd2depth_ros2_node src/pcd2depth_ros2.cpp)
  target_link_libraries(pcd2depth_ros2_node
    depth_image_ros2_node_lib
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp
  )
  ament_target_dependencies(pcd2depth_ros2_node
    rclcpp sensor_msgs std_msgs
    cv_bridge image_transport
    pcl_conversions message_filters
  )

  # ---- 独立 mapping_node ----
  add_executable(mapping_node src/mapping_node.cpp)
  target_link_libraries(mapping_node
    ${PCL_LIBRARIES}
    yaml-cpp
  )
  ament_target_dependencies(mapping_node
    rclcpp sensor_msgs nav_msgs geometry_msgs
    pcl_conversions std_srvs
  )

  # 安装（注意：ament_package() 之前声明所有 install）
  install(TARGETS
    host_sdk_sample
    pcd2depth_ros2_node
    mapping_node
    pointcloud_depth_converter_ros2
    depth_image_ros2_node_lib
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY include/
    DESTINATION include
  )

  # 安装配置/启动文件（若存在）
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
    install(DIRECTORY config/
      DESTINATION share/${PROJECT_NAME}/config
    )
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch_ROS2")
    install(DIRECTORY launch_ROS2/
      DESTINATION share/${PROJECT_NAME}/launch
    )
    message(STATUS "Installing launch_ROS2 directory to share/${PROJECT_NAME}/launch")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
    install(DIRECTORY launch/
      DESTINATION share/${PROJECT_NAME}/launch
    )
  endif()

  ament_export_targets(export_${PROJECT_NAME})
  ament_export_dependencies(
    rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs
    cv_bridge image_transport
    pcl_conversions message_filters
    std_srvs
  )

  # 导出头文件目录（便于被其他包 include）
  ament_export_include_directories(include)

  ament_package()

else()
  message(FATAL_ERROR "Invalid ROS_VERSION: ${ROS_VERSION}")
endif()

# =========================
# 4) ARM 平台 RPATH / 链接设置
# =========================
# 给所有可能的可执行/库施加 ARM 特定链接选项（如果存在）
if(TARGET_PLATFORM STREQUAL "arm")
  foreach(tgt IN ITEMS host_sdk_sample pcd2depth_ros2_node mapping_node pointcloud_depth_converter_ros2 depth_image_ros2_node_lib pointcloud_depth_converter depth_image_ros_node pcd2depth_node)
    if(TARGET ${tgt})
      set_target_properties(${tgt} PROPERTIES
        LINK_FLAGS "-Wl,--no-as-needed -Wl,--rpath=${LIB_DIR}"
      )
    endif()
  endforeach()
  message(STATUS "Adding ARM-specific link options and RPATH")
endif()

# =========================
# 5) 构建信息
# =========================
message(STATUS "=======================================")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "ROS_VERSION: ${ROS_VERSION}")
message(STATUS "Target platform: ${TARGET_PLATFORM}")
message(STATUS "lydHostApi library: ${LYD_HOST_API_LIB}")
message(STATUS "libusb library: ${LIBUSB_LIBRARIES}")
message(STATUS "=======================================")
