cmake_minimum_required(VERSION 3.8)
project(autotaxi_trt_detector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include/aarch64-linux-gnu
  REQUIRED
)

find_library(TENSORRT_LIBRARY nvinfer
  PATHS /usr/lib/aarch64-linux-gnu
  REQUIRED
)

set(CUDART_LIBRARY
  /usr/local/cuda-10.2/targets/aarch64-linux/lib/libcudart.so.10.2.300
)

add_executable(trt_detector src/trt_detector.cpp)

target_include_directories(trt_detector PRIVATE
  ${TENSORRT_INCLUDE_DIR}
  /usr/local/cuda-10.2/targets/aarch64-linux/include
)

ament_target_dependencies(trt_detector
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(trt_detector
  ${OpenCV_LIBS}
  ${TENSORRT_LIBRARY}
  /usr/lib/aarch64-linux-gnu/tegra/libnvdla_compiler.so
  /usr/lib/aarch64-linux-gnu/tegra/libnvmedia.so
  ${CUDART_LIBRARY}
)

set_target_properties(trt_detector PROPERTIES
  BUILD_RPATH "/usr/local/cuda-10.2/targets/aarch64-linux/lib;/usr/lib/aarch64-linux-gnu/tegra"
INSTALL_RPATH "/usr/local/cuda-10.2/targets/aarch64-linux/lib;/usr/lib/aarch64-linux-gnu/tegra"
)

install(TARGETS trt_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
