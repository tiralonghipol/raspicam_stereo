cmake_minimum_required(VERSION 2.8.3)
project(raspicam_stereo)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
  genmsg
  tf
  geometry_msgs
  message_generation
  dynamic_reconfigure
  message_filters
)

find_package(OpenCV 3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime sensor_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
libarducam_mipicamera.so
)


