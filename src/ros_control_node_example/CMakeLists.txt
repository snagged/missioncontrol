cmake_minimum_required(VERSION 2.8.3)
project(ros_control_node_example)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  gazebo_ros
  urdf
  message_generation
  gazebo_dll_controller
  std_msgs
)
find_package(OpenCV REQUIRED)



find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
catkin_package()
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(include ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(/opt/ros/indigo/lib)

add_executable(ros_control_node_example src/demo.cpp include/ros_control_node_example/demo.h)

add_dependencies(ros_control_node_example gazebo_dll_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(ros_control_node_example ${catkin_LIBRARIES})
target_link_libraries(ros_control_node_example opencv_calib3d opencv_contrib opencv_core opencv_features2d opencv_flann)# -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_ts -lopencv_video -lopencv_videostab)
