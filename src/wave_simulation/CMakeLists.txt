cmake_minimum_required(VERSION 2.8.3)
project(wave_simulation)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)


include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(wave_simulation src/wave_simulation.cpp include/wave_simulation/wave_simulation.h)
target_link_libraries(wave_simulation ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})