cmake_minimum_required(VERSION 2.8.3)
project(seven_engines_force)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  urdf
  std_msgs
  message_generation  
)

find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  engine_forces.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS geometry_msgs std_msgs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
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

add_library(seven_engines_force src/seven_engines_force.cpp include/seven_engines_force/seven_engines_force.h)

add_dependencies(seven_engines_force seven_engines_force_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(seven_engines_force ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
