cmake_minimum_required(VERSION 2.8.3)
project(relative_force_engine)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  gazebo_ros
  urdf
  std_msgs
  message_generation  
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(gazebo REQUIRED)

add_message_files(
    FILES
    object_params.msg
)

generate_messages(
    DEPENDENCIES geometry_msgs std_msgs
    )

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs message_runtime)

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

add_library(relative_force_engine src/relative_force_engine.cpp include/relative_force_engine/relative_force_engine.h)

add_dependencies(relative_force_engine relative_force_engine_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(relative_force_engine ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
