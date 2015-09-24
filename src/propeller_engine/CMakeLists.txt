cmake_minimum_required(VERSION 2.8.3)
project(propeller_engine)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  geometry_msgs
  std_msgs
  message_generation
  
)
find_package(gazebo REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
    FILES
    engine_msg.msg
)

generate_messages(
    DEPENDENCIES geometry_msgs std_msgs
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

add_library(propeller_engine src/propeller_engine.cpp include/propeller_engine/propeller_engine.h)

add_dependencies(propeller_engine propeller_engine_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(propeller_engine ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

