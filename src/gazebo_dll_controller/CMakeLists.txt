cmake_minimum_required(VERSION 2.8.3)
project(gazebo_dll_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  gazebo_ros
  urdf
  message_generation
  #ros_control_node_example
)

find_package(gazebo REQUIRED)

add_message_files(
    FILES
    command.msg
)

generate_messages(
    DEPENDENCIES std_msgs
    )

  
catkin_package(
INCLUDE_DIRS include src
CATKIN_DEPENDS message_runtime)


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

# add_library(gazebo_dll_controller src/gazebo_dll_controller.cpp include/gazebo_dll_controller/gazebo_dll_controller.h)
# target_link_libraries(gazebo_dll_controller ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

# add_library(ros_read_write src/ros_read_write.cpp include/gazebo_dll_controller/ros_read_write.h)
add_library(gazebo_dll_controller src/ros_read_write.cpp include/gazebo_dll_controller/ros_read_write.h)

add_dependencies(gazebo_dll_controller gazebo_dll_controller_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(gazebo_dll_controller ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(propeller_engine propeller_engine_generate_messages_cpp)

