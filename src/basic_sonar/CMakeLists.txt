cmake_minimum_required(VERSION 2.8.3)
project(basic_sonar)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  gazebo_ros
)

find_package(gazebo REQUIRED)

add_service_files(
  FILES
  sonar_serv.srv 
  sonar_image_serv.srv
)

generate_messages(
    DEPENDENCIES geometry_msgs
    )

catkin_package(CATKIN_DEPENDS message_runtime)    

find_package(Boost REQUIRED COMPONENTS system)
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

add_library(basic_sonar src/basic_sonar.cpp include/basic_sonar/basic_sonar.h)

add_dependencies(basic_sonar basic_sonar_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(basic_sonar ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
