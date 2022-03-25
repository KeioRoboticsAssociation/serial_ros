cmake_minimum_required(VERSION 2.8.3)
project(serial_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  rogi_link_msgs
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(
  Boost REQUIRED COMPONENTS 
  system
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES serial_ros
  CATKIN_DEPENDS rogi_link_msgs roscpp rospy std_msgs 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rogi_link_node src/rogi_link.cpp)
add_dependencies(rogi_link_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rogi_link_node ${catkin_LIBRARIES})


# add_executable(serial_ros src/serial_ros.cpp)
# add_dependencies(serial_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(serial_ros ${catkin_LIBRARIES})

# add_executable(serial_test_sync_node src/serial_test_sync.cpp)
# add_dependencies(serial_test_sync_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(serial_test_sync_node ${catkin_LIBRARIES})

