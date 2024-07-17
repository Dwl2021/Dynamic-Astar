cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)
set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  path_searching
  cv_bridge
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(plan_manage_node src/plan_manage_node.cpp)
target_link_libraries(plan_manage_node ${catkin_LIBRARIES})
add_dependencies(plan_manage_node ${catkin_EXPORTED_TARGETS})

