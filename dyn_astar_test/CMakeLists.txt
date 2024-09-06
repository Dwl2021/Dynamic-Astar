cmake_minimum_required(VERSION 3.0.2)
project(dyn_astar_test)
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
  dyn_astar
  cv_bridge
  pcl_ros
  pcl_conversions
  sensor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(dyn_astar_test_node src/plan_manage_node.cpp)
target_link_libraries(dyn_astar_test_node ${catkin_LIBRARIES})
add_dependencies(dyn_astar_test_node ${catkin_EXPORTED_TARGETS})


add_executable(visual_pcl_node src/visual_pcl.cpp)
target_link_libraries(visual_pcl_node ${catkin_LIBRARIES})
add_dependencies(visual_pcl_node ${catkin_EXPORTED_TARGETS})
