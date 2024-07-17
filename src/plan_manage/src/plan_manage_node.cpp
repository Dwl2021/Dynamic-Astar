#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>

#include "path_searching/kinodynamic_astar.h"
#include "plan_env/edt_environment.h"
#include "plan_env/sdf_map.h"
using namespace fast_planner;

void visualizePath(const std::vector<Eigen::Vector3d>& path, ros::Publisher& marker_pub)
{
  visualization_msgs::Marker points;
  points.header.frame_id = "map";  // 根据你的需求调整
  points.header.stamp = ros::Time::now();
  points.ns = "path_visualization";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;

  // 设置标记的比例
  points.scale.x = 0.1;  // 点的直径
  points.scale.y = 0.1;

  // 设置标记的颜色
  points.color.r = 1.0;
  points.color.g = 0.0;
  points.color.b = 0.0;
  points.color.a = 1.0;

  for (const auto& point : path)
  {
    geometry_msgs::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    points.points.push_back(p);
  }

  marker_pub.publish(points);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinodynamic_astar_node");
  ros::NodeHandle nh;

  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  std::vector<Eigen::Vector3d> path;

  Eigen::Vector3d start(0, 0, 0);
  Eigen::Vector3d goal(10, 10, 0);
  Eigen::Vector3d zero(0, 0, 0);

  // Initialize KinodynamicAstar
  std::shared_ptr<MapUtil<3>> map_util;
  map_util = std::make_shared<MapUtil<3>>();
  ROS_INFO("Loading map");
  map_util->setParam(nh);
  ROS_INFO("Map loaded");

  KinodynamicAstar kinodynamic_astar;
  ROS_INFO("Setting parameters");
  kinodynamic_astar.setParam(nh);
  ROS_INFO("Parameters set");
  kinodynamic_astar.setMap(map_util);
  ROS_INFO("Map set");
  ros::Duration(2.0).sleep();
  ros::Rate r(10);
  while (ros::ok())
  {
    if (map_util->has_map_())
    {
      ROS_INFO("READY");
      if (kinodynamic_astar.search(start, zero, zero, goal, zero, true))
      {
        path = kinodynamic_astar.getKinoTraj(0.01);
        ROS_INFO("Path found!");
        visualizePath(path, marker_pub);
      }
      else
      {
        ROS_WARN("Path not found!");
      }
    }
    else
    {
      ROS_INFO("NOT READY");
    }
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
