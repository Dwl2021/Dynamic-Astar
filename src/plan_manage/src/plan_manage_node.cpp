#include <path_searching/kinodynamic_astar.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>

using namespace fast_planner;

void visualizePath(const std::vector<Eigen::Vector3d>& path, ros::Publisher& marker_pub);
int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinodynamic_astar_node");
  ros::NodeHandle nh;
  ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>("path", 1);
  std::vector<Eigen::Vector3d> path;

  /*
    path searching example
  */
  Eigen::Vector3d start(0, 0, 1);
  Eigen::Vector3d start_vel(0, 0, 0);
  Eigen::Vector3d start_acc(0, 0, 0);
  Eigen::Vector3d goal(30, 0, 1);
  Eigen::Vector3d goal_vel(-2, 0, 0);

  /* init map util */
  std::shared_ptr<MapUtil<3>> map_util;
  map_util = std::make_shared<MapUtil<3>>();
  map_util->setParam(nh);

  /* init dynamic astar */
  KinodynamicAstar kinodynamic_astar;
  kinodynamic_astar.setParam(nh);
  kinodynamic_astar.setMap(map_util);
  kinodynamic_astar.init();

  ros::Duration(1.0).sleep();
  ros::Rate rate(10);
  bool plan_once = false;
  bool success = false;
  while (ros::ok())
  {
    if (map_util->has_map_()) /* if MAP is not ready */
    {
      if (!plan_once) /* plan for only once */
      {
        ROS_INFO("READY TO PLAN");
        plan_once = true;

        /* main plan function */
        success =
            kinodynamic_astar.search(start, start_vel, start_acc, goal, goal_vel, false);
        if (success)
        {
          path = kinodynamic_astar.getKinoTraj(0.01); /* sample */
          ROS_INFO("\033[1;32mPATH FOUND!\033[0m");
        }
        else
        {
          ROS_INFO("\033[1;31mPATH NOT FOUND\033[0m");
        }
      }
      else /* plan for onces */
      {
        if (success)
        {
          ROS_INFO("PATH PUBLISH!");
          visualizePath(path, marker_pub);
        }
      }
    }
    else
    {
      ROS_INFO("MAP NOT READY!");
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}

void visualizePath(const std::vector<Eigen::Vector3d>& path, ros::Publisher& marker_pub)
{
  visualization_msgs::Marker points;
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "path_visualization";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.05;
  points.scale.y = 0.05;
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

  mpath_pub.publish(points);
}