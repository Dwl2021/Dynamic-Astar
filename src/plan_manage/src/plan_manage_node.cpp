#include <path_searching/kinodynamic_astar.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>

void visualizePath(const std::vector<Eigen::Vector3d>& path, ros::Publisher& marker_pub);
void visualizeEndpoints(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                        ros::Publisher& start_pub, ros::Publisher& goal_pub);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinodynamic_astar_node");
  ros::NodeHandle nh, nh_priv("~");
  ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>("path", 1);
  ros::Publisher start_pub = nh.advertise<visualization_msgs::Marker>("start_point", 1);
  ros::Publisher goal_pub = nh.advertise<visualization_msgs::Marker>("end_point", 1);
  std::vector<Eigen::Vector3d> path;
  bool plan_once = false;
  int success = 0;

  /*
    path searching example
  */
  double start_x, start_y, start_z;
  double start_vel_x, start_vel_y, start_vel_z;
  double start_acc_x, start_acc_y, start_acc_z;
  double goal_x, goal_y, goal_z;
  double goal_vel_x, goal_vel_y, goal_vel_z;
  nh_priv.param("start_x", start_x, 0.0);
  nh_priv.param("start_y", start_y, 0.0);
  nh_priv.param("start_z", start_z, 1.0);
  nh_priv.param("start_vel_x", start_vel_x, 0.0);
  nh_priv.param("start_vel_y", start_vel_y, 1.0);
  nh_priv.param("start_vel_z", start_vel_z, 0.0);
  nh_priv.param("start_acc_x", start_acc_x, 0.0);
  nh_priv.param("start_acc_y", start_acc_y, 0.0);
  nh_priv.param("start_acc_z", start_acc_z, 0.0);
  nh_priv.param("goal_x", goal_x, 30.0);
  nh_priv.param("goal_y", goal_y, -20.0);
  nh_priv.param("goal_z", goal_z, 1.0);
  nh_priv.param("goal_vel_x", goal_vel_x, -3.0);
  nh_priv.param("goal_vel_y", goal_vel_y, 0.0);
  nh_priv.param("goal_vel_z", goal_vel_z, 0.0);

  Eigen::Vector3d start(start_x, start_y, start_z);
  Eigen::Vector3d start_vel(start_vel_x, start_vel_y, start_vel_z);
  Eigen::Vector3d start_acc(start_acc_x, start_acc_y, start_acc_z);
  Eigen::Vector3d goal(goal_x, goal_y, goal_z);
  Eigen::Vector3d goal_vel(goal_vel_x, goal_vel_y, goal_vel_z);

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
  while (ros::ok())
  {
    visualizeEndpoints(start, goal, start_pub, goal_pub);
    if (map_util->has_map_()) /* if MAP is not ready */
    {
      if (!plan_once) /* plan for only once */
      {
        ROS_INFO("READY TO PLAN");
        plan_once = true;

        /* main plan function */
        success =
            kinodynamic_astar.search(start, start_vel, start_acc, goal, goal_vel, false);
        if (success == 2)
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
        if (success == 2)
        {
          ROS_INFO("PATH PUBLISH!");
          visualizePath(path, path_pub);
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

void visualizePath(const std::vector<Eigen::Vector3d>& path, ros::Publisher& path_pub)
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

  path_pub.publish(points);
}

void visualizeEndpoints(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                        ros::Publisher& start_pub, ros::Publisher& goal_pub)
{
  visualization_msgs::Marker start_marker, goal_marker;

  start_marker.header.frame_id = goal_marker.header.frame_id = "world";
  start_marker.header.stamp = goal_marker.header.stamp = ros::Time::now();
  start_marker.ns = goal_marker.ns = "endpoints_visualization";
  start_marker.action = goal_marker.action = visualization_msgs::Marker::ADD;
  start_marker.pose.orientation.w = goal_marker.pose.orientation.w = 1.0;

  start_marker.id = 1;
  start_marker.type = visualization_msgs::Marker::SPHERE;
  start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.4;
  start_marker.color.r = 0.0;
  start_marker.color.g = 1.0;
  start_marker.color.b = 0.0;
  start_marker.color.a = 1.0;
  start_marker.pose.position.x = start.x();
  start_marker.pose.position.y = start.y();
  start_marker.pose.position.z = start.z();

  goal_marker.id = 2;
  goal_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.4;
  goal_marker.color.r = 0.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 1.0;
  goal_marker.color.a = 1.0;
  goal_marker.pose.position.x = goal.x();
  goal_marker.pose.position.y = goal.y();
  goal_marker.pose.position.z = goal.z();

  start_pub.publish(start_marker);
  goal_pub.publish(goal_marker);
}
