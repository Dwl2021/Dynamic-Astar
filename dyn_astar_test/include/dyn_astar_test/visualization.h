#include <data_type/data_type.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <vector>

class Visualizer
{
 public:
  Visualizer(ros::NodeHandle& nh)
  {
    path_pub_ = nh.advertise<visualization_msgs::Marker>("path", 1);
    start_pub_ = nh.advertise<visualization_msgs::Marker>("start_point", 1);
    goal_pub_ = nh.advertise<visualization_msgs::Marker>("end_point", 1);
  }

  void visualizePath(const vec_Vec3f& path)
  {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "path_visualization";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.05;  // Line width
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    for (const auto& point : path)
    {
      geometry_msgs::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      line_strip.points.push_back(p);
    }

    path_pub_.publish(line_strip);
  }

  void visualizeEndpoints(const Vec3f& start, const Vec3f& goal)
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

    start_pub_.publish(start_marker);
    goal_pub_.publish(goal_marker);
  }

 private:
  ros::Publisher path_pub_;
  ros::Publisher start_pub_;
  ros::Publisher goal_pub_;
};
