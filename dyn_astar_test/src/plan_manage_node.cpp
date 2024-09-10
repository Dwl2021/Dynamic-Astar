#include <dyn_astar/kinodynamic_astar.h>
#include <dyn_astar_test/visualization.h>
#include <ros/ros.h>

#include <Eigen/Core>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kastar_node");
  ros::NodeHandle nh, nh_priv("~");

  /* The result path can be obtain in two type */
  vec_Vec3f path;

  Visualizer visualizer(nh);
  bool plan_once = false;
  int success = 0;

  Vec3f start, start_vel, start_acc, goal, goal_vel;
  nh_priv.param("start_x", start(0), 0.0);
  nh_priv.param("start_y", start(1), 0.0);
  nh_priv.param("start_z", start(2), 1.0);
  nh_priv.param("start_vel_x", start_vel(0), 0.0);
  nh_priv.param("start_vel_y", start_vel(1), 1.0);
  nh_priv.param("start_vel_z", start_vel(2), 0.0);
  nh_priv.param("start_acc_x", start_acc(0), 0.0);
  nh_priv.param("start_acc_y", start_acc(1), 0.0);
  nh_priv.param("start_acc_z", start_acc(2), 0.0);
  nh_priv.param("goal_x", goal(0), 10.0);
  nh_priv.param("goal_y", goal(1), 0.0);
  nh_priv.param("goal_z", goal(2), 1.0);
  nh_priv.param("goal_vel_x", goal_vel(0), 0.0);
  nh_priv.param("goal_vel_y", goal_vel(1), 0.0);
  nh_priv.param("goal_vel_z", goal_vel(2), 0.0);

  /* init map util */
  std::shared_ptr<MapUtil<3>> map_util;
  map_util = std::make_shared<MapUtil<3>>();
  map_util->setParam(nh);

  /* init dynamic astar */
  KinodynamicAstar kastar;
  kastar.setParam(nh);
  kastar.setMap(map_util);
  kastar.init();

  ros::Rate rate(1);

  while (ros::ok())
  {
    /* visual start and goal points */
    visualizer.visualizeEndpoints(start, goal);

    /* Plan once */
    if (map_util->has_map_() && !plan_once)
    {
      success = kastar.search(start, start_vel, start_acc, goal, goal_vel);
      ROS_INFO(success == KinodynamicAstar::REACH_END
                   ? "\033[1;32mPATH FOUND!\033[0m"
                   : "\033[1;31mPATH NOT FOUND\033[0m");
      /* visual path */
      if (success == KinodynamicAstar::REACH_END)
      {
        /* vec_Vec3f */
        double effort = kastar.getKinoTraj(0.01, path);
        ROS_INFO("effort: %lf", effort);

        /* visualize */
        visualizer.visualizePath(path);
        plan_once = true;
      }
    }
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
