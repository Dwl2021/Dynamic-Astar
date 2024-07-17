#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

std::string pcd_file_path, frame_id;
double object_theta;
ros::Publisher pcl_pub;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_pcl_node");
  ros::NodeHandle nh("~");

  nh.param("pcd_file_path", pcd_file_path, std::string("package://map_pcl/pcd/map.pcd"));
  nh.param("frame_id", frame_id, std::string("world"));
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);

  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, cloud) == -1)
  {
    PCL_ERROR("Couldn't read file %s\n", pcd_file_path.c_str());
    return (-1);
  }

  pcl::toROSMsg(cloud, output);
  output.header.frame_id = frame_id;

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
