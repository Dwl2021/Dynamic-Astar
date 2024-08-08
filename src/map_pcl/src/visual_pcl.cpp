#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

std::string pcd_file_path, frame_id;
Eigen::Vector3d object_axis, object_p;
double object_theta;
ros::Publisher pcl_pub;

void calPose(const Eigen::Vector3d& axis, const double& theta, Eigen::Quaterniond& q,
             Eigen::Vector3d& zd)
{
  Eigen::Quaterniond land_q;
  land_q.w() = cos(theta / 2);
  land_q.x() = axis(0) * sin(theta / 2);
  land_q.y() = axis(1) * sin(theta / 2);
  land_q.z() = axis(2) * sin(theta / 2);
  q = land_q;
  Eigen::MatrixXd land_R = land_q.toRotationMatrix();
  zd = land_R.col(2).normalized();
}

void genPlaneCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  int resolution = 100;
  int length = 1;
  int width = 1;
  int depth = 10;        // Depth below the plane
  double plane_z = 0.0;  // Z coordinate of the plane

  Eigen::Quaterniond q;
  Eigen::Vector3d zd;
  calPose(object_axis, object_theta, q, zd);
  Eigen::MatrixXd R = q.toRotationMatrix();

  for (int i = -resolution / 2; i < resolution / 2; ++i)
  {
    for (int j = -resolution / 2; j < resolution / 2; ++j)
    {
      Eigen::Vector3d vec;
      vec.x() = static_cast<float>(j) / resolution * length;
      vec.y() = static_cast<float>(i) / resolution * width;
      vec.z() = 0;  // Negative to go below the plane

      vec = R * vec;
      vec.x() += object_p.x();
      vec.y() += object_p.y();
      vec.z() += object_p.z() + plane_z;  // Adjust for the plane's z position

      pcl::PointXYZ point;
      point.x = vec.x();
      point.y = vec.y();
      point.z = vec.z();

      for (int k = 1; k <= resolution; ++k)
      {
        pcl::PointXYZ tmp_point = point;
        tmp_point.z = vec.z() * k / resolution;
        cloud.points.push_back(tmp_point);
      }

      cloud.points.push_back(point);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_pcl_node");
  ros::NodeHandle nh("~");

  nh.param("pcd_file_path", pcd_file_path, std::string("package://map_pcl/pcd/map.pcd"));
  nh.param("frame_id", frame_id, std::string("world"));
  nh.getParam("object_px", object_p.x());
  nh.getParam("object_py", object_p.y());
  nh.getParam("object_pz", object_p.z());
  nh.getParam("object_axis_x", object_axis.x());
  nh.getParam("object_axis_y", object_axis.y());
  nh.getParam("object_axis_z", object_axis.z());
  nh.getParam("object_theta", object_theta);
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);

  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  object_axis.normalize();

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, cloud) == -1)
  {
    PCL_ERROR("Couldn't read file %s\n", pcd_file_path.c_str());
    return (-1);
  }
  pcl::PointCloud<pcl::PointXYZ> plane_cloud;
  genPlaneCloud(plane_cloud);
  cloud += plane_cloud;
  // cloud = plane_cloud;

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
