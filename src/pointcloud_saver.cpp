/**
 * @file pointcloud_saver.cpp
 * @author Toshiki Nakamura
 * @brief Save point cloud data
 * @copyright Copyright (c) 2024
 */

#include <geometry_msgs/PoseStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <vector>

class PointCloudSaver
{
public:
  PointCloudSaver(void);

private:
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  bool request_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::ServiceServer request_server_;
  bool request_flag_ = false;
};

PointCloudSaver::PointCloudSaver(void) : pnh_("~")
{
  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");

  cloud_sub_ = nh_.subscribe("cloud", 1, &PointCloudSaver::cloud_callback, this);
  request_server_ = pnh_.advertiseService("request_save", &PointCloudSaver::request_callback, this);
}

void PointCloudSaver::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  if (!request_flag_)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);
  pcl::io::savePCDFileASCII("map_cloud.pcd", cloud);
  ROS_INFO("Map cloud data is saved to ~/.ros/map_cloud.pcd");
  request_flag_ = false;
}

bool PointCloudSaver::request_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  request_flag_ = true;
  res.success = true;
  res.message = "Start saving point cloud data";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_saver");
  PointCloudSaver pointcloud_saver;
  ros::spin();
  return 0;
}
