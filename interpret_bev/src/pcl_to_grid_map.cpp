#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void pcl_callback(const sensor_msgs::PointCloud2& pcl_msg)
{
  // convert pointcloud message to usable point cloud object
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(pcl_msg, *pointCloud);

  // initialize GridMap
  grid_map::GridMap gridMap;
  gridMap.setFrameId("map");
  gridMap.setGeometry(grid_map::Length(10, 10), 0.05);
  gridMap.add("elevation", 0.0);

  for (const auto& point : pointCloud->points) {
    grid_map::Position position(point.x, point.y);
    gridMap.atPosition("elevation", position) = point.z;
  }

  // convert GridMap to message and publish
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(gridMap, msg);
  pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_to_grid_map");

  ros::NodeHandle nh;
  pub = nh.advertise<grid_map_msgs::GridMap>("/bev_grid_map", 1, true);
  ros::Subscriber sub = nh.subscribe("/pcl/segmented", 1, pcl_callback);
  
  ros::spin();
}
