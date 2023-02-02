#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub;

void pcl_callback(const sensor_msgs::ImageConstPtr& image)
{
	const double REGULAR_HEIGHT = 1;
	const double OBSTACLE_HEIGHT = 2;

  cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvShare(image, "bgr8");

  grid_map::GridMap gridMap;
  gridMap.setFrameId("map");
  gridMap.setGeometry(grid_map::Length(10, 10), 0.05);
  gridMap.add("elevation", 0.0);

  // place image pixels on the GridMap according to color
  size_t z = 0;
	for (size_t y = 0; y < cv_img->image.rows; y++)
	{
		for (size_t x = 0; x < cv_img->image.cols; x++)
		{
      auto color = cv_img->image.at<cv::Vec3b>(cv::Point(x, y));

      grid_map::Position position(x * 0.03, y * 0.03);
      if ( color == cv::Vec3b(50, 234, 157))
			{
				gridMap.atPosition("elevation", position) = OBSTACLE_HEIGHT;
			}
			else
			{
				gridMap.atPosition("elevation", position) = REGULAR_HEIGHT;
			}
    }
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
  ros::Subscriber sub = nh.subscribe("/carla/ego_vehicle/bev_view/image", 1, pcl_callback);
  
  ros::spin();
}
