#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

class ImageConverterNode: public rclcpp::Node
{
    public:
        ImageConverterNode() : Node("img_to_pcl")
        {
            // advertise topic
            pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("bev_pcl", 10);

            // subscribe to image topic
            image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/carla/ego_vehicle/bev_view/image",
             10,
              std::bind(&ImageConverterNode::image_callback,
               this,
                std::placeholders::_1));
        }

    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
        {
            // Convert image message to OpenCV Mat
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Create empty 3D point cloud with dimensions of image
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            // point_cloud->width = img.cols;
            // point_cloud->height = img.rows;
            // point_cloud->is_dense = false;

            // Fill point cloud data
            for (int x = 0; x < img.rows; x++) {
                for (int y = 0; y < img.cols; y++) {
                    pcl::PointXYZRGB point;

                    point.z = 0.0;
                    point.x = (y - img.cols / 2.0) * DISTANCE_PER_PIXEL;
                    point.y = (x - img.rows / 2.0) * DISTANCE_PER_PIXEL;

                    auto color = img.at<cv::Vec3b>(cv::Point(x, y));

                    point.r = color[2];
                    point.g = color[1];
                    point.b = color[0];

                    // Check if the pixel color matches the target color
                    if (color[0] == 30 && color[1] == 170 && color[2] == 250)
                    {
                        point.z = 1;
                        point_cloud->points.push_back(point);
                    }
                    else if (color[0] == 0 && color[1] == 0 && color[2] == 0)
                    {
                        point.z = 1;
                        point_cloud->points.push_back(point);
                    }
                }
            }


            // Convert point cloud to ROS message and publish
            sensor_msgs::msg::PointCloud2 msg_out;
            pcl::toROSMsg(*point_cloud, msg_out);
            msg_out.header.frame_id = "base_link";
            msg_out.header.stamp = msg->header.stamp;
            pointcloud_publisher_->publish(msg_out);
        }

        const double DISTANCE_PER_PIXEL = 0.1;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageConverterNode>());
  rclcpp::shutdown();
  return 0;
}
