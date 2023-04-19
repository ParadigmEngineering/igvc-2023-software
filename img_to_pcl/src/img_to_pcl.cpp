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
            point_cloud->width = img.cols;
            point_cloud->height = img.rows;
            point_cloud->is_dense = false;

            // fill point cloud data
            size_t z = 0;
            for (int x = 0; x < img.rows; x++) {
                for (int y = 0; y < img.cols; y++) {
                    auto color = img.at<cv::Vec3b>(cv::Point(x, y));

                    if ( color == cv::Vec3b(30, 170, 250))
                    {
                        z = 1;
                    }
                    else
                    {
                        z = 0;
                    }

                    pcl::PointXYZRGB point;

                    point.z = z;
                    point.x = x;
                    point.y = y;

                    point.r = img.at<cv::Vec3b>(x, y)[2];
                    point.g = img.at<cv::Vec3b>(x, y)[1];
                    point.b = img.at<cv::Vec3b>(x, y)[0];

                    point_cloud->points.push_back(point);
                }
            }

            // // Convert point cloud to ROS message and publish
            // sensor_msgs::msg::PointCloud2 msg_out;
            // pcl::toROSMsg(*point_cloud, msg_out);
            // msg_out.header.frame_id = "ego_vehicle/bev_view";
            // msg_out.header.stamp = msg->header.stamp;
            // pointcloud_publisher_->publish(msg_out);
            
            // Convert point cloud to ROS message
            sensor_msgs::msg::PointCloud2 msg_out;
            pcl::toROSMsg(*point_cloud, msg_out);

            // Set the frame_id of the point cloud to base_link
            msg_out.header.frame_id = "base_link";
            msg_out.header.stamp = msg->header.stamp;

            // Publish point cloud
            pointcloud_publisher_->publish(msg_out);

        }

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
