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

class ImageConverterNode : public rclcpp::Node
{
public:
    ImageConverterNode() : Node("img_to_pcl"), DISTANCE_PER_PIXEL(0.1)
    {
        // Fetch parameter for image topic
        std::string image_sub_topic = this->declare_parameter<std::string>("image_sub_topic", "para_ai/bev_obstcale_pred");
        std::string image_pub_topic = this->declare_parameter<std::string>("image_pub_topic", "para_ai/bev_obstcale_pcl");
        obs_color = this->declare_parameter<int>("obstcale_color", 255);

        // Advertise topic
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(image_pub_topic, 10);

        // Subscribe to image topic
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_sub_topic, 10,
            std::bind(&ImageConverterNode::image_callback, this, std::placeholders::_1)
        );
    }

private:
    int obs_color;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        // Convert image message to OpenCV Mat
        cv::Mat img = cv_bridge::toCvShare(msg, "rgb8")->image;

        // Create an unorganized 3D point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);

        // Fill point cloud data
        for (int x = 0; x < img.rows; ++x) 
        {
            for (int y = 0; y < img.cols; ++y)
            {
                auto color = img.at<cv::Vec3b>(cv::Point(y, x));

                // Check if the pixel color matches the target color or is black
                if ((color[0] == obs_color && color[1] == obs_color && color[2] == obs_color))
                {
                    pcl::PointXYZRGB point;

                    point.z = 1.0;
                    point.x = (x - img.rows / 2.0) * DISTANCE_PER_PIXEL;
                    point.y = (y - img.cols / 2.0) * DISTANCE_PER_PIXEL;

                    point.r = color[2];
                    point.g = color[1];
                    point.b = color[0];

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

    const double DISTANCE_PER_PIXEL;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pointcloud_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageConverterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
