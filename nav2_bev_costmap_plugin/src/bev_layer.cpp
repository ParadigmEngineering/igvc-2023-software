#include "nav2_bev_costmap_plugin/bev_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_bev_costmap_plugin
{

BEVLayer::BEVLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
BEVLayer::onInitialize()
{
  RCLCPP_INFO(rclcpp::get_logger("nav2_bev_costmap_plugin"), "BEVLayer initializing...");

  auto node = node_; //.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  need_recalculation_ = false;
  current_ = true;

  image_subscription_ = node->create_subscription<sensor_msgs::msg::Image>(
    "/carla/ego_vehicle/bev_view/image", 10,
    std::bind(&BEVLayer::image_callback, this, std::placeholders::_1),
    rclcpp::SubscriptionOptions());
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
BEVLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
BEVLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "BEVLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

void BEVLayer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("nav2_bev_costmap_plugin"), "Received image: %d x %d",
    msg->width, msg->height);

  std::lock_guard<std::mutex> lock(bev_mutex_);
  // Convert image message to OpenCV Mat
  processed_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
  // Set flag to re-calculate window bounds
  need_recalculation_ = true;
}

void BEVLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  RCLCPP_INFO(rclcpp::get_logger("nav2_bev_costmap_plugin"), "Updating Costs...");

  std::lock_guard<std::mutex> lock(bev_mutex_);

  if (!enabled_ || processed_image_.empty()) {
    return;
  }

  // Find the common area between the image and the costmap
  int common_max_i = std::min(static_cast<int>(processed_image_.cols), max_i);
  int common_max_j = std::min(static_cast<int>(processed_image_.rows), max_j);

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  common_max_i = std::min(static_cast<int>(master_grid.getSizeInCellsX()), common_max_i);
  common_max_j = std::min(static_cast<int>(master_grid.getSizeInCellsY()), common_max_j);

  // Loop over the common area and update the costmap accordingly
  for (int j = min_j; j < common_max_j; j++) {
    for (int i = min_i; i < common_max_i; i++) {
      cv::Vec3b color = processed_image_.at<cv::Vec3b>(j, i);

      if (color == cv::Vec3b(50, 234, 157)) {
        master_grid.setCost(i, j, LETHAL_OBSTACLE);
      } else {
        master_grid.setCost(i, j, 0);
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("nav2_bev_costmap_plugin"), "Updated Costs!");
}

}  // namespace nav2_bev_costmap_plugin

// This is the macro allowing a nav2_bev_costmap_plugin::BEVLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_bev_costmap_plugin::BEVLayer, nav2_costmap_2d::Layer)
