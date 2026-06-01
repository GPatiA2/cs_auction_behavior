// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

#include "coordinate_item/coordinate_item.hpp"

#include <cmath>
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace coordinate_item
{

std::shared_ptr<as2_auction_behavior::AuctionItemPluginBase> Plugin::create(
  const as2_msgs::msg::AuctionItem & item_msg) const
{
  auto instance = std::make_shared<Plugin>();
  instance->item_ = item_msg;
  instance->name_ = item_msg.name;
  if (item_msg.features.size() >= 2) {
    instance->x_ = item_msg.features[0];
    instance->y_ = item_msg.features[1];
  } else {
    RCLCPP_WARN(rclcpp::get_logger("coordinate_item"),
      "Item '%s' has %zu features, expected at least 2 (x, y). Defaulting to origin.",
      item_msg.name.c_str(), item_msg.features.size());
  }
  return instance;
}

float Plugin::evaluate(const geometry_msgs::msg::PoseStamped & current_pose) const
{
  const double dx = current_pose.pose.position.x - x_;
  const double dy = current_pose.pose.position.y - y_;
  return static_cast<float>(std::sqrt(dx * dx + dy * dy));
}

std::string Plugin::to_string() const
{
  return "CoordinateItem(name='" + name_ + "', x=" + std::to_string(x_) +
         ", y=" + std::to_string(y_) + ")";
}

std::string Plugin::get_name() const {return name_;}
as2_msgs::msg::AuctionItem Plugin::get_item() const {return item_;}

}  // namespace coordinate_item

PLUGINLIB_EXPORT_CLASS(coordinate_item::Plugin, as2_auction_behavior::AuctionItemPluginBase)
