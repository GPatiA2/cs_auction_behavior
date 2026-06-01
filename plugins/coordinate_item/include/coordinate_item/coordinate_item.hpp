// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

#ifndef COORDINATE_ITEM__COORDINATE_ITEM_HPP_
#define COORDINATE_ITEM__COORDINATE_ITEM_HPP_

#include <memory>
#include <string>

#include "auction_behavior/auction_item_plugin_base.hpp"
#include "as2_msgs/msg/auction_item.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace coordinate_item
{

class Plugin : public as2_auction_behavior::AuctionItemPluginBase
{
public:
  Plugin() = default;

  std::shared_ptr<AuctionItemPluginBase> create(
    const as2_msgs::msg::AuctionItem & item_msg) const override;

  float evaluate(const geometry_msgs::msg::PoseStamped & current_pose) const override;

  std::string get_name() const override;
  std::string to_string() const override;
  as2_msgs::msg::AuctionItem get_item() const override;

private:
  as2_msgs::msg::AuctionItem item_;
  std::string name_;
  double x_{0.0};
  double y_{0.0};
};

}  // namespace coordinate_item

#endif  // COORDINATE_ITEM__COORDINATE_ITEM_HPP_
