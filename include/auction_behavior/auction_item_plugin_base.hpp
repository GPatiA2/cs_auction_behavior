// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

/*!*******************************************************************************************
 *  \file       auction_item_plugin_base.hpp
 *  \brief      Auction item plugin base (cs4home version)
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef AUCTION_BEHAVIOR__AUCTION_ITEM_PLUGIN_BASE_HPP_
#define AUCTION_BEHAVIOR__AUCTION_ITEM_PLUGIN_BASE_HPP_

#include <memory>
#include <string>

#include "as2_msgs/msg/auction_item.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace as2_auction_behavior
{

class AuctionItemPluginBase
{
public:
  virtual ~AuctionItemPluginBase() = default;

  virtual std::shared_ptr<AuctionItemPluginBase> create(
    const as2_msgs::msg::AuctionItem & item_msg) const = 0;

  // Evaluate the cost/utility of this item given the agent's current pose.
  virtual float evaluate(
    const geometry_msgs::msg::PoseStamped & current_pose) const = 0;

  virtual std::string get_name() const = 0;
  virtual std::string to_string() const = 0;
  virtual as2_msgs::msg::AuctionItem get_item() const = 0;

protected:
  AuctionItemPluginBase() {}
};

}  // namespace as2_auction_behavior

#endif  // AUCTION_BEHAVIOR__AUCTION_ITEM_PLUGIN_BASE_HPP_
