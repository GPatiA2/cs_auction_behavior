// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

#ifndef AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_AFFERENT_HPP_
#define AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_AFFERENT_HPP_

#include "cs4home_core/Afferent.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace auction_behavior
{

class AuctionBehaviorAfferent : public cs4home_core::Afferent
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AuctionBehaviorAfferent)

  explicit AuctionBehaviorAfferent(rclcpp_lifecycle::LifecycleNode::SharedPtr parent);

  bool configure() override;
};

}  // namespace auction_behavior

#endif  // AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_AFFERENT_HPP_
