// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

#include "auction_behavior/auction_behavior_afferent.hpp"

namespace auction_behavior
{

AuctionBehaviorAfferent::AuctionBehaviorAfferent(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
: cs4home_core::Afferent("auction_behavior_afferent", parent)
{
}

bool AuctionBehaviorAfferent::configure()
{
  // Subscribe to the robot's self-localization pose in ONDEMAND mode so Core
  // can pull the latest pose when computing item costs.
  return create_subscriber("self_localization/pose", "geometry_msgs/msg/PoseStamped");
}

}  // namespace auction_behavior
