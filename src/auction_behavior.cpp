// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

/*!*******************************************************************************************
 *  \file       auction_behavior.cpp
 *  \brief      AuctionBehavior CognitiveModule — on_configure wires components
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "auction_behavior/auction_behavior.hpp"

#include <memory>

namespace auction_behavior
{

AuctionBehavior::AuctionBehavior()
: cs4home_core::CognitiveModule("AuctionBehavior")
{
}

AuctionBehavior::CallbackReturnT
AuctionBehavior::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto self = std::dynamic_pointer_cast<rclcpp_lifecycle::LifecycleNode>(shared_from_this());

  // Afferent: subscribes to self_localization/pose (ONDEMAND)
  afferent_ = std::make_shared<AuctionBehaviorAfferent>(self);
  afferent_->configure();

  // Core: action server + plugin loading + CA registrations
  auto core = std::make_shared<AuctionBehaviorCore>(self);
  core->set_afferent(afferent_);
  if (!core->configure()) {
    RCLCPP_ERROR(get_logger(), "AuctionBehaviorCore::configure() failed");
    return CallbackReturnT::FAILURE;
  }
  core_ = core;

  RCLCPP_INFO(get_logger(), "AuctionBehavior configured.");
  return CallbackReturnT::SUCCESS;
}

}  // namespace auction_behavior
