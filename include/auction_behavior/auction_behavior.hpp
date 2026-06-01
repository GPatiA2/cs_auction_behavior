// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

/*!*******************************************************************************************
 *  \file       auction_behavior.hpp
 *  \brief      Auction behavior CognitiveModule (cs4home version)
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_HPP_
#define AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_HPP_

#include <memory>

#include "cs4home_core/CognitiveModule.hpp"

#include "auction_behavior/auction_behavior_afferent.hpp"
#include "auction_behavior/auction_behavior_core.hpp"

namespace auction_behavior
{

class AuctionBehavior : public cs4home_core::CognitiveModule
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AuctionBehavior)
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  AuctionBehavior();

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;
};

}  // namespace auction_behavior

#endif  // AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_HPP_
