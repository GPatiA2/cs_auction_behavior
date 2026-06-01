// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

#ifndef AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_CORE_HPP_
#define AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_CORE_HPP_

#include <memory>
#include <string>

#include "cs4home_core/Core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pluginlib/class_loader.hpp"

#include "ca_structure/ca_gateway_client.hpp"

#include "as2_msgs/action/auction.hpp"
#include "as2_msgs/msg/auction_item_array.hpp"
#include "as2_msgs/msg/bid.hpp"
#include "as2_msgs/msg/start_auction.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "auction_behavior/auction_behavior_plugin_base.hpp"
#include "auction_behavior/auction_item_plugin_base.hpp"

namespace auction_behavior
{

class AuctionBehaviorCore : public cs4home_core::Core
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AuctionBehaviorCore)

  using AuctionAction = as2_msgs::action::Auction;
  using GoalHandleT   = rclcpp_action::ServerGoalHandle<AuctionAction>;

  explicit AuctionBehaviorCore(rclcpp_lifecycle::LifecycleNode::SharedPtr parent);

  bool configure() override;
  bool activate()  override;
  bool deactivate() override;

private:
  ca_structure::CAGatewayClient ca_client_;

  std::shared_ptr<pluginlib::ClassLoader<as2_auction_behavior::AuctionBehaviorPluginBase>>
  loader_;
  std::shared_ptr<as2_auction_behavior::AuctionBehaviorPluginBase> auction_plugin_;

  std::shared_ptr<pluginlib::ClassLoader<as2_auction_behavior::AuctionItemPluginBase>>
  item_loader_;
  std::shared_ptr<as2_auction_behavior::AuctionItemPluginBase> item_plugin_;
  std::string loaded_item_type_;

  rclcpp_action::Server<AuctionAction>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleT> goal_handle_;
  rclcpp::TimerBase::SharedPtr run_timer_;
  rclcpp_action::Client<AuctionAction>::SharedPtr self_action_client_;

  bool started_{false};
  bool is_participant_{false};
  std::string auction_id_;
  AuctionAction::Goal   goal_;
  AuctionAction::Result result_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const AuctionAction::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandleT> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandleT> goal_handle);

  bool do_activate(std::shared_ptr<const AuctionAction::Goal> goal);
  bool do_deactivate();
  void do_run();
  void do_execution_end(bool success);

  bool load_item_plugin(const std::string & item_type);
  geometry_msgs::msg::PoseStamped get_current_pose();
};

}  // namespace auction_behavior

#endif  // AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_CORE_HPP_
