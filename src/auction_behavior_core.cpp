// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

/*!*******************************************************************************************
 *  \file       auction_behavior_core.cpp
 *  \brief      AuctionBehaviorCore — cs4home Core component
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include "auction_behavior/auction_behavior_core.hpp"

#include <memory>
#include <string>
#include <vector>

namespace auction_behavior
{

namespace
{
std::string strip_ns(const std::string & ns)
{
  return (!ns.empty() && ns.front() == '/') ? ns.substr(1) : ns;
}
}  // namespace

AuctionBehaviorCore::AuctionBehaviorCore(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
: cs4home_core::Core("auction_behavior_core", parent)
{
}

bool AuctionBehaviorCore::configure()
{
  parent_->declare_parameter("plugin_name", "greedy_sequential");
  const std::string plugin_name =
    parent_->get_parameter("plugin_name").as_string() + "::Plugin";
  parent_->declare_parameter("run_frequency", 10.0);

  // CA gateway client
  auto self = std::dynamic_pointer_cast<rclcpp_lifecycle::LifecycleNode>(parent_);
  ca_client_ = ca_structure::CAGatewayClient(self);

  // Auction behavior plugin
  loader_ = std::make_shared<
    pluginlib::ClassLoader<as2_auction_behavior::AuctionBehaviorPluginBase>>(
    "auction_behavior", "as2_auction_behavior::AuctionBehaviorPluginBase");

  try {
    auction_plugin_ = loader_->createSharedInstance(plugin_name);
    auction_plugin_->initialize(parent_, ca_client_);
    auction_plugin_->configure(parent_);
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(parent_->get_logger(),
      "Failed to load auction plugin '%s': %s", plugin_name.c_str(), e.what());
    return false;
  }

  // Item plugin loader
  item_loader_ = std::make_shared<
    pluginlib::ClassLoader<as2_auction_behavior::AuctionItemPluginBase>>(
    "auction_behavior", "as2_auction_behavior::AuctionItemPluginBase");
  loaded_item_type_.clear();

  // CA message registrations
  ca_client_.register_module<as2_msgs::msg::StartAuction>(
    "auction_item_array", "auction_behavior",
    [this](const as2_msgs::msg::StartAuction & msg, const std::string & agent_id) {
      RCLCPP_INFO(parent_->get_logger(),
        "StartAuction from '%s': %zu items, %zu participants",
        agent_id.c_str(), msg.items.list.size(), msg.participants.size());

      if (!load_item_plugin(msg.items.item_type)) {return;}

      auction_plugin_->set_current_pose(get_current_pose());
      auction_plugin_->set_participans(msg.participants);
      auction_plugin_->on_auction_items_received(msg.items, agent_id);

      AuctionAction::Goal participant_goal;
      participant_goal.type     = msg.items.item_type;
      participant_goal.elements = msg.items.list;
      participant_goal.bidders  = msg.participants;

      is_participant_ = true;
      self_action_client_->async_send_goal(participant_goal);
    });

  ca_client_.register_module<as2_msgs::msg::Bid>(
    "bid", "auction_behavior",
    [this](const as2_msgs::msg::Bid & msg, const std::string & agent_id) {
      auction_plugin_->on_bid_received(msg, agent_id);
    });

  // Action server
  const std::string action_name =
    std::string(parent_->get_namespace()) + "/AuctionBehavior";

  action_server_ = rclcpp_action::create_server<AuctionAction>(
    parent_,
    action_name,
    [this](const rclcpp_action::GoalUUID & uuid, auto goal) {
      return handle_goal(uuid, goal);
    },
    [this](auto gh) {return handle_cancel(gh);},
    [this](auto gh) {handle_accepted(gh);});

  self_action_client_ = rclcpp_action::create_client<AuctionAction>(parent_, action_name);

  RCLCPP_INFO(parent_->get_logger(), "AuctionBehaviorCore configured.");
  return true;
}

bool AuctionBehaviorCore::activate()  {return true;}

bool AuctionBehaviorCore::deactivate()
{
  if (run_timer_) {
    run_timer_->cancel();
    run_timer_.reset();
  }
  return true;
}

// ── Action server callbacks ─────────────────────────────────────────────────

rclcpp_action::GoalResponse AuctionBehaviorCore::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const AuctionAction::Goal> goal)
{
  return do_activate(goal) ?
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
    rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse AuctionBehaviorCore::handle_cancel(
  std::shared_ptr<GoalHandleT> /*goal_handle*/)
{
  do_deactivate();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AuctionBehaviorCore::handle_accepted(std::shared_ptr<GoalHandleT> goal_handle)
{
  goal_handle_ = goal_handle;
  const double run_freq =
    parent_->get_parameter("run_frequency").as_double();
  run_timer_ = parent_->create_wall_timer(
    std::chrono::duration<double>(1.0 / run_freq),
    [this]() {do_run();});
}

// ── Behavior logic ──────────────────────────────────────────────────────────

bool AuctionBehaviorCore::do_activate(std::shared_ptr<const AuctionAction::Goal> goal)
{
  const std::string item_type = goal->type;
  auction_id_ = goal->name;

  if (!load_item_plugin(item_type)) {
    is_participant_ = false;
    return false;
  }

  if (!is_participant_) {
    auction_plugin_->set_participans(goal->bidders);

    as2_msgs::msg::AuctionItemArray items_msg;
    items_msg.list      = goal->elements;
    items_msg.item_type = item_type;

    auction_plugin_->set_current_pose(get_current_pose());
    auction_plugin_->on_auction_items_received(items_msg, parent_->get_namespace());

    as2_msgs::msg::StartAuction start_msg;
    start_msg.participants = goal->bidders;
    start_msg.items        = items_msg;
    start_msg.itemtype     = item_type;

    const std::string my_ns = parent_->get_namespace();
    std::vector<std::string> others;
    for (const auto & b : goal->bidders) {
      if (b != my_ns) {others.push_back(b);}
    }
    ca_client_.forward_IA_msg<as2_msgs::msg::StartAuction>(
      start_msg, "auction_item_array", others);

    auction_plugin_->on_activate(goal);
  }

  is_participant_ = false;
  goal_ = *goal;
  started_ = true;
  return true;
}

bool AuctionBehaviorCore::do_deactivate()
{
  auction_plugin_->on_deactivate();
  started_ = false;
  return true;
}

void AuctionBehaviorCore::do_run()
{
  if (!goal_handle_ || !goal_handle_->is_active()) {return;}
  if (!started_) {return;}

  auto feedback = std::make_shared<AuctionAction::Feedback>();
  auto result   = std::make_shared<AuctionAction::Result>();

  if (!auction_plugin_->check_convergence()) {
    auction_plugin_->on_run();
    auto fb = auction_plugin_->get_feedback();
    feedback->asignees = fb.asignees;
    feedback->items    = fb.items;
    feedback->amounts  = fb.amounts;
    goal_handle_->publish_feedback(feedback);
    return;
  }

  auto res = auction_plugin_->get_result();
  for (const auto & w : res.winners) {
    if (w.empty()) {
      RCLCPP_ERROR(parent_->get_logger(), "Auction converged with unassigned items.");
      run_timer_->cancel();
      goal_handle_->abort(result);
      do_execution_end(false);
      return;
    }
  }

  result->winners  = res.winners;
  result->elements = res.elements;
  result_ = res;
  started_ = false;

  run_timer_->cancel();
  goal_handle_->succeed(result);
  do_execution_end(true);
}

void AuctionBehaviorCore::do_execution_end(bool success)
{
  auction_plugin_->on_execution_end();
  if (success) {
    RCLCPP_INFO(parent_->get_logger(), "=== AUCTION RESULT ===");
    for (size_t i = 0; i < result_.elements.size(); ++i) {
      RCLCPP_INFO(parent_->get_logger(), "  %s -> %s",
        strip_ns(result_.winners[i]).c_str(),
        result_.elements[i].name.c_str());
    }
  }
  goal_handle_.reset();
}

// ── Helpers ─────────────────────────────────────────────────────────────────

bool AuctionBehaviorCore::load_item_plugin(const std::string & item_type)
{
  if (loaded_item_type_ == item_type) {return true;}
  try {
    item_plugin_ = item_loader_->createSharedInstance(item_type + "::Plugin");
    auction_plugin_->set_item_plugin(item_plugin_);
    loaded_item_type_ = item_type;
    return true;
  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(parent_->get_logger(),
      "Failed to load item plugin '%s': %s", item_type.c_str(), e.what());
    return false;
  }
}

geometry_msgs::msg::PoseStamped AuctionBehaviorCore::get_current_pose()
{
  auto msg = afferent_->get_msg<geometry_msgs::msg::PoseStamped>("self_localization/pose");
  return msg ? *msg : geometry_msgs::msg::PoseStamped{};
}

}  // namespace auction_behavior
