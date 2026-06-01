// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

#ifndef CBBA__CBBA_HPP_
#define CBBA__CBBA_HPP_

#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "auction_behavior/auction_behavior_plugin_base.hpp"
#include "as2_msgs/action/auction.hpp"
#include "as2_msgs/msg/bid.hpp"

namespace cbba
{

class Plugin : public as2_auction_behavior::AuctionBehaviorPluginBase
{
  using GoalT     = as2_msgs::action::Auction::Goal;
  using FeedbackT = as2_msgs::action::Auction::Feedback;
  using ResultT   = as2_msgs::action::Auction::Result;

public:
  Plugin() = default;

  void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    ca_structure::CAGatewayClient & client) override
  {
    AuctionBehaviorPluginBase::initialize(node, client);
    node->declare_parameter("bundle_size", 1);
  }

  void on_auction_items_received(
    const as2_msgs::msg::AuctionItemArray & msg,
    const std::string & agent_id) override;

  void on_activate(std::shared_ptr<const GoalT> goal) override;
  void on_deactivate() override;
  void on_execution_end() override;

  as2_msgs::msg::Bid compute_bid() override;
  void update(const as2_msgs::msg::Bid & bid_msg, const std::string & agent_id) override;
  bool check_convergence() override;

  FeedbackT get_feedback() override;
  ResultT   get_result()   override;
  std::map<std::string, std::string> get_global_assignment() const override;

protected:
  std::map<std::string, double> costs_;
  std::map<std::string, double> y_;
  std::map<std::string, std::string> z_;
  std::vector<std::string> bundle_;
  std::unordered_set<std::string> received_from_;
  bool changed_ = false;
  int  bundle_size_ = 1;

  void build_bundle();
  void cascade_remove(size_t n_bar);

private:
  void reset();
};

}  // namespace cbba

#endif  // CBBA__CBBA_HPP_
