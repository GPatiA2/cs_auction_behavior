// Copyright 2026 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!*******************************************************************************************
 *  \file       auction_behavior_plugin_base.hpp
 *  \brief      Auction behavior plugin base (cs4home version — uses ca_structure)
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#ifndef AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_PLUGIN_BASE_HPP_
#define AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_PLUGIN_BASE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ca_structure/ca_gateway_client.hpp"

#include "auction_behavior/auction_item_plugin_base.hpp"
#include "as2_msgs/action/auction.hpp"
#include "as2_msgs/msg/auction_item_array.hpp"
#include "as2_msgs/msg/bid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace as2_auction_behavior
{

class AuctionBehaviorPluginBase
{
  using GoalT = as2_msgs::action::Auction::Goal;

public:
  virtual ~AuctionBehaviorPluginBase() = default;

  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    ca_structure::CAGatewayClient & client)
  {
    client_     = client;
    namespace_  = node->get_namespace();
    if (!namespace_.empty() && namespace_.front() == '/') {
      namespace_ = namespace_.substr(1);
    }
    node_ = node;
  }

  virtual void on_activate(std::shared_ptr<const GoalT> goal) = 0;
  virtual void on_deactivate() = 0;
  virtual void on_execution_end() = 0;

  void set_item_plugin(std::shared_ptr<AuctionItemPluginBase> item_plugin)
  {
    item_plugin_ = item_plugin;
  }

  void set_current_pose(const geometry_msgs::msg::PoseStamped & pose)
  {
    current_pose_ = pose;
  }

  virtual void on_auction_items_received(
    const as2_msgs::msg::AuctionItemArray & msg,
    const std::string & agent_id)
  {
    auction_items_.clear();
    for (const auto & item_msg : msg.list) {
      auction_items_.push_back(item_plugin_->create(item_msg));
    }
  }

  virtual as2_msgs::msg::Bid compute_bid() = 0;

  virtual void update(const as2_msgs::msg::Bid & bid_msg, const std::string & agent_id) = 0;

  virtual bool check_convergence() = 0;

  void send_bid(const as2_msgs::msg::Bid & bid)
  {
    if (bid.name.empty() || participants_.empty()) {
      return;
    }
    client_.forward_IA_msg<as2_msgs::msg::Bid>(bid, "bid", participants_);
  }

  void on_bid_received(const as2_msgs::msg::Bid & msg, const std::string & agent_id)
  {
    update(msg, agent_id);
    if (check_convergence()) {
      return;
    }
    as2_msgs::msg::Bid new_bid = compute_bid();
    send_bid(new_bid);
  }

  virtual void on_run() {}

  void set_participans(const std::vector<std::string> & participants)
  {
    participants_ = participants;
  }

  virtual void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  {
    (void)node;
  }

  virtual as2_msgs::action::Auction::Feedback get_feedback() = 0;
  virtual as2_msgs::action::Auction::Result   get_result()   = 0;

  virtual std::map<std::string, std::string> get_global_assignment() const = 0;

protected:
  AuctionBehaviorPluginBase() = default;

  ca_structure::CAGatewayClient client_;
  std::vector<std::shared_ptr<AuctionItemPluginBase>> auction_items_;
  std::shared_ptr<AuctionItemPluginBase> item_plugin_;
  std::vector<std::string> participants_;
  std::string namespace_;
  geometry_msgs::msg::PoseStamped current_pose_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace as2_auction_behavior

#endif  // AUCTION_BEHAVIOR__AUCTION_BEHAVIOR_PLUGIN_BASE_HPP_
