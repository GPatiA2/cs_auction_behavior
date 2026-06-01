// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

/*!*******************************************************************************************
 *  \file       auction_behavior_node.cpp
 *  \brief      Auction behavior node executable (cs4home version)
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include "auction_behavior/auction_behavior.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<auction_behavior::AuctionBehavior>();

  // Spin the executor in a background thread so DDS discovery runs while
  // on_configure() waits for the CA gateway service.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  std::thread spin_thread([&executor]() {executor.spin();});

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
