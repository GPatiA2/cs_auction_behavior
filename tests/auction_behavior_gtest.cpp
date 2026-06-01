// Copyright 2026 Universidad Politécnica de Madrid
//
// BSD-3-Clause License

/*!*******************************************************************************************
 *  \file       auction_behavior_gtest.cpp
 *  \brief      Tests for coordinate_item plugin (cs4home version)
 *  \authors    Guillermo GP-Lenza
 ********************************************************************************************/

#include <gtest/gtest.h>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "coordinate_item/coordinate_item.hpp"

// ── coordinate_item tests ────────────────────────────────────────────────────

TEST(CoordinateItemTest, CreateStoresNameAndCoords)
{
  as2_msgs::msg::AuctionItem item_msg;
  item_msg.name = "target_A";
  item_msg.features = {3.0, 4.0};

  coordinate_item::Plugin factory;
  auto item = factory.create(item_msg);

  EXPECT_EQ(item->get_name(), "target_A");
  EXPECT_EQ(item->get_item().name, "target_A");
}

TEST(CoordinateItemTest, EvaluateDistanceFromOrigin)
{
  as2_msgs::msg::AuctionItem item_msg;
  item_msg.name = "target";
  item_msg.features = {3.0, 4.0};

  coordinate_item::Plugin factory;
  auto item = factory.create(item_msg);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;

  EXPECT_NEAR(item->evaluate(pose), 5.0f, 1e-4f);
}

TEST(CoordinateItemTest, EvaluateDistanceFromNonOriginPose)
{
  as2_msgs::msg::AuctionItem item_msg;
  item_msg.name = "target";
  item_msg.features = {4.0, 4.0};

  coordinate_item::Plugin factory;
  auto item = factory.create(item_msg);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.0;

  EXPECT_NEAR(item->evaluate(pose), 5.0f, 1e-4f);
}

TEST(CoordinateItemTest, EvaluateZeroDistanceWhenAtTarget)
{
  as2_msgs::msg::AuctionItem item_msg;
  item_msg.name = "target";
  item_msg.features = {2.0, 3.0};

  coordinate_item::Plugin factory;
  auto item = factory.create(item_msg);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 2.0;
  pose.pose.position.y = 3.0;

  EXPECT_NEAR(item->evaluate(pose), 0.0f, 1e-4f);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
