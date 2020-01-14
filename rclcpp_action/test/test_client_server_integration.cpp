// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <test_msgs/action/fibonacci.hpp>

#include <chrono>
#include <memory>

#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/create_server.hpp"

using namespace std::chrono_literals;

const auto WAIT_FOR_SERVER_TIMEOUT = 10s;

class TestClientServerIntegration : public ::testing::Test
{
protected:
  using ActionType = test_msgs::action::Fibonacci;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node_ = std::make_shared<rclcpp::Node>("test_client_server_node", "/");
    executor_.add_node(node_);
  }

  void TearDown()
  {
    executor_.remove_node(node_);
    client_.reset();
    server_.reset();
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp_action::Client<ActionType>::SharedPtr client_;
  rclcpp_action::Server<ActionType>::SharedPtr server_;
};  // class TestClientServerIntegration

TEST_F(TestClientServerIntegration, test_terminate_in_accepted_callback) {
  // Create a client
  client_ = rclcpp_action::create_client<ActionType>(node_, "test_fibonacci");
  // Create a server
  server_ = rclcpp_action::create_server<ActionType>(
    node_, "test_fibonacci",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ActionType::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](std::shared_ptr<ServerGoalHandle>) {
      return rclcpp_action::CancelResponse::REJECT;
    },
    [](std::shared_ptr<ServerGoalHandle> handle) {
      // Succeed immediately after goal is accepted
      auto result = std::make_shared<ActionType::Result>();
      handle->succeed(result);
    });

  // Wait for discovery
  ASSERT_TRUE(client_->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT));

  // Send goal
  ActionType::Goal goal;
  auto future_goal_handle = client_->async_send_goal(goal);
  auto spin_ret = executor_.spin_until_future_complete(
    future_goal_handle, std::chrono::seconds(5));
  ASSERT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS, spin_ret);

  auto goal_handle = future_goal_handle.get();
  auto goal_status = goal_handle->get_status();
  EXPECT_EQ(rclcpp_action::GoalStatus::STATUS_ACCEPTED, goal_status);

  // Get result
  auto future_result = client_->async_get_result(goal_handle);
  EXPECT_TRUE(goal_handle->is_result_aware());
  spin_ret = executor_.spin_until_future_complete(
    future_result, std::chrono::seconds(5));
  ASSERT_EQ(rclcpp::executor::FutureReturnCode::SUCCESS, spin_ret);
  auto wrapped_result = future_result.get();
  EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, wrapped_result.code);
}
