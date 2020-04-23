// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/msg/stamped.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(TestNode, test_new_mesg) {
  rclcpp::msg::Stamped stamp;

  rclcpp::init(0, nullptr);

  rclcpp::Node node("test_foo");

  auto pub = node.create_publisher<rclcpp::msg::Stamped>("chatter", 10);
  rclcpp::shutdown();
}
