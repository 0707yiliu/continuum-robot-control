// Copyright 2021 ROBOTIS CO., LTD.
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

#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/get_motor_state.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_current.hpp"
// #include "geometry_msgs/msg/vector3.hpp"


class ReadWriteNode : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetMotorState = dynamixel_sdk_custom_interfaces::msg::GetMotorState;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;
  using GetCurrent = dynamixel_sdk_custom_interfaces::srv::GetCurrent;
  

  ReadWriteNode(int control_mode);
  virtual ~ReadWriteNode();

private:
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;

  rclcpp::Publisher<GetMotorState>::SharedPtr rotator_motor_state_publisher;

  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Service<GetCurrent>::SharedPtr get_current_server_;

  rclcpp::TimerBase::SharedPtr timer_;

  int present_position;
  int present_current;
  int present_velocity;

  int control_mode_;

  void pub_motor1_callback();
};

#endif  // READ_WRITE_NODE_HPP_
