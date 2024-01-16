// Copyright (c) 2020 Samsung Research America
// This code is licensed under MIT license (see LICENSE.txt for details)


#include <cmath>
#include <chrono>
#include <memory>

#include "/opt/ros/humble/include/pluginlib/pluginlib/class_list_macros.hpp"
#include "/home/hb/ros2_turtle/src/ros2_amr/include/ros2_amr/send_sms.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_amr::SendSms, nav2_core::Behavior)
//#include "nav2_sms_behavior/send_sms.hpp"

namespace ros2_amr
{

SendSms::SendSms()
: TimedBehavior<Action>()
{
}

SendSms::~SendSms()
{
}

void SendSms::onConfigure()
{
  auto node = node_.lock();
  node->declare_parameter("account_sid","");
  _account_sid = node->get_parameter("account_sid").as_string();
  node->declare_parameter("auth_token","");
  _auth_token = node->get_parameter("auth_token").as_string();
  node->declare_parameter("from_number","");
  _from_number = node->get_parameter("from_number").as_string();
  node->declare_parameter("to_number","");
  _to_number = node->get_parameter("to_number").as_string();
  _twilio = std::make_shared<twilio::Twilio>(_account_sid, _auth_token);
}

ResultStatus SendSms::onRun(const std::shared_ptr<const Action::Goal> command)
{
  auto node = node_.lock();
  std::string response;
  bool message_success = _twilio->send_message(
    _to_number,
    _from_number,
    command->message,
    response,
    "",
    false);
  if (!message_success) {
    RCLCPP_INFO(node->get_logger(), "SMS send failed.");
    return ResultStatus{Status::FAILED};
  }

  RCLCPP_INFO(node->get_logger(), "SMS sent successfully!");
  return ResultStatus{Status::SUCCEEDED};
}

ResultStatus SendSms::onCycleUpdate()
{
  return ResultStatus{Status::SUCCEEDED};
}

}  // namespace nav2_sms_behavior

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_amr::SendSms, nav2_core::Behavior)
