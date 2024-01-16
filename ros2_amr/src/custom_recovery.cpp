// REFERENCE: /opt/ros/humble/share/nav2_behaviors/plugins
// Here, the include path is not set so we use angle brackets to reference the hpp file from pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ros2_amr::SendSms, nav2_core::Behavior)

Status SendSms::onRun(const std::shared_ptr<const Action::Goal> command)
{
  std::string response;
  bool message_success = _twilio->send_message(
    _to_number,
    _from_number,
    command->message,
    response,
    "",
    false);

  if (!message_success) {
    RCLCPP_INFO(node_->get_logger(), "SMS send failed.");
    return ResultStatus{Status::FAILED};
  }

  RCLCPP_INFO(node_->get_logger(), "SMS sent successfully!");
  return ResultStatus{Status::SUCCEEDED};
}

Status SendSms::onCycleUpdate()
{
  return Status::SUCCEEDED;
}