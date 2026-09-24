#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <optional>
#include <string>

namespace uclv_systems_ros
{

class TwistStampedSinePublisher : public rclcpp::Node
{
public:
  struct SineParameters
  {
    double amplitude;
    double frequency;
    double phase;
    double offset;
  };

  explicit TwistStampedSinePublisher(const rclcpp::NodeOptions& options);

private:
  void publishMessage();

  std::array<SineParameters, 6> sine_parameters_;
  std::optional<rclcpp::Time> time_origin_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uclv_systems_ros