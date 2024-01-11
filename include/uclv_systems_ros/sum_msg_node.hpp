#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sum_msgs.hpp"

namespace uclv_systems_ros
{

template <typename MessageT>
class SumMsgNode : public rclcpp::Node
{
public:
  SumMsgNode(const rclcpp::NodeOptions& options) : Node("sum_msg_node", options)
  {
    topics_in_ = this->declare_parameter<std::vector<std::string>>("topics_in", std::vector<std::string>());
    signs_ = this->declare_parameter<std::vector<bool>>("signs", std::vector<bool>());
    pub_msg_ = this->create_publisher<MessageT>("msg_out", rclcpp::SensorDataQoS());
  }

  void reset_subscriptions()
  {
    last_msgs_.clear();
    sub_msgs_.clear();
    last_msgs_.reserve(topics_in_.size());
    sub_msgs_.reserve(topics_in_.size());
    for (size_t i = 0; i < topics_in_.size(); i++)
    {
      last_msgs_.push_back(nullptr);
      sub_msgs_.push_back(this->create_subscription<MessageT>(
          topics_in_[i], rclcpp::SensorDataQoS(), std::bind(&SumMsgNode::msgCallback, this, std::placeholders::_1, i)));
    }
  }

  void msgCallback(const typename MessageT::ConstSharedPtr source_msg, size_t index)
  {
    last_msgs_[index] = source_msg;
    publish_sum();
  }

  void publish_sum()
  {
    typename MessageT::UniquePtr msg_out = std::make_unique<MessageT>();

    sum_msgs(last_msgs_, *msg_out, signs_);

    pub_msg_->publish(std::move(msg_out));
  }

protected:
  std::vector<std::string> topics_in_;
  std::vector<typename MessageT::ConstSharedPtr> last_msgs_;
  std::vector<std::shared_ptr<rclcpp::Subscription<MessageT>>> sub_msgs_;
  std::vector<bool> signs_;

  std::shared_ptr<rclcpp::Publisher<MessageT>> pub_msg_;
};

}  // namespace uclv_systems_ros