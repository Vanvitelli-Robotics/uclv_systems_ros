#pragma once

#include <vector>
#include <memory>
#include <std_msgs/msg/header.hpp>

namespace uclv_systems_ros
{

// Propagate the header of the message with the latest timestamp
inline void propagate_header_in_sum(const std_msgs::msg::Header& msg1_header, const std_msgs::msg::Header& msg2_header,
                                    std_msgs::msg::Header& result_header)
{
  result_header = (rclcpp::Time(msg1_header.stamp) > rclcpp::Time(msg2_header.stamp)) ? msg1_header : msg2_header;
}

template <typename MessageT>
inline void set_zero(MessageT& target_msg)
{
  // As default we will assume that the message has been already initialized to zero
}

template <typename MessageT>
inline void sum_msg(const MessageT& msg1, const MessageT& msg2, MessageT& result);

template <typename MessageT>
inline void sum_msgs(const std::vector<typename MessageT::ConstSharedPtr>& source_msg, MessageT& target_msg)
{
  for (size_t i = 0; i < source_msg.size(); i++)
  {
    if (source_msg[i])
    {
      sum_msg(target_msg, *source_msg[i], target_msg);
    }
  }
}

}  // namespace uclv_systems_ros
