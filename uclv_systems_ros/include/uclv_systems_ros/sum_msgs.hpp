#pragma once

#include <vector>
#include <memory>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/rclcpp.hpp>

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
  (void)target_msg;
  // As default we will assume that the message has been already initialized to zero
}

template <typename MessageT>
inline void sum_msg(const MessageT& msg1, const MessageT& msg2, MessageT& result, bool positive_sign = true);

template <typename MessageT>
inline void sum_msgs(const std::vector<typename MessageT::ConstSharedPtr>& source_msg, MessageT& target_msg,
                     const std::vector<bool>& positive_signs = std::vector<bool>())
{
  size_t i = 0;

  // Find the first non-null message
  for (; i < source_msg.size(); i++)
  {
    if (source_msg[i])
    {
      bool sign_first_msg = (positive_signs.size() > i) ? positive_signs[i] : true;
      if (sign_first_msg)
      {
        target_msg = *source_msg[i];
      }
      else
      {
        // TODO A better option could be the implementation of a negative_msg function:
        // negative_msg(*source_msg[i], target_msg);
        // but it should be implemented for each message type
        set_zero(target_msg);
        sum_msg(target_msg, *source_msg[i], target_msg, false);
      }
      i++;
      break;
    }
  }

  // Sum the rest of the messages
  for (; i < source_msg.size(); i++)
  {
    if (source_msg[i])
    {
      sum_msg(target_msg, *source_msg[i], target_msg, (positive_signs.size() > i) ? positive_signs[i] : true);
    }
  }
}

}  // namespace uclv_systems_ros
