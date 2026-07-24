#pragma once

#include "sum_msgs.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace uclv_systems_ros
{

template <>
inline void sum_msg(const geometry_msgs::msg::Twist& msg1, const geometry_msgs::msg::Twist& msg2,
                    geometry_msgs::msg::Twist& result, bool positive_sign)
{
  if (positive_sign)
  {
    result.linear.x = msg1.linear.x + msg2.linear.x;
    result.linear.y = msg1.linear.y + msg2.linear.y;
    result.linear.z = msg1.linear.z + msg2.linear.z;
    result.angular.x = msg1.angular.x + msg2.angular.x;
    result.angular.y = msg1.angular.y + msg2.angular.y;
    result.angular.z = msg1.angular.z + msg2.angular.z;
  }
  else
  {
    result.linear.x = msg1.linear.x - msg2.linear.x;
    result.linear.y = msg1.linear.y - msg2.linear.y;
    result.linear.z = msg1.linear.z - msg2.linear.z;
    result.angular.x = msg1.angular.x - msg2.angular.x;
    result.angular.y = msg1.angular.y - msg2.angular.y;
    result.angular.z = msg1.angular.z - msg2.angular.z;
  }
}

template <>
inline void sum_msg(const geometry_msgs::msg::TwistStamped& msg1, const geometry_msgs::msg::TwistStamped& msg2,
                    geometry_msgs::msg::TwistStamped& result, bool positive_sign)
{
  propagate_header_in_sum(msg1.header, msg2.header, result.header);
  sum_msg(msg1.twist, msg2.twist, result.twist, positive_sign);
}

}  // namespace uclv_systems_ros
