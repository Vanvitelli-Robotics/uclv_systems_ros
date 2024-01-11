
#include <uclv_systems_ros/sum_msg_node.hpp>
#include <uclv_systems_ros/sum_geometry_msgs.hpp>

namespace uclv_systems_ros
{

using SumTwistNode = SumMsgNode<geometry_msgs::msg::Twist>;
using SumTwistStampedNode = SumMsgNode<geometry_msgs::msg::TwistStamped>;

}  // namespace uclv_systems_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uclv_systems_ros::SumTwistNode)
RCLCPP_COMPONENTS_REGISTER_NODE(uclv_systems_ros::SumTwistStampedNode)
