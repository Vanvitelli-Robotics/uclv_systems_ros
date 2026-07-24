#include <uclv_eigen_ros_conversions/eigen_ros_conversions_geometry_msgs.hpp>

#include <uclv_systems_ros/filter_msg_node.hpp>

namespace uclv_systems_ros
{

using FilterTwistNode = FilterMsgNode<geometry_msgs::msg::Twist, 6>;
using FilterTwistStampedNode = FilterMsgNode<geometry_msgs::msg::TwistStamped, 6, true>;
using FilterWrenchNode = FilterMsgNode<geometry_msgs::msg::Wrench, 6>;
using FilterWrenchStampedNode = FilterMsgNode<geometry_msgs::msg::WrenchStamped, 6, true>;

}  // namespace uclv_systems_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uclv_systems_ros::FilterTwistNode)
RCLCPP_COMPONENTS_REGISTER_NODE(uclv_systems_ros::FilterTwistStampedNode)
RCLCPP_COMPONENTS_REGISTER_NODE(uclv_systems_ros::FilterWrenchNode)
RCLCPP_COMPONENTS_REGISTER_NODE(uclv_systems_ros::FilterWrenchStampedNode)
