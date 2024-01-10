#include <uclv_eigen_ros_conversions/eigen_ros_conversions_geometry_msgs.hpp>

#include <uclv_filters_ros/filter_msg_node.hpp>

namespace uclv_filters_ros
{

using FilterTwistStampedNode = FilterMsgNode<geometry_msgs::msg::TwistStamped, 6>;

}  // namespace uclv_filters_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uclv_filters_ros::FilterTwistStampedNode)
