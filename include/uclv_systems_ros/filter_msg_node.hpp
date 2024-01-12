#pragma once

#include <rclcpp/rclcpp.hpp>
#include <uclv_eigen_ros_conversions/eigen_ros_conversions.hpp>
#include <Eigen/Dense>
#include <uclv_systems_lib/tf/first_order_filter.hpp>

namespace uclv_systems_ros
{

template <typename MessageT, int FILTER_SIZE, bool MSG_HAS_EXTRA_FIELDS = false>
class FilterMsgNode : public rclcpp::Node
{
public:
  FilterMsgNode(const rclcpp::NodeOptions& options) : Node("filter_msg_node", options)
  {
    // TODO make parameters reconfigurable
    sample_time_ = this->declare_parameter<double>("sample_time", -1.0);
    cut_freq_ = this->declare_parameter<double>("cut_frequency", -1.0);
    gain_ = this->declare_parameter<double>("gain", 1.0);

    if (sample_time_ <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "FilterMsgNode Invalid sample time %f", sample_time_);
      throw std::runtime_error("Invalid sample time");
    }
    if (cut_freq_ <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "FilterMsgNode Invalid time frequency %f", cut_freq_);
      throw std::runtime_error("Invalid time frequency");
    }

    init_filter();

    sub_msg_ = this->create_subscription<MessageT>("msg_in", rclcpp::SensorDataQoS(),
                                                   std::bind(&FilterMsgNode::msgCallback, this, std::placeholders::_1));

    pub_msg_ = this->create_publisher<MessageT>("msg_out", rclcpp::SensorDataQoS());

    timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::duration<double>(sample_time_),
                                  std::bind(&FilterMsgNode::timerCallback, this));
  }

  void init_filter()
  {
    if (!filter_)
    {
      filter_ =
          std::make_unique<uclv::systems::tf::mimo::FirstOrderFilter<FILTER_SIZE>>(cut_freq_, sample_time_, gain_);
    }
    else
    {
      filter_->set_parameters(cut_freq_, sample_time_, gain_);
    }
  }

  void msgCallback(const typename MessageT::ConstSharedPtr source_msg)
  {
    last_msg_ = source_msg;
  }

  void timerCallback()
  {
    if (last_msg_ == nullptr)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "FilterMsgNode No msg received yet");
      return;
    }

    typename MessageT::ConstSharedPtr msg_in = last_msg_;

    Eigen::Matrix<double, FILTER_SIZE, 1> input;
    uclv::ros::conversions::convert(*msg_in, input);

    const auto& output = filter_->step(input);

    typename MessageT::UniquePtr msg_out = std::make_unique<MessageT>();
    // convert from eigen to msg, use msg_in as template to copy non-eigen fields
    uclv::ros::conversions::convert(output, *msg_out);
    if (MSG_HAS_EXTRA_FIELDS)
    {
      uclv::ros::conversions::copy_extra_fields(*msg_in, *msg_out, this->now());
    }

    pub_msg_->publish(std::move(msg_out));
  }

protected:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::Subscription<MessageT>> sub_msg_;
  std::shared_ptr<rclcpp::Publisher<MessageT>> pub_msg_;

  double sample_time_, cut_freq_, gain_;
  typename MessageT::ConstSharedPtr last_msg_;

  typename uclv::systems::tf::mimo::FirstOrderFilter<FILTER_SIZE>::UniquePtr filter_;
};

}  // namespace uclv_systems_ros