#include <uclv_systems_ros/twist_stamped_sine_publisher.hpp>

#include <rclcpp_components/register_node_macro.hpp>
#include <yaml-cpp/yaml.h>

#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

namespace uclv_systems_ros
{
namespace
{

constexpr double kTwoPi = 2.0 * std::acos(-1.0);

template <typename ValueT>
ValueT getRequiredValue(const YAML::Node& node, const std::string& name)
{
  const YAML::Node value = node[name];
  if (!value)
  {
    throw std::runtime_error("Missing required YAML field: " + name);
  }
  return value.as<ValueT>();
}

TwistStampedSinePublisher::SineParameters parseSineParameters(const YAML::Node& root, const std::string& group,
                                                               const std::string& component)
{
  const YAML::Node node = root[group][component];
  if (!node)
  {
    throw std::runtime_error("Missing sine configuration for " + group + "." + component);
  }

  return {
      getRequiredValue<double>(node, "amplitude"),
      getRequiredValue<double>(node, "frequency"),
      getRequiredValue<double>(node, "phase"),
      getRequiredValue<double>(node, "offset"),
  };
}

}  // namespace

TwistStampedSinePublisher::TwistStampedSinePublisher(const rclcpp::NodeOptions& options)
: Node("twist_stamped_sine_publisher", options)
{
  const std::string config_file = this->declare_parameter<std::string>("config_file", "");

  if (config_file.empty())
  {
    const std::string message =
        "Parameter 'config_file' is required. Start this node with: ros2 run uclv_systems_ros "
        "twist_stamped_sine_publisher --ros-args -p config_file:=/path/to/config.yaml";
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    throw std::runtime_error(message);
  }

  try
  {
    const YAML::Node config = YAML::LoadFile(config_file)["twist_stamped_sine_publisher"];
    if (!config)
    {
      throw std::runtime_error("Missing top-level YAML key: twist_stamped_sine_publisher");
    }

    const bool use_sim_time = getRequiredValue<bool>(config, "use_sim_time");
    this->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

    const double publish_rate = getRequiredValue<double>(config, "publish_rate");
    if (publish_rate <= 0.0)
    {
      throw std::runtime_error("YAML field 'publish_rate' must be greater than zero");
    }

    const std::array<std::pair<const char*, const char*>, 6> components = {
        {{"linear", "x"}, {"linear", "y"}, {"linear", "z"}, {"angular", "x"}, {"angular", "y"}, {"angular", "z"}}};
    for (std::size_t index = 0; index < components.size(); ++index)
    {
      sine_parameters_[index] = parseSineParameters(config, components[index].first, components[index].second);
    }

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        getRequiredValue<std::string>(config, "topic"), rclcpp::SensorDataQoS());
    timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::duration<double>(1.0 / publish_rate),
                                  std::bind(&TwistStampedSinePublisher::publishMessage, this));
  }
  catch (const YAML::Exception& error)
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to parse configuration file '%s': %s", config_file.c_str(), error.what());
    throw;
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid configuration file '%s': %s", config_file.c_str(), error.what());
    throw;
  }
}

void TwistStampedSinePublisher::publishMessage()
{
  const rclcpp::Time now = this->now();
  if (!time_origin_)
  {
    time_origin_ = now;
  }
  const double elapsed_time = (now - *time_origin_).seconds();
  geometry_msgs::msg::TwistStamped message;
  message.header.stamp = now;
  std::array<double*, 6> values = {
      &message.twist.linear.x,
      &message.twist.linear.y,
      &message.twist.linear.z,
      &message.twist.angular.x,
      &message.twist.angular.y,
      &message.twist.angular.z,
  };

  for (std::size_t index = 0; index < sine_parameters_.size(); ++index)
  {
    const SineParameters& parameters = sine_parameters_[index];
    *values[index] = parameters.offset + parameters.amplitude *
                                              std::sin(kTwoPi * parameters.frequency * elapsed_time + parameters.phase);
  }
  publisher_->publish(message);
}

}  // namespace uclv_systems_ros

RCLCPP_COMPONENTS_REGISTER_NODE(uclv_systems_ros::TwistStampedSinePublisher)