#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/QR>
#include <yaml-cpp/yaml.h>

namespace uclv_systems_ros
{

struct MeasurementFilterWindow
{
  std::size_t before_including_current{0};
  std::size_t after{0};

  std::size_t size() const
  {
    return before_including_current + after;
  }
};

enum class MeasurementCovarianceFilterType
{
  MovingAverage,
  SavitzkyGolay,
};

struct MeasurementCovarianceFilterConfig
{
  bool enabled{false};
  MeasurementCovarianceFilterType type{
    MeasurementCovarianceFilterType::MovingAverage};
  MeasurementFilterWindow window;
  int polyorder{0};
};

inline std::int64_t parseInteger(
  const YAML::Node & node, const std::string & description)
{
  if (!node || !node.IsScalar()) {
    throw std::runtime_error(description + " must be an integer");
  }

  try {
    return node.as<std::int64_t>();
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            description + " must be an integer: " + exception.what());
  }
}

inline MeasurementFilterWindow parseMeasurementFilterWindow(
  const YAML::Node & node)
{
  const std::string description =
    "measurement_covariance_filter.window_size";

  if (!node) {
    throw std::runtime_error("Missing YAML key '" + description + "'");
  }

  if (node.IsScalar()) {
    const std::int64_t window_size = parseInteger(node, description);
    if (window_size <= 0 || window_size % 2 == 0) {
      throw std::runtime_error(
              description +
              " must be a positive odd integer when specified as a scalar");
    }
    if (window_size > std::numeric_limits<int>::max()) {
      throw std::runtime_error(description + " is too large");
    }
    return MeasurementFilterWindow{
      static_cast<std::size_t>((window_size + 1) / 2),
      static_cast<std::size_t>((window_size - 1) / 2)};
  }

  if (!node.IsSequence() || node.size() != 2) {
    throw std::runtime_error(
            description +
            " must be a scalar or a two-element [before, after] vector");
  }

  const std::int64_t before = parseInteger(node[0], description + "[0]");
  const std::int64_t after = parseInteger(node[1], description + "[1]");
  if (before < 1) {
    throw std::runtime_error(
            description +
            "[0] must be at least one because it includes the current sample");
  }
  if (after < 0) {
    throw std::runtime_error(description + "[1] cannot be negative");
  }
  if (before > std::numeric_limits<int>::max() ||
    after > std::numeric_limits<int>::max() ||
    before + after > std::numeric_limits<int>::max())
  {
    throw std::runtime_error(description + " is too large");
  }

  return MeasurementFilterWindow{
    static_cast<std::size_t>(before), static_cast<std::size_t>(after)};
}

inline MeasurementCovarianceFilterConfig loadMeasurementCovarianceFilterConfig(
  const std::string & file_path)
{
  YAML::Node document;
  try {
    document = YAML::LoadFile(file_path);
  } catch (const YAML::Exception & exception) {
    throw std::runtime_error(
            "Failed to load measurement covariance filter configuration '" +
            file_path + "': " + exception.what());
  }

  if (!document.IsMap()) {
    throw std::runtime_error(
            "YAML root must be a map in measurement covariance filter file " +
            file_path);
  }

  const YAML::Node filter_node = document["measurement_covariance_filter"];
  if (!filter_node || !filter_node.IsMap()) {
    throw std::runtime_error(
            "Missing YAML map 'measurement_covariance_filter' in " + file_path);
  }

  MeasurementCovarianceFilterConfig config;
  const YAML::Node enabled_node = filter_node["enabled"];
  if (enabled_node) {
    try {
      config.enabled = enabled_node.as<bool>();
    } catch (const YAML::Exception & exception) {
      throw std::runtime_error(
              "measurement_covariance_filter.enabled must be a boolean: " +
              std::string(exception.what()));
    }
  } else {
    config.enabled = true;
  }

  if (!config.enabled) {
    return config;
  }

  const YAML::Node type_node = filter_node["type"];
  const YAML::Node polyorder_node = filter_node["polyorder"];
  if (type_node && !type_node.IsScalar()) {
    throw std::runtime_error(
            "measurement_covariance_filter.type must be 'moving_average', "
            "'ma', 'savitzky_golay', or 'sg'");
  }

  const std::string type = type_node ? type_node.as<std::string>() :
    (polyorder_node ? "savitzky_golay" : "moving_average");
  if (type == "moving_average" || type == "ma") {
    config.type = MeasurementCovarianceFilterType::MovingAverage;
    config.polyorder = 0;
  } else if (type == "savitzky_golay" || type == "sg") {
    config.type = MeasurementCovarianceFilterType::SavitzkyGolay;
    if (polyorder_node) {
      const std::int64_t polyorder = parseInteger(
        polyorder_node, "measurement_covariance_filter.polyorder");
      if (polyorder > std::numeric_limits<int>::max()) {
        throw std::runtime_error(
                "measurement_covariance_filter.polyorder is too large");
      }
      config.polyorder = static_cast<int>(polyorder);
    } else {
      config.polyorder = 3;
    }
  } else {
    throw std::runtime_error(
            "Unsupported measurement covariance filter type '" + type + "'");
  }

  config.window = parseMeasurementFilterWindow(filter_node["window_size"]);
  if (config.polyorder < 0) {
    throw std::runtime_error(
            "measurement_covariance_filter.polyorder cannot be negative");
  }
  if (static_cast<std::size_t>(config.polyorder) >= config.window.size()) {
    throw std::runtime_error(
            "measurement_covariance_filter.polyorder must be smaller than "
            "the total window_size");
  }

  return config;
}

template<typename Scalar, int Dimension>
class MeasurementSignalFilter
{
public:
  using Sample = Eigen::Matrix<Scalar, Dimension, 1>;

  struct SignalEstimate
  {
    std::size_t sample_index;
    Sample value;
  };

  virtual ~MeasurementSignalFilter() = default;

  virtual std::vector<SignalEstimate> estimateSignal(
    const std::vector<Sample> & measurements) const = 0;

  virtual std::string name() const = 0;
};

template<typename Scalar, int Dimension>
class SavitzkyGolayMeasurementSignalFilter final
  : public MeasurementSignalFilter<Scalar, Dimension>
{
public:
  using Base = MeasurementSignalFilter<Scalar, Dimension>;
  using Sample = typename Base::Sample;
  using SignalEstimate = typename Base::SignalEstimate;

  SavitzkyGolayMeasurementSignalFilter(
    MeasurementFilterWindow window, int polyorder, std::string filter_name)
  : window_(window), polyorder_(polyorder), filter_name_(std::move(filter_name))
  {
    if (window_.before_including_current == 0) {
      throw std::invalid_argument(
              "The number of samples before must include the current sample");
    }
    if (polyorder_ < 0 || static_cast<std::size_t>(polyorder_) >= window_.size()) {
      throw std::invalid_argument(
              "Savitzky-Golay polyorder must be non-negative and smaller than "
              "the total window size");
    }

    computeWeights();
  }

  std::vector<SignalEstimate> estimateSignal(
    const std::vector<Sample> & measurements) const override
  {
    std::vector<SignalEstimate> estimates;
    if (measurements.size() < window_.size()) {
      return estimates;
    }

    estimates.reserve(measurements.size() - window_.size() + 1);
    const std::size_t first_index = window_.before_including_current - 1;
    const std::size_t end_index = measurements.size() - window_.after;
    for (std::size_t current_index = first_index; current_index < end_index;
      ++current_index)
    {
      const std::size_t window_start = current_index - first_index;
      Sample estimate = Sample::Zero();
      for (std::size_t offset = 0; offset < window_.size(); ++offset) {
        estimate.noalias() +=
          weights_(static_cast<Eigen::Index>(offset)) *
          measurements[window_start + offset];
      }
      estimates.push_back(SignalEstimate{current_index, estimate});
    }
    return estimates;
  }

  std::string name() const override
  {
    return filter_name_;
  }

private:
  void computeWeights()
  {
    const Eigen::Index window_size = static_cast<Eigen::Index>(window_.size());
    const Eigen::Index coefficient_count = static_cast<Eigen::Index>(polyorder_ + 1);
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> design_matrix(
      window_size, coefficient_count);

    const std::size_t largest_offset = std::max(
      window_.before_including_current - 1, window_.after);
    const Scalar offset_scale = static_cast<Scalar>(std::max<std::size_t>(1, largest_offset));
    for (Eigen::Index row = 0; row < window_size; ++row) {
      const auto integer_offset =
        static_cast<std::int64_t>(row) -
        static_cast<std::int64_t>(window_.before_including_current - 1);
      const Scalar x = static_cast<Scalar>(integer_offset) / offset_scale;
      design_matrix(row, 0) = Scalar(1);
      for (Eigen::Index column = 1; column < coefficient_count; ++column) {
        design_matrix(row, column) = design_matrix(row, column - 1) * x;
      }
    }

    const auto decomposition = design_matrix.colPivHouseholderQr();
    if (decomposition.rank() != coefficient_count) {
      throw std::runtime_error(
              "Savitzky-Golay design matrix is rank deficient");
    }

    const auto identity = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(
      window_size, window_size);
    const auto pseudoinverse = decomposition.solve(identity);
    weights_ = pseudoinverse.row(0).transpose();
    if (!weights_.allFinite()) {
      throw std::runtime_error(
              "Failed to compute finite Savitzky-Golay coefficients");
    }
  }

  MeasurementFilterWindow window_;
  int polyorder_;
  std::string filter_name_;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> weights_;
};

template<typename Scalar, int Dimension>
std::unique_ptr<MeasurementSignalFilter<Scalar, Dimension>>
makeMeasurementSignalFilter(const MeasurementCovarianceFilterConfig & config)
{
  if (!config.enabled) {
    return nullptr;
  }

  const bool moving_average =
    config.type == MeasurementCovarianceFilterType::MovingAverage;
  return std::make_unique<SavitzkyGolayMeasurementSignalFilter<Scalar, Dimension>>(
    config.window,
    moving_average ? 0 : config.polyorder,
    moving_average ? "moving_average" : "savitzky_golay");
}

template<typename Scalar, int Dimension>
std::vector<Eigen::Matrix<Scalar, Dimension, 1>> estimateNoiseResiduals(
  const std::vector<Eigen::Matrix<Scalar, Dimension, 1>> & measurements,
  const MeasurementSignalFilter<Scalar, Dimension> & filter)
{
  using Sample = Eigen::Matrix<Scalar, Dimension, 1>;
  const auto signal_estimates = filter.estimateSignal(measurements);
  std::vector<Sample> residuals;
  residuals.reserve(signal_estimates.size());
  for (const auto & estimate : signal_estimates) {
    residuals.push_back(measurements[estimate.sample_index] - estimate.value);
  }
  return residuals;
}

}  // namespace uclv_systems_ros
