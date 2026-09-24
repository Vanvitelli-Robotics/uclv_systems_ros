#include <atomic>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <uclv_systems_ros/measurement_covariance_filter.hpp>

namespace
{

class TemporaryYamlFile
{
public:
  explicit TemporaryYamlFile(const std::string & contents)
  {
    static std::atomic_uint64_t counter{0};
    path_ = std::filesystem::temp_directory_path() /
      ("uclv_measurement_filter_" + std::to_string(counter++) + ".yaml");
    std::ofstream stream(path_);
    stream << contents;
    if (!stream) {
      throw std::runtime_error("Failed to create temporary YAML file");
    }
  }

  ~TemporaryYamlFile()
  {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  std::string path() const
  {
    return path_.string();
  }

private:
  std::filesystem::path path_;
};

using FilterType = uclv_systems_ros::MeasurementCovarianceFilterType;
using Sample = Eigen::Matrix<double, 1, 1>;

std::vector<Sample> makeSamples(const std::vector<double> & values)
{
  std::vector<Sample> samples;
  samples.reserve(values.size());
  for (const double value : values) {
    Sample sample;
    sample << value;
    samples.push_back(sample);
  }
  return samples;
}

TEST(MeasurementCovarianceFilterConfig, ScalarWindowIsCentered)
{
  const auto window = uclv_systems_ros::parseMeasurementFilterWindow(YAML::Load("5"));
  EXPECT_EQ(window.before_including_current, 3u);
  EXPECT_EQ(window.after, 2u);
  EXPECT_EQ(window.size(), 5u);
}

TEST(MeasurementCovarianceFilterConfig, RejectsEvenScalarWindow)
{
  EXPECT_THROW(
    uclv_systems_ros::parseMeasurementFilterWindow(YAML::Load("4")),
    std::runtime_error);
}

TEST(MeasurementCovarianceFilterConfig, SupportsAliases)
{
  const TemporaryYamlFile moving_average(
    "measurement_covariance_filter:\n"
    "  enabled: true\n"
    "  type: ma\n"
    "  window_size: [3, 2]\n");
  const TemporaryYamlFile savitzky_golay(
    "measurement_covariance_filter:\n"
    "  enabled: true\n"
    "  type: sg\n"
    "  window_size: 7\n");

  const auto moving_config =
    uclv_systems_ros::loadMeasurementCovarianceFilterConfig(moving_average.path());
  const auto sg_config =
    uclv_systems_ros::loadMeasurementCovarianceFilterConfig(savitzky_golay.path());

  EXPECT_EQ(moving_config.type, FilterType::MovingAverage);
  EXPECT_EQ(moving_config.polyorder, 0);
  EXPECT_EQ(moving_config.window.before_including_current, 3u);
  EXPECT_EQ(moving_config.window.after, 2u);
  EXPECT_EQ(sg_config.type, FilterType::SavitzkyGolay);
  EXPECT_EQ(sg_config.polyorder, 3);
}

TEST(MeasurementCovarianceFilterConfig, InfersTypeFromPolyorderPresence)
{
  const TemporaryYamlFile implicit_moving_average(
    "measurement_covariance_filter:\n"
    "  window_size: 5\n");
  const TemporaryYamlFile implicit_savitzky_golay(
    "measurement_covariance_filter:\n"
    "  window_size: 5\n"
    "  polyorder: 2\n");

  const auto moving_config =
    uclv_systems_ros::loadMeasurementCovarianceFilterConfig(
    implicit_moving_average.path());
  const auto sg_config =
    uclv_systems_ros::loadMeasurementCovarianceFilterConfig(
    implicit_savitzky_golay.path());

  EXPECT_EQ(moving_config.type, FilterType::MovingAverage);
  EXPECT_EQ(moving_config.polyorder, 0);
  EXPECT_EQ(sg_config.type, FilterType::SavitzkyGolay);
  EXPECT_EQ(sg_config.polyorder, 2);
}

TEST(MeasurementCovarianceFilterConfig, DisabledFilterNeedsNoOtherKeys)
{
  const TemporaryYamlFile yaml(
    "measurement_covariance_filter:\n"
    "  enabled: false\n");

  const auto config =
    uclv_systems_ros::loadMeasurementCovarianceFilterConfig(yaml.path());
  const auto filter =
    uclv_systems_ros::makeMeasurementSignalFilter<double, 1>(config);

  EXPECT_FALSE(config.enabled);
  EXPECT_EQ(filter, nullptr);
}

TEST(MeasurementCovarianceFilterConfig, RejectsPolyorderNotSmallerThanWindow)
{
  const TemporaryYamlFile yaml(
    "measurement_covariance_filter:\n"
    "  type: sg\n"
    "  window_size: 5\n"
    "  polyorder: 5\n");

  EXPECT_THROW(
    uclv_systems_ros::loadMeasurementCovarianceFilterConfig(yaml.path()),
    std::runtime_error);
}

TEST(MeasurementSignalFilter, MovingAverageUsesFullAsymmetricWindow)
{
  uclv_systems_ros::MeasurementCovarianceFilterConfig config;
  config.enabled = true;
  config.type = FilterType::MovingAverage;
  config.window = {2, 1};
  const auto filter =
    uclv_systems_ros::makeMeasurementSignalFilter<double, 1>(config);
  const auto estimates = filter->estimateSignal(
    makeSamples({1.0, 2.0, 6.0, 3.0, 9.0}));

  ASSERT_EQ(estimates.size(), 3u);
  EXPECT_EQ(estimates[0].sample_index, 1u);
  EXPECT_EQ(estimates[2].sample_index, 3u);
  EXPECT_NEAR(estimates[0].value(0), 3.0, 1.0e-12);
  EXPECT_NEAR(estimates[1].value(0), 11.0 / 3.0, 1.0e-12);
  EXPECT_NEAR(estimates[2].value(0), 6.0, 1.0e-12);
}

TEST(MeasurementSignalFilter, SavitzkyGolayReconstructsPolynomial)
{
  uclv_systems_ros::MeasurementCovarianceFilterConfig config;
  config.enabled = true;
  config.type = FilterType::SavitzkyGolay;
  config.window = {3, 1};
  config.polyorder = 2;
  const auto filter =
    uclv_systems_ros::makeMeasurementSignalFilter<double, 1>(config);

  std::vector<double> values;
  for (int index = 0; index < 8; ++index) {
    values.push_back(2.0 + 3.0 * index + 0.5 * index * index);
  }
  const auto samples = makeSamples(values);
  const auto estimates = filter->estimateSignal(samples);
  const auto residuals = uclv_systems_ros::estimateNoiseResiduals(samples, *filter);

  ASSERT_EQ(estimates.size(), 5u);
  ASSERT_EQ(residuals.size(), estimates.size());
  for (std::size_t index = 0; index < estimates.size(); ++index) {
    EXPECT_NEAR(
      estimates[index].value(0), values[estimates[index].sample_index], 1.0e-11);
    EXPECT_NEAR(residuals[index](0), 0.0, 1.0e-11);
  }
}

TEST(MeasurementSignalFilter, MovingAverageIsOrderZeroSavitzkyGolay)
{
  const uclv_systems_ros::MeasurementFilterWindow window{3, 2};
  uclv_systems_ros::SavitzkyGolayMeasurementSignalFilter<double, 1> moving_average(
    window, 0, "moving_average");
  uclv_systems_ros::SavitzkyGolayMeasurementSignalFilter<double, 1> sg_order_zero(
    window, 0, "savitzky_golay");
  const auto samples = makeSamples({1.0, -2.0, 4.0, 8.0, 3.0, 7.0, 0.0});

  const auto moving_estimates = moving_average.estimateSignal(samples);
  const auto sg_estimates = sg_order_zero.estimateSignal(samples);

  ASSERT_EQ(moving_estimates.size(), sg_estimates.size());
  for (std::size_t index = 0; index < moving_estimates.size(); ++index) {
    EXPECT_EQ(moving_estimates[index].sample_index, sg_estimates[index].sample_index);
    EXPECT_NEAR(moving_estimates[index].value(0), sg_estimates[index].value(0), 1.0e-12);
  }
}

}  // namespace
