#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Eigenvalues>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <yaml-cpp/yaml.h>

#include <uclv_eigen_ros_conversions/eigen_ros_conversions.hpp>
#include <uclv_systems_interfaces/srv/estimate_measurement_covariance.hpp>
#include <uclv_systems_lib/observers/ekf.hpp>

namespace uclv_systems_ros
{

    template <typename Scalar_t, typename InputMsgT, typename MeasurementMsgT, typename StateMsgT, int dim_state, int dim_input, int dim_measurement>
    class EKFNode : public rclcpp::Node
    {

    protected:
        using EKF_t = uclv::systems::ExtendedKalmanFilter<Scalar_t, dim_state, dim_input, dim_measurement>;

        using StateSpaceInterface_t = typename EKF_t::StateSpaceInterface_t;

        using StateNoiseCovariance_t = typename EKF_t::StateNoiseCovariance_t;
        using OutputNoiseCovariance_t = typename EKF_t::OutputNoiseCovariance_t;

    public:
        using Input_t = typename EKF_t::Input_t;
        using Output_t = typename EKF_t::Output_t;
        using State_t = typename EKF_t::State_t;
        using EstimateMeasurementCovarianceSrv_t =
            uclv_systems_interfaces::srv::EstimateMeasurementCovariance;

        EKFNode(const rclcpp::NodeOptions &options)
            : Node("ekf_node", options)
        {
            sample_time_ = this->declare_parameter<double>("sample_time", -1.0);
            run_on_measurement_ = this->declare_parameter<bool>("run_on_measurement", false);
            use_msg_timestamp_ = this->declare_parameter<bool>("use_msg_timestamp", true);
            running_.store(this->declare_parameter<bool>("start_running", false));

            std::string input_topic = "input_topic";                               // use remapping to change this topic name
            std::string measurement_topic = "measurement_topic";                   // use remapping to change this topic name
            std::string filtered_state_topic = "filtered_state_topic";             // use remapping to change this topic name
            std::string filtered_measurement_topic = "filtered_measurement_topic"; // use remapping to change this topic name

            bool publish_measurement = this->declare_parameter<bool>("publish_measurement", false);
            bool publish_state = this->declare_parameter<bool>("publish_state", true);

            if (!run_on_measurement_ && sample_time_ <= 0.0)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid sample_time %.6f for timer-based EKF", sample_time_);
                throw std::runtime_error("Invalid sample_time");
            }

            auto qos = rclcpp::SensorDataQoS();
            if (publish_state)
            {
                pub_state_ = this->create_publisher<StateMsgT>(filtered_state_topic, qos);
            }
            if (publish_measurement)
            {
                pub_measurement_ = this->create_publisher<MeasurementMsgT>(filtered_measurement_topic, qos);
            }

            if (dim_input > 0)
            {
                sub_input_ = this->create_subscription<InputMsgT>(
                    input_topic, qos, std::bind(&EKFNode::inputCallback, this, std::placeholders::_1));
            }
            sub_measurement_ = this->create_subscription<MeasurementMsgT>(
                measurement_topic, qos, std::bind(&EKFNode::measurementCallback, this, std::placeholders::_1));

            srv_start_ = this->create_service<std_srvs::srv::SetBool>(
                "~/start", std::bind(&EKFNode::startCallback, this, std::placeholders::_1, std::placeholders::_2));

            srv_estimate_measurement_covariance_ =
                this->create_service<EstimateMeasurementCovarianceSrv_t>(
                    "~/estimate_measurement_covariance",
                    [this](
                        const std::shared_ptr<rclcpp::Service<EstimateMeasurementCovarianceSrv_t>> service,
                        const std::shared_ptr<rmw_request_id_t> request_header,
                        const std::shared_ptr<typename EstimateMeasurementCovarianceSrv_t::Request> request)
                    {
                        estimateMeasurementCovarianceCallback(service, request_header, request);
                    });

            if (!run_on_measurement_)
            {
                timer_ = rclcpp::create_timer(
                    this,
                    this->get_clock(),
                    std::chrono::duration<double>(sample_time_),
                    std::bind(&EKFNode::processLatestData, this));

                if (!running_.load())
                {
                    timer_->cancel();
                }
            }
        }

    protected:
        // to be implemented by derived classes to create the system model for the EKF
        virtual typename StateSpaceInterface_t::SharedPtr make_system() = 0;

        void init_ekf()
        {
            process_noise_covariance_configured_ = false;
            measurement_noise_covariance_configured_ = false;

            auto process_noise_diagonal =
                this->declare_parameter<std::vector<double>>(
                    "process_noise_diagonal", std::vector<double>{});
            auto measurement_noise_diagonal = this->declare_parameter<std::vector<double>>(
                "measurement_noise_diagonal", std::vector<double>{});
            const auto process_noise_covariance_file =
                this->declare_parameter<std::string>("process_noise_covariance_file", "");
            const auto measurement_noise_covariance_file =
                this->declare_parameter<std::string>("measurement_noise_covariance_file", "");

            StateNoiseCovariance_t W = StateNoiseCovariance_t::Zero();
            OutputNoiseCovariance_t V = OutputNoiseCovariance_t::Zero();

            if (!process_noise_covariance_file.empty())
            {
                loadCovarianceFromYaml(
                    process_noise_covariance_file, "process_noise_covariance", W);
                process_noise_covariance_configured_ = true;
                RCLCPP_INFO(
                    this->get_logger(), "Loaded process-noise covariance from %s",
                    process_noise_covariance_file.c_str());
            }
            else if (!process_noise_diagonal.empty())
            {
                setCovarianceFromDiagonal(
                    process_noise_diagonal, "process_noise_diagonal", W);
                process_noise_covariance_configured_ = true;
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Process-noise covariance is not configured: set "
                    "'process_noise_diagonal' or 'process_noise_covariance_file'");
            }

            if (!measurement_noise_covariance_file.empty())
            {
                loadCovarianceFromYaml(
                    measurement_noise_covariance_file, "measurement_noise_covariance", V);
                measurement_noise_covariance_configured_ = true;
                RCLCPP_INFO(
                    this->get_logger(), "Loaded measurement-noise covariance from %s",
                    measurement_noise_covariance_file.c_str());
            }
            else if (!measurement_noise_diagonal.empty())
            {
                setCovarianceFromDiagonal(
                    measurement_noise_diagonal, "measurement_noise_diagonal", V);
                measurement_noise_covariance_configured_ = true;
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Measurement-noise covariance is not configured: set "
                    "'measurement_noise_diagonal' or 'measurement_noise_covariance_file'");
            }

            initializeEkf(W, V);
            updateCovarianceReadiness();
        }

        void init_ekf(StateNoiseCovariance_t W, OutputNoiseCovariance_t V)
        {
            validateCovarianceMatrix(W, "Process-noise covariance");
            validateCovarianceMatrix(V, "Measurement-noise covariance");
            process_noise_covariance_configured_ = true;
            measurement_noise_covariance_configured_ = true;
            initializeEkf(W, V);
            updateCovarianceReadiness();
        }

        void initializeEkf(StateNoiseCovariance_t W, OutputNoiseCovariance_t V)
        {
            auto system = make_system();
            if (!system)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to create system model for EKF");
                throw std::runtime_error("Failed to create system model for EKF");
            }
            ekf_ = std::make_shared<EKF_t>(system, W, V);
            ekf_->reset();
        }

        void updateCovarianceReadiness()
        {
            const bool covariances_configured =
                process_noise_covariance_configured_ &&
                measurement_noise_covariance_configured_;
            covariances_configured_.store(covariances_configured);

            if (!covariances_configured)
            {
                running_.store(false);
                if (timer_)
                {
                    timer_->cancel();
                }
                RCLCPP_WARN(
                    this->get_logger(),
                    "EKF start is disabled until both process-noise and "
                    "measurement-noise covariances are configured");
            }
        }

        void inputCallback(const typename InputMsgT::ConstSharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            last_input_msg_ = msg;
        }

        void measurementCallback(const typename MeasurementMsgT::ConstSharedPtr msg)
        {
            recordMeasurementForCovarianceEstimation(msg);

            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_measurement_msg_ = msg;
            }

            if (run_on_measurement_)
            {
                processLatestData();
            }
        }

        void processLatestData()
        {

            if (!running_.load())
            {
                return;
            }

            if (!ekf_)
            {
                RCLCPP_ERROR(this->get_logger(), "EKF has not been initialized");
                return;
            }

            typename InputMsgT::ConstSharedPtr input_msg;
            typename MeasurementMsgT::ConstSharedPtr measurement_msg;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                input_msg = last_input_msg_;
                measurement_msg = last_measurement_msg_;
            }

            if ((dim_input > 0 && !input_msg) || !measurement_msg)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "EKF waiting for input and/or measurement messages");
                return;
            }

            Input_t input;
            input.setZero();
            if (dim_input > 0)
            {
                uclv::ros::conversions::convert(*input_msg, input);
            }
            Output_t measurement;
            uclv::ros::conversions::convert(*measurement_msg, measurement);

            ekf_->obs_apply(input, measurement);

            // state conversion to message
            typename StateMsgT::UniquePtr filtered_state_msg = std::make_unique<StateMsgT>();
            // convert from eigen to msg, use msg_in as template to copy non-eigen fields
            uclv::ros::conversions::convert(ekf_->get_state(), *filtered_state_msg);
            if (use_msg_timestamp_)
            {
                uclv::ros::conversions::copy_extra_fields(*measurement_msg, *filtered_state_msg);
            }
            else
            {
                uclv::ros::conversions::copy_extra_fields(*measurement_msg, *filtered_state_msg, this->now());
            }

            // measurement conversion to message
            typename MeasurementMsgT::UniquePtr measurement_msg_out = std::make_unique<MeasurementMsgT>();
            uclv::ros::conversions::convert(ekf_->get_output(), *measurement_msg_out);
            if (use_msg_timestamp_)
            {
                uclv::ros::conversions::copy_extra_fields(*measurement_msg, *measurement_msg_out);
            }
            else
            {
                uclv::ros::conversions::copy_extra_fields(*measurement_msg, *measurement_msg_out, this->now());
            }

            if (pub_state_)
            {
                pub_state_->publish(std::move(filtered_state_msg));
            }
            if (pub_measurement_)
            {
                pub_measurement_->publish(std::move(measurement_msg_out));
            }
        }

        void recordMeasurementForCovarianceEstimation(
            const typename MeasurementMsgT::ConstSharedPtr &msg)
        {
            if (!covariance_recording_active_.load())
            {
                return;
            }

            Output_t measurement;
            measurement.setZero();
            try
            {
                uclv::ros::conversions::convert(*msg, measurement);
            }
            catch (const std::exception &exception)
            {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Failed to convert a measurement for covariance estimation: %s",
                    exception.what());
                return;
            }

            std::lock_guard<std::mutex> lock(covariance_samples_mutex_);
            if (covariance_recording_active_.load())
            {
                covariance_samples_.push_back(measurement);
            }
        }

        void estimateMeasurementCovarianceCallback(
            const std::shared_ptr<rclcpp::Service<EstimateMeasurementCovarianceSrv_t>> service,
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<typename EstimateMeasurementCovarianceSrv_t::Request> request)
        {
            auto response = std::make_shared<typename EstimateMeasurementCovarianceSrv_t::Response>();
            response->dimension = static_cast<std::uint32_t>(dim_measurement);

            const auto reject_request =
                [this, &service, &request_header, &response](const std::string &message)
                {
                    response->success = false;
                    response->message = message;
                    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
                    sendCovarianceEstimationResponse(service, request_header, response);
                };

            if (request->recording_duration.sec < 0 ||
                request->recording_duration.nanosec >= 1000000000u)
            {
                reject_request("recording_duration must be a valid, positive duration");
                return;
            }

            const rclcpp::Duration recording_duration(request->recording_duration);
            if (recording_duration.nanoseconds() <= 0)
            {
                reject_request("recording_duration must be greater than zero");
                return;
            }

            try
            {
                validateCovarianceOutputPath(request->output_path, request->overwrite);
            }
            catch (const std::exception &exception)
            {
                reject_request(exception.what());
                return;
            }

            std::string state_error;
            {
                std::scoped_lock lock(operation_mutex_, covariance_samples_mutex_);
                if (running_.load())
                {
                    state_error = "Measurement covariance estimation is available only while the EKF is stopped";
                }
                else if (covariance_estimation_active_.load())
                {
                    state_error = "A measurement covariance estimation is already active";
                }
                else
                {
                    covariance_samples_.clear();
                    covariance_estimation_active_.store(true);
                    covariance_recording_active_.store(true);
                }
            }

            if (!state_error.empty())
            {
                reject_request(state_error);
                return;
            }

            try
            {
                covariance_estimation_timer_ = this->create_wall_timer(
                    recording_duration.to_chrono<std::chrono::nanoseconds>(),
                    [this, service, request_header,
                     output_path = request->output_path,
                     overwrite = request->overwrite,
                     diagonal_only = request->diagonal_only]()
                    {
                        finishMeasurementCovarianceEstimation(
                            service, request_header, output_path, overwrite,
                            diagonal_only);
                    });
            }
            catch (const std::exception &exception)
            {
                {
                    std::scoped_lock lock(operation_mutex_, covariance_samples_mutex_);
                    covariance_recording_active_.store(false);
                    covariance_estimation_active_.store(false);
                    covariance_samples_.clear();
                }
                reject_request(
                    std::string("Failed to start covariance recording timer: ") + exception.what());
                return;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Recording measurements for covariance estimation for %.9f seconds",
                recording_duration.seconds());
        }

        void finishMeasurementCovarianceEstimation(
            const std::shared_ptr<rclcpp::Service<EstimateMeasurementCovarianceSrv_t>> service,
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::string &output_path,
            const bool overwrite,
            const bool diagonal_only)
        {
            if (!covariance_recording_active_.exchange(false))
            {
                return;
            }

            if (covariance_estimation_timer_)
            {
                covariance_estimation_timer_->cancel();
            }

            std::vector<Output_t> samples;
            {
                std::lock_guard<std::mutex> lock(covariance_samples_mutex_);
                samples.swap(covariance_samples_);
            }

            auto response = std::make_shared<typename EstimateMeasurementCovarianceSrv_t::Response>();
            response->dimension = static_cast<std::uint32_t>(dim_measurement);

            if (samples.size() < 2)
            {
                response->success = false;
                response->message =
                    "At least two measurement samples are required; received " +
                    std::to_string(samples.size());
                finishCovarianceEstimationOperation();
                sendCovarianceEstimationResponse(service, request_header, response);
                return;
            }

            Output_t mean = Output_t::Zero();
            for (const auto &sample : samples)
            {
                mean += sample;
            }
            mean /= static_cast<Scalar_t>(samples.size());

            OutputNoiseCovariance_t covariance = OutputNoiseCovariance_t::Zero();
            for (const auto &sample : samples)
            {
                const Output_t centered = sample - mean;
                covariance.noalias() += centered * centered.transpose();
            }
            covariance /= static_cast<Scalar_t>(samples.size() - 1u);
            covariance = (covariance + covariance.transpose()).eval() * Scalar_t(0.5);
            if (diagonal_only)
            {
                const Output_t covariance_diagonal = covariance.diagonal();
                covariance = covariance_diagonal.asDiagonal();
            }

            response->covariance.reserve(
                static_cast<std::size_t>(dim_measurement * dim_measurement));
            for (int row = 0; row < dim_measurement; ++row)
            {
                for (int column = 0; column < dim_measurement; ++column)
                {
                    response->covariance.push_back(
                        static_cast<double>(covariance(row, column)));
                }
            }

            try
            {
                if (!output_path.empty())
                {
                    saveCovarianceToYaml(
                        output_path, "measurement_noise_covariance", covariance, overwrite);
                }

                response->success = true;
                response->message =
                    std::string("Estimated ") +
                    (diagonal_only ? "diagonal " : "full ") +
                    "measurement covariance from " +
                    std::to_string(samples.size()) + " samples";
                if (!output_path.empty())
                {
                    response->message += " and saved it to " + output_path;
                }
            }
            catch (const std::exception &exception)
            {
                response->success = false;
                response->message =
                    std::string("Covariance computed, but saving failed: ") + exception.what();
            }

            finishCovarianceEstimationOperation();
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
            sendCovarianceEstimationResponse(service, request_header, response);
        }

        void finishCovarianceEstimationOperation()
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            covariance_estimation_active_.store(false);
        }

        void sendCovarianceEstimationResponse(
            const std::shared_ptr<rclcpp::Service<EstimateMeasurementCovarianceSrv_t>> &service,
            const std::shared_ptr<rmw_request_id_t> &request_header,
            const std::shared_ptr<typename EstimateMeasurementCovarianceSrv_t::Response> &response)
        {
            try
            {
                service->send_response(*request_header, *response);
            }
            catch (const std::exception &exception)
            {
                RCLCPP_ERROR(
                    this->get_logger(), "Failed to send covariance estimation response: %s",
                    exception.what());
            }
        }

        static void validateCovarianceOutputPath(
            const std::string &output_path, const bool overwrite)
        {
            if (output_path.empty())
            {
                return;
            }

            const std::filesystem::path path(output_path);
            if (std::filesystem::exists(path))
            {
                if (std::filesystem::is_directory(path))
                {
                    throw std::runtime_error(
                        "Covariance output path is a directory: " + output_path);
                }
                if (!overwrite)
                {
                    throw std::runtime_error(
                        "Covariance output file already exists and overwrite is false: " +
                        output_path);
                }
            }

            const auto parent = path.parent_path();
            if (!parent.empty() &&
                (!std::filesystem::exists(parent) || !std::filesystem::is_directory(parent)))
            {
                throw std::runtime_error(
                    "Covariance output directory does not exist: " + parent.string());
            }
        }

        template <typename MatrixT>
        static void setCovarianceFromDiagonal(
            const std::vector<double> &diagonal,
            const std::string &parameter_name,
            MatrixT &matrix)
        {
            if (diagonal.size() != static_cast<std::size_t>(matrix.rows()))
            {
                throw std::runtime_error(
                    "Parameter '" + parameter_name + "' must contain exactly " +
                    std::to_string(matrix.rows()) + " coefficients");
            }

            matrix.setZero();
            for (Eigen::Index index = 0; index < matrix.rows(); ++index)
            {
                matrix(index, index) =
                    static_cast<Scalar_t>(diagonal[static_cast<std::size_t>(index)]);
            }
            validateCovarianceMatrix(matrix, "Covariance from '" + parameter_name + "'");
        }

        template <typename MatrixT>
        static void validateCovarianceMatrix(
            const MatrixT &matrix, const std::string &description)
        {
            if (!matrix.allFinite())
            {
                throw std::runtime_error(description + " contains NaN or Inf");
            }

            const Scalar_t scale =
                std::max(Scalar_t(1), matrix.cwiseAbs().maxCoeff());
            const Scalar_t tolerance =
                Scalar_t(100) * std::numeric_limits<Scalar_t>::epsilon() * scale;
            const Scalar_t symmetry_error =
                (matrix - matrix.transpose()).cwiseAbs().maxCoeff();
            if (symmetry_error > tolerance)
            {
                throw std::runtime_error(description + " is not symmetric");
            }

            Eigen::SelfAdjointEigenSolver<MatrixT> eigen_solver(
                matrix, Eigen::EigenvaluesOnly);
            if (eigen_solver.info() != Eigen::Success)
            {
                throw std::runtime_error(
                    "Failed to compute the eigenvalues of " + description);
            }

            const Scalar_t positive_semidefinite_tolerance =
                tolerance * static_cast<Scalar_t>(matrix.rows());
            if (eigen_solver.eigenvalues().minCoeff() < -positive_semidefinite_tolerance)
            {
                throw std::runtime_error(description + " is not positive semidefinite");
            }
        }

        template <typename MatrixT>
        static void loadCovarianceFromYaml(
            const std::string &file_path,
            const std::string &key,
            MatrixT &matrix)
        {
            try
            {
                const YAML::Node document = YAML::LoadFile(file_path);
                if (!document.IsMap())
                {
                    throw std::runtime_error(
                        "YAML root must be a map in file " + file_path);
                }

                const YAML::Node matrix_node = document[key];
                if (!matrix_node)
                {
                    throw std::runtime_error(
                        "Missing YAML key '" + key + "' in file " + file_path);
                }
                if (!matrix_node.IsSequence())
                {
                    throw std::runtime_error(
                        "YAML key '" + key + "' must be either a diagonal vector "
                        "or a full matrix");
                }

                matrix.setZero();

                const bool is_diagonal_vector = std::all_of(
                    matrix_node.begin(), matrix_node.end(),
                    [](const YAML::Node &entry)
                    { return entry.IsScalar(); });
                const bool is_full_matrix = std::all_of(
                    matrix_node.begin(), matrix_node.end(),
                    [](const YAML::Node &entry)
                    { return entry.IsSequence(); });

                if (is_diagonal_vector)
                {
                    if (matrix_node.size() != static_cast<std::size_t>(matrix.rows()))
                    {
                        throw std::runtime_error(
                            "Diagonal vector in YAML key '" + key +
                            "' must contain exactly " +
                            std::to_string(matrix.rows()) + " values");
                    }

                    for (Eigen::Index index = 0; index < matrix.rows(); ++index)
                    {
                        matrix(index, index) =
                            matrix_node[static_cast<std::size_t>(index)]
                                .template as<Scalar_t>();
                    }
                }
                else if (is_full_matrix)
                {
                    if (matrix_node.size() != static_cast<std::size_t>(matrix.rows()))
                    {
                        throw std::runtime_error(
                            "Full matrix in YAML key '" + key +
                            "' must contain exactly " +
                            std::to_string(matrix.rows()) + " rows");
                    }

                    for (Eigen::Index row = 0; row < matrix.rows(); ++row)
                    {
                        const YAML::Node row_node =
                            matrix_node[static_cast<std::size_t>(row)];
                        if (row_node.size() != static_cast<std::size_t>(matrix.cols()))
                        {
                            throw std::runtime_error(
                                "Row " + std::to_string(row + 1) +
                                " of YAML key '" + key +
                                "' must contain exactly " +
                                std::to_string(matrix.cols()) + " columns");
                        }

                        for (Eigen::Index column = 0; column < matrix.cols(); ++column)
                        {
                            matrix(row, column) =
                                row_node[static_cast<std::size_t>(column)]
                                    .template as<Scalar_t>();
                        }
                    }
                }
                else
                {
                    throw std::runtime_error(
                        "YAML key '" + key + "' mixes scalar values and matrix rows; "
                        "use either a diagonal vector or a full matrix");
                }

                validateCovarianceMatrix(matrix, "Covariance matrix '" + key + "'");
            }
            catch (const YAML::Exception &exception)
            {
                throw std::runtime_error(
                    "Failed to read covariance YAML file " + file_path + ": " +
                    exception.what());
            }
        }

        template <typename MatrixT>
        static void saveCovarianceToYaml(
            const std::string &file_path,
            const std::string &key,
            const MatrixT &matrix,
            const bool overwrite)
        {
            validateCovarianceOutputPath(file_path, overwrite);

            YAML::Node document(YAML::NodeType::Map);
            if (std::filesystem::exists(file_path))
            {
                try
                {
                    document = YAML::LoadFile(file_path);
                }
                catch (const YAML::Exception &exception)
                {
                    throw std::runtime_error(
                        "Failed to read existing YAML file " + file_path + ": " +
                        exception.what());
                }

                if (!document.IsMap())
                {
                    throw std::runtime_error(
                        "YAML root must be a map in existing file " + file_path);
                }
            }

            YAML::Node matrix_node(YAML::NodeType::Sequence);
            for (Eigen::Index row = 0; row < matrix.rows(); ++row)
            {
                YAML::Node row_node(YAML::NodeType::Sequence);
                row_node.SetStyle(YAML::EmitterStyle::Flow);
                for (Eigen::Index column = 0; column < matrix.cols(); ++column)
                {
                    row_node.push_back(static_cast<double>(matrix(row, column)));
                }
                matrix_node.push_back(row_node);
            }
            document[key] = matrix_node;

            YAML::Emitter emitter;
            emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
            emitter << document;
            if (!emitter.good())
            {
                throw std::runtime_error(
                    "Failed to serialize covariance YAML: " + emitter.GetLastError());
            }

            std::ofstream output(file_path, std::ios::out | std::ios::trunc);
            if (!output.is_open())
            {
                throw std::runtime_error(
                    "Failed to open covariance output file: " + file_path);
            }
            output << emitter.c_str() << '\n';
            output.close();
            if (!output)
            {
                throw std::runtime_error(
                    "Failed to write covariance output file: " + file_path);
            }
        }

        void startCallback(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response)
        {
            {
                std::lock_guard<std::mutex> lock(operation_mutex_);
                if (request->data && covariance_estimation_active_.load())
                {
                    response->success = false;
                    response->message =
                        "Cannot start the EKF while measurement covariance estimation is active";
                    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                    return;
                }
                if (request->data && !covariances_configured_.load())
                {
                    response->success = false;
                    response->message =
                        "Cannot start the EKF: both process-noise and "
                        "measurement-noise covariances must be configured";
                    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
                    return;
                }
                running_.store(request->data);
            }

            if (run_on_measurement_)
            {
                response->success = true;
                response->message = request->data
                    ? "Processing on measurement enabled"
                    : "Processing on measurement disabled";
                RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
                return;
            }

            if (request->data)
            {
                timer_->reset();
                RCLCPP_INFO_STREAM(this->get_logger(), "EKF timer started with sample time: " << sample_time_ << " seconds");
                response->message = "EKF started with timer";
            }
            else
            {
                timer_->cancel();
                RCLCPP_INFO_STREAM(this->get_logger(), "EKF timer stopped");
                response->message = "EKF stopped";
            }
            response->success = true;
        }

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<rclcpp::Subscription<InputMsgT>> sub_input_;
        std::shared_ptr<rclcpp::Subscription<MeasurementMsgT>> sub_measurement_;
        std::shared_ptr<rclcpp::Publisher<StateMsgT>> pub_state_;
        std::shared_ptr<rclcpp::Publisher<MeasurementMsgT>> pub_measurement_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_start_;
        typename rclcpp::Service<EstimateMeasurementCovarianceSrv_t>::SharedPtr
            srv_estimate_measurement_covariance_;
        rclcpp::TimerBase::SharedPtr covariance_estimation_timer_;

        std::atomic_bool running_{false};
        std::atomic_bool covariances_configured_{false};
        std::atomic_bool covariance_estimation_active_{false};
        std::atomic_bool covariance_recording_active_{false};
        bool process_noise_covariance_configured_{false};
        bool measurement_noise_covariance_configured_{false};
        bool run_on_measurement_{false};
        bool use_msg_timestamp_{true};
        double sample_time_{-1.0};

        std::mutex operation_mutex_;
        std::mutex data_mutex_;
        typename InputMsgT::ConstSharedPtr last_input_msg_;
        typename MeasurementMsgT::ConstSharedPtr last_measurement_msg_;

        std::mutex covariance_samples_mutex_;
        std::vector<Output_t> covariance_samples_;

        typename EKF_t::SharedPtr ekf_;
    };

} // namespace uclv_systems_ros
