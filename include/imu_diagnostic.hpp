#pragma once

#include "diagnostic_base.hpp"

#include <deque>
#include <string>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace sensor_diagnostic{
class ImuDiagnostic : public DiagnosticBase{
public:
  explicit ImuDiagnostic(const rclcpp::NodeOptions & options);
  ~ImuDiagnostic() override = default;
  
private:
  void init_param() override;
  void init_diag() override;
  void diag_device(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_timeout(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_frequency(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_data(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_outlier(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_stuck(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  bool check_device(std::string & message);
  bool check_timeout(std::string & message);
  bool check_frequency(double & frequency, std::string & message);
  bool check_data_validity(std::string & message);
  bool check_outlier(std::string & message);
  bool check_stuck_sensor(std::string & message);

  bool is_valid_double(double value) const;

  struct AccelSample {
    rclcpp::Time stamp;
    geometry_msgs::msg::Vector3 acc;
  };

  std::string device_path_;
  std::string imu_topic_;
  double imu_timeout_sec_{};
  double min_expected_frequency_{};
  double accel_threshold_{};
  double gyro_threshold_{};
  double frequency_window_sec_{1.0};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Time last_imu_time_;
  sensor_msgs::msg::Imu last_imu_msg_{};

  std::deque<rclcpp::Time> imu_timestamps_;
  std::deque<AccelSample> accel_history_;

  bool is_connected_{false};
};
}// namespace sensor_diagnostic