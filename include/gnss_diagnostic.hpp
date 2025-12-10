#pragma once

#include "diagnostic_base.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <skyautonet_msgs/msg/gphdt.hpp>

#include <string>

namespace sensor_diagnostic{
class GnssDiagnostic : public DiagnosticBase{
public:
  explicit GnssDiagnostic(const rclcpp::NodeOptions & options);
  ~GnssDiagnostic() override = default;

private:
  void init_param() override;
  void init_diag() override;
  void diag_device(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_timeout(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_fix(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_data(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_jump(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_heading(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void navsat_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void heading_callback(const skyautonet_msgs::msg::Gphdt::SharedPtr msg);

  bool check_device(std::string & message);
  bool check_timeout(std::string & message);
  bool check_fix_status(std::string & message);
  bool check_data_validity(std::string & message);
  bool check_position_jump(std::string & message);
  bool check_heading_jump(std::string & message);

  double haversine(double lat1, double lon1, double lat2, double lon2) const;

  std::string device_path_;
  std::string fix_topic_;
  std::string heading_topic_;
  double fix_timeout_sec_{};
  double position_jump_threshold_{};
  double max_jump_speed_kmph_{};
  double jump_warn_hold_sec_{};
  double heading_jump_threshold_deg_{};
  double heading_warn_hold_sec_{};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<skyautonet_msgs::msg::Gphdt>::SharedPtr heading_sub_;
  rclcpp::Time last_fix_time_;
  rclcpp::Time last_jump_time_;
  rclcpp::Time last_heading_jump_time_;
  sensor_msgs::msg::NavSatFix last_fix_msg_{};
  double last_heading_deg_{0.0};
  double last_heading_jump_diff_{0.0};
  bool fix_received_{false};
  bool jump_detected_{false};
  bool heading_received_{false};
  bool heading_jump_detected_{false};
};
}// namespace sensor_diagnostic
