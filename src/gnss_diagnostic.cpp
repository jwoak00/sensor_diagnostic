#include "gnss_diagnostic.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <unistd.h>

namespace sensor_diagnostic{
namespace {
constexpr double kEarthRadiusMeters = 6371000.0;
constexpr double kMetersPerSecondToKmph = 3.6;

double normalize_heading_deg(double heading_deg)
{
  double normalized = std::fmod(heading_deg, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  return normalized;
}

double heading_difference_deg(double current_deg, double previous_deg)
{
  const double current = normalize_heading_deg(current_deg);
  const double previous = normalize_heading_deg(previous_deg);
  double diff = std::fabs(current - previous);
  if (diff > 180.0) {
    diff = 360.0 - diff;
  }
  return diff;
}
}  // namespace

GnssDiagnostic::GnssDiagnostic(const rclcpp::NodeOptions & options)
: DiagnosticBase(options)
{
  init_param();
  init_diag();
}

void GnssDiagnostic::init_param()
{
  this->declare_parameter<std::string>("hardware_id", "HI-EDGE_Dual");
  this->declare_parameter<std::string>("sensor_name", "gnss");
  this->declare_parameter<std::string>("device_path", "/dev/ttyUSB1");
  this->declare_parameter<std::string>("fix_topic", "/sensing/gnss/fix");
  this->declare_parameter<std::string>("heading_topic", "/sensing/gnss/heading");
  this->declare_parameter<double>("fix_timeout_sec", 2.0);
  this->declare_parameter<double>("position_jump_threshold", 50.0);
  this->declare_parameter<double>("max_jump_speed_kmph", 200.0);
  this->declare_parameter<double>("jump_warn_hold_sec", 5.0);
  this->declare_parameter<double>("heading_jump_threshold_deg", 90.0);
  this->declare_parameter<double>("heading_warn_hold_sec", 5.0);

  this->get_parameter("hardware_id", hardware_id_);
  this->get_parameter("sensor_name", sensor_name_);
  this->get_parameter("device_path", device_path_);
  this->get_parameter("fix_topic", fix_topic_);
  this->get_parameter("heading_topic", heading_topic_);
  this->get_parameter("fix_timeout_sec", fix_timeout_sec_);
  this->get_parameter("position_jump_threshold", position_jump_threshold_);
  this->get_parameter("max_jump_speed_kmph", max_jump_speed_kmph_);
  this->get_parameter("jump_warn_hold_sec", jump_warn_hold_sec_);
  this->get_parameter("heading_jump_threshold_deg", heading_jump_threshold_deg_);
  this->get_parameter("heading_warn_hold_sec", heading_warn_hold_sec_);

  last_fix_time_ = this->now();
  last_jump_time_ = this->now();
  last_heading_jump_time_ = this->now();
}

void GnssDiagnostic::init_diag()
{
  diagnostic_updater_.setHardwareID(hardware_id_);
  diagnostic_updater_.add("Device", this, &GnssDiagnostic::diag_device);
  diagnostic_updater_.add("Timeout", this, &GnssDiagnostic::diag_timeout);
  diagnostic_updater_.add("Fix", this, &GnssDiagnostic::diag_fix);
  diagnostic_updater_.add("Data", this, &GnssDiagnostic::diag_data);
  diagnostic_updater_.add("Jump", this, &GnssDiagnostic::diag_jump);
  diagnostic_updater_.add("Heading", this, &GnssDiagnostic::diag_heading);

  fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    fix_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&GnssDiagnostic::navsat_callback, this, std::placeholders::_1));

  heading_sub_ = this->create_subscription<skyautonet_msgs::msg::Gphdt>(
    heading_topic_,
    rclcpp::QoS{1},
    std::bind(&GnssDiagnostic::heading_callback, this, std::placeholders::_1));
}

void GnssDiagnostic::navsat_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  const auto now = this->now();

  if (fix_received_) {
    const double time_diff_sec = (now - last_fix_time_).seconds();
    if (time_diff_sec > 0.1) {
      const double distance = haversine(last_fix_msg_.latitude, last_fix_msg_.longitude,
        msg->latitude, msg->longitude);
      const double speed_kmph = (distance / time_diff_sec) * kMetersPerSecondToKmph;

      if (max_jump_speed_kmph_ > 0.0 && speed_kmph > max_jump_speed_kmph_) {
        last_jump_time_ = now;
        jump_detected_ = true;
      }
    }
  }

  fix_received_ = true;
  last_fix_time_ = now;
  last_fix_msg_ = *msg;
}

void GnssDiagnostic::heading_callback(const skyautonet_msgs::msg::Gphdt::SharedPtr msg)
{
  if (!msg || !std::isfinite(msg->heading)) {
    return;
  }

  const double heading_deg = normalize_heading_deg(msg->heading);
  const auto now = this->now();

  if (heading_received_) {
    const double diff = heading_difference_deg(heading_deg, last_heading_deg_);
    if (heading_jump_threshold_deg_ > 0.0 && diff > heading_jump_threshold_deg_) {
      heading_jump_detected_ = true;
      last_heading_jump_diff_ = diff;
      last_heading_jump_time_ = now;
    }
  }

  heading_received_ = true;
  last_heading_deg_ = heading_deg;
}

void GnssDiagnostic::diag_device(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  diagnostics.add("Device Path", device_path_);

  std::string message;
  if (check_device(message)) {
    diagnostics.add("Device", message);
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
    return;
  }

  diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 연결");
}

void GnssDiagnostic::diag_timeout(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  diagnostics.add("Topic", fix_topic_);

  if (fix_received_) {
    diagnostics.addf("Latitude", "%.7f", last_fix_msg_.latitude);
    diagnostics.addf("Longitude", "%.7f", last_fix_msg_.longitude);
    diagnostics.addf("Altitude", "%.2f", last_fix_msg_.altitude);
  }

  std::string message;
  if (check_timeout(message)) {
    diagnostics.add("Timeout", message);
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
    return;
  }

  diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

void GnssDiagnostic::diag_fix(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (!fix_received_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "[Fix] 데이터 수신 대기 중");
    return;
  }

  std::string message;
  if (check_fix_status(message)) {
    diagnostics.add("Fix", message);
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
    return;
  }

  diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

void GnssDiagnostic::diag_data(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (!fix_received_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "[Data] 데이터 수신 대기 중");
    return;
  }

  std::string message;
  if (check_data_validity(message)) {
    diagnostics.add("Data", message);
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
    return;
  }

  diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

void GnssDiagnostic::diag_jump(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (!fix_received_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "[Jump] 데이터 수신 대기 중");
    return;
  }

  std::string message;
  if (check_position_jump(message)) {
    diagnostics.add("Jump", message);
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
    return;
  }

  diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

void GnssDiagnostic::diag_heading(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  if (!heading_received_) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "[Heading] 데이터 수신 대기 중");
    return;
  }

  diagnostics.addf("Heading(deg)", "%.2f", last_heading_deg_);

  std::string message;
  if (check_heading_jump(message)) {
    diagnostics.add("Heading", message);
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
    return;
  }

  diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

bool GnssDiagnostic::check_device(std::string & message)
{
  if (device_path_.empty()) {
    return false;
  }

  if (access(device_path_.c_str(), F_OK) != 0) {
    message = "[Device] 장치 파일을 찾을 수 없음";
    return true;
  }
  return false;
}

bool GnssDiagnostic::check_timeout(std::string & message)
{
  const auto now = this->now();
  if (!fix_received_ || (now - last_fix_time_).seconds() > fix_timeout_sec_) {
    message = "[Fix] 데이터 수신 없음";
    return true;
  }
  return false;
}

bool GnssDiagnostic::check_fix_status(std::string & message)
{
  if (!fix_received_) {
    return false;
  }

  if (last_fix_msg_.status.status < 0) {
    message = "[Fix] GNSS 상태 비정상";
    return true;
  }
  return false;
}

bool GnssDiagnostic::check_data_validity(std::string & message)
{
  if (!fix_received_) {
    return false;
  }

  if (!std::isfinite(last_fix_msg_.latitude) ||
      !std::isfinite(last_fix_msg_.longitude) ||
      !std::isfinite(last_fix_msg_.altitude)) {
    message = "[Data] 비정상 값(NaN/Inf) 포함";
    return true;
  }
  return false;
}

bool GnssDiagnostic::check_position_jump(std::string & message)
{
  if (!jump_detected_) {
    return false;
  }

  const double elapsed = (this->now() - last_jump_time_).seconds();
  if (elapsed <= jump_warn_hold_sec_) {
    message = "[Jump] 위치 점프 감지 (" + std::to_string(elapsed) + "초 경과)";
    return true;
  }

  jump_detected_ = false;
  return false;
}

bool GnssDiagnostic::check_heading_jump(std::string & message)
{
  if (!heading_jump_detected_) {
    return false;
  }

  const double elapsed = (this->now() - last_heading_jump_time_).seconds();
  if (elapsed <= heading_warn_hold_sec_) {
    message = "[Heading] 헤딩 점프 감지 (Δ" + std::to_string(last_heading_jump_diff_) +
      "deg, " + std::to_string(elapsed) + "초 경과)";
    return true;
  }

  heading_jump_detected_ = false;
  return false;
}

double GnssDiagnostic::haversine(double lat1, double lon1, double lat2, double lon2) const
{
  const double to_rad = M_PI / 180.0;
  const double dlat = (lat2 - lat1) * to_rad;
  const double dlon = (lon2 - lon1) * to_rad;
  const double a = std::pow(std::sin(dlat / 2.0), 2) +
    std::cos(lat1 * to_rad) * std::cos(lat2 * to_rad) * std::pow(std::sin(dlon / 2.0), 2);
  const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
  return kEarthRadiusMeters * c;
}

}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_diagnostic::GnssDiagnostic)
