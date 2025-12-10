#include "lidar_diagnostic.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace sensor_diagnostic{

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;
}

LidarDiagnostic::LidarDiagnostic(const rclcpp::NodeOptions & options)
: DiagnosticBase(options)
{
  init_param();
  init_diag();
}

LidarDiagnostic::~LidarDiagnostic()
{
  stop_ping_thread_.store(true);
  if (ping_thread_.joinable()) {
    ping_thread_.join();
  }
}

void LidarDiagnostic::init_param()
{
  this->declare_parameter<std::string>("hardware_id", "Leishen_C32");
  this->declare_parameter<std::string>("sensor_name", "left_lidar");
  this->declare_parameter<std::string>("target_ip", "192.168.0.201");
  this->declare_parameter<double>("cloud_timeout_sec", 5.0);
  this->declare_parameter<double>("min_valid_intensity", 3.0);
  this->declare_parameter<double>("low_intensity_ratio_threshold", 0.3);
  this->declare_parameter<int>("sector_divisions", 24);
  this->declare_parameter<int>("min_points_per_sector", 20);
  this->declare_parameter<int>("max_empty_sectors", 3);
  this->declare_parameter<int>("ring_upper_start", 24);
  this->declare_parameter<int>("ring_upper_end", 31);
  this->declare_parameter<int>("ring_min_points", 1000);
  this->declare_parameter<double>("ring_min_intensity", 5.0);

  this->get_parameter("hardware_id", hardware_id_);
  this->get_parameter("sensor_name", sensor_name_);
  this->get_parameter("target_ip", target_ip_);
  this->get_parameter("cloud_timeout_sec", cloud_timeout_sec_);
  this->get_parameter("min_valid_intensity", min_valid_intensity_);
  this->get_parameter("low_intensity_ratio_threshold", low_intensity_ratio_threshold_);
  this->get_parameter("sector_divisions", sector_divisions_);
  this->get_parameter("min_points_per_sector", min_points_per_sector_);
  this->get_parameter("max_empty_sectors", max_empty_sectors_);
  this->get_parameter("ring_upper_start", ring_upper_start_);
  this->get_parameter("ring_upper_end", ring_upper_end_);
  this->get_parameter("ring_min_points", ring_min_points_);
  this->get_parameter("ring_min_intensity", ring_min_intensity_);

  last_cloud_time_ = this->now();
}

void LidarDiagnostic::init_diag()
{
  diagnostic_updater_.setHardwareID(hardware_id_);
  diagnostic_updater_.add("Timeout", this, &LidarDiagnostic::diag_timeout);
  diagnostic_updater_.add("Point Distribution", this, &LidarDiagnostic::diag_point_distribution);
  diagnostic_updater_.add("Ring", this, &LidarDiagnostic::diag_ring);

  const std::string cloud_topic = "/sensing/lidar/" + sensor_name_ + "/pointcloud";
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      last_cloud_time_ = this->now();
      last_cloud_msg_ = msg;
    });

  start_ping_thread();
}

void LidarDiagnostic::start_ping_thread()
{
  if (target_ip_.empty()) {
    ping_success_.store(false);
    latest_latency_ms_.store(-1.0f);
    return;
  }

  ping_thread_ = std::thread([this]() {
    while (rclcpp::ok() && !stop_ping_thread_.load()) {
      float latency_ms = -1.0f;
      const bool success = perform_ping(latency_ms);
      ping_success_.store(success);
      latest_latency_ms_.store(success ? latency_ms : -1.0f);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });
}

bool LidarDiagnostic::perform_ping(float & latency_ms)
{
  if (target_ip_.empty()) {
    return false;
  }

  std::string cmd = "ping -c 1 -W 1 " + target_ip_;
  std::array<char, 256> buffer{};
  std::string result;
  FILE * pipe = popen(cmd.c_str(), "r");
  if (pipe == nullptr) {
    return false;
  }

  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    result += buffer.data();
  }
  pclose(pipe);

  std::smatch match;
  std::regex rtt_regex("time=([0-9.]+) ms");
  if (std::regex_search(result, match, rtt_regex)) {
    latency_ms = std::stof(match[1].str());
    return true;
  }
  return false;
}

void LidarDiagnostic::diag_timeout(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  diagnostics.add("Target IP", target_ip_);
  const float latency_ms = latest_latency_ms_.load();
  if (latency_ms >= 0.0f) {
    diagnostics.addf("Ping Latency (ms)", "%.2f", static_cast<double>(latency_ms));
  }

  if (!ping_success_.load()) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "[Network] ping 응답 없음");
    return;
  }

  double elapsed = 0.0;
  if (check_timeout(elapsed)) {
    diagnostics.addf("Cloud Timeout (s)", "%.2f", elapsed);
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "[Pointcloud] 수신 지연");
    return;
  }
  diagnostics.addf("Cloud Age (s)", "%.2f", elapsed);
  diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

void LidarDiagnostic::diag_point_distribution(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud = last_cloud_msg_;
  }

  if (!cloud) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "[Pointcloud] 데이터 수신 대기 중");
    return;
  }

  std::string message;
  const uint8_t level = check_point_distribution(*cloud, message);
  if (!message.empty()) {
    diagnostics.add("Point Distribution", message);
  }

  if (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    diagnostics.summary(level, message.empty() ? "[Pointcloud] 포인트 수 없음" : message);
  } else if (level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    diagnostics.summary(level, message.empty() ? "[Pointcloud] 주의 필요" : message);
  } else {
    diagnostics.summary(level, "정상 수신 중");
  }
}

void LidarDiagnostic::diag_ring(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud = last_cloud_msg_;
  }

  if (!cloud) {
    diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "[Pointcloud] 데이터 수신 대기 중");
    return;
  }

  std::string message;
  const uint8_t level = check_ring(*cloud, message);
  if (!message.empty()) {
    diagnostics.add("Ring", message);
  }

  if (level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    diagnostics.summary(level, message.empty() ? "[Ring] 오류" : message);
  } else if (level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
    diagnostics.summary(level, message.empty() ? "[Ring] 주의 필요" : message);
  } else {
    diagnostics.summary(level, "정상 수신 중");
  }
}

bool LidarDiagnostic::check_timeout(double & elapsed)
{
  const auto now = this->now();
  elapsed = (now - last_cloud_time_).seconds();
  return elapsed > cloud_timeout_sec_;
}

uint8_t LidarDiagnostic::check_point_distribution(const sensor_msgs::msg::PointCloud2 & cloud, std::string & message)
{
  const size_t point_count = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  if (point_count == 0) {
    message = "[Pointcloud] 포인트 수 없음";
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  bool all_zero = true;
  bool has_invalid = false;
  std::vector<int> sector_counts(static_cast<size_t>(std::max(sector_divisions_, 1)), 0);
  const float sector_angle = 360.0f / static_cast<float>(std::max(sector_divisions_, 1));

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      has_invalid = true;
      continue;
    }

    if (x != 0.0f || y != 0.0f || z != 0.0f) {
      all_zero = false;
    }

  float angle_deg = std::atan2(y, x) * kRadToDeg;
    if (angle_deg < 0.0f) {
      angle_deg += 360.0f;
    }
    const int sector_idx = static_cast<int>(angle_deg / sector_angle);
    if (sector_idx >= 0 && sector_idx < sector_divisions_) {
      sector_counts[static_cast<size_t>(sector_idx)]++;
    }
  }

  if (has_invalid) {
    message = "[Pointcloud] 유효하지 않은 값 존재";
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  if (all_zero) {
    message = "[Pointcloud] 좌표 0만 존재";
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  const int empty_sectors = std::count_if(
    sector_counts.begin(), sector_counts.end(),
    [this](int count) { return count < min_points_per_sector_; });

  if (empty_sectors > max_empty_sectors_) {
    message = "[Sector] 빈 시야 영역 수 초과: " + std::to_string(empty_sectors) +
      " > " + std::to_string(max_empty_sectors_);
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  message.clear();
  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

uint8_t LidarDiagnostic::check_ring(const sensor_msgs::msg::PointCloud2 & cloud, std::string & message)
{
  const bool has_ring_field = std::any_of(
    cloud.fields.begin(), cloud.fields.end(),
    [](const auto & field) { return field.name == "ring"; });

  if (!has_ring_field) {
    message.clear();
    return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  const size_t point_count = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  std::vector<int> ring_counts(32, 0);
  std::vector<float> ring_intensity(32, 0.0f);

  sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(cloud, "ring");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(cloud, "intensity");

  for (size_t i = 0;
       i < point_count && iter_ring != iter_ring.end() && iter_intensity != iter_intensity.end();
       ++i, ++iter_ring, ++iter_intensity)
  {
    const int ring = static_cast<int>(*iter_ring);
    if (ring < static_cast<int>(ring_counts.size())) {
      ring_counts[static_cast<size_t>(ring)]++;
      ring_intensity[static_cast<size_t>(ring)] += static_cast<float>(*iter_intensity);
    }
  }

  int upper_count = 0;
  float upper_sum = 0.0f;
  for (int ring = ring_upper_start_; ring <= ring_upper_end_ && ring < static_cast<int>(ring_counts.size()); ++ring) {
    upper_count += ring_counts[static_cast<size_t>(ring)];
    upper_sum += ring_intensity[static_cast<size_t>(ring)];
  }

  const float upper_avg = (upper_count > 0) ? (upper_sum / static_cast<float>(upper_count)) : 0.0f;

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (upper_count < ring_min_points_) {
    message = "[Ring] 상단 ring 개수 부족: " + std::to_string(upper_count) +
      " < " + std::to_string(ring_min_points_);
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  if (upper_avg < ring_min_intensity_) {
    std::string intensity_msg = "[Ring] 상단 ring intensity 평균: " + std::to_string(upper_avg) +
      " < " + std::to_string(ring_min_intensity_);
    if (message.empty()) {
      message = intensity_msg;
    } else {
      message += ", " + intensity_msg;
    }
    level = std::max(level, static_cast<uint8_t>(diagnostic_msgs::msg::DiagnosticStatus::WARN));
  }

  if (level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    message.clear();
  }
  return level;
}

}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_diagnostic::LidarDiagnostic)