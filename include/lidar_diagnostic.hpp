#pragma once

#include "diagnostic_base.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sensor_diagnostic{
class LidarDiagnostic : public DiagnosticBase{
public:
  explicit LidarDiagnostic(const rclcpp::NodeOptions & options);
  ~LidarDiagnostic() override;
  
private:
  void init_param() override;
  void init_diag() override;
  void diag_timeout(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_point_distribution(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_ring(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void start_ping_thread();
  bool perform_ping(float & latency_ms);
  bool check_timeout(double & elapsed);
  uint8_t check_point_distribution(const sensor_msgs::msg::PointCloud2 & cloud, std::string & message);
  uint8_t check_ring(const sensor_msgs::msg::PointCloud2 & cloud, std::string & message);

  std::string target_ip_;
  double cloud_timeout_sec_{};
  double min_valid_intensity_{};
  double low_intensity_ratio_threshold_{};
  int sector_divisions_{};
  int min_points_per_sector_{};
  int max_empty_sectors_{};
  int ring_upper_start_{};
  int ring_upper_end_{};
  int ring_min_points_{};
  double ring_min_intensity_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Time last_cloud_time_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_cloud_msg_;
  std::mutex cloud_mutex_;

  std::thread ping_thread_;
  std::atomic<bool> stop_ping_thread_{false};
  std::atomic<bool> ping_success_{false};
  std::atomic<float> latest_latency_ms_{-1.0f};
};
}// namespace sensor_diagnostic