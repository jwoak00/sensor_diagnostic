#pragma once

#include "diagnostic_base.hpp"

#include <deque>
#include <string>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace cv
{
class Mat;
}

namespace sensor_diagnostic{
class CameraDiagnostic : public DiagnosticBase{
public:
  explicit CameraDiagnostic(const rclcpp::NodeOptions & options);
  ~CameraDiagnostic() override = default;
  
private:
  void init_param() override;
  void init_diag() override;
  void diag_timeout(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_blocked(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);
  void diag_framerate(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  void setup_image_subscriber(const std::string & image_topic, const std::string & transport_type);
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void analyze_image_blocking(const cv::Mat & image);

  bool check_timeout(std::string & message);
  bool check_blocked(std::string & message);
  bool check_fps(std::string & message);

  std::string transport_type_;
  std::string image_topic_;
  double timeout_sec_{};
  double expected_fps_{};
  double fps_warn_threshold_{};
  double brightness_dark_threshold_{};
  double brightness_bright_threshold_{};
  double brightness_stddev_threshold_{};
  double histogram_dominance_threshold_{};
  double diagnostic_throttle_period_sec_{};

  std::deque<rclcpp::Time> frame_times_;
  rclcpp::Time last_image_time_;
  rclcpp::Time last_check_time_;
  bool image_received_{false};
  bool image_blocked_{false};

  image_transport::Subscriber image_transport_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
};
}// namespace sensor_diagnostic
