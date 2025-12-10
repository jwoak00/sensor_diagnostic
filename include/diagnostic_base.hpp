#pragma once

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

namespace sensor_diagnostic{
class DiagnosticBase : public rclcpp::Node {
public:
  explicit DiagnosticBase(const rclcpp::NodeOptions & options)
  : Node("sensor_diagnostic", options)
  {
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(10.0).period(), std::bind(&DiagnosticBase::on_timer, this));
  }

  virtual void init_diag() = 0;
  virtual void init_param() = 0;
  virtual void update_diag(diagnostic_updater::DiagnosticStatusWrapper &) {}

  diagnostic_updater::Updater diagnostic_updater_{this};
  
  std::string hardware_id_{};
  std::string sensor_name_{};

private:
  rclcpp::TimerBase::SharedPtr timer_;

  void on_timer(){
    diagnostic_updater_.force_update();
  }

};

}// namespace sensor_diagnostic
