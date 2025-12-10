#include "imu_diagnostic.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <string>
#include <vector>

#include <unistd.h>

namespace sensor_diagnostic{

ImuDiagnostic::ImuDiagnostic(const rclcpp::NodeOptions & options)
: DiagnosticBase(options)
{
	init_param();
	init_diag();
}

void ImuDiagnostic::init_param()
{
	this->declare_parameter<std::string>("hardware_id", "MTi-670-DK");
	this->declare_parameter<std::string>("sensor_name", "imu");
	this->declare_parameter<std::string>("device_path", "/dev/ttyUSB0");
	this->declare_parameter<std::string>("imu_topic", "/imu_raw");
	this->declare_parameter<double>("imu_timeout_sec", 1.0);
	this->declare_parameter<double>("min_expected_frequency", 50.0);
	this->declare_parameter<double>("accel_threshold", 5.0); // 이상치 판단 기준 (m/s^2)
	this->declare_parameter<double>("gyro_threshold", 3.0); // 이상치 판단 기준 (rad/s)
	this->declare_parameter<double>("frequency_window_sec", 1.0);

	this->get_parameter("hardware_id", hardware_id_);
	this->get_parameter("sensor_name", sensor_name_);
	this->get_parameter("device_path", device_path_);
	this->get_parameter("imu_topic", imu_topic_);
	this->get_parameter("imu_timeout_sec", imu_timeout_sec_);
	this->get_parameter("min_expected_frequency", min_expected_frequency_);
	this->get_parameter("accel_threshold", accel_threshold_);
	this->get_parameter("gyro_threshold", gyro_threshold_);
	this->get_parameter("frequency_window_sec", frequency_window_sec_);

	last_imu_time_ = this->now();
	is_connected_ = (access(device_path_.c_str(), F_OK) == 0);
}

void ImuDiagnostic::init_diag()
{
	diagnostic_updater_.setHardwareID(hardware_id_);
	diagnostic_updater_.add("Device", this, &ImuDiagnostic::diag_device);
	diagnostic_updater_.add("Timeout", this, &ImuDiagnostic::diag_timeout);
	diagnostic_updater_.add("Frequency", this, &ImuDiagnostic::diag_frequency);
	diagnostic_updater_.add("Data", this, &ImuDiagnostic::diag_data);
	diagnostic_updater_.add("Outlier", this, &ImuDiagnostic::diag_outlier);
	diagnostic_updater_.add("Stuck", this, &ImuDiagnostic::diag_stuck);

	imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
		imu_topic_,
		rclcpp::SensorDataQoS(),
		std::bind(&ImuDiagnostic::imu_callback, this, std::placeholders::_1));
}

void ImuDiagnostic::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
	const auto now = this->now();
	last_imu_time_ = now;
	last_imu_msg_ = *msg;
	is_connected_ = true;

	imu_timestamps_.push_back(now);
	while (!imu_timestamps_.empty() && (now - imu_timestamps_.front()).seconds() > frequency_window_sec_) {
		imu_timestamps_.pop_front();
	}

	accel_history_.push_back({now, msg->linear_acceleration});
	while (!accel_history_.empty() && (now - accel_history_.front().stamp).seconds() > frequency_window_sec_) {
		accel_history_.pop_front();
	}
}

void ImuDiagnostic::diag_device(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
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

void ImuDiagnostic::diag_timeout(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
	diagnostics.add("Topic", imu_topic_);

	std::string message;
	if (check_timeout(message)) {
		diagnostics.add("Timeout", message);
		diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
		return;
	}

	diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

void ImuDiagnostic::diag_frequency(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
	double frequency = 0.0;
	std::string message;
	const bool warn = check_frequency(frequency, message);

	diagnostics.addf("Observed Frequency", "%.2f", frequency);
	diagnostics.addf("Expected Min Frequency", "%.2f", min_expected_frequency_);

	if (!is_connected_ || imu_timestamps_.empty()) {
		diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "[Frequency] 데이터 수신 대기 중");
		return;
	}

	if (warn) {
		diagnostics.add("Frequency", message);
		diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
		return;
	}

	diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

void ImuDiagnostic::diag_data(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
	if (!is_connected_ || imu_timestamps_.empty()) {
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

void ImuDiagnostic::diag_outlier(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
	if (!is_connected_ || imu_timestamps_.empty()) {
		diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "[Outlier] 데이터 수신 대기 중");
		return;
	}

	std::string message;
	if (check_outlier(message)) {
		diagnostics.add("Outlier", message);
		diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
		return;
	}

	diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

void ImuDiagnostic::diag_stuck(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
	if (!is_connected_ || imu_timestamps_.empty()) {
		diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "[Stuck] 데이터 수신 대기 중");
		return;
	}

	std::string message;
	if (check_stuck_sensor(message)) {
		diagnostics.add("Stuck", message);
		diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
		return;
	}

	diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

bool ImuDiagnostic::check_device(std::string & message)
{
	is_connected_ = (access(device_path_.c_str(), F_OK) == 0);
	if (!is_connected_) {
		message = "[Device] 장치 없음";
		return true;
	}
	return false;
}

bool ImuDiagnostic::check_timeout(std::string & message)
{
	const auto now = this->now();
	if ((now - last_imu_time_).seconds() > imu_timeout_sec_) {
		message = "[Device] Time Out";
		return true;
	}
	return false;
}

bool ImuDiagnostic::check_frequency(double & frequency, std::string & message)
{
	if (imu_timestamps_.empty()) {
		frequency = 0.0;
	} else {
		frequency = static_cast<double>(imu_timestamps_.size()) / frequency_window_sec_;
	}

	if (frequency < min_expected_frequency_) {
		message = "[Frequency] 수신 주기 저하: " +
			std::to_string(frequency) + " < " + std::to_string(min_expected_frequency_);
		return true;
	}
	return false;
}

bool ImuDiagnostic::check_data_validity(std::string & message)
{
	const auto & a = last_imu_msg_.linear_acceleration;
	const auto & g = last_imu_msg_.angular_velocity;
	const auto & q = last_imu_msg_.orientation;

	const bool valid = is_valid_double(a.x) && is_valid_double(a.y) && is_valid_double(a.z) &&
		is_valid_double(g.x) && is_valid_double(g.y) && is_valid_double(g.z) &&
		is_valid_double(q.x) && is_valid_double(q.y) && is_valid_double(q.z) && is_valid_double(q.w);

	if (!valid) {
		message = "[Data] 유효하지 않은 값 포함";
		return true;
	}
	return false;
}

bool ImuDiagnostic::check_outlier(std::string & message)
{
	const auto & a = last_imu_msg_.linear_acceleration;
	const auto & g = last_imu_msg_.angular_velocity;

	const double acc_norm = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
	if (acc_norm > accel_threshold_) {
		message = "[Outlier] 가속도 크기 과도: " + std::to_string(acc_norm) + " > " + std::to_string(accel_threshold_);
		return true;
	}

	if (std::abs(g.x) > gyro_threshold_ || std::abs(g.y) > gyro_threshold_ || std::abs(g.z) > gyro_threshold_) {
		message = "[Outlier] 각속도 크기 과도";
		return true;
	}

	const auto & q = last_imu_msg_.orientation;
	const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	if (std::abs(q_norm - 1.0) > 0.01) {
		message = "[Outlier] 쿼터니언 정규화 오류";
		return true;
	}

	return false;
}

bool ImuDiagnostic::check_stuck_sensor(std::string & message)
{
	if (accel_history_.size() < 2) {
		return false;
	}

	double sum_x = 0.0;
	double sum_y = 0.0;
	double sum_z = 0.0;
	for (const auto & sample : accel_history_) {
		sum_x += sample.acc.x;
		sum_y += sample.acc.y;
		sum_z += sample.acc.z;
	}

	const double count = static_cast<double>(accel_history_.size());
	const double mean_x = sum_x / count;
	const double mean_y = sum_y / count;
	const double mean_z = sum_z / count;

	double var_x = 0.0;
	double var_y = 0.0;
	double var_z = 0.0;
	for (const auto & sample : accel_history_) {
		var_x += std::pow(sample.acc.x - mean_x, 2);
		var_y += std::pow(sample.acc.y - mean_y, 2);
		var_z += std::pow(sample.acc.z - mean_z, 2);
	}

	const double std_x = std::sqrt(var_x / count);
	const double std_y = std::sqrt(var_y / count);
	const double std_z = std::sqrt(var_z / count);

	constexpr double kThreshold = 0.01;  // m/s^2
	if (std_x < kThreshold && std_y < kThreshold && std_z < kThreshold) {
		message = "[Stuck] 센서 값 변동 없음";
		return true;
	}
	return false;
}

bool ImuDiagnostic::is_valid_double(double value) const
{
	return std::isfinite(value);
}

}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_diagnostic::ImuDiagnostic)
