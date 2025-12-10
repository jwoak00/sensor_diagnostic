#include "camera_diagnostic.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

namespace sensor_diagnostic{

CameraDiagnostic::CameraDiagnostic(const rclcpp::NodeOptions & options)
: DiagnosticBase(options)
{
	init_param();
	init_diag();
}

void CameraDiagnostic::init_param(){
	this->declare_parameter<std::string>("hardware_id", "STURDeCAM25-60");
	this->declare_parameter<std::string>("sensor_name", "auto_center_camera");
	this->declare_parameter<std::string>("image_transport", "compressed");
	this->declare_parameter<double>("timeout_sec", 2.0);
	this->declare_parameter<double>("expected_fps", 30.0);
	this->declare_parameter<double>("fps_warn_threshold", 0.9);
	this->declare_parameter<double>("brightness_dark_threshold", 30.0);
	this->declare_parameter<double>("brightness_bright_threshold", 180.0);
	this->declare_parameter<double>("brightness_stddev_threshold", 12.0);
	this->declare_parameter<double>("histogram_dominance_threshold", 0.5);

	this->get_parameter("hardware_id", hardware_id_);
	this->get_parameter("sensor_name", sensor_name_);
	this->get_parameter("image_transport", transport_type_);
	this->get_parameter("timeout_sec", timeout_sec_);
	this->get_parameter("expected_fps", expected_fps_);
	this->get_parameter("fps_warn_threshold", fps_warn_threshold_);
	this->get_parameter("brightness_dark_threshold", brightness_dark_threshold_);
	this->get_parameter("brightness_bright_threshold", brightness_bright_threshold_);
	this->get_parameter("brightness_stddev_threshold", brightness_stddev_threshold_);
	this->get_parameter("histogram_dominance_threshold", histogram_dominance_threshold_);

	diagnostic_throttle_period_sec_ = 1.0 / 5.0;
	image_topic_ = "/sensing/camera/" + sensor_name_ + "/image_raw";

	rclcpp::Clock steady_clock(RCL_STEADY_TIME);
	last_image_time_ = steady_clock.now();
	last_check_time_ = steady_clock.now();
	image_received_ = false;
	image_blocked_ = false;
}

void CameraDiagnostic::init_diag(){
	diagnostic_updater_.setHardwareID(hardware_id_);
	diagnostic_updater_.add("Timeout", this, &CameraDiagnostic::diag_timeout);
	diagnostic_updater_.add("Blocked", this, &CameraDiagnostic::diag_blocked);
	diagnostic_updater_.add("FrameRate", this, &CameraDiagnostic::diag_framerate);

	setup_image_subscriber(image_topic_, transport_type_);
}

void CameraDiagnostic::setup_image_subscriber(const std::string & image_topic, const std::string & transport_type)
{
	if (transport_type == "compressed" || transport_type == "raw") {
		image_transport_sub_ = image_transport::create_subscription(
			this,
			image_topic,
			std::bind(&CameraDiagnostic::image_callback, this, std::placeholders::_1),
			transport_type,
			rmw_qos_profile_sensor_data);
	} else {
		RCLCPP_WARN(
			this->get_logger(),
			"Unknown image_transport '%s'. Falling back to raw subscription.",
			transport_type.c_str());

		raw_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
			image_topic,
			rclcpp::SensorDataQoS(),
			std::bind(&CameraDiagnostic::image_callback, this, std::placeholders::_1));
	}
}

void CameraDiagnostic::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
	rclcpp::Clock steady_clock(RCL_STEADY_TIME);
	const auto now = steady_clock.now();

	frame_times_.push_back(now);
	while (!frame_times_.empty() && (now - frame_times_.front()).seconds() > 1.0) {
		frame_times_.pop_front();
	}

	last_image_time_ = now;
	image_received_ = true;

	if ((now - last_check_time_).seconds() < diagnostic_throttle_period_sec_) {
		return;
	}
	last_check_time_ = now;

		try {
			const auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
		analyze_image_blocking(cv_ptr->image);
	} catch (const cv_bridge::Exception & e) {
		RCLCPP_WARN(this->get_logger(), "cv_bridge 변환 실패: %s", e.what());
		image_blocked_ = false;
	} catch (const std::exception & e) {
		RCLCPP_WARN(this->get_logger(), "이미지 분석 중 예외: %s", e.what());
		image_blocked_ = false;
	}
}

void CameraDiagnostic::analyze_image_blocking(const cv::Mat & image)
{
	if (image.empty()) {
		image_blocked_ = true;
		return;
	}

	cv::Mat gray;
	if (image.channels() == 1) {
		gray = image;
	} else {
		cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	}

	cv::Scalar mean, stddev;
	cv::meanStdDev(gray, mean, stddev);
	const double avg_brightness = mean[0];
	const double stddev_brightness = stddev[0];

	image_blocked_ = ((avg_brightness < brightness_dark_threshold_ &&
										stddev_brightness < brightness_stddev_threshold_) ||
										(avg_brightness > brightness_bright_threshold_ &&
										stddev_brightness < brightness_stddev_threshold_));

	if (image.channels() < 3) {
		return;
	}

	std::vector<cv::Mat> channels;
	cv::split(image, channels);

	constexpr int histSize = 256;
	float range[] = {0, 256};
	const float * histRange = {range};

	cv::Mat r_hist, g_hist, b_hist;
	cv::calcHist(&channels[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange);
	cv::calcHist(&channels[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange);
	cv::calcHist(&channels[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange);

	const double r_sum = cv::sum(r_hist)[0];
	const double g_sum = cv::sum(g_hist)[0];
	const double b_sum = cv::sum(b_hist)[0];
	const double total = r_sum + g_sum + b_sum + 1e-6;

	const double r_ratio = r_sum / total;
	const double g_ratio = g_sum / total;
	const double b_ratio = b_sum / total;

	const double max_ratio = std::max({r_ratio, g_ratio, b_ratio});
	const double min_ratio = std::min({r_ratio, g_ratio, b_ratio});

	if ((max_ratio - min_ratio) > histogram_dominance_threshold_) {
		image_blocked_ = true;
	}
}

void CameraDiagnostic::diag_timeout(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
	diagnostics.add("Topic", image_topic_);

	std::string message;
	if (check_timeout(message)) {
		diagnostics.add("Timeout", message);
		diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
		return;
	}

	diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

void CameraDiagnostic::diag_blocked(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
	std::string message;
	if (check_blocked(message)) {
		diagnostics.add("Blocked", message);
		diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
		return;
	}

	diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}

void CameraDiagnostic::diag_framerate(diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
	diagnostics.addf("Observed FPS", "%.2f", static_cast<double>(frame_times_.size()));
	diagnostics.addf("Expected FPS", "%.2f", expected_fps_);

	std::string message;
	if (check_fps(message)) {
		diagnostics.add("FrameRate", message);
		diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
		return;
	}

	diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "정상 수신 중");
}


bool CameraDiagnostic::check_timeout(std::string & message)
{
	const auto now = rclcpp::Clock(RCL_STEADY_TIME).now();

	if (!image_received_ || (now - last_image_time_).seconds() > timeout_sec_) {
		message = "[Communication] 이미지 수신 없음";
		return true;
	}
	return false;
}

bool CameraDiagnostic::check_blocked(std::string & message)
{
	if (!image_received_) {
		message = "[Visibility] 이미지 수신 없음";
		return true;
	}
	if (image_blocked_) {
		message = "[Visibility] 카메라가 가려짐";
		return true;
	}
	return false;
}

bool CameraDiagnostic::check_fps(std::string & message)
{
	const double current_fps = static_cast<double>(frame_times_.size());
	const double threshold = expected_fps_ * fps_warn_threshold_;
	if (current_fps < threshold) {
		message = "[Framerate] 프레임 수신 속도가 낮음: " +
			std::to_string(current_fps) + " < " + std::to_string(threshold);
		return true;
	}
	return false;
}


}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_diagnostic::CameraDiagnostic)
