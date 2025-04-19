#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <libserial/SerialPort.h>
#include <nlohmann/json.hpp>
#include <math.h>
#include "serial_manager.hpp"

class EncoderSerialNode : public rclcpp::Node {
 public:
	 EncoderSerialNode(const rclcpp::NodeOptions & options) 
	 : Node("encoder_serial_node", options),  x_(0.0), y_(0.0), theta_(0.0),
	 linear_velocity_(0.0), angular_velocity_(0.0) {

		this->declare_parameter<int>("ticks_per_rev", 48);
		this->declare_parameter<double>("wheel_radius", 0.06248 / 2);
		this->declare_parameter<double>("track",        0.2580);

		ticks_per_rev_ = this->get_parameter("ticks_per_rev").as_int();
		wheel_radius_  = this->get_parameter("wheel_radius").as_double();
		track_         = this->get_parameter("track").as_double();
		
		SerialManager::get().open_port("/dev/ttyACM0");

		odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
			"wheel_odom", 10); 

		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100),
			std::bind(&EncoderSerialNode::readEncoderData, this)		
		);

		prev_time_ = this->get_clock()->now();
	 }

 private:
	 void readEncoderData() {
		auto json_msg = SerialManager::get().popEnc();

		if (!json_msg.has_value())
			return;

		try {
			RCLCPP_INFO(this->get_logger(), "Processing encoder data");

			rclcpp::Time current_time = this->get_clock()->now();
			double dt = (current_time - prev_time_).seconds();
			prev_time_ = current_time;

			auto& data = json_msg->at("data");
			double left_ticks  = data["left_wheel_ticks"].get<double>();
			double right_ticks = data["right_wheel_ticks"].get<double>();

			double left_distance  = (left_ticks / ticks_per_rev_) * (2 * M_PI * wheel_radius_);
			double right_distance = (right_ticks / ticks_per_rev_) * (2 * M_PI * wheel_radius_);

			double left_velocity  = left_distance / dt;
			double right_velocity = right_distance / dt;
			
			double linear_velocity_  = (left_velocity + right_velocity) / 2.0;
			double angular_velocity_ = (right_velocity - left_velocity) / track_;

			theta_ += angular_velocity_;      // aka delta_theta
			x_ += linear_velocity_ * cos(theta_);
			y_ += linear_velocity_ * sin(theta_);
			
			nav_msgs::msg::Odometry odom_msg;
			odom_msg.header.stamp = current_time;
			odom_msg.header.frame_id = "odom";
			odom_msg.child_frame_id = "base_link";

			odom_msg.pose.pose.position.x = x_;
			odom_msg.pose.pose.position.y = y_;

			odom_msg.twist.twist.linear.x  = linear_velocity_;
			odom_msg.twist.twist.angular.z = angular_velocity_ / dt;

			// covariance (adjust based on sensor noise);
			odom_msg.twist.covariance[0] = 0.01;  // x velocity variance
			odom_msg.twist.covariance[35] = 0.01; // yaw variance

			odom_publisher_->publish(odom_msg);
			RCLCPP_INFO(this->get_logger(), "Successfully published data to wheel_odom");
		} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "JSON Parsing Error: %s", e.what());
		}
	 }

	 int    ticks_per_rev_;
	 double wheel_radius_;
	 double track_;
	 
     double x_, y_, theta_;
	 double linear_velocity_, angular_velocity_;

	 rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
	 rclcpp::TimerBase::SharedPtr timer_;
	 rclcpp::Time prev_time_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(EncoderSerialNode)
