#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nlohmann/json.hpp>
#include "serial_manager.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

class GPSSerialNode : public rclcpp::Node {
 public:
	 GPSSerialNode(const rclcpp::NodeOptions & options) 
	 : Node("gps_serial_node", options) {
		 
		SerialManager::get().open_port("/dev/ttyACM0");

		gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
			"gps/fix", 10);

		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100), // read GPS data every 100ms
			std::bind(&GPSSerialNode::readGPSData, this)		
		);
	 }

 private:
	 void readGPSData() {
		auto json_msg = SerialManager::get().popGps();

		if (!json_msg)
			return;

		try {
			RCLCPP_INFO(this->get_logger(), "Processing GPS data");

			auto& data = json_msg->at("data");
			double latitude  = data["latitude"].get<double>();
			double longitude = data["longitude"].get<double>();
			
			RCLCPP_INFO(this->get_logger(), "GPS-FollowBot->Received latitude: %f, longitude: %f", latitude, longitude);

			sensor_msgs::msg::NavSatFix gps_msg;
			gps_msg.header.stamp = this->now();
			gps_msg.header.frame_id = "gps_link";

			gps_msg.latitude = latitude;
			gps_msg.longitude = longitude;

			// default covariance
			for (size_t i = 0; i < 9; i++)
				gps_msg.position_covariance[i] = 1.0;

			gps_publisher_->publish(gps_msg);
			RCLCPP_INFO(this->get_logger(), 
				"Successfully published GPS data to `gps/fix`");

		} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "GPS error: %s", e.what());
		}
	 }

	 rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
	 rclcpp::TimerBase::SharedPtr timer_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(GPSSerialNode)
