#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "serial_manager.hpp"
#include <iostream>
#include <string>

/**
 * This node:
 *	 - Subscribes to /cmd_vel
 *   - Extracts linear.x and angular.z form geometry_msgs::msg::Twist message
 *   - Sends them over Serial to an Arduino via /dev/ttyUSB0
 */
class cmd_velSerialNode : public rclcpp::Node {
 public: 
	cmd_velSerialNode(const rclcpp::NodeOptions & options) 
		: Node("cmd_velSerialNode", options) {

		SerialManager::get().open_port("/dev/ttyACM0");

		cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist> (
			"/cmd_vel", 10,
			std::bind(&cmd_velSerialNode::cmd_velCallback, this, std::placeholders::_1)
		);

		RCLCPP_INFO(this->get_logger(), "Started cmd_velSerialNode");
	}

 private:
	//TODO: send as JSON
	void cmd_velCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(), "cmd_velCallback: linear.x = %f", msg->linear.x);
		nlohmann::json js;
		
		js["sensor_type"] = "cmd_vel"; // Add sensor_type field

		// Creating a data object
		nlohmann::json data_obj;

		data_obj["linear"]["x"] = msg->linear.x;
		data_obj["linear"]["y"] = msg->linear.y;
		data_obj["linear"]["z"] = msg->linear.z;

		// Add angular velocity components
		data_obj["angular"]["x"] = msg->angular.x;
		data_obj["angular"]["y"] = msg->angular.y;
		data_obj["angular"]["z"] = msg->angular.z;
		
		// Assign the populated "data" object to the main JSON
		js["data"] = data_obj;
		std::string json_str = js.dump(-1); // Use 4 for pretty printing if desired, or -1 for compact

		SerialManager::get().write_line(json_str.c_str());
		RCLCPP_INFO(this->get_logger(), "published json: \n\t%s", json_str.c_str());
	}
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cmd_velSerialNode)
