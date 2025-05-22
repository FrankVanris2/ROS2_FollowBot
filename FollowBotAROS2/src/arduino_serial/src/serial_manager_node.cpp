#include "serial_manager.hpp"
#include <rclcpp/rclcpp.hpp>

class SerialManagerNode : public rclcpp::Node {
 public: 
	 SerialManagerNode(const rclcpp::NodeOptions& options)
	 : Node("serial_manager_node", options) {
		// get parameters
		auto port_name = declare_parameter<std::string>("port_name", "/dev/ttyACM0");
		// auto baud_rate = declare_parameter<int>("baud_rate", 9600);

		SerialManager::get().open_port(port_name);
		RCLCPP_INFO(get_logger(), 
			"SerialManager initialized on %s.", port_name.c_str());
		SerialManager::get().start_demux();
		RCLCPP_INFO(get_logger(), 
			"SerialManager initialized on %s.", port_name.c_str());
	 }

};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SerialManagerNode)
