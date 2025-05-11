#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h> // RFCOMM protocol
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // for goal poses
#include <sensor_msgs/msg/nav_sat_fix.hpp>    // for initial GPS pose
#include <GeographicLib/UTMUPS.hpp>

#include <string>
#include <thread> // keep main ROS 2 threads responsive
#include <csignal>
#include <atomic>


class BluetoothGPSNode : public rclcpp::Node {
 public:
 	BluetoothGPSNode() : Node("bluetooth_gps_node") {
		// capture Ctrl+C to close sockets
 		signal(SIGINT, [](int) { rclcpp::shutdown(); });

		this->declare_parameter<std::string>("pi_mac", "2c:cf:67:2b:b0:fa");
 		pi_mac_ = this->get_parameter("pi_mac").as_string();
 
 		pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
 			"/goal_pose", 10);
 
 		// subscribe to the first gps/fix to set UTM origin
 		gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
 			"gps/fix", 10,
 			[this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
 				if (!utm_origin_set_) {
 					int zone; bool northp;
 					double easting0, northing0;
 					GeographicLib::UTMUPS::Forward(
 						msg->latitude, msg->longitude,
 						zone, northp, easting0, northing0
 					);
 					utm_origin_easting_  = easting0;
 					utm_origin_northing_ = northing0;
 					utm_origin_set_ = true;
 					RCLCPP_INFO(this->get_logger(), "UTM origin set (BT)");
 				}
 			}
 		);
 
 		// blocking operations in a thread
 		bt_thread_ = std::thread(&BluetoothGPSNode::bluetooth_loop, this);
 	}
 
 	~BluetoothGPSNode() {
 		if (bt_thread_.joinable())
 			bt_thread_.join();
 		close(client_sock_);
 		close(server_sock_);
 	}

 private:
 	void bluetooth_loop() {
 		/** setup Bluetooth server */
 		server_sock_ = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
 		sockaddr_rc addr{}; // initialize to all zeros
 		addr.rc_family = AF_BLUETOOTH;
 		addr.rc_channel = 1; // channel 1 of 30, sync with connecting device
 		str2ba(pi_mac_.c_str(), &addr.rc_bdaddr); // to binary, byte array
 
 		bind(server_sock_, (sockaddr*)&addr, sizeof(addr));
 		listen(server_sock_, 1);
 
 		RCLCPP_INFO(this->get_logger(), "Waiting for connection...");
 		client_sock_ = accept(server_sock_, nullptr, nullptr);
 		RCLCPP_INFO(this->get_logger(), "Connected.");
 
 		/** read data and publish */
 		char buffer[1024];
 		while (rclcpp::ok()) {
 			int bytes_read = read(client_sock_, buffer, sizeof(buffer));
 			if (bytes_read > 0) {
 				std::string data(buffer, bytes_read);
 				process_data(data);
 			}
 		}
 	}
 
 	void process_data(const std::string& data) {
 		try {
 			// expect "lat,lon"
 			size_t comma_idx = data.find(',');
			if (comma_idx == std::string::npos) {
				RCLCPP_ERROR(this->get_logger(), "Invalid data format");
				return;
			}

 			double latitude  = std::stod(data.substr(0, comma_idx));
 			double longitude = std::stod(data.substr(comma_idx + 1));
 
 			int zone; bool northp;
 			double easting, northing;
 			GeographicLib::UTMUPS::Forward(
 				latitude, longitude,
 				zone, northp, easting, northing		
 			);
 			double utm_x = easting - utm_origin_easting_;
 			double utm_y = northing - utm_origin_northing_;
 
 			geometry_msgs::msg::PoseStamped msg;
 			msg.header.stamp = this->now();
 			msg.header.frame_id = "map";
 			msg.pose.position.x = utm_x;
 			msg.pose.position.y = utm_y;
 
 			pose_pub_->publish(msg);
 			RCLCPP_INFO(this->get_logger(), 
 				"Published: x: %f, y: %f", utm_x, utm_y);
 		} catch (...) {
 			RCLCPP_ERROR(this->get_logger(), "Bluetooth error");
 		}
 	}
 	std::string pi_mac_;
 	int server_sock_{-1}, client_sock_{-1};
 	std::thread bt_thread_;
 	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
 
 	// for setting origin
 	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
	std::atomic<bool> utm_origin_set_{false};
 	double utm_origin_easting_, utm_origin_northing_;
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BluetoothGPSNode>());
	rclcpp::shutdown();
	return 0;
}
