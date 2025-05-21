#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // for goal poses
#include <sensor_msgs/msg/nav_sat_fix.hpp>    // for intial GPS pose
#include <nlohmann/json.hpp>
#include "serial_manager.hpp"
#include <GeographicLib/UTMUPS.hpp>

#include <chrono>
#include <memory>
#include <optional>
#include <string>

class GoalSerialNode : public rclcpp::Node {
 public:
	 GoalSerialNode(const rclcpp::NodeOptions & options) 
	 : Node("goal_serial_node", options) {

		goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10);

		// Subscribe to the robot's GPS data continuously
		robot_gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
			"gps/fix", 10,
			[this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
				latest_robot_gps_ = msg;
			}
		);

		/*utm_origin_set_ = false;

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
					RCLCPP_INFO(this->get_logger(), "UTM origin set");
				}
			}
		);*/

		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100),
			[this]() { this->process_goal(); }
		);
	 
	 }

 private:
	 void process_goal() {
		auto json_msg = SerialManager::get().popGoal();
		
		if (!json_msg || !latest_robot_gps_) {
			if (!latest_robot_gps_) {
				RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for initial robot GPS data...");
			}
			return;
		}

		try {
			RCLCPP_INFO(this->get_logger(), "Processing goal data");

			auto& data = json_msg->at("data");
			double latitude = data["latitude"].get<double>();
			double longitude = data["longitude"].get<double>();

			RCLCPP_INFO(this->get_logger(), 
				"GPS coords received> latitude: %f, longitude: %f", latitude, longitude);

			// Convert phone's GPS to UTM
			int zone; bool northp;
			double easting, northing;
			GeographicLib::UTMUPS::Forward(
				latitude, longitude,
				zone, northp, easting, northing
			);

			// Convert robot's GPS to UTM
			int robot_zone; bool robot_northp;
			double robot_easting, robot_northing;
			GeographicLib::UTMUPS::Forward(
				latest_robot_gps_->latitude, latest_robot_gps_->longitude,
				robot_zone, robot_northp, robot_easting, robot_northing
			);

			// Ensure both are in the same UTM zone
			if(zone == robot_zone && northp == robot_northp) {
				double utm_x = easting - robot_easting;
				double utm_y = northing - robot_northing;

				geometry_msgs::msg::PoseStamped goal_msg;
				goal_msg.header.stamp = this->now();
				goal_msg.header.frame_id = "base_link"; // Express goal relative to the robot
											
				goal_msg.pose.position.x = utm_x;
				goal_msg.pose.position.y = utm_y;

				goal_pub_->publish(goal_msg);
				RCLCPP_INFO(this->get_logger(),
					"Successfully published goal {x:%f, y:%f} to /goal_pose", utm_x, utm_y);
			} else {
				RCLCPP_WARN(this->get_logger(), 
					"Phone and robot GPS are in different UTM zones. Goal not published.");
			}
		} catch (const std:: exception& e) {
			RCLCPP_ERROR(this->get_logger(), "Goal error: %s", e.what());
		}
			

		//OLD CODE
		/*if (!json_msg || !utm_origin_set_)
			return;

		try {
			RCLCPP_INFO(this->get_logger(), "Processing goal data");

			auto& data = json_msg->at("data");
			double latitude  = data["latitude"].get<double>();
			double longitude = data["longitude"].get<double>();
			
			RCLCPP_INFO(this->get_logger(), "Phone GPS->Received latitude: %f, longitidue: %f", latitude, longitude);

			int zone; bool northp;
			double easting, northing;
			GeographicLib::UTMUPS::Forward(
				latitude, longitude, 
				zone, northp, easting, northing
			);
			double utm_x = easting - utm_origin_easting_;
			double utm_y = northing - utm_origin_northing_;

			geometry_msgs::msg::PoseStamped goal_msg;
			goal_msg.header.stamp = this->now();         
			goal_msg.header.frame_id = "map";
			goal_msg.pose.position.x = utm_x;
			goal_msg.pose.position.y = utm_y;
			
			goal_pub_->publish(goal_msg);
			RCLCPP_INFO(this->get_logger(), 
				"Successfully published goal {x:%f, y:%f} to /goal_pose", utm_x, utm_y);

		} catch (const std::exception& e) {
			RCLCPP_ERROR(this->get_logger(), "Goal error: %s", e.what());	
		}*/
	 }
	

	 // NEW CODE
	 rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
	 rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr robot_gps_sub_;
	 rclcpp::TimerBase::SharedPtr timer_;
	 sensor_msgs::msg::NavSatFix::SharedPtr latest_robot_gps_;
	 // OLD CODE
	 /*rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
	 rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
	 rclcpp::TimerBase::SharedPtr timer_;

	 bool utm_origin_set_;
	 double utm_origin_easting_, utm_origin_northing_;*/
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(GoalSerialNode)
