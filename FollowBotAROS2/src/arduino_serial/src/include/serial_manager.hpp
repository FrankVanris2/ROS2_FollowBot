#pragma once
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <optional>
#include <string>
#include <algorithm> // For std::remove_if, std::isspace
#include <cctype> 	 // For std::isspace

class SerialManager {
 public:
	 /** get function for thread-safe singleton object*/
	 static SerialManager &get() {
		static SerialManager instance;
		return instance;
	 }

	 /** open the serial port (once) */
	 void open_port(const std::string &port_name) {
		std::lock_guard<std::mutex> lock(port_mutex_);
		if (!port_.IsOpen()) {
			port_.Open(port_name);
			port_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);;
		}
	 }

	 /**
	 * start the demuxing thread if it is not already running.
	 * the thread continuously calls read_and_demux() to handle incoming serial data
	 * sleeps briefly between iterations to reduce CPU usage
	 */
	 void start_demux() {
		if (demux_thread_.joinable()) 
			return;

		running_ = true;
		demux_thread_ = std::thread([this]() {
			RCLCPP_INFO(rclcpp::get_logger("serial_manager"), 
			"Demux thread started");
			while (running_) {
				this->read_and_demux();
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}		
		});
	 }
	 
	 /**
	  * Sends a string through serial port
	  */
	 void write_line(const std::string& line_to_write) {
		RCLCPP_INFO(rclcpp::get_logger("serial_manager"), 
					"Attempting to write to serial: %s", line_to_write.c_str()); 
		std::lock_guard<std::mutex> lock(port_mutex_);
		if (port_.IsOpen()) {
			try {
				port_.Write(line_to_write);
				port_.Write("\n"); // Ensure a newline is sent if arduino expects it for each JSON Message
				RCLCPP_INFO(rclcpp::get_logger("serial_manager"), 
							"Serial write successful");
			} catch (const std::exception& e) {
				RCLCPP_ERROR(rclcpp::get_logger("serial_manager"), 
							"Serial write error: %s", e.what());
			}
		} else {
			RCLCPP_WARN(rclcpp::get_logger("serial_manager"), 
						"Serial port not open. Cannot write.");
		}
     }

	 /**
	 * pop one message for each message type
	 * return std::nullopt if queue empty
	 */
	 std::optional<nlohmann::json> popImu() { return pop_queue(imu_q_, imu_mutex_); }
	 std::optional<nlohmann::json> popEnc() { return pop_queue(enc_q_, enc_mutex_); }
	 std::optional<nlohmann::json> popGps() { return pop_queue(gps_q_, gps_mutex_); }
	 std::optional<nlohmann::json> popGoal() { return pop_queue(goal_q_, goal_mutex_); }
	 
	 /**
	 * destructor for SerialManager 
	 * if the thread is still joinable, waits for it to finish
	 */
	 ~SerialManager() {
		running_ = false;
        if (demux_thread_.joinable()) {
            demux_thread_.join();
            RCLCPP_INFO(rclcpp::get_logger("serial_manager"), 
				"Demux thread stopped");
        }
    }

 private:
	 SerialManager() = default;

	 // ensure singleton object when copy or delete
	 SerialManager(const SerialManager &) = delete;
	 SerialManager & operator=(const SerialManager &) = delete;

	 bool read_line(std::string& out) {
		std::lock_guard<std::mutex> lock(port_mutex_);
        if (port_.IsDataAvailable()) {
            port_.ReadLine(out);
			//RCLCPP_INFO(rclcpp::get_logger("serial_manager"), "Received: %s", out.c_str()); // NEED TO REMOVE
            return true;
        }
        return false;
     }

	 /**
	 * trim lines to remove leading/trailing whitespace
	 * If there is whitespace, remove it
	 * @param line the line to trim
	 */
	 std::string trim(const std::string& line) {
		size_t first = line.find_first_not_of(" \t\n\r");
		if (std::string::npos == first) {
			return line; // No non-whitespace characters found.
		}
		size_t last = line.find_last_not_of(" \t\n\r");
		return line.substr(first, (last - first + 1));
	 }

	 bool read_and_demux() {
		std::string line;
        if (!read_line(line)) 
			return false;

		// --- THE CRITICAL STEP: TRIM THE LINE ---
		std::string trimmed_line = trim(line); // only needed for debugging purposes.
		
		// FOR DEBUGGING PURPOSES
		// Check if error occurs on arduino side for Json Deserialization
		if (!trimmed_line.empty() && trimmed_line == "JSON_ERROR_ACK") {
			RCLCPP_INFO(rclcpp::get_logger("serial_manager"), 
				"Arduino acknowledged JSON error");
			return true;
		}

		// FOR DEBUGGING PURPOSES
		// Check for specific confirmation message first
		if (!trimmed_line.empty() && trimmed_line.find("ARDUINO_CMD_VEL_ACK") == 0) {
			RCLCPP_INFO(rclcpp::get_logger("serial_manager"),
				"Arduino acknowledged cmd_vel receipt ACKNOWLEDGED");
			return true; // Successfully handled this line
		}

        // message validation
        if (line.empty() || line.front() != '{') 
            return false;

        try {
            auto js = nlohmann::json::parse(line);
            const std::string sensor_type = js["sensor_type"];

            if (sensor_type == "imu") {
				std::lock_guard<std::mutex> lock(imu_mutex_);
                if (imu_q_.size() < MAX_QUEUE_SIZE) {
					imu_q_.push(std::move(js));
				}
            } else if (sensor_type == "encoder") {
                std::lock_guard<std::mutex> lock(enc_mutex_);
				if (enc_q_.size() < MAX_QUEUE_SIZE) {
					enc_q_.push(std::move(js));
				}
            } else if (sensor_type == "gps") {
                std::lock_guard<std::mutex> lock(gps_mutex_);
				if (gps_q_.size() < MAX_QUEUE_SIZE) {
					gps_q_.push(std::move(js));
				}
			} else if (sensor_type == "goal") {
				std::lock_guard<std::mutex> lock(goal_mutex_);
				if (goal_q_.size() < MAX_QUEUE_SIZE) {
					goal_q_.push(std::move(js));
				}
			} else if (sensor_type == "cmd_vel") {
				//see what you get back
				std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
				RCLCPP_INFO(rclcpp::get_logger("cmd_vel"), "received json: \n\t%s", line.c_str());
            } else {
                RCLCPP_WARN(rclcpp::get_logger("serial_manager"),
                    "Unknown sensor type: %s", sensor_type.c_str());
                return false;
            }

            return true;

        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("serial_manager"),
                "JSON error: %s", e.what());
            return false;
        }
    }

	 template<typename Q>
	 std::optional<nlohmann::json> pop_queue(Q &q, std::mutex& mtx) {
		std::lock_guard<std::mutex> lock(mtx);
		if (q.empty())
			return std::nullopt;

		auto js = std::move(q.front());
		q.pop();
		return js;
	 }

	 static constexpr size_t MAX_QUEUE_SIZE = 1000;

	 // thread control
	 std::atomic<bool> running_{false};
	 std::thread demux_thread_;

	 // serial port
	 LibSerial::SerialPort port_;
	 std::mutex port_mutex_;

	 // data queues
	 std::queue<nlohmann::json> imu_q_, enc_q_, gps_q_, goal_q_;
	 std::mutex imu_mutex_, enc_mutex_, gps_mutex_, goal_mutex_;
	 std::mutex cmd_vel_mutex_;
};
