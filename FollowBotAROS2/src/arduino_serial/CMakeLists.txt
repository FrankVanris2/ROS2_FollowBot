cmake_minimum_required(VERSION 3.8)
project(arduino_serial)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)      # for encoder/odom msg
find_package(sensor_msgs REQUIRED)   # for imu and gps
find_package(geometry_msgs REQUIRED) # for PoseStamped msgs and cmd_vel
find_package(nlohmann_json REQUIRED)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GEOLIB REQUIRED geographiclib)
pkg_check_modules(LibSerial REQUIRED IMPORTED_TARGET libserial)
include_directories(${LibSerial_INCLUDE_DIRS})

# == Serial Manager Component ==
add_library(serial_manager_component SHARED src/serial_manager_node.cpp)
target_link_libraries(serial_manager_component PkgConfig::LibSerial)
ament_target_dependencies(serial_manager_component
    rclcpp
    rclcpp_components
    nlohmann_json
)
target_include_directories(serial_manager_component PUBLIC src/include)
rclcpp_components_register_node(serial_manager_component
    PLUGIN "SerialManagerNode"
    EXECUTABLE serial_manager_node_exec
)

# ======= IMU component =======
add_library(madgwick_filter src/imu_filter/MadgwickAHRS.cpp)
target_include_directories(madgwick_filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/imu_filter) 

add_library(imu_serial_component SHARED src/imu_serial_node.cpp)
target_link_libraries(imu_serial_component 
	madgwick_filter 
	PkgConfig::LibSerial
)
ament_target_dependencies(imu_serial_component 
	rclcpp 
	rclcpp_components 
	sensor_msgs
	nlohmann_json	
) 
target_include_directories(imu_serial_component PUBLIC src/include)

rclcpp_components_register_node(
	imu_serial_component
	PLUGIN "IMUSerialNode"
	EXECUTABLE imu_serial_node_exec
)

# ===== encoder component =====
add_library(encoder_serial_component SHARED src/encoder_serial_node.cpp)
target_link_libraries(encoder_serial_component PkgConfig::LibSerial)
ament_target_dependencies(encoder_serial_component 
	rclcpp 
	rclcpp_components 
	nav_msgs
	sensor_msgs
	nlohmann_json
)
target_include_directories(encoder_serial_component PUBLIC src/include)

rclcpp_components_register_node(
	encoder_serial_component
	PLUGIN "EncoderSerialNode"
	EXECUTABLE encoder_serial_node_exec
)

# ======= gps component =======
add_library(gps_serial_component SHARED src/gps_serial_node.cpp)
target_link_libraries(gps_serial_component PkgConfig::LibSerial)
ament_target_dependencies(gps_serial_component 
	rclcpp 
	rclcpp_components 
	sensor_msgs 
	nlohmann_json)
target_include_directories(gps_serial_component PUBLIC src/include)

rclcpp_components_register_node(
	gps_serial_component
	PLUGIN "GPSSerialNode"
	EXECUTABLE gps_serial_node_exec
)

# ====== cmd_vel component ======
add_library(cmd_vel_serial_component SHARED src/cmd_vel_serial_node.cpp)
target_link_libraries(cmd_vel_serial_component PkgConfig::LibSerial)
ament_target_dependencies(cmd_vel_serial_component 
	rclcpp 
	rclcpp_components 
	geometry_msgs
)
target_include_directories(cmd_vel_serial_component PUBLIC src/include)

rclcpp_components_register_node(
	cmd_vel_serial_component
	PLUGIN "cmd_velSerialNode"
	EXECUTABLE cmd_vel_serial_node_exec
)

# ======= goal component ======
add_library(goal_serial_component SHARED src/goal_serial_node.cpp)
target_link_libraries(goal_serial_component 
	PkgConfig::LibSerial 
	${GEOLIB_LIBRARIES})
ament_target_dependencies(goal_serial_component
	rclcpp
	rclcpp_components
	geometry_msgs
	sensor_msgs
	nlohmann_json
)
target_include_directories(goal_serial_component PUBLIC 
	src/include
	${GEOLIB_INCLUDE_DIRS}
)

rclcpp_components_register_node(
	goal_serial_component
	PLUGIN "GoalSerialNode"
	EXECUTABLE goal_serial_node_exec
)

# FollowBotClientNode EXECUTABLE
add_executable(follow_path_client_node src/follow_path_client_node.cpp)
# Target dependencies for action client
ament_target_dependencies(follow_path_client_node
	rclcpp
	rclcpp_action
	nav2_msgs
	nav_msgs
	geometry_msgs
)
# Install the EXECUTABLE
install(TARGETS follow_path_client_node
	DESTINATION lib/${PROJECT_NAME}
)

install(
	TARGETS serial_manager_component
			serial_manager_node_exec
			imu_serial_component 
			imu_serial_node_exec
			encoder_serial_component
			encoder_serial_node_exec	
			gps_serial_component
			gps_serial_node_exec
			goal_serial_component
			goal_serial_node_exec
			cmd_vel_serial_component
			cmd_vel_serial_node_exec
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
	rclcpp 
	rclcpp_components
	rclcpp_action
	nav2_msgs
	nav_msgs
	sensor_msgs
	geometry_msgs
	nlohmann_json
)
ament_package()
