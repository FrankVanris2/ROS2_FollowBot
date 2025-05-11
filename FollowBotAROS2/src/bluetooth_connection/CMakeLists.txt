cmake_minimum_required(VERSION 3.8)
project(bluetooth_connection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(BLUETOOTH REQUIRED bluez)
pkg_check_modules(GEOLIB REQUIRED geographiclib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(bluetooth_gps_node src/bluetooth_gps_node.cpp)

target_link_libraries(bluetooth_gps_node
	${BLUETOOTH_LIBRARIES}  # from libbluetooth-dev
	${GEOLIB_LIBRARIES}
)
ament_target_dependencies(bluetooth_gps_node
	rclcpp
	geometry_msgs
	sensor_msgs
)

target_include_directories(bluetooth_gps_node PUBLIC
	${BLUETOOTH_INCLUDE_DIRS}
	${GEOLIB_INCLUDE_DIRS}
)

install(
	TARGETS bluetooth_gps_node
	DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
	rclcpp
	geometry_msgs
	sensor_msgs
)

ament_package()
