from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='serial_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='arduino_serial',
                    plugin='SerialManagerNode',
                    name='serial_manager'
                ),
                ComposableNode(
                    package='arduino_serial',
                    plugin='IMUSerialNode',
                    name='imu_serial_component',
                    parameters=[{'timer_period_ms': 10}]
                ),
                ComposableNode(
                    package='arduino_serial',
                    plugin='EncoderSerialNode',
                    name='encoder_serial_component',
                    parameters=[{'timer_period_ms': 20}]
                ),
                ComposableNode(
                    package='arduino_serial',
                    plugin='GPSSerialNode',
                    name='gps_serial_component',
                    parameters=[{'timer_period_ms': 1000}]
                ),
                ComposableNode(
                    package='arduino_serial',
                    plugin='cmd_velSerialNode',
                    name='cmd_vel_serialcomponent',
                    parameters=[{'timer_period_ms': 1000}]
                ),
            ],
            output='screen',
        )
    return LaunchDescription([container])
