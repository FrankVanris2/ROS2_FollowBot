from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ==================== #
    #   paths and params   #
    # ==================== #

    mapping_pkg = get_package_share_directory('mapping')
    ldlidar_pkg = get_package_share_directory('ldlidar_sl_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    slam_pkg = get_package_share_directory('slam_toolbox')

    # URDF path
    urdf_path = PathJoinSubstitution([mapping_pkg, 'urdf', 'followbot.urdf.xacro'])

    # config files
    slam_config = PathJoinSubstitution([mapping_pkg, 'config', 'mapper_params_online_async.yaml'])
    ekf_config = PathJoinSubstitution([mapping_pkg, 'config', 'dual_ekf_navsat_params.yaml'])
    nav2_config = PathJoinSubstitution([mapping_pkg, 'config', 'nav2_params.yaml'])

    rviz_config = PathJoinSubstitution([mapping_pkg, 'rviz2', 'navigation.rviz'])
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command([
                        'xacro ',
                        urdf_path
                    ]),
                    value_type=str
                ),
                'use_sim_time': use_sim_time
            }]
        ),


        # LDLidar driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([ldlidar_pkg, 'launch', 'ld14p.launch.py'])
            ])
        ),

        # `robot_localization` EKF and NavSat nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([mapping_pkg, 'launch', 'dual_ekf_navsat.launch.py'])
            ]),
            launch_arguments={
                'params_file': ekf_config,
                'use_sim_time': use_sim_time
            }.items()
        ),

        # `slam_toolbox`
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([slam_pkg, 'launch', 'online_async_launch.py'])
            ]),
            launch_arguments={
                'params_file': slam_config,
                'slam_params_file': slam_config,
                'publish_map': 'true', 
                'use_sim_time': use_sim_time
            }.items()
        ),

        # delayed `nav2` launch (ensure SLAM is ready)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([nav2_pkg, 'launch', 'bringup_launch.py'])
                    ]),
                    launch_arguments={
                        'params_file': nav2_config,
                        'use_sim_time': use_sim_time,
                        'autostart': 'true'
                    }.items()
                )
            ]
        ),

        LogInfo(msg="Launch complete.")
    ])
