#!/usr/bin/env python3

from rclpy.node import Node
import time # Time library
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
import os
import sys

file_dir = os.path.dirname(__file__)
sys.path.append(file_dir)
from robot_navigator import BasicNavigator
 
class Path_Making(Node):

    def __init__(self):
        super().__init__("nav_through_poses")
        self.get_logger().info("Hello from ROS2")

def main(args=None):
    rclpy.init(args=args)
    node = Path_Making()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # Where robot should go
    goal_poses = []
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -5.0
    goal_pose.pose.position.y = -4.2
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    goal_poses.append(goal_pose)

    navigator.goThroughPoses(goal_poses)

    while not navigator.isNavComplete():
        # Print the current status of the navigation
        status = navigator.getState()
        node.get_logger().info(f'Current navigation state: {status}')
        time.sleep(1)
        
    # Do something depending on the return code
    result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')
    print("Result: " + result)
    
    # Close the ROS 2 Navigation Stack
    navigator.lifecycleShutdown()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()