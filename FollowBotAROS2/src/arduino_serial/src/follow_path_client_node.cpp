/**
 * By: Frank Vanris
 * Date: 7/21/2025
 * Desc: Creating an action client to communicate with the Nav2
 * action server (will test things out with dummy code)
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <mav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <thread>

/**
 * @brief ROS2 Action Client for Nav2's FollowPath action.
 *
 * This node demonstrates how to send a Path goal to the Nav2 controller's
 * action server (`controller_server/follow_path`), receive feedback,
 * and process the final result of the action.
 *
 * The `FollowPath` action takes a `nav_msgs/msg/Path` as its goal,
 * which is a sequence of desired robot poses. The `controller_server`
 * then generates `geometry_msgs/msg/Twist` commands on `/cmd_vel`
 * to make the robot follow this path.
 */
class FollowPathClientNode : public rclcpp::Node {
public:

    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    // Constructor
    explicit FollowPathClientNode(const rclcpp::NodeOptions & options) : 
        Node("follow_path_client_node", options) {
            // Creating the action client
            this->client_ptr_ = rclcpp_action::create_client<FollowPath>(
                this,
                "controller_server/follow_path");

            RCLCPP_INFO(this->get_logger(), "FollowPathClientNode created. Waiting for action server...");
    }

    /// @brief Sends a path goal to the action server.
    /// @param path The path to follow.
    void send_path_goal(const nav_msgs::msg::Path& path) {
        // wait for the action server to be available
        if(!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server 'controller_server/follow_path' not available after waiting.");
            return;
        }

        // Create the goal message
        auto goal_msg = FollowPath::Goal();
        goal_msg.path = path;

        goal_msg.controller_id = "FollowPath";
        goal_msg.smoother_id = "SimpleSmoother";

        RCLCPP_INFO(this->get_logger(), "Sending FollowPath goal with %zu poses...", goal_msg.path.poses.size());

        // Set up options for sending the goal including callbacks
        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&FollowPathClientNode::goal_response_callback, this, std::placeholders::_1);

        send_goal_options.feedback_callback = 
            std::bind(&FollowPathClientNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback = 
            std::bind(&FollowPathClientNode::result_callback, this, std::placeholders::_1);

        // Asynchronously send the goal.
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;

    /// @brief Callback for goal response.
    /// @param goal_handle The goal handle.
    void goal_response_callback(const GoalHandleFollowPath::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by action server.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server. Goal ID: %s", goal_handle->get_goal_id().uuid.str().c_str());
        }
    }

    /// @brief Callback for feedback.
    /// @param goal_handle The goal handle.
    /// @param feedback The feedback message.
    void feedback_callback(
        GoalHandleFollowPath::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Received feedback: Current pose (x,y): (%.2f, %.2f)",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y);
    }

    /// @brief Callback for result.
    /// @param result The result message.
    void result_callback(const GoalHandleFollowPath::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "FollowPath action SUCCEEDED!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "FollowPath action ABORTED.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "FollowPath action CANCELED.");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code for FollowPath action.");
                break;
        }

        rclcpp::shutdown();
    }
};

// Main function (Don't know how else to configure yet for our project)
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Creating a node option object
    rclcpp::NodeOptions options;
    auto client_node = std::make_shared<FollowPathClientNode>(options);

    // Creating a dummy path for demonstration purposes and testing purposes
    nav_msgs::msg::Path path_to_follow;
    path_to_follow.header.frame_id = "map"; // Important: the frame ID must match the global frame
    path_to_follow.header.stamp = client_node->now();

    // defining a few poses to form a simple path
    geometry_msgs::msg::PoseStamped pose1, pose2, pose3, pose4, pose5;

    // Pose 1: starts at (0,0)
    pose1.header.frame_id = "map";
    pose1.header.stamp = client_node->now();
    pose1.pose.position.x = 0.0;
    pose1.pose.position.y = 0.0;
    pose1.pose.orientation.w = 1.0;

    // Pose 2: moves 1 meter forward
    pose2.header.frame_id = "map";
    pose2.header.stamp = client_node->now();
    pose2.pose.position.x = 1.0;
    pose2.pose.position.y = 0.0;
    pose2.pose.orientation.w = 1.0;

    // Pose 3: turns and move 1 meter left
    pose3.header.frame_id = "map";
    pose3.header.stamp = client_node->now();
    pose3.pose.position.x = 1.0;
    pose3.pose.position.y = 1.0;
    pose3.pose.orientation.w = 1.0;

    // Pose 4: Move 1 meter backward (in X)
    pose4.header.frame_id = "map";
    pose4.header.stamp = client_node->now();
    pose4.pose.position.x = 0.0;
    pose4.pose.position.y = 1.0;
    pose4.pose.orientation.w = 1.0;

    // Pose 5: Return to start (in Y)
    pose5.header.frame_id = "map";
    pose5.header.stamp = client_node->now();
    pose5.pose.position.x = 0.0;
    pose5.pose.position.y = 0.0;
    pose5.pose.orientation.w = 1.0;

    // Adding the poses to the path
    path_to_follow.poses.push_back(pose1);
    path_to_follow.poses.push_back(pose2);
    path_to_follow.poses.push_back(pose3);
    path_to_follow.poses.push_back(pose4);
    path_to_follow.poses.push_back(pose5);

    // Sent the constructed path goal
    client_node->send_path_goal(path_to_follow);

    rclcpp::spin(client_node);
    rclcpp::shutdown();
    return 0;
}