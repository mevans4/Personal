#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class MissionNode : public rclcpp::Node
{
public:
    MissionNode() : Node("mission_node")
    {
    throughpose_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this,
      "navigate_through_poses");

    topose_client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose");

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 10, std::bind(&MissionNode::odometry_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/drone/status", 10);

    start_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/drone/cmd/start", 10,
        std::bind(&MissionNode::start_callback, this, std::placeholders::_1));

    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/drone/cmd/stop", 10,
        std::bind(&MissionNode::stop_callback, this, std::placeholders::_1));

    home_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/drone/cmd/return_home", 10,
        std::bind(&MissionNode::return_home_callback, this, std::placeholders::_1));    

    run_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MissionNode::run, this));

    while (!throughpose_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for action server to be available...");
    }
    while (!topose_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for action server to be available...");
    }
    RCLCPP_INFO(this->get_logger(), "Action server available.");

    // Publish initial status
    publish_status("Status: IDLE");

    run();
    }

private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr throughpose_client_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr topose_client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr run_timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr home_sub_;
    float flight_height = 0.5;
    float current_height = 0.0;
    bool takeoff_ = false;
    bool running_ = false;
    bool takeoff_complete = false;
    bool mission_complete = false;
    float error = 0.0;
    float vz;
    bool start_status = false;
    bool stop_status = false;
    bool home_status = false;
    int number_of_waypoints_left = 0;
    int number_of_waypoints_total = 0;
    int starting_waypoint = 0;
    std::string current_status_ = "Status: IDLE";

    void publish_status(const std::string& status) {
        std_msgs::msg::String msg;
        msg.data = status;
        status_pub_->publish(msg);
        current_status_ = status;
    }

    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_height = msg->pose.pose.position.z;
    }
    void start_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Start command received.");
            start_status = true;
            stop_status = false;
        }
    }
    void stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && start_status) {
            RCLCPP_INFO(this->get_logger(), "Stop command received.");
            start_status = false;
            stop_status = true;
            starting_waypoint = number_of_waypoints_total - number_of_waypoints_left;
        }
    }
    void return_home_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && stop_status && stop_status) {
            RCLCPP_INFO(this->get_logger(), "Return home command received.");
            home_status = true;
        }
    }
    void run()
    {
        if (!takeoff_complete && !running_ && start_status) {
            takeoff();
        }
        if (takeoff_complete && !running_ && !stop_status) {
            send_mission();
        }
        if (running_ && stop_status){
            stop_mission();
        }
        if (mission_complete || home_status) {
            return_home();
        }
    }
    void takeoff()
    {
        RCLCPP_INFO(this->get_logger(), "Taking off to height: %.2f meters", flight_height);
        // Implement takeoff logic here
        if (!takeoff_complete) {
            publish_status("Status: TAKING OFF");
            error = flight_height - current_height;
            if (error > 0.05) {
                vz = 0.3;
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.z = vz;
                cmd_vel_pub_->publish(cmd_vel);
            } else {
                vz = 0.0;
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.z = vz;
                cmd_vel_pub_->publish(cmd_vel);
                takeoff_complete = true;
                publish_status("Status: TAKEOFF COMPLETE");
            }
        }

    }

    void return_home()
    {
        publish_status("Status: RETURNING HOME");
        NavigateToPose::Goal home_pose_;
        home_pose_.pose.header.frame_id = "map";
        home_pose_.pose.pose.position.x = -21.529921570879463;
        home_pose_.pose.pose.position.y = -0.8135906984216432;
        home_pose_.pose.pose.orientation.z = 0.17897474707207528;
        home_pose_.pose.pose.orientation.w = 0.9838536679356776;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.feedback_callback =
        [this](GoalHandleToPose::SharedPtr,
               const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Nav2 feedback: distance_remaining = %.2f",
                feedback->distance_remaining);
        };

        send_goal_options.result_callback =
            [this](const GoalHandleToPose::WrappedResult & result) {
                RCLCPP_INFO(this->get_logger(), "Mission finished with result code: %d", result.code);
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    mission_complete = false;
                    home_status = false;
                    takeoff_complete = false;
                    publish_status("Status: IDLE");
                    RCLCPP_INFO(this->get_logger(), "Mission complete!");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Mission did not succeed!");
                    publish_status("Status: ERROR - Return home failed");
                }
            };
        // Send the goal to the action server
        topose_client_->async_send_goal(home_pose_, send_goal_options);
    }

    void stop_mission()
    {
        if (!running_) {
            RCLCPP_WARN(this->get_logger(), "No active mission to stop.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Stopping current mission...");
        publish_status("Status: STOPPED");

        auto cancel_future = throughpose_client_->async_cancel_all_goals();
        running_ = false;
        stop_status = true;
        takeoff_complete = false;

    }

    void send_mission()
    {
        publish_status("Status: FLYING MISSION");
        NavigateThroughPoses::Goal goal;

        geometry_msgs::msg::PoseStamped pose1, pose2, pose3;
        running_ = true;

        pose1.header.frame_id = "map";
        pose1.pose.position.x = 0.9557;
        pose1.pose.position.y = 7.5768;
        pose1.pose.orientation.z = 0.17897474707207528;
        pose1.pose.orientation.w = 0.9838536679356776;

        pose2.header.frame_id = "map";
        pose2.pose.position.x = 2.3621;
        pose2.pose.position.y = 3.8324;
        pose2.pose.orientation.z = 0.9838536679356776;
        pose2.pose.orientation.w = -0.17897474707207522;

        pose3.header.frame_id = "map";
        pose3.pose.position.x = -20.121241879882;
        pose3.pose.position.y = -4.557335017706;
        pose3.pose.orientation.z = 0.9838536679356776;
        pose3.pose.orientation.w = -0.17897474707207522;  

        std::vector<geometry_msgs::msg::PoseStamped> all_poses = {pose1, pose2, pose3};

        // Add poses starting from the starting_waypoint
        for (int i = starting_waypoint; i < all_poses.size(); ++i) {
            goal.poses.push_back(all_poses[i]);
        }

        number_of_waypoints_total = all_poses.size();
        RCLCPP_INFO(this->get_logger(), "Sending goal with %zu waypoints, starting from waypoint %d", 
                    goal.poses.size(), starting_waypoint);

        auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    
        send_goal_options.feedback_callback =
        [this](GoalHandle::SharedPtr,
               const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Number of waypoints remaining %u, distance_remaining = %.2f",
                number_of_waypoints_left = feedback->number_of_poses_remaining,
                feedback->distance_remaining);

        };

        send_goal_options.result_callback =
            [this](const GoalHandle::WrappedResult & result) {
                RCLCPP_INFO(this->get_logger(), "Mission finished with result code: %d", result.code);
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    mission_complete = true;
                    publish_status("Status: MISSION COMPLETE");
                    RCLCPP_INFO(this->get_logger(), "Mission complete!");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Mission did not succeed!");
                    publish_status("Status: ERROR - Mission failed");
                }
            };

        // Actually send and store the goal future
        auto goal_future = throughpose_client_->async_send_goal(goal, send_goal_options);


    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}