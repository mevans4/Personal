#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class DroneController : public rclcpp::Node
{
public:
    DroneController() : Node("drone_controller")
    {
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&DroneController::run, this));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 1000, std::bind(&DroneController::odoCallback,this,_1));
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&DroneController::goalPoseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Drone controller node started.");
    }

private:
    bool first_run_ = true;

    void odoCallback(const nav_msgs::msg::Odometry& msg)
    { 
        pose_ = msg;
    }

    nav_msgs::msg::Odometry getOdometry(void)
    {
        return pose_;
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        latest_goal_pose_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received goal pose: x=%.2f, y=%.2f, z=%.2f",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    void run()
    {
        // Wait for odometry update only on the first run
        if (first_run_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for odometry update...");
            rclcpp::sleep_for(std::chrono::seconds(5));
            first_run_ = false;
        }

        size_t current_waypoint_index = 0;
        current_pose = getOdometry();
        RCLCPP_INFO(this->get_logger(), "Current Pose: x=%.2f, y=%.2f, z=%.2f",
                    current_pose.pose.pose.position.x,
                    current_pose.pose.pose.position.y,
                    current_pose.pose.pose.position.z);

        auto msg = geometry_msgs::msg::Twist();

        Coordinate goal1 = {4, 26, 0.5, 0, 0, 1.57};
        Coordinate goal2 = {8, 26, 0.5, 0, 0, -1.57};

        std::vector<Coordinate> waypoints = {goal1, goal2};

        double dx = goal1.x - current_pose.pose.pose.position.x;
        double dy = goal1.y - current_pose.pose.pose.position.y;
        RCLCPP_INFO(this->get_logger(), "Delta: dx=%.2f, dy=%.2f", dx, dy);
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < 0.3) // Threshold to consider waypoint reached
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached.");
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.linear.z = 0.0;
        }
        else
        {
            // Simple proportional controller to move towards the waypoint
            msg.linear.x = (dx / distance) * 0.5; // Scale speed
            msg.linear.y = (dy / distance) * 0.5;
            //msg.linear.z = std::clamp(height_ - current_pose.pose.pose.position.z, -1.0, 1.0);
            msg.linear.z = 0.0; // Maintain current altitude
            
            // Yaw control can be added here if needed
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = 0.0;
        }

        sendCmd(msg);
    }

    // void publish_command()
    // {
    //     auto msg = geometry_msgs::msg::Twist();

    //     // Hover command: Apply slight upward force to maintain altitude
    //     msg.linear.x = 0.3;
    //     msg.linear.y = 0.0;
    //     msg.linear.z = 0.0;  // Adjust this value depending on gravity & tuning

    //     msg.angular.x = 0.0;
    //     msg.angular.y = 0.0;
    //     msg.angular.z = 0.0;

    //     sendCmd(msg);
    // }

    void sendCmd(const geometry_msgs::msg::Twist& cmd)
    {
        vel_publisher_->publish(cmd);
    }

    struct Coordinate
    {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };

    int height_ = 0.5;
    geometry_msgs::msg::PoseStamped latest_goal_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    nav_msgs::msg::Odometry current_pose;
    nav_msgs::msg::Odometry pose_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}
