#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class AltitudeController : public rclcpp::Node
{
public:
    AltitudeController() : Node("altitude_controller")
    {
        target_altitude_ = 0.5;
        kp_ = 1.0;      
        ki_ = 0.2;      
        kd_ = 0.3;      
        max_vz_ = 0.5;  

        nav_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10,
            std::bind(&AltitudeController::navVelCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            std::bind(&AltitudeController::odomCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        lastTime = this->now();
    }

private:
    double target_altitude_;
    double kp_;
    double ki_;
    double kd_;
    double max_vz_;

    double current_altitude_ = 0.0;
    double integral_error_ = 0.0;      
    double previous_error_ = 0.0;  

    rclcpp::Time lastTime; 
    geometry_msgs::msg::Twist lastNav_vel_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_altitude_ = msg->pose.pose.position.z;
    }

    void navVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        lastNav_vel_ = *msg;
        geometry_msgs::msg::Twist combined = *msg;

        // Calculate time difference between last readings, used for the ki & kd
        rclcpp::Time currentTime = this->now();
        double dt = (currentTime - lastTime).seconds();

        // Safety barrier for when the sim first starts, and if it is laggy, defaults to 50Hz
        if (dt <= 0.0 || dt > 1.0)(
            dt = 0.02
        ); 

        lastTime = currentTime;

        double error = target_altitude_ - current_altitude_;
        
        // Integral term 
        integral_error_ += error * dt;
        
        // Anti-windup to prevent the integral term from being too large
        if (integral_error_ > 2.0)( 
            integral_error_ = 2.0
        );  

        if (integral_error_ < -2.0)( 
            integral_error_ = -2.0
        );
        
        // Derivative term
        double derivative_error = (error - previous_error_) / dt;
        previous_error_ = error;
        
        // PID output
        double vz = kp_ * error + ki_ * integral_error_ + kd_ * derivative_error;
        
        // Clamp for safety
        if (vz > max_vz_)(
            vz = max_vz_
        );

        if (vz < -max_vz_)(
            vz = -max_vz_
        );
        
        combined.linear.z = vz;

        cmd_vel_pub_->publish(combined);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AltitudeController>());
    rclcpp::shutdown();
    return 0;
}