#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace distobee_cleverhive {

class CmdVelOdometry : public rclcpp::Node
{
public:
  CmdVelOdometry(const rclcpp::NodeOptions &options) : Node("cmd_vel_odometry", options)
  {
    // Initialize position and orientation
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    
    // Subscribe to cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel",
        10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          this->on_cmd_vel(msg);
        });
    
    // Publish odometry
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    
    last_update_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "cmd_vel_odometry node started");
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    
    if (dt <= 0.0) {
      return;  // Ignore if time didn't advance
    }
    
    // Extract linear and angular velocities
    double vx = msg->twist.linear.x;
    double vy = msg->twist.linear.y;
    double omega = msg->twist.angular.z;
    
    // Update pose by integration
    // For differential drive or holonomic robots:
    // If omega is significant, use circular arc integration
    // Otherwise use simple linear integration
    
    if (std::abs(omega) > 1e-6) {
      // Circular arc integration
      double radius = std::sqrt(vx * vx + vy * vy) / omega;
      double dtheta = omega * dt;
      
      // Current heading
      double theta_mid = theta_ + dtheta / 2.0;
      
      x_ += radius * std::sin(dtheta) * std::cos(theta_mid);
      y_ += radius * std::sin(dtheta) * std::sin(theta_mid);
      theta_ += dtheta;
    } else {
      // Linear integration
      double dx = vx * dt;
      double dy = vy * dt;
      
      x_ += dx * std::cos(theta_) - dy * std::sin(theta_);
      y_ += dx * std::sin(theta_) + dy * std::cos(theta_);
    }
    
    // Normalize theta to [-pi, pi]
    while (theta_ > M_PI) theta_ -= 2 * M_PI;
    while (theta_ < -M_PI) theta_ += 2 * M_PI;
    
    // Publish odometry message
    auto odometry_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odometry_msg->header.stamp = current_time;
    odometry_msg->header.frame_id = "odom";
    odometry_msg->child_frame_id = "base_link";
    
    // Set position
    odometry_msg->pose.pose.position.x = x_;
    odometry_msg->pose.pose.position.y = y_;
    odometry_msg->pose.pose.position.z = 0.0;
    
    // Set orientation as quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odometry_msg->pose.pose.orientation.x = q.x();
    odometry_msg->pose.pose.orientation.y = q.y();
    odometry_msg->pose.pose.orientation.z = q.z();
    odometry_msg->pose.pose.orientation.w = q.w();
    
    // Set velocity (from current cmd_vel message)
    odometry_msg->twist.twist.linear.x = vx;
    odometry_msg->twist.twist.linear.y = vy;
    odometry_msg->twist.twist.linear.z = 0.0;
    odometry_msg->twist.twist.angular.x = 0.0;
    odometry_msg->twist.twist.angular.y = 0.0;
    odometry_msg->twist.twist.angular.z = omega;
    
    // Set covariance (default values, can be tuned)
    const double pose_cov = 0.1;
    const double twist_cov = 0.1;
    
    std::fill(odometry_msg->pose.covariance.begin(), odometry_msg->pose.covariance.end(), 0.0);
    odometry_msg->pose.covariance[0] = pose_cov;    // x
    odometry_msg->pose.covariance[7] = pose_cov;    // y
    odometry_msg->pose.covariance[35] = pose_cov;   // theta
    
    std::fill(odometry_msg->twist.covariance.begin(), odometry_msg->twist.covariance.end(), 0.0);
    odometry_msg->twist.covariance[0] = twist_cov;  // vx
    odometry_msg->twist.covariance[7] = twist_cov;  // vy
    odometry_msg->twist.covariance[35] = twist_cov; // omega
    
    odometry_pub_->publish(*odometry_msg);
    
    last_update_time_ = current_time;
  }
  
  double x_, y_, theta_;
  rclcpp::Time last_update_time_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
};

} // namespace distobee_cleverhive

RCLCPP_COMPONENTS_REGISTER_NODE(distobee_cleverhive::CmdVelOdometry)
