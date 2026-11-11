#ifndef WORKSHOP_OFFBOARD_NODE_HPP_
#define WORKSHOP_OFFBOARD_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

enum class TrajectoryType {
  CIRCLE,
  WAYPOINTS,
  FIGURE_EIGHT
};

class WorkshopOffboardNode : public rclcpp::Node {
public:
  WorkshopOffboardNode();

private:
  // Timer callback (Main control loop at 50 Hz)
  void control_loop_callback();

  // Trajectory Generators
  void compute_circle_trajectory(double t, Eigen::Vector3d &pos_d, Eigen::Vector3d &vel_d, Eigen::Vector3d &acc_d);
  void compute_waypoints_trajectory(double t, Eigen::Vector3d &pos_d, Eigen::Vector3d &vel_d, Eigen::Vector3d &acc_d);
  void compute_figure_eight_trajectory(double t, Eigen::Vector3d &pos_d, Eigen::Vector3d &vel_d, Eigen::Vector3d &acc_d);

  // Helper Functions for PX4 Offboard Communication
  void publish_offboard_control_mode();
  void publish_trajectory_setpoint(const Eigen::Vector3d &acc_cmd);
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

  // Callbacks
  void odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void reference_trajectory_callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);

  // ROS 2 Subscriptions & Publishers
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // State Variables
  Eigen::Vector3d current_pos_{0.0, 0.0, 0.0};
  Eigen::Vector3d current_vel_{0.0, 0.0, 0.0};

  bool odom_received_{false};
  uint64_t offboard_setpoint_counter_{0};
  rclcpp::Time start_time_;

  // Drone Parameters & Controller Gains
  double mass_{2.0};       // Drone mass [kg]
  double kx_{3.0};         // Position gain [N/m]
  double kv_{1.5};         // Velocity gain [N*s/m]
  
  // Circle Parameters
  double circle_radius_{1.0};
  double circle_omega_{0.5};
  double circle_altitude_{-1.2}; // NED Frame: Negative is UP

  // Selected Mode
  TrajectoryType traj_type_{TrajectoryType::CIRCLE};
};

#endif // WORKSHOP_OFFBOARD_NODE_HPP_