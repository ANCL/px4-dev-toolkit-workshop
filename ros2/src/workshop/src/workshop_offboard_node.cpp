#include "workshop_offboard_node.hpp"

using namespace std::chrono_literals;

WorkshopOffboardNode::WorkshopOffboardNode() : Node("workshop_offboard_node") {
  // Declare Parameters
  this->declare_parameter<double>("mass", 2.0);
  this->declare_parameter<double>("kx", 3.0);
  this->declare_parameter<double>("kv", 1.5);
  this->declare_parameter<std::string>("trajectory_mode", "circle"); // "circle", "figure_eight"

  this->get_parameter("mass", mass_);
  this->get_parameter("kx", kx_);
  this->get_parameter("kv", kv_);

  std::string mode_str;
  this->get_parameter("trajectory_mode", mode_str);
  
  // Set trajectory type based on parameter
  if (mode_str == "figure_eight") {
    traj_type_ = TrajectoryType::FIGURE_EIGHT;
  } else {
    traj_type_ = TrajectoryType::CIRCLE;
  }

  // Publishers
  
  // This publisher is used to send the offboard control mode to PX4
  // i.e. whether we are sending position, velocity, acceleration, attitude, or body rate commands
  offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
  
  // This publisher is used to send the desired trajectory setpoint to PX4
  trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);

  // This publisher is used to send vehicle commands to PX4 (e.g., arm, disarm, set mode)
  vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

  // Quality of Service Profile for Sensor Data
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);


  // === TO REMOVE ===
  // ============================================================================
  // TASK 1: Subscribe to ROS 2 topic for vehicle odometry so you can get the 
  // current position and velocity of the vehicle
  //
  // Hint 1: the topic name is "/fmu/out/vehicle_odometry"
  // Hint 2: the callback function is odometry_callback()
  // ============================================================================
  vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      qos,
      std::bind(&WorkshopOffboardNode::odometry_callback, this, std::placeholders::_1));
  
  // ================

  // Subscribe to the reference trajectory topic
  trajectory_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/workshop/reference_trajectory",
      10,
      std::bind(&WorkshopOffboardNode::reference_trajectory_callback, this, std::placeholders::_1));

  // control_loop_callback() will be called every 10ms (100Hz)
  timer_ = this->create_timer(10ms, std::bind(&WorkshopOffboardNode::control_loop_callback, this));
  start_time_ = this->now();
}

// ============================================================================
// TASK 2: Finish implementing the odometry_callback() function to get the 
// current velocity of the vehicle
// 
// ============================================================================
void WorkshopOffboardNode::odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  current_pos_ = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
  
  // === TO REMOVE ===
  current_vel_ = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
  // ================
  
  odom_received_ = true;
}

// ============================================================================
// TRAJECTORY GENERATORS
// ============================================================================

// Circle Trajectory Generator
void WorkshopOffboardNode::compute_circle_trajectory(double t, Eigen::Vector3d &pos_d, Eigen::Vector3d &vel_d, Eigen::Vector3d &acc_d) {
  double R = circle_radius_;
  double w = circle_omega_;

  // desired position, velocity, and acceleration for a circular trajectory
  // the circle is in the XY plane at a fixed altitude (circle_altitude_)
  pos_d = Eigen::Vector3d(R * std::cos(w * t), R * std::sin(w * t), circle_altitude_);
  vel_d = Eigen::Vector3d(-R * w * std::sin(w * t), R * w * std::cos(w * t), 0.0);
  acc_d = Eigen::Vector3d(-R * w * w * std::cos(w * t), -R * w * w * std::sin(w * t), 0.0);
}

// Step Trajectory Generator
void WorkshopOffboardNode::compute_step_trajectory(double t, Eigen::Vector3d &pos_d, Eigen::Vector3d &vel_d, Eigen::Vector3d &acc_d) {
  // Step trajectory: hover at a fixed position for 15 seconds, then step to a new position
  if (t < 15.0) {
    pos_d = Eigen::Vector3d(0.0, 0.0, circle_altitude_);
    vel_d = Eigen::Vector3d(0.0, 0.0, 0.0);
    acc_d = Eigen::Vector3d(0.0, 0.0, 0.0);
  } else {
    pos_d = Eigen::Vector3d(0.5, 0.5, circle_altitude_);
    vel_d = Eigen::Vector3d(0.0, 0.0, 0.0);
    acc_d = Eigen::Vector3d(0.0, 0.0, 0.0);
  }
}


// Figure-eight Trajectory Generator
void WorkshopOffboardNode::compute_figure_eight_trajectory(double t, Eigen::Vector3d &pos_d, Eigen::Vector3d &vel_d, Eigen::Vector3d &acc_d) {
  double A = circle_radius_;
  double B = circle_radius_ / 2.0;
  double w = circle_omega_;

  // desired position, velocity, and acceleration for a figure-eight trajectory
  pos_d = Eigen::Vector3d(A * std::sin(w * t), B * std::sin(w * t) * std::cos(w * t), circle_altitude_);
  vel_d = Eigen::Vector3d(A * w * std::cos(w * t), B * w * (std::cos(2.0 * w * t) - std::sin(2.0 * w * t)), 0.0);
  acc_d = Eigen::Vector3d(-A * w * w * std::sin(w * t), -2.0 * B * w * w * std::cos(2.0 * w * t), 0.0);
}

// ============================================================================
// MAIN CONTROL LOOP
// ============================================================================
void WorkshopOffboardNode::control_loop_callback() {
  // Send initial setpoints before switching to offboard mode
  // This is required by PX4 before it will allow offboard mode to be engaged
  publish_offboard_control_mode();

  if (offboard_setpoint_counter_ == 10) {
    // Request PX4 to switch to Offboard mode and arm vehicle
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  }

  if (offboard_setpoint_counter_ <= 10) {
    offboard_setpoint_counter_++;
    return;
  }

  if (!odom_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for Vehicle Odometry...");
    return;
  }

  // Compute desired trajectory (pos_d, vel_d, acc_d)
  double t = (this->now() - start_time_).seconds();
  Eigen::Vector3d pos_d = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_d = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_d = Eigen::Vector3d::Zero();

  switch (traj_type_) {
    case TrajectoryType::CIRCLE:
      compute_circle_trajectory(t, pos_d, vel_d, acc_d);
      break;
    case TrajectoryType::FIGURE_EIGHT:
      compute_figure_eight_trajectory(t, pos_d, vel_d, acc_d);
      break;
    case TrajectoryType::STEP:
      compute_step_trajectory(t, pos_d, vel_d, acc_d);
      break;
  }

  // Controller implementation

  // ============================================================================
  // TASK 4: Implement the acceleration command calculation in the control loop callback function
  // 
  // The acceleration command is calculated using the following formula:
  // a_cmd = a_d - (k_x / m) * e_x - (k_v / m) * e_v
  // ============================================================================
  
  Eigen::Vector3d a_cmd = Eigen::Vector3d::Zero();

  // === TO REMOVE ===
  Eigen::Vector3d e_x = current_pos_ - pos_d;
  Eigen::Vector3d e_v = current_vel_ - vel_d;

  a_cmd = acc_d - (kx_ / mass_) * e_x - (kv_ / mass_) * e_v;
  // ================

  // Publish acceleration command to PX4
  publish_trajectory_setpoint(a_cmd);
}

// ============================================================================
// PX4 HELPER FUNCTIONS
// ============================================================================
void WorkshopOffboardNode::publish_offboard_control_mode() {
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = true; // only giving acceleration commands to PX4
  msg.attitude = false;
  msg.body_rate = false;
  offboard_control_mode_publisher_->publish(msg);
}

void WorkshopOffboardNode::publish_trajectory_setpoint(const Eigen::Vector3d &acc_cmd) {
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.position = {NAN, NAN, NAN}; // we are not sending position commands to PX4
  msg.velocity = {NAN, NAN, NAN}; // same for velocity commands
  msg.acceleration = {
    static_cast<float>(acc_cmd.x()), 
    static_cast<float>(acc_cmd.y()), 
    static_cast<float>(acc_cmd.z())
  };
  trajectory_setpoint_publisher_->publish(msg);
}

void WorkshopOffboardNode::publish_vehicle_command(uint16_t command, float param1, float param2) {
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorkshopOffboardNode>());
  rclcpp::shutdown();
  return 0;
}