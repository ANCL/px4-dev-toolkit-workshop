# PX4 & ROS 2 Offboard Control Workshop Guide

This guide provides step-by-step instructions to set up the simulation environment, complete the missing node implementation tasks, build your ROS 2 workspace, and launch the offboard controller node.

---

## Environment Setup & Execution Flow

### Step 1: Launch the SITL Environment

Open a terminal and start the PX4 SITL simulation environment (Gazebo, QGC, and MicroXRCE-DDS agent inside TMUX):

```bash
tools/sitl start
```

---

### Step 2: Complete the Implementation Tasks

Go to `ros2/src/workshop/src/`

```bash
cd ros2/src/workshop/src/
```

Open `workshop_offboard_node.cpp` in a text editor and complete the four required implementation tasks marked with `// YOUR CODE HERE`:

#### Task 1: Subscribe to Vehicle Odometry

In the constructor, set up the ROS 2 subscriber for the topic `"/fmu/out/vehicle_odometry"`:

<details>
<summary>Answer</summary>

```cpp

vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", 
    qos,
    std::bind(&WorkshopOffboardNode::odometry_callback, this, std::placeholders::_1));

```
</details>


#### Task 2: Extract Current Velocity

Inside `odometry_callback()`, parse the current velocity vector from the incoming odometry message:

<details>
<summary>Answer</summary>

```cpp
current_vel_ = Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
```

</details>

#### Task 3: Implement Waypoint Trajectory Generation

In `compute_waypoints_trajectory()`, create logic to switch between 4 waypoints over time within the bounding box ($\vert{}x\vert{}, \vert{}y\vert{} \le 1.0$, $-1.2 < z < -0.6$).

<details>
<summary>Answer</summary>

```cpp
// Cycle through 4 waypoints every 10 seconds
int cycle = static_cast<int>(t / 10.0) % 4;

switch (cycle) {
    case 0: pos_d = Eigen::Vector3d(0.5, 0.5, -0.8); break;
    case 1: pos_d = Eigen::Vector3d(-0.5, 0.5, -1.0); break;
    case 2: pos_d = Eigen::Vector3d(-0.5, -0.5, -0.8); break;
    case 3: pos_d = Eigen::Vector3d(0.5, -0.5, -1.0); break;
}
```

</details>

#### Task 4: Implement Acceleration Control Law

In `control_loop_callback()`, calculate position error $e_x$, velocity error $e_v$, and compute the target acceleration vector $a_{cmd}$:

$$\mathbf{a}_{cmd} = \mathbf{a}_d - \frac{k_x}{m}\mathbf{e}_x - \frac{k_v}{m}\mathbf{e}_v$$

<details>
<summary>Answer</summary>

```cpp
Eigen::Vector3d e_x = current_pos_ - pos_d;
Eigen::Vector3d e_v = current_vel_ - vel_d;

Eigen::Vector3d a_cmd = acc_d - (kx_ / mass_) * e_x - (kv_ / mass_) * e_v;
```

</details>

---

### Step 3: Build the ROS 2 Package

Open a **second terminal** and compile the ROS 2 workspace:

```bash
ros2/build.sh
```

---

### Step 4: Source the Environment and Run the Node

1. Source the workspace environment variables:

```bash
source ros2/runtime_env.sh
```

2. Run the offboard control node with your chosen trajectory mode (`waypoints`, `circle`, or `figure_eight`):

* **Waypoints Mode:**

```bash
ros2 run workshop_offboard_controller workshop_offboard_node --ros-args -p trajectory_mode:="waypoints"

```

* **Circle Mode:**

```bash
ros2 run workshop_offboard_controller workshop_offboard_node --ros-args -p trajectory_mode:="circle"

```

* **Figure-Eight Mode:**

```bash
ros2 run workshop_offboard_controller workshop_offboard_node --ros-args -p trajectory_mode:="figure_eight"

```

---

## Quick Reference Summary

| Terminal | Command | Purpose |
| --- | --- | --- |
| **Terminal 1** | `tools/sitl start` | Launches Gazebo, QGC, and MicroXRCE-DDS |
| **Terminal 2** | `ros2/build.sh` | Compiles ROS 2 code |
| **Terminal 2** | `source ros2/runtime_env.sh` | Sets ROS 2 environment variables |
| **Terminal 2** | `ros2 run workshop_offboard_controller workshop_offboard_node --ros-args -p trajectory_mode:="waypoints"` | Executes controller node |
