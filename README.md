# TurtleNav-ROS2-Path-Planning-and-Monitoring


This project focuses on path planning, waypoint management, and motion tracking for mobile robots using a TurtleBot3 simulation in ROS 2 Humble with Gazebo. Students will develop a system that enables the robot to move between specified waypoints, record movement data, and analyze the robot's performance. The project also covers continuous looping (cyclic movement), distance calculations using sensor data, and comprehensive data logging.

---

## Table of Contents

- [Project Purpose](#project-purpose)
- [Requirements](#requirements)
- [Project Steps and Technical Details](#project-steps-and-technical-details)
  - [1. Environment Setup and Simulation Preparation](#1-environment-setup-and-simulation-preparation)
  - [2. Waypoint Definition](#2-waypoint-definition)
  - [3. Reading Waypoints and Action Execution](#3-reading-waypoints-and-action-execution)
  - [4. Task Outcome Recording](#4-task-outcome-recording)
  - [5. Continuous Tour and Cyclic Movement](#5-continuous-tour-and-cyclic-movement)
  - [6. Distance Calculation and Monitoring](#6-distance-calculation-and-monitoring)
  - [7. Final Data Aggregation and Analysis](#7-final-data-aggregation-and-analysis)
- [Technical Tips and Recommendations](#technical-tips-and-recommendations)
- [Delivery and File Management](#delivery-and-file-management)
- [Project Outcomes and Evaluation](#project-outcomes-and-evaluation)
- [Extra Information and Suggestions](#extra-information-and-suggestions)
- [License](#license)

---

## Project Purpose

- **Objective:**
  - Learn and implement path planning, waypoint management, and motion tracking on a TurtleBot3 simulation using ROS 2 Humble and Gazebo.
- **Goals:**
  - Develop a system that navigates the robot between predefined waypoints.
  - Record sensor and movement data during navigation.
  - Analyze the robot’s performance by evaluating planned paths and actual movement data.
  - Gain hands-on experience with ROS 2 topics, actions, and JSON-based data logging.

---

## Requirements

- **Software:**
  - ROS 2 Humble
  - TurtleBot3 Simulation packages
  - Gazebo Simulation Software
- **Hardware:**
  - A computer capable of running ROS 2 and Gazebo simulations

---

## Project Steps and Technical Details

### 1. Environment Setup and Simulation Preparation
- **Install ROS 2 Humble:**
  - Follow the official ROS 2 Humble installation guide.
- **Install TurtleBot3 Packages:**
  - Update package list:
    - `sudo apt update`
  - Install TurtleBot3 package:
    - `sudo apt install ros-humble-turtlebot3`
  - Configure your TurtleBot3 model by adding the following line to your `~/.bashrc` file:
    - `export TURTLEBOT3_MODEL=burger`
- **Set Up Gazebo Simulation:**
  - Create a new map using Gazebo or use an existing map that represents the robot's operating environment with obstacles and open areas.

### 2. Waypoint Definition
- **Define Waypoints:**
  - Choose 10 waypoints on the simulation map where the robot is required to navigate.
- **Determine Waypoint Coordinates:**
  - Use the Gazebo graphical interface or ROS commands to set the (x, y) coordinates and orientation (w) for each waypoint.
- **Save Waypoints to JSON:**
  - Create a JSON file named with your student number (e.g., `waypoints_123456.json`) with the following structure:
    ```json
    {
      "waypoints": [
        {"id": 1, "x": 1.0, "y": 2.0, "w": 0},
        {"id": 2, "x": 3.5, "y": 4.0, "w": 1.57},
        ...
        {"id": 10, "x": 5.0, "y": 1.5, "w": 2}
      ]
    }
    ```

### 3. Reading Waypoints and Action Execution
- **Read Waypoints from JSON:**
  - Within a ROS node, use Python’s `json` module to load the waypoints from `waypoints_[student_number].json`.
- **Connect to the TurtleBot3 Action Server:**
  - Utilize the NavigateToPose action (or similar) to command the robot to move to each waypoint.
- **Execute Action and Process Feedback:**
  - For every waypoint:
    - Send an action goal to the action server.
    - Process feedback to capture the planned path length.
  - Save the path lengths in a JSON file (e.g., `path_lengths_123456.json`) using the structure:
    ```json
    {
      "waypoint_paths": [
        {"waypoint_id": 1, "planned_length": 2.5},
        {"waypoint_id": 2, "planned_length": 3.0},
        ...
        {"waypoint_id": 10, "planned_length": 1.8}
      ]
    }
    ```

### 4. Task Outcome Recording
- **Monitor Action Results:**
  - Each waypoint navigation can result in:
    - **Success:** The robot reached the target waypoint.
    - **Failure:** The robot could not reach the waypoint due to obstacles or errors.
    - **Canceled:** The task was canceled by the user.
- **Log Task Results:**
  - Record the outcome along with the robot’s current position in a JSON file (e.g., `task_results_123456.json`):
    ```json
    {
      "tasks": [
        {
          "waypoint_id": 1,
          "result": "success",
          "current_position": {"x": 1.0, "y": 2.0, "theta": 0.0}
        },
        ...
      ]
    }
    ```

### 5. Continuous Tour and Cyclic Movement
- **Implement Cyclic Navigation:**
  - After completing all 10 waypoints, instruct the robot to return to the first waypoint.
  - Ensure that the system continuously repeats the tour, allowing for long-term performance monitoring.

### 6. Distance Calculation and Monitoring
- **Subscribe to ROS Topics:**
  - Use the `/odom` topic to obtain odometric data.
  - Use the `/amcl_pose` topic to acquire the robot’s estimated pose from AMCL.
- **Calculate Distances:**
  - Compute the Euclidean distance between successive waypoints using the formula:
    ```python
    import math
    def calculate_distance(x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    ```
- **Save Distance Data:**
  - Save distances calculated from `/odom` data to `distance_odom_[student_number].json`:
    ```json
    {
      "distances": [
        {"from_waypoint": 1, "to_waypoint": 2, "distance": 2.5},
        ...
      ],
      "total_distance": 25.0
    }
    ```
  - Save distances calculated from `/amcl_pose` data to `distance_amcl_[student_number].json`:
    ```json
    {
      "distances": [
        {"from_waypoint": 1, "to_waypoint": 2, "distance": 2.7},
        ...
      ],
      "total_distance": 26.5
    }
    ```

### 7. Final Data Aggregation and Analysis
- **Aggregate All Data:**
  - At the end of a complete tour, compile all collected data (waypoints, path lengths, task results, and distance calculations) into a single JSON summary file (e.g., `tour_summary_[student_number].json`):
    ```json
    {
      "waypoints": [...],
      "path_lengths": [...],
      "task_results": [...],
      "distance_odom": {...},
      "distance_amcl": {...},
      "total_distance_odom": 25.0,
      "total_distance_amcl": 26.5
    }
    ```
- **Analyze Discrepancies:**
  - Compare distances from odom and AMCL sources.
  - Discuss potential errors due to odometric drift and sensor fusion differences.

---

## Technical Tips and Recommendations

- **ROS Topic Subscriptions:**
  - Use `rclpy` (Python) or `rclcpp` (C++) for subscribing to topics like `/odom` and `/amcl_pose`.
  - Example for subscribing to `/odom`:
    ```python
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry

    class OdomSubscriber(Node):
        def __init__(self):
            super().__init__('odom_subscriber')
            self.subscription = self.create_subscription(
                Odometry,
                '/odom',
                self.listener_callback,
                10
            )
        
        def listener_callback(self, msg):
            # Process odometry data for distance calculation
            pass
    ```
- **Action Client Usage:**
  - Use the NavigateToPose action for robot navigation.
  - Process action feedback to record planned path lengths.
- **Data Logging:**
  - Use Python’s `json` module to write data to JSON files.
  - Ensure proper file naming conventions by including your student number.
- **Distance Calculation:**
  - Use the Euclidean distance formula to calculate distances between waypoints.

---

## Delivery and File Management

- **Files to be Delivered:**
  - **Source Code:**  
    - All Python/C++ source files (e.g., in `src/` or `scripts/` directories).
  - **Map Files:**  
    - The map files (e.g., `map.pgm`, `map.yaml`).
  - **JSON Files:**  
    - `waypoints_[student_number].json`
    - `path_lengths_[student_number].json`
    - `task_results_[student_number].json`
    - `distance_odom_[student_number].json`
    - `distance_amcl_[student_number].json`
    - `tour_summary_[student_number].json`
  - **Project Details File:**  
    - A text file (e.g., `project_details_[student_number].txt`) documenting subscriptions, publications, actions, and learning outcomes.
- **File Naming Conventions:**
  - Ensure all JSON and text files include your student number.
- **Zip Archive:**
  - Package all required files and directories in a single zip file (e.g., `mobile_robot_project_123456.zip`).
  - Exclude directories generated during build/compilation (e.g., `build/`, `devel/`, `install/`).

---

## Project Outcomes and Evaluation

- **Data Analysis:**
  - Analyze the collected data to evaluate the robot’s navigation performance.
  - Compare differences between odom and AMCL distance measurements.
- **Performance Improvement:**
  - Use the analyzed data to suggest improvements in navigation algorithms.
  - Identify sources of error such as sensor drift and implement potential corrective measures.
- **Reporting:**
  - Prepare a detailed report that includes:
    - An explanation of the project steps.
    - Data visualizations (e.g., graphs of distance measurements).
    - Discussions on the discrepancies between measurement methods.
    - Recommendations for future improvements.

---

## Extra Information and Suggestions

- **File Organization:**
  - Organize your project files in a clear folder structure, for example:
    ```
    mobile_robot_project_123456.zip
    ├── src/
    │   ├── navigate_client.py
    │   └── odom_subscriber.py
    ├── maps/
    │   ├── map.pgm
    │   └── map.yaml
    ├── waypoints_123456.json
    ├── path_lengths_123456.json
    ├── task_results_123456.json
    ├── distance_odom_123456.json
    ├── distance_amcl_123456.json
    ├── tour_summary_123456.json
    └── project_details_123456.txt
    ```
- **Exclude Unnecessary Files:**
  - Do not include temporary build folders (e.g., `build/`, `devel/`, `install/`).
- **Optimize File Size:**
  - Exclude unnecessary logs or temporary data files from the zip archive.

---

## License

- This project is provided under the MIT License. For complete details, please refer to the LICENSE file included in the project.

