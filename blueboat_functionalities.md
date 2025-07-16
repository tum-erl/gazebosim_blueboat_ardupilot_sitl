
# BlueBoat Interaction and Control Analysis

This document summarizes the analysis of the BlueBoat simulation workspace and outlines the functionalities available for interacting with and controlling the BlueBoat model.

## 1. Initial Workspace Analysis

The first step was to analyze the entire workspace to identify all functionalities related to the BlueBoat. This was done by searching for all files containing "BlueBoat" in their names.

The analysis revealed a set of files including documentation, ROS2 nodes, and simulation models. The most relevant files for understanding the BlueBoat's functionalities were identified as:

-   `SITL_Models/Gazebo/docs/BlueBoat.md`: The primary documentation file.
-   `gz_ws/extras_interface/ros2_blueboat_interface.py`: A ROS2 node for low-level control and interfacing with the BlueBoat.
-   `gz_ws/src/move_blueboat/move_blueboat/run_blueboat_mission.py`: A ROS2 node for autonomous mission execution.
-   `blueboat_sitl/blueboat_sitl.repos`: A file containing information about the repositories used in the simulation.

## 2. BlueBoat Functionalities

Based on the analysis of the identified files, the following functionalities are available for interacting with the BlueBoat:

### High-Level Control and Mission Execution:

-   **`run_blueboat_mission.py`**: This script provides autonomous navigation capabilities for the BlueBoat. It can be used to:
    -   **Follow a predefined path**: The script loads waypoints from a YAML file (`waypoints_asv.yaml`), generates a smooth spline path, and then follows that path using a line-of-sight (LOS) guidance algorithm.
    -   **Control motors**: It calculates and publishes thrust commands to the port and starboard motors to navigate the boat along the path.
    -   **Monitor position and orientation**: It subscribes to odometry data to get the boat's current position and yaw.

### Low-Level Control and Interfacing:

-   **`ros2_blueboat_interface.py`**: This script acts as a bridge between ROS2 and the ArduPilot running on the BlueBoat. It provides the following functionalities:
    -   **MAVLink Communication**: It establishes a MAVLink connection with the ArduPilot to send and receive data.
    -   **Joystick Control**: It can be used with a joystick to manually control the BlueBoat. It maps joystick axes to motor thrust commands and buttons to arm/disarm the vehicle.
    -   **ROS2 Topics**: It exposes several ROS2 topics for interacting with the BlueBoat:
        -   `/blueboat/cmd_vel`: Allows you to send `Twist` messages to control the boat's linear and angular velocity.
        -   `/{uuv_name}/send_port_motor_0_100_thrust` and `/{uuv_name}/send_stbd_motor_0_100_thrust`: Allows you to send thrust commands directly to the port and starboard motors.
        -   `/blueboat/servo_outputs`: Publishes the raw PWM values of the servo outputs.
        -   `/compass`: Publishes the compass heading.
        -   `/imu`: Publishes IMU data.
        -   `/blueboat_roll`, `/blueboat_pitch`, `/blueboat_yaw`: Publishes the roll, pitch, and yaw angles.
    -   **Arming and Disarming**: It provides functions to arm and disarm the vehicle.
    -   **Mode Setting**: It can set the vehicle's mode (e.g., STABILIZE).

### Simulation Environment:

-   **`BlueBoat.md`**: This file provides instructions on how to set up and run the BlueBoat simulation in Gazebo. It explains how to:
    -   **Launch the simulation**: It provides the `gz sim` command to launch the Gazebo world with the BlueBoat model.
    -   **Run the ArduPilot SITL**: It provides the `sim_vehicle.py` command to run the ArduPilot SITL and connect it to the Gazebo simulation.
    -   **Configure the simulation**: It mentions the use of a hydrodynamics plugin for wave simulation and provides some recommended ArduPilot parameters.

### Summary of Interaction Methods:

1.  **Autonomous Missions**: Use the `run_blueboat_mission.py` script to execute missions based on a set of waypoints.
2.  **Manual Control**: Use a joystick with the `ros2_blueboat_interface.py` script to manually control the boat's movement.
3.  **Programmatic Control via ROS2**:
    -   Publish `Twist` messages to the `/blueboat/cmd_vel` topic to control the boat's velocity.
    -   Publish thrust commands to the `/model/blueboat/joint/motor_port_joint/cmd_thrust` and `/model/blueboat/joint/motor_stbd_joint/cmd_thrust` topics to directly control the motors.
4.  **Data Monitoring**: Subscribe to the various ROS2 topics published by `ros2_blueboat_interface.py` to monitor the boat's status, including its position, orientation, and sensor data.
5.  **Simulation Environment**: Use the Gazebo simulation environment to test and visualize the BlueBoat's behavior in a virtual world.

## 3. Position Holding Controller

The possibility of writing a position-holding controller with the current setup was analyzed. The main challenge is the presence of two controllers: the ArduPilot and the new ROS node. To avoid conflicts, it is recommended to work with the ArduPilot's built-in functionalities.

### Option 1: The Recommended Approach (Leveraging ArduPilot's `GUIDED` Mode)

This is the most robust and standard way to achieve position holding.

1.  **Tell ArduPilot the target**: The ROS node sends a `SET_POSITION_TARGET_GLOBAL_INT` or similar MAVLink message to ArduPilot with the desired latitude and longitude.
2.  **Tell ArduPilot the mode**: The ROS node commands ArduPilot to switch to `GUIDED` mode.
3.  **Let ArduPilot do the work**: ArduPilot's internal, highly-tuned PID controllers then calculate the necessary motor outputs to move the boat to the target and hold it there, compensating for drift from wind or waves.

**System Tweaks Required**:

-   The `gz_ws/extras_interface/ros2_blueboat_interface.py` script needs to be modified to add functions for:
    1.  Setting the vehicle mode (e.g., to `GUIDED`).
    2.  Sending a position target command.

### Option 2: The Alternative Approach (Using `/cmd_vel`)

This approach is less direct and less robust than using `GUIDED` mode.

1.  **Create a PID Node**: A ROS node is written that:
    -   Subscribes to `/model/blueboat/odometry` to get the current position.
    -   Has a target position hardcoded or received from another topic.
    -   Calculates the X and Y error between the current and target positions.
    -   Uses a PID controller to convert this position error into a desired velocity (a `Twist` message).
    -   Publishes this `Twist` message to the `/blueboat/cmd_vel` topic.
2.  **ArduPilot's Role**: The `ros2_blueboat_interface.py` script will receive this `Twist` message and translate it into RC channel overrides, which ArduPilot (in `STABILIZE` or a similar mode) will then interpret to produce motor commands.

**System Tweaks Required**:

-   None. This would work with the existing scripts. However, you are essentially implementing a position controller on top of ArduPilot's velocity controller. Tuning your external PID gains while ArduPilot has its own internal gains can be tricky and may not perform as well as the native `GUIDED` mode.

### Summary and Recommendation

| Feature | Option 1: `GUIDED` Mode (Recommended) | Option 2: `cmd_vel` Controller |
| :--- | :--- | :--- |
| **How it Works** | Tell ArduPilot *where* to go. | Tell ArduPilot *how fast* to go. |
| **Pros** | - Extremely robust and stable. <br>- Uses ArduPilot's expert-tuned internal controllers. <br>- Simpler ROS node logic. | - Requires no changes to existing interface scripts. |
| **Cons** | - Requires modifying `ros2_blueboat_interface.py`. | - You are re-implementing a controller that already exists. <br>- Tuning can be complex and performance may be suboptimal. |
| **System Impact**| **Tweak:** Modify the ROS/MAVLink interface script. | **None:** Works with the current setup as-is. |

**Conclusion**: The recommended and most professional approach is **Option 1**. It leverages the power of the existing ArduPilot flight controller, resulting in better performance and reliability. It requires a small modification to the interface script to expose the necessary MAVLink commands to ROS.
