# SAE Abaja 2026: Autonomous Lane Keeping Assist (LKA) System

## 1. Project Overview
This repository contains the complete software stack for the SAE Abaja 2026 Lane Keeping Assist (LKA) system. The architecture is designed to bridge a **Windows Host (IPG CarMaker)** with a **WSL2 Ubuntu 22.04 environment (ROS 2 Humble)**.

The system features real-time neural network lane detection, a finely-tuned Stanley lateral controller, and a fully automated performance logging suite that generates BAJA SAEINDIA Article K.2 compliant dashboards.

---

## 2. Current Status & Known Issues

**Recent Fixes (v4.2 - Optimization Update):**
- **UFLDv2 0 Lanes Fixed**: Adjusted row and column anchor thresholds to allow the UFLDv2 model to reliably detect lane boundaries on synthetic CarMaker images with varying horizons.
- **UFLDv2 Load Time Optimized**: The massive PyTorch parsingNet initialization overhead was eliminated by converting the model into a localized TorchScript binary (`ufldv2_traced.pt`), dropping load times from ~2.8s to <0.8s.
- **Steering ISO Convention Fixed**: Inverted the Stanley steering output to properly conform to CarMaker's ISO 8855 standard (positive turns left).
- **Graceful Cruise Control Implemented**: Replaced the harsh 30 km/h emergency limiter that caused engine stalling with a proportional 10 km/h cruise control that explicitly unhooks the IPG Driver while LKA is active, allowing for smooth, continuous autonomous driving.
- **Controller Stability**: Disabled curvature feedforward, lowered base gains, and restricted the steering ratio mapping to 180 degrees to eliminate extreme cornering oversteer and twitchiness.

**Known Issues:**
- None at the moment. System is stable.

---

## 3. Core Architecture

### Control Strategy
| Control Axis | Who Controls | How |
|---|---|---|
| **Steering (Lateral)** | ROS2 Stanley Controller | UFLDv2 detects lanes → Stanley computes steering → DVA writes `VC.Steer.Ang` |
| **Throttle/Brake (Longitudinal)** | ROS2 Cruise Control / IPG Driver | While LKA is connected (`latswitch=1`), bridge assumes graceful 10 km/h cruise control. If LKA is offline, control passes back to IPG Driver via DVA `Off` mode. |

### Data Flow
```text
[Windows Host: IPG CarMaker]
         │
   RSDS/CameraRSI (TCP 2210) & APO (TCP 16660)
         │
[WSL2: ROS2 Network]
         ▼
[IPG Telemetry Bridge (carmakercamera.py)]
  Publish: /VehicleSpeed (feedback/Velocity)
  Publish: /InertialData (inertial_msgs/Pose)
  Subscribe: /vehicle_control (vehiclecontrol/Control)
  DVA Write: VC.Steer.Ang, Driver.Lat.passive=1
  DVA Write: DM.Gas, DM.Brake (Cruise Control)
         │
[IPG Camera Node (CameraFramePublisher.py)]
  Publish: /RGBImage (sensor_msgs/Image)
         │
         ▼
[Lane Detection Node (lane_detection_node.py)]
  Process: UFLDv2 Inference (ResNet34 TorchScript)
  Publish: /lka/lane_detection (lka_interfaces/LaneDetection)
         │
         ▼
[Stanley Controller Node (stanley_controller_node.py)]
  Process: Tuned Stanley Controller (Cross-track error only)
  Publish: /lka/steering_cmd (std_msgs/Float32)
         │
         ▼
[Vehicle Control Node (vehicle_control_node.py)]
  Relay: /lka/steering_cmd → /vehicle_control (latswitch=1, longswitch=0)
         │
         ▼
[Performance Logger Node (performance_logger_node.py)]
  Process: Calculates RMSE and ILC %; Generates Article K.2 Dashboards
```

---

## 4. Advanced Features

### A. Steering Override via DVA
The system uses the IPG CarMaker **Direct Variable Access (DVA)** interface:
- `Driver.Lat.passive = 1` — Makes IPG Driver passive on lateral control
- `VC.Steer.Ang` — Writes the Stanley controller's steering angle (radians)

### B. Autonomous Cruise Control
When LKA is connected, the bridge locks into a smooth 10 km/h cruise control. When the ROS 2 node stops sending lateral override commands, the bridge sends a `DVARelease` command, cleanly giving the pedals back to the IPG Driver.

### C. Smoothed Stanley Controller
* **Dynamic Speed Gains:** The steering gain ($k$) automatically scales down as vehicle speed increases to prevent high-speed oscillations.
* **Low-Pass Filtering:** Applies exponential smoothing to the steering commands, heavily rejecting noisy frames and eliminating high-frequency steering vibrations.

### D. Performance Logging (Article K.2)
The `performance_logger_node` automatically captures data and saves a comprehensive dashboard every 5 seconds to `lka_logs/` (created in the directory where the launch command is executed).
* **Lateral Deviation (tRoad):** Captures ground-truth deviation from the lane center directly from the CarMaker physics engine.
* **RMSE:** Computes the Root Mean Square Error of the lateral deviation for the entire run.
* **ILC Percentage:** Calculates the "In-Lane Capability"—the percentage of time the vehicle stayed within $\pm 0.9m$ of the lane center.

---

## 5. Execution Instructions (Running on a New Laptop)

Follow these instructions to build and run the complete LKA software stack from scratch. The entire system has been packaged to be highly portable and will dynamically resolve its internal directory paths.

### Step 1: Open the CarMaker Scenario (Windows)
On your Windows host, open IPG CarMaker, load your specific Baja scenario, and click **Start/Play**. The simulation must be actively playing for the network telemetry ports to open.

### Step 2: Download Model Weights
Download the pre-trained UFLDv2 ResNet34 CULane weights from the following link:
- [Download culane_res34.pth](https://drive.google.com/file/d/1AjnvAD3qmqt_dGPveZJsLZ1bOyWv62Yj/view)

Place the downloaded `culane_res34.pth` file into the `abaja_lka_ws/Ultra-Fast-Lane-Detection-v2/weights/` directory. *(Create the `weights/` folder if it doesn't exist)*.

### Step 3: Build the ROS 2 Workspace (WSL2 Ubuntu)
Open your WSL2 terminal and navigate to the root of the workspace (`abaja_lka_ws`). Build all the packages using `colcon`:

```bash
cd <path_to_workspace>/abaja_lka_ws
source /opt/ros/humble/setup.bash
colcon build
```

### Step 4: Source and Launch the LKA Stack
Once the build is complete, source the local installation and launch the entire system using a single command. 

This single launch file automatically spins up:
1. The CarMaker Telemetry Bridge
2. The RSDS Camera Publisher
3. The UFLDv2 Lane Detection Node
4. The Stanley Controller Node
5. The Vehicle Control Relay Node
6. The Performance Logger
7. The Real-time Lane Visualization Window

```bash
source install/setup.bash
ros2 launch lka lka_system.launch.py
```
*(The lane detection visualizer window will open automatically, and the car will begin driving at 10 km/h.)*

---

## 6. Cleaning Up
To gracefully stop the system and ensure the final Performance Dashboard is saved, use SIGINT (2):
```bash
pkill -2 -f "ros2|python"
```

To forcefully kill all processes and free network ports:
```bash
pkill -9 -f "carmakercamera|CameraFramePublisher|lane_detection|stanley_controller|vehicle_control|performance_logger|showimage|ros2|python"
```