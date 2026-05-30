# SVW_aBAJA2026_264018_Team_Equinox

Software submission repository for *aBAJA 2026 Virtual World Simulation Competition*.

This repository contains the implementation and simulation assets developed by *Team Equinox* for the following autonomous driving modules:

- Autonomous Emergency Braking (AEB)
- Lane Keeping Assist (LKA)
- Endurance Event
- Lane Detection
- Scenario Definitions
- Performance Results and Evaluation

---

# Repository Structure

```text
SVW_aBAJA2026_264018_Team_Equinox
│
├── Ultra-Fast-Lane-Detection-v2/
│   ├── lane detection model
│   ├── training scripts
│   ├── inference scripts
│   └── model weights
│
├── config/
│   ├── vehicle parameters
│   ├── sensor configurations
│   └── simulation settings
│
├── docs/
│   ├── architecture diagrams
│   ├── software design documents
│   ├── system workflow
│   └── competition reports
│
├── results/
│   ├── AEB results
│   ├── LKA results
│   ├── Endurance results
│   └── evaluation screenshots
│
├── scenarios/
│   ├── AEB scenarios
│   ├── LKA scenarios
│   ├── Endurance scenarios
│   └── testing environments
│
├── src/
│   ├── AEB implementation
│   ├── LKA implementation
│   ├── Endurance implementation
│   └── ROS2 nodes
│
├── README.md
├── Submission_Guidelines.md
├── results.7z
└── scenarios.7z
```

---

# Execution Instructions

Follow these instructions to build and run the autonomous driving modules. All modules are designed to run in a **WSL2 Ubuntu 22.04** environment with **ROS2 Humble**, interfacing with **IPG CarMaker** on a Windows host.

## Prerequisites

1.  **IPG CarMaker 11+** installed on Windows.
2.  **ROS2 Humble** installed on WSL2.
3.  Python dependencies:
    ```bash
    pip install torch torchvision opencv-python numpy scipy gdown
    ```

## 1. Lane Keeping Assist (LKA)

The LKA system uses the UFLDv2 model for lane detection and a Stanley controller for lateral steering.

### Setup & Run
1.  **Start CarMaker:** Load an LKA TestRun on Windows and click **Start/Play**.
2.  **Download Weights:**
    ```bash
    mkdir -p src/lka/abaja_lka_ws/Ultra-Fast-Lane-Detection-v2/weights
    # Ensure culane_res34.pth is placed in the weights/ folder
    ```
3.  **Build & Launch:**
    ```bash
    cd src/lka/abaja_lka_ws
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash
    ros2 launch lka lka_system.launch.py
    ```

## 2. Autonomous Emergency Braking (AEB)

The AEB system uses fuzzy logic to determine the required braking force based on radar data.

### Setup & Run
1.  **Start CarMaker:** Load an AEB TestRun (e.g., Scenario M1) on Windows and click **Start/Play**.
2.  **Build & Launch:**
    The AEB package is integrated into the Endurance workspace.
    ```bash
    cd src/endurance/ENDURANCE
    source /opt/ros/humble/setup.bash
    colcon build --packages-select aeb endurance
    source install/setup.bash
    # Note: Running via the integrated endurance launch will activate the AEB node
    ros2 launch endurance endurance_system.launch.py
    ```

## 3. Endurance Event (Integrated AEB + LKA)

The Endurance event combines full lateral and longitudinal control.

### Setup & Run
1.  **Start CarMaker:** Load the Endurance TestRun on Windows and click **Start/Play**.
2.  **Export Path:**
    ```bash
    cd src/endurance/ENDURANCE
    export UFLDV2_DIR=$(pwd)/Ultra-Fast-Lane-Detection-v2
    ```
3.  **Build & Launch:**
    ```bash
    source /opt/ros/humble/setup.bash
    colcon build
    source install/setup.bash
    ros2 launch endurance endurance_system.launch.py
    ```

---

# Portability & Troubleshooting

If you are running this stack on a new system, you may need to adjust the following configurations:

### 1. IP Configuration (WSL2 to Windows Host)
The ROS2 nodes in WSL2 communicate with IPG CarMaker on Windows via a TCP socket. By default, the scripts are configured to use **`172.23.128.1`** as the Windows host IP. 

If your connection fails, verify your host IP from WSL2:
```bash
grep nameserver /etc/resolv.conf | awk '{print $2}'
```
Update the `CARMAKER_IP` variable in the following files if your IP differs:
- `src/lka/abaja_lka_ws/src/pycarmaker/pycarmaker/carmakercamera.py`
- `src/lka/abaja_lka_ws/src/camera_sensor/camera_sensor/CameraFramePublisher.py`
- `src/endurance/ENDURANCE/src/endurance/scripts/carmakercamera.py`
- `src/endurance/ENDURANCE/src/endurance/scripts/rsds_camera_publisher.py`

### 2. Network Ports
Ensure the following ports are open and not blocked by the Windows Firewall:
- **16660:** CarMaker APO/DVA (Control & Telemetry)
- **2210:** CarMaker RSDS (Camera Stream)

### 3. Model Weights Path
The lane detection module relies on the `UFLDV2_DIR` environment variable to find the UFLDv2 model architecture and weights.
- Always ensure you `export UFLDV2_DIR=$(pwd)/Ultra-Fast-Lane-Detection-v2` before launching if you are not in the default workspace root.
- Ensure `culane_res34.pth` is present in the `weights/` subdirectory of the UFLDv2 folder.

### 4. Shared Memory / TCP Issues
If the video stream does not appear, ensure that **RSDS** is enabled in your CarMaker TestRun configuration and that the "Image Generation" is set to "Active".

---
📋 See [Submission_Guidelines.md](./Submission_Guidelines.md) for full rules, checklist, and scoring.
