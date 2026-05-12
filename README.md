

# AI Powered Autonomous Medical Mobile Robot 🏥🤖

An autonomous mobile robot (AMR) built using **ROS 2 Humble**, designed for medical environments. The system utilizes a Raspberry Pi 4 as the primary compute module and an Arduino Mega for real-time motor control and sensor fusion.

## 🌟 Features

* **Autonomous Navigation:** Powered by the ROS 2 Nav2 Stack.
* **SLAM (Simultaneous Localization and Mapping):** Uses `slam_toolbox` and LD19 LiDAR to generate high-accuracy floor plans.
* **Closed-Loop Control:** PID velocity control via Arduino and high-resolution encoders.
* **Hardware Handshake:** Optimized Python-to-Arduino serial bridge with DTR-reset and buffer clearing to ensure zero-lag odometry.
* **Medical Use-Case Ready:** Pre-configured parameters for smooth, safe indoor navigation.

---

## 🛠 Hardware Requirements

| Component | Description |
| --- | --- |
| **Compute** | Raspberry Pi 4 (8GB recommended) |
| **Controller** | Arduino Mega 2560 |
| **LiDAR** | LD19 (ldlidar_stl_ros2) |
| **Motors** | 2x JGY-370 DC Motors with Encoders |
| **Drivers** | 2x BTS7960 High-Power H-Bridge |
| **Chassis** | Differential Drive (0.165m wheel base) |

---

## 💻 Software Setup

### Prerequisites

1. **OS:** Ubuntu 22.04 LTS (Server or Desktop).
2. **ROS 2:** [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html).
3. **Dependencies:**
```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard \
                 python3-serial

```



### Installation

1. Clone this repository into your workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Wahdan-03/amr_robot.git

```


2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select amr_robot
source install/setup.bash

```



---

## 🚀 Usage

### 1. Mapping (Building the environment)

To start the LiDAR, Arduino bridge, and SLAM toolbox, run:

```bash
ros2 launch amr_robot mapping.launch.py

```

* Use the teleop window that appears to drive the robot around.
* Once finished, save the map:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"

```



### 2. Navigation (Autonomous Driving)

To navigate using a saved map:

```bash
ros2 launch amr_robot nav2.launch.py map:=/path/to/your/map.yaml

```

* Open **RViz2**, set the "Initial Pose," and use "Nav2 Goal" to send the robot to its destination.

---

## 📂 File Structure

* `amr_robot/`
* `launch/`
* `mapping.launch.py`: Full stack for building maps.
* `nav2.launch.py`: Full stack for localization and navigation.


* `config/`
* `nav2_params.yaml`: Tuned parameters for Nav2 (Humble).
* `slam_config.yaml`: Async SLAM settings optimized for Pi 4.


* `amr_robot/`
* `arduino_bridge.py`: The Python node communicating with the Arduino Mega.





---

## ⚠️ Important Notes

* **Hardware Pinout:** The `arduino_bridge` assumes the Arduino is connected via `/dev/arduino_mega`.
* **Rebuilds:** Remember to run `colcon build` whenever you modify `.yaml` or `.py` launch files to ensure the changes are installed to the `share` directory.

---

## 🔮 Future Work

* [ ] **AI Integration:** Implementing YOLOv8-Nano for doctor/patient detection.
* [ ] **Medical Payload:** Integration of UV-C sterilization or medicine delivery lockbox.
* [ ] **Voice Interface:** Natural language processing for nurse-to-robot commands.
