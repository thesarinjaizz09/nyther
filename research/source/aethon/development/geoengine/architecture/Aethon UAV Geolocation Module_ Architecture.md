# Aethon UAV Geolocation Module: Architecture

## 1. Hardware Stack
To achieve accurate geolocation with YOLO and DeepSORT, the Aethon platform requires a high-performance companion computer and synchronized sensors.

| Component | Recommendation | Purpose |
| :--- | :--- | :--- |
| **Companion Computer** | NVIDIA Jetson Orin NX (16GB) | Real-time YOLOv8/v10 inference and DeepSORT tracking. |
| **Flight Controller** | Cube Orange+ (ArduPilot/PX4) | Provides high-frequency IMU and GPS data via MAVLink. |
| **Camera** | Sony IMX-based Global Shutter | Reduces rolling shutter distortion for better pixel-to-ray mapping. |
| **Gimbal** | 3-Axis Brushless (e.g., Gremsy) | Stabilizes the camera and provides precise encoder feedback for pointing angles. |
| **GNSS** | Dual-Antenna RTK GPS | Provides centimeter-level position and accurate heading (yaw). |

## 2. Software Stack
The software architecture is built on **ROS2 (Humble)** for modularity and real-time data handling.

- **OS**: Ubuntu 22.04 with NVIDIA JetPack.
- **Middleware**: ROS2 Humble.
- **Communication**: `MAVROS` or `Micro-XRCE-DDS` to bridge MAVLink data from the flight controller.
- **Vision**:
  - **Detection**: YOLOv8/v10 (TensorRT optimized).
  - **Tracking**: DeepSORT (integrating appearance features for ID persistence).
- **Geolocation Engine**: Custom C++/Python node for ray-casting and EKF.

## 3. Data Synchronization Strategy
The biggest challenge in geolocation is the "Time Offset" between the image and the sensor data.
- **Hardware Triggering**: The flight controller should trigger the camera shutter.
- **MAVLink `CAMERA_IMAGE_CAPTURED`**: Use this message to get the exact GPS/IMU state at the moment of capture.
- **Message Filter**: Use ROS2 `message_filters` (ApproximateTime Synchronizer) to align YOLO bounding boxes with the corresponding aircraft state.
