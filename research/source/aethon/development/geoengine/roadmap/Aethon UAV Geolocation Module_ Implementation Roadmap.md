# Aethon UAV Geolocation Module: Implementation Roadmap

This guide details the mathematical framework and software structure required to build the real-time geolocation module for the Aethon UAV, integrating YOLO/DeepSORT detections with high-fidelity sensor data.

## 1. Mathematical Framework: From Pixel to Geodetic Coordinates

The core of the geolocation module is the transformation of a detected pixel coordinate $(u, v)$ into a geodetic coordinate $(\text{Lat}, \text{Lon}, \text{Alt})$. This requires a chain of coordinate transformations based on the **Pinhole Camera Model** and the **Direct Georeferencing** principle.

### 1.1. Step 1: Image Plane to Camera Frame

The pixel coordinates $(u, v)$ of the target (e.g., the center of the bottom edge of the YOLO bounding box) are converted to a 3D vector $\vec{v}_{cam}$ in the camera's coordinate frame.

$$\vec{v}_{cam} = K^{-1} \cdot \begin{pmatrix} u \\ v \\ 1 \end{pmatrix}$$

Where $K$ is the **Camera Intrinsic Matrix**:
$$K = \begin{pmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{pmatrix}$$
*   $f_x, f_y$: Focal lengths in terms of pixel units.
*   $c_x, c_y$: Principal point coordinates (center of the image).

### 1.2. Step 2: Camera Frame to Navigation Frame (NED)

The vector $\vec{v}_{cam}$ must be rotated through the gimbal and the UAV's body to align with the local North-East-Down (NED) frame.

$$\vec{v}_{NED} = R_{body}^{NED} \cdot R_{cam}^{body} \cdot \vec{v}_{cam}$$

*   **$R_{cam}^{body}$ (Camera to Body)**: This is a fixed rotation matrix defined by the physical mounting of the camera/gimbal on the UAV body. If a gimbal is used, this matrix will incorporate the gimbal's current encoder angles (e.g., $\text{Roll}_{gimbal}, \text{Pitch}_{gimbal}, \text{Yaw}_{gimbal}$).
*   **$R_{body}^{NED}$ (Body to NED)**: This is the rotation matrix derived from the UAV's attitude (Roll $\phi$, Pitch $\theta$, Yaw $\psi$) provided by the IMU. This is a sequence of three rotations (e.g., Z-Y-X Euler angles).

### 1.3. Step 3: Ray-Terrain Intersection

The ray's starting point is the UAV's position $P_{UAV}$ (from GNSS) and its direction is $\vec{v}_{NED}$. The target position $P_{target}$ lies along this ray:

$$P_{target}(t) = P_{UAV} + t \cdot \vec{v}_{NED}$$

Where $t$ is the distance along the ray. The value of $t$ is found by solving for the intersection of the ray with the terrain model (DEM).

#### Iterative Ray-Tracing Algorithm
1.  **Initialization**: Set $t_{min} = 0$ and $t_{max}$ to a large value (e.g., max camera range).
2.  **Iteration**:
    *   Calculate a test point $P_{test}$ along the ray at $t_{current}$.
    *   Convert $P_{test}$'s NED coordinates to Geodetic coordinates $(\text{Lat}, \text{Lon})$.
    *   Query the DEM for the terrain altitude $Z_{DEM}$ at $(\text{Lat}, \text{Lon})$.
    *   Compare $P_{test}$'s altitude $Z_{NED}$ with $Z_{DEM}$.
3.  **Refinement**: Use a binary search approach to quickly converge on the intersection point where $Z_{NED} \approx Z_{DEM}$. This is the most robust method for real-time systems.

### 1.4. Step 4: NED to Geodetic (WGS84)

Once the distance $t$ is found, $P_{target}$ is calculated in NED coordinates. This local coordinate is then converted to the final geodetic coordinates $(\text{Lat}, \text{Lon}, \text{Alt})$ using standard Earth model transformations (e.g., using the `pyproj` library in Python or a dedicated C++ library).

## 2. Software Structure: ROS2 Implementation

The Aethon geolocation system should be implemented as a set of modular ROS2 nodes, communicating via topics.

| ROS2 Node | Input Topics | Output Topics | Core Functionality |
| :--- | :--- | :--- | :--- |
| **`vision_node`** | `/camera/image_raw` | `/vision/detections` | YOLO inference, DeepSORT tracking, outputting bounding boxes and track IDs. |
| **`sensor_sync_node`** | `/mavros/state`, `/mavros/imu/data`, `/mavros/global_position/global`, `/gimbal/status` | `/sensor/synced_state` | Time-synchronization of all sensor data (GNSS, IMU, Gimbal) using `message_filters`. |
| **`geolocation_node`** | `/vision/detections`, `/sensor/synced_state` | `/geolocation/target_track` | **Core Logic**: Implements the 4-step mathematical framework (Ray Casting, DEM Intersection, Coordinate Transform). |
| **`ekf_fusion_node`** | `/geolocation/target_track` (Measurement), `/sensor/synced_state` (UAV State) | `/geolocation/final_target_pos` | **State Estimation**: Runs the Extended Kalman Filter (EKF) to smooth the target track and provide a stable, high-precision estimate. |

## 3. Advanced Algorithm: Extended Kalman Filter (EKF)

The EKF is essential for a military-grade system to handle sensor noise and provide a stable track for moving targets.

### EKF State and Measurement
*   **State Vector ($\mathbf{x}$)**: The EKF estimates the target's true state, typically its 3D position and velocity:
    $$\mathbf{x} = [x_{target}, y_{target}, z_{target}, \dot{x}_{target}, \dot{y}_{target}, \dot{z}_{target}]^T$$
*   **Measurement Vector ($\mathbf{z}$)**: The input to the EKF is the noisy measurement from the ray-casting module, which is the estimated target position $P_{target}$ at time $k$.
    $$\mathbf{z} = [x_{target}^{meas}, y_{target}^{meas}, z_{target}^{meas}]^T$$

### EKF Process
1.  **Prediction**: The filter uses a motion model (e.g., constant velocity) to predict the target's state at the next time step.
2.  **Update**: The filter uses the new measurement from the `geolocation_node` to correct the predicted state, minimizing the error covariance.

The EKF output (`/geolocation/final_target_pos`) is the most accurate and stable real-time position of the target.

## 4. Implementation Roadmap

| Phase | Task | Key Deliverable |
| :--- | :--- | :--- |
| **I. Data Acquisition** | Configure Flight Controller (FC) and Gimbal. | High-rate, time-stamped MAVLink stream of GNSS, IMU, and Gimbal angles. |
| **II. Vision Pipeline** | Train and optimize YOLO/DeepSORT on the Jetson. | `vision_node` publishing bounding boxes and track IDs at $\ge 10$ Hz. |
| **III. Synchronization** | Implement the `sensor_sync_node`. | Synchronized ROS2 topic `/sensor/synced_state` containing all necessary metadata. |
| **IV. Geolocation Core** | Implement the 4-step mathematical framework in `geolocation_node`. | Unfiltered target position topic `/geolocation/target_track`. |
| **V. State Estimation** | Implement the EKF in `ekf_fusion_node`. | Stable, filtered target position topic `/geolocation/final_target_pos`. |
| **VI. Validation** | Test with simulated and real flight data. | Geolocation error under a specified threshold (e.g., $\le 5$ meters). |

This structured approach ensures that each component is developed and tested independently before integration, leading to a robust and accurate final system.
