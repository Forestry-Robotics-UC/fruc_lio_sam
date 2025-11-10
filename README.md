# LIO-SAM for Unitree Go1 (RS-Bpearl + RTK Integration)

This repository contains a **ROS1 Noetic** implementation of **LIO-SAM**, adapted and tuned for the **Unitree Go1** robot equipped with an **RS-Bpearl LiDAR**, **internal IMU**, and **Reach RTK GNSS module**.  
It includes custom preprocessing scripts and configuration adjustments to make the raw Go1 dataset directly compatible with LIO-SAM.

This work was developed as part of forest mapping research under the **Institute of Systems and Robotics – University of Coimbra (ISR-UC)**.

---

## 1. Dependencies

This is based on the original ROS1 implementation of [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM).  
For a ROS2 version, refer to the `ros2` branch (under progress, will be available soon).

### ROS (tested with Noetic)
```bash
sudo apt-get install -y ros-noetic-navigation
sudo apt-get install -y ros-noetic-robot-localization
sudo apt-get install -y ros-noetic-robot-state-publisher
```

### GTSAM (Georgia Tech Smoothing and Mapping Library)
```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

### Source your workspace
```bash
source /opt/ros/noetic/setup.bash
```

## 2. Installation

Use the following commands to download and compile the package in your ROS Noetic workspace.

```bash
cd ~/catkin_ws/src
git clone https://github.com/Forestry-Robotics-UC/fruc_lio_sam.git
cd ..
catkin_make
```

Once compilation is complete, source your workspace:

```bash
source devel/setup.bash
```

If you are working inside a Docker container, remember to source ROS Noetic as well:

```bash
source /opt/ros/noetic/setup.bash
```

This will build the LIO-SAM package adapted for the **Unitree Go1 + RS-Bpearl + RTK** setup.


## 3. Data Preprocessing

Before running LIO-SAM, the raw ROS bag data collected from the **Unitree Go1** robot must be preprocessed to match the topic and data-format requirements of LIO-SAM.

Two Python scripts in the `scripts/` directory handle this conversion and synchronization pipeline.

---

### (1) unitree_to_lio_offline_rtk.py

This script converts the raw topics recorded from the Unitree Go1 robot into a format compatible with LIO-SAM.

**Main operations**
- Extracts and renames key topics:
  - `/rslidar_points` → `/rslidar_points`
  - `/high_state` → `/imu_data`
  - `/reach/fix`, `/reach/vel`, `/reach/time_ref` → `/gps/fix`, `/gps/vel`, `/gps/time_ref`
- Converts Unitree’s `HighState` IMU data into standard `sensor_msgs/Imu` messages.

**Usage**
Specify the input bag path(s) inside the script under `input_bags`, then run:

```bash
python3 scripts/unitree_to_lio_offline_rtk.py
```

---

### (2) go1_lio_preprocessor.py

This script refines and harmonizes the converted data, ensuring LiDAR–IMU–GPS consistency before running LIO-SAM.

**Main operations**
- Filters RS-Bpearl LiDAR scans (removes invalid points, retains `ring` and `time` fields).
- Computes IMU bias, applies low-pass smoothing, and resamples IMU data to **100 Hz**.
- Normalizes all frame names:
  - `base` → `base_link`
  - `rslidar` → `velodyne`
  - `gps` → `navsat_link`
- Outputs LiDAR, IMU, and GPS topics ready for LIO-SAM.

**Usage**
Edit the `IN_BAG` and `OUT_BAG` paths at the top of the script, then run:

```bash
python3 scripts/go1_lio_preprocessor.py
```

After these steps, the resulting preprocessed bag can be used directly for mapping and odometry with LIO-SAM.


## 4. Running the Package

Once preprocessing is complete and the workspace has been built, you can run LIO-SAM using your preprocessed ROS bag.

### 1. Launch LIO-SAM
Start the main LIO-SAM node:

```bash
roslaunch lio_sam run.launch
```

This will start all required components for LiDAR–IMU–GNSS odometry and mapping.

### 2. Play the preprocessed ROS bag
In a new terminal (with your workspace sourced), play the final preprocessed bag:

```bash
rosbag play /path/to/your_preprocessed.bag --clock
```

The `--clock` flag is required so that LIO-SAM receives simulated ROS time from the bag file.

### 3. Saving the map
You can optionally save the generated map using the provided ROS service:

```bash
rosservice call /lio_sam/save_map 0.2 "/path/to/save/folder/"
```

This will export a `.pcd` point cloud map at the specified resolution (e.g., 0.2 m).

## 5. Notes and Practical Guidance

This section summarizes important configuration parameters, tuning notes, and practical recommendations for running LIO-SAM on the **Unitree Go1** platform with **RS-Bpearl LiDAR** and **Reach RTK GNSS**.

---

### 5.1 Key Configuration Parameters

The following parameters in `config/params.yaml` are tuned for the Go1 + RS-Bpearl + RTK setup:

```yaml
lidarTopic:             "/rslidar_points_dense"
imuTopic:               "/imu_data_resampled"
gpsTopic:               "/gps/fix"
sensor:                 "rslidar"
N_SCAN:                 16
Horizon_SCAN:           1800
useImuHeadingInitialization: true
imuRate:                100
wait_for_datum:         true
gpsCovThreshold:        2.0
poseCovThreshold:       2.0
```

**Notes:**
- Ensure that `extrinsicRot` and `extrinsicRPY` accurately represent the LiDAR–IMU transform (measured in your setup).  
- `wait_for_datum: true` should be enabled when a known GPS base-station datum is available.  
- If your IMU frequency differs from 100 Hz, update `imuRate` accordingly.

---

### 5.2 EKF and GPS Integration

The **robot_localization** and **navsat_transform_node** modules depend on IMU data being correctly referenced through the `imu0` parameter in their respective configurations.

Example snippet in the EKF configuration (`ekf.yaml`):

```yaml
imu0: /imu_data_resampled
imu0_config: [false, false, false,
              true,  true,  true,
              true,  true,  true,
              false, false, false,
              false, false, false]
imu0_remove_gravitational_acceleration: true
```

**Explanation:**
- The `imu0` parameter specifies which IMU topic feeds the EKF and NavSat fusion modules.  
- LIO-SAM relies on this IMU input for orientation alignment and for computing `/gps_odom` through the **NavSatTransform** node.  
- Without it, GPS data will not be fused into the odometry stream, resulting in missing or inconsistent `/odometry/gps` output.

---

### 5.3 Dataset and Timing Guidelines

For reliable operation:
- **LiDAR frequency:** 10 Hz  
- **IMU frequency:** 100 Hz (uniformly resampled using `go1_lio_preprocessor.py`)  
- **GPS frequency:** 1 Hz (RTK Fix recommended)  
- All topics must share a consistent time reference; use the preprocessed bag to ensure synchronization.

---

### 5.4 Troubleshooting Tips

| Issue | Likely Cause | Suggested Fix |
|-------|---------------|----------------|
| Zigzag / jerky motion | IMU timestamps not synchronized | Use `/imu_data_resampled` from preprocessing |
| Map tilted / drifting | Incorrect IMU extrinsic rotation | Check `extrinsicRot` and `extrinsicRPY` |
| GPS odometry missing | `imu0` not set or missing transform | Verify EKF and NavSat configs |
| Low mapping rate | High voxel density | Increase `mappingCornerLeafSize` and `mappingSurfLeafSize` |
| Sudden jumps in map | Inconsistent GPS covariance | Tune `gpsCovThreshold` and ensure RTK Fix |

---

### 5.5 General Advice

- Always start playback **after** launching `lio_sam` so the system initializes correctly.  
- When using GPS, uncheck “Map (cloud)” and enable “Map (global)” in RViz (if visualization is needed).  
- Keep IMU and LiDAR frames consistent with REP-105 conventions (`x` – forward, `y` – left, `z` – up).  
- For repeatability, store calibration files (`extrinsics.yaml`, `params.yaml`) alongside each dataset.  
- Periodically verify `tf_tree` to confirm proper transforms between `base_link`, `velodyne`, and `navsat_link`.

