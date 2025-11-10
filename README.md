LIO-SAM (Go1 Adaptation)

This repository is a ROS Noetic adaptation of LIO-SAM for the Unitree Go1 quadruped, developed as part of the Forestry Robotics UC (FRUC) project at the Institute of Systems and Robotics – University of Coimbra (ISR-UC).

The package provides real-time LiDAR–IMU–GNSS odometry using LIO-SAM as a base framework, modified and tuned for the Go1 robot and its sensor suite. All modifications were implemented and tested inside a CUDA-enabled ROS Noetic Docker environment.

Overview

The original LIO-SAM
 system fuses LiDAR, IMU, and optional GPS data through nonlinear optimization (iSAM2). This version adapts it to the Unitree Go1 robot platform, with configuration and topic-level modifications that allow processing of Go1 sensor data and other non-Velodyne datasets.

The following updates were made:

Integration of Unitree Go1 IMU and external LiDAR sensors (e.g., RS-LiDAR or RealSense D435i).

Adjusted extrinsic parameters and coordinate frames to match the Go1 robot model (base_link, imu_link, lidar_link, gps_link).

Configured robot_localization and navsat_transform_node to fuse GPS odometry with LIO-SAM mapping.

Tuned params.yaml for stable operation at lower IMU frequencies (100 Hz).

Verified operation on ROS Noetic with CUDA support.

Validated results using multiple datasets, including the Adões Go1 dataset.

Resolved GPS alignment issues through TF corrections and datum configuration.

Environment

All tests and deployments were performed inside a ROS Noetic Docker container with GPU support.

Component	Version / Details
Base Image	nvidia/cuda:12.8.0-devel-ubuntu20.04
ROS	Noetic
CUDA	12.8
Python	3.10 (via pyenv)
PyTorch	2.8.0
CMake / G++	3.16+ / 9.4
OS	Ubuntu 20.04
Hardware	Unitree Go1 (IMU, LiDAR, GNSS)

Docker Execution Example:

docker run -it --gpus all --net=host --ipc=host \
    -v ~/catkin_ws:/root/catkin_ws \
    liosam-noetic-gpu

Key Configuration Modifications
1. IMU Setup

The Go1 IMU operates at 100 Hz; LIO-SAM parameters were adjusted to handle this rate.

imuTopic updated to /unitree/imu or /imu_data_resampled.

imu0_remove_gravitational_acceleration: true for accurate motion compensation.

Extrinsic calibration values updated for IMU–LiDAR alignment (extrinsicRot, extrinsicRPY).

2. LiDAR Input

Adapted for RS-LiDAR and RealSense D435i point clouds.

Primary point cloud topics: /rslidar_points and /rslidar_points_dense.

Extrinsics calibrated for offset between LiDAR and base_link.

Scan parameters adjusted in params.yaml for non-Velodyne sensors.

3. GPS Integration

Configured robot_localization and navsat_transform_node for GPS odometry fusion.

Relevant parameters in params.yaml:

useGpsElevation: true
zero_altitude: false
wait_for_datum: true
datum: [latitude, longitude, altitude]


GNSS topic: /gps/fix

Frame: gps_link

Alignment verified between /map, /odom, and /base_link.

4. Frame Structure

All frames follow the REP-105 convention:

map → odom → base_link → lidar_link / imu_link / gps_link


Extrinsics were tuned to ensure correct orientation and coordinate alignment.

5. Dataset-Specific Adaptations

Tested on the Adões Go1 dataset containing synchronized LiDAR, IMU, and GNSS data.

Verified IMU deskewing, timestamp synchronization, and stable trajectory reconstruction.

Optimized GPS factor integration by refining covariance thresholds and iSAM smoothing parameters.

Build and Usage

Build:

cd ~/catkin_ws/src
git clone https://github.com/ArushMendon-dev/lio-sam-go1.git
cd ..
catkin_make


Launch:

roslaunch lio_sam run.launch


Run Dataset:

rosbag play your_dataset.bag -r 1 --clock

Notes and Observations

The RealSense D435i IMU is 6-axis only; the Go1 onboard IMU provides full 9-axis data for LIO-SAM.

Recommended IMU frequency: ≥100 Hz.

For GPS-based corrections:

useImuHeadingInitialization: true


This ensures yaw alignment with GNSS data without excessive correction.

Altitude parameters (useGpsElevation, zero_altitude) control whether GPS height affects map elevation.

A tilted or angled map (15–20°) generally indicates incorrect IMU–LiDAR extrinsics or gravity direction settings.

Repository Structure
lio-sam-go1/
├── config/              # Parameter and calibration files
├── launch/              # Launch files for mapping and GPS integration
├── src/                 # Core LIO-SAM source code
├── scripts/             # Utility and Python helper nodes
├── msg/ & srv/          # ROS message and service definitions
├── urdf/                # Robot model (base_link, IMU, GPS frames)
├── Dockerfile           # Optional environment definition
├── LICENSE              # BSD 3-Clause with modification notice
└── README.md

License and Attribution

This project extends the original LIO-SAM (Tixiao Shan, 2020)
 implementation under the BSD 3-Clause License.
Modifications for the Go1 platform © 2025 Arush Mendon, Forestry Robotics UC (FRUC), Institute of Systems and Robotics – University of Coimbra (ISR-UC).

If used for academic or research purposes, please cite both the original LIO-SAM publication and this adaptation.

Contact

Author: Arush Mendon
Affiliation: Institute of Systems and Robotics – University of Coimbra (ISR-UC)
Email: arushmendon2001@gmail.com

GitHub: ArushMendon-dev
