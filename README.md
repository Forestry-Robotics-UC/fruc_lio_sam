LIO-SAM (Go1 Adaptation)

This repository provides a ROS Noetic adaptation of LIO-SAM for the Unitree Go1 quadruped, developed within the Forestry Robotics UC (FRUC) project at the Institute of Systems and Robotics – University of Coimbra (ISR-UC).

The package enables real-time LiDAR–IMU–GNSS odometry using the LIO-SAM framework, modified and tuned for the Go1 robot and its onboard/external sensors.
All tests were performed inside a CUDA-enabled ROS Noetic Docker environment.

1. Overview

The original LIO-SAM
 fuses LiDAR, IMU, and GNSS data using nonlinear optimization (iSAM2).
This fork adapts the system for the Unitree Go1, introducing changes in topics, coordinate frames, and configuration to support Go1-specific sensors and data.

Key Updates

Integration of Unitree Go1 IMU and external LiDAR sensors (RS-LiDAR / RealSense D435i).

Adjusted extrinsics and frames: base_link, imu_link, lidar_link, gps_link.

Added GPS fusion using robot_localization + navsat_transform_node.

Tuned parameters for stable performance at 100 Hz IMU rate.

Verified compatibility with ROS Noetic + CUDA.

Validated using Adões Go1 dataset and FRUC mapping trials.

2. Environment

All experiments were executed in a Dockerized ROS Noetic workspace with GPU acceleration.

Component	Version / Details
Base Image	nvidia/cuda:12.8.0-devel-ubuntu20.04
ROS	Noetic
CUDA	12.8
Python	3.10 (via pyenv)
PyTorch	2.8.0
CMake / G++	≥ 3.16 / 9.4
OS	Ubuntu 20.04
Platform	Unitree Go1 (IMU, LiDAR, GNSS)
Docker Example
docker run -it --gpus all --net=host --ipc=host \
    -v ~/catkin_ws:/root/catkin_ws \
    liosam-noetic-gpu

3. Configuration Modifications
3.1 IMU

Go1 IMU frequency: 100 Hz

Updated topic: /unitree/imu or /imu_data_resampled

Parameter changes:

imu0_remove_gravitational_acceleration: true


Calibrated extrinsicRot and extrinsicRPY for IMU–LiDAR alignment.

3.2 LiDAR

Adapted for RS-LiDAR and RealSense D435i.

Topics: /rslidar_points, /rslidar_points_dense

Tuned extrinsic offsets and FOV parameters in params.yaml.

3.3 GPS Fusion

Configured robot_localization and navsat_transform_node for GNSS odometry.

useGpsElevation: true
zero_altitude: false
wait_for_datum: true
datum: [latitude, longitude, altitude]


GNSS topic: /gps/fix

Frame: gps_link

Verified consistent transforms among /map, /odom, and /base_link.

3.4 Frame Structure

Coordinate hierarchy (ROS REP-105 compliant):

map → odom → base_link → lidar_link / imu_link / gps_link


Extrinsic matrices tuned for correct orientation and gravity alignment.

3.5 Dataset Adaptations

Tested on Adões Go1 dataset (LiDAR + IMU + GNSS).

Verified timestamp alignment, IMU deskewing, and trajectory stability.

Optimized GPS factor covariance and smoothing for robust map consistency.

4. Build and Usage
Build
cd ~/catkin_ws/src
git clone https://github.com/ArushMendon-dev/lio-sam-go1.git
cd ..
catkin_make

Launch
roslaunch lio_sam run.launch

Play Dataset
rosbag play your_dataset.bag -r 1 --clock

5. Notes and Observations

The RealSense D435i provides only a 6-axis IMU; Go1 IMU supplies 9-axis data used by LIO-SAM.

Recommended IMU frequency ≥ 100 Hz.

To align yaw with GNSS data:

useImuHeadingInitialization: true


Adjust altitude integration using:

useGpsElevation: true
zero_altitude: false


If the reconstructed map tilts (≈15–20 °), recheck IMU–LiDAR extrinsics and gravity compensation.

6. Repository Structure
lio-sam-go1/
├── config/              # Parameter and calibration files
├── launch/              # Launch files for mapping and GPS fusion
├── src/                 # Core C++ source
├── scripts/             # Python/utility nodes
├── msg/  srv/           # ROS message/service definitions
├── urdf/                # Robot model frames
├── Dockerfile           # Optional build environment
├── LICENSE              # BSD 3-Clause with modification notice
└── README.md

7. License and Attribution

This work extends the original LIO-SAM (Tixiao Shan, 2020)
 under the BSD 3-Clause License.
Modifications for the Unitree Go1 platform © 2025 Arush Mendon, Forestry Robotics UC (FRUC), Institute of Systems and Robotics – University of Coimbra.

When used in academic or research contexts, please cite both the original LIO-SAM paper and this adaptation.

8. Contact

Author: Arush Mendon
Affiliation: Institute of Systems and Robotics – University of Coimbra (ISR-UC)
Email: arushmendon2001@gmail.com

GitHub: ArushMendon-dev
