# LIO-SAM ROS1

This repository contains a **ROS1 Noetic** implementation of **LIO-SAM**, adapted for general LiDAR–IMU odometry and mapping applications.

The package is based on the original [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) implementation and includes configuration changes for easier integration with different robotic platforms and sensors.

This work was developed as part of robotics and forest mapping research at the **Institute of Systems and Robotics – University of Coimbra (ISR-UC)**.

---

## 1. Dependencies

Tested with **ROS Noetic**.

```bash
sudo apt-get install -y ros-noetic-navigation
sudo apt-get install -y ros-noetic-robot-localization
sudo apt-get install -y ros-noetic-robot-state-publisher
```

### GTSAM

```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

---

## 2. Installation

Clone the repository into your ROS Noetic workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/Forestry-Robotics-UC/fruc_lio_sam.git
cd ..
catkin_make
```

Source the workspace:

```bash
source devel/setup.bash
```

If running inside a Docker container:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

---

## 3. Sensor Inputs

The main required inputs are:

- LiDAR point cloud: `sensor_msgs/PointCloud2`
- IMU: `sensor_msgs/Imu`

The corrected IMU should be published on:

```text
/imu/data/corrected
```

Set the LiDAR topic in:

```text
config/params.yaml
```

For example:

```yaml
lidarTopic: "/hesai/points"
imuTopic: "/imu/data/corrected"
```

Replace `/hesai/points` with the topic published by your LiDAR.

The LiDAR point cloud should contain the fields required by LIO-SAM:

```text
x
y
z
intensity
ring
time
```

---

## 4. Configuration

The main configuration file is:

```text
config/params.yaml
```

Set the LiDAR parameters according to your sensor:

```yaml
sensor: velodyne
N_SCAN: 128
Horizon_SCAN: 1800
```

The exact values depend on the LiDAR being used.

Also verify the LiDAR–IMU extrinsics:

```yaml
extrinsicTrans:
extrinsicRot:
extrinsicRPY:
```

These parameters must correctly represent the physical transformation between the LiDAR and IMU.

Incorrect extrinsics or IMU orientation can result in tilted maps, unstable odometry, or drift.

---

## 5. Running LIO-SAM

Launch LIO-SAM:

```bash
roslaunch lio_sam run.launch
```

Then start the sensors or play a recorded ROS bag:

```bash
rosbag play /path/to/dataset.bag --clock
```

If the recorded IMU topic has a different name, remap it to `/imu/data/corrected`.

For example:

```bash
rosbag play /path/to/dataset.bag --clock \
  /imu/data:=/imu/data/corrected
```

Alternatively, change the `imuTopic` parameter directly in `config/params.yaml`.

---

## 6. Saving the Map

The generated map can be saved using the LIO-SAM map-saving service:

```bash
rosservice call /lio_sam/save_map 0.2 "/path/to/save/folder/"
```

The first argument specifies the voxel resolution in meters.

For example:

```text
0.2
```

corresponds to a map resolution of **0.2 m**.

The resulting map is exported as a `.pcd` point cloud.

---

## 7. Notes

- Recommended IMU frequency: **200 Hz or higher**
- Ensure LiDAR and IMU timestamps are synchronized.
- Verify that the LiDAR contains valid `ring` and `time` fields.
- Ensure the IMU orientation follows the expected coordinate convention.
- Check `extrinsicRot`, `extrinsicRPY`, and `extrinsicTrans` for each sensor setup.
- Start LIO-SAM before playing a recorded ROS bag.
- Verify the TF tree if the map is tilted, unstable, or incorrectly oriented.

---

Developed at **ISR-UC, University of Coimbra**.
