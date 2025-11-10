#!/usr/bin/env python3
"""
Offline converter: Extract LiDAR, IMU (Unitree + Camera), and GPS (RTK) topics
from Unitree/Go1 dataset and save in LIO-SAM compatible format.
"""

import rosbag
from tqdm import tqdm
from sensor_msgs.msg import Imu

input_bags = [
    f"/root/Downloads/bagfile_2024-07-30-12-09-40_{i}.bag" for i in range(7)
]

for idx, in_bag_path in enumerate(input_bags):
    out_bag_path = f"/root/Downloads/bpearl_fixed_synced_camimu_{idx}.bag"
    print(f"\nProcessing {in_bag_path} → {out_bag_path}")

    in_bag = rosbag.Bag(in_bag_path, 'r')
    out_bag = rosbag.Bag(out_bag_path, 'w')

    lidar_count, imu_count, camimu_count, gps_count = 0, 0, 0, 0

    for topic, msg, t in tqdm(in_bag.read_messages()):
        # LiDAR data
        if topic == "/rslidar_points":
            out_bag.write("/rslidar_points", msg, t)
            lidar_count += 1

        # Unitree HighState IMU
        elif topic == "/high_state":
            imu_msg = Imu()
            imu_msg.header.stamp = t
            imu_msg.header.frame_id = "base"
            imu_msg.orientation.x = msg.imu.quaternion[0]
            imu_msg.orientation.y = msg.imu.quaternion[1]
            imu_msg.orientation.z = msg.imu.quaternion[2]
            imu_msg.orientation.w = msg.imu.quaternion[3]
            imu_msg.angular_velocity.x = msg.imu.gyroscope[0]
            imu_msg.angular_velocity.y = msg.imu.gyroscope[1]
            imu_msg.angular_velocity.z = msg.imu.gyroscope[2]
            imu_msg.linear_acceleration.x = msg.imu.accelerometer[0]
            imu_msg.linear_acceleration.y = msg.imu.accelerometer[1]
            imu_msg.linear_acceleration.z = msg.imu.accelerometer[2]
            out_bag.write("/imu_data", imu_msg, t)
            imu_count += 1

        # Camera IMU (already standard Imu)
        elif topic == "/camera/imu":
            msg.header.frame_id = "base"   # optional: unify frame name
            out_bag.write("/camera/imu", msg, t)
            camimu_count += 1

        # GPS topics
        elif topic == "/reach/fix":
            out_bag.write("/gps/fix", msg, t)
            gps_count += 1
        elif topic == "/reach/vel":
            out_bag.write("/gps/vel", msg, t)
            gps_count += 1
        elif topic == "/reach/time_ref":
            out_bag.write("/gps/time_ref", msg, t)
            gps_count += 1

    in_bag.close()
    out_bag.close()
    print(f" Done — LiDAR: {lidar_count}, IMU: {imu_count}, CamIMU: {camimu_count}, GPS: {gps_count}")
    print(f"Output saved: {out_bag_path}")
