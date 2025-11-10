#!/usr/bin/env python3
import rosbag
import math
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField, Imu
import rospy

# ==========================
# CONFIGURATION
# ==========================
IN_BAG  = '/root/Downloads/bpearl_fixed_synced_camimu_0.bag'
OUT_BAG = '/root/Downloads/bpearl_camimu_0.bag'

# --------------------------
# LiDAR filtering parameters
# --------------------------
FIELDS_OUT = [
    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PointField.UINT16,  count=1),
    PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
]

def finite(*vals):
    return all(math.isfinite(v) for v in vals)

def filter_lidar(msg):
    """Filters invalid points, keeps ring/time."""
    field_names = [f.name for f in msg.fields]
    has_ring = 'ring' in field_names
    has_time = 'time' in field_names

    read_names = ['x', 'y', 'z', 'intensity']
    if has_ring: read_names.append('ring')
    if has_time: read_names.append('time')

    pts = []
    for tpl in pc2.read_points(msg, field_names=read_names, skip_nans=True):
        x, y, z, intensity = map(float, tpl[:4])
        if not finite(x, y, z, intensity):
            continue
        i = 4
        ring_i = int(tpl[i]) if has_ring else 0
        if has_ring: i += 1
        r = math.sqrt(x*x + y*y + z*z)
        if r < 0.5 or r > 100.0:
            continue
        t_rel = float(tpl[i]) if has_time else 0.0
        pts.append((x, y, z, intensity, ring_i, t_rel))

    out = pc2.create_cloud(msg.header, FIELDS_OUT, pts)
    out.is_dense = True
    return out

# --------------------------
# IMU resampling parameters
# --------------------------
PUB_RATE = 100.0
MAX_GAP  = 0.08
ACC_MAX  = 25.0
ACC_MIN  = 5.0
GYR_MAX  = 6.0
EMA_ALPHA = 0.3
GRAVITY_N = 9.8
BIAS_INIT = 200

def lerp(a,b,u): return a + (b-a)*u

def slerp(q1,q2,u):
    q1 = np.array(q1, dtype=float)
    q2 = np.array(q2, dtype=float)
    dot = np.dot(q1,q2)
    if dot < 0.0:
        q2 = -q2; dot = -dot
    DOT_THRESH = 0.9995
    if dot > DOT_THRESH:
        r = q1 + u*(q2-q1)
        return r / np.linalg.norm(r)
    theta_0 = math.acos(dot)
    sin_0 = math.sin(theta_0)
    theta = theta_0*u
    sin_t = math.sin(theta)
    s0 = math.cos(theta) - dot*sin_t/sin_0
    s1 = sin_t/sin_0
    return s0*q1 + s1*q2

def interp_msg(t_target, A, B):
    ta, a = A; tb, b = B
    dt = tb - ta
    u = 0.0 if dt <= 0 else min(1.0, max(0.0, (t_target - ta)/dt))
    out = Imu()
    out.header.stamp = a.header.stamp
    out.header.frame_id = a.header.frame_id
    for axis in ['x','y','z']:
        setattr(out.angular_velocity, axis, lerp(getattr(a.angular_velocity, axis), getattr(b.angular_velocity, axis), u))
        setattr(out.linear_acceleration, axis, lerp(getattr(a.linear_acceleration, axis), getattr(b.linear_acceleration, axis), u))
    qA = [a.orientation.x,a.orientation.y,a.orientation.z,a.orientation.w]
    qB = [b.orientation.x,b.orientation.y,b.orientation.z,b.orientation.w]
    qI = slerp(qA,qB,u)
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = qI
    out.angular_velocity_covariance = a.angular_velocity_covariance
    out.linear_acceleration_covariance = a.linear_acceleration_covariance
    out.orientation_covariance = a.orientation_covariance
    return out

def filter_and_clip(msg, acc_bias, gyr_bias, last_acc, last_gyr):
    ax,ay,az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
    gx,gy,gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
    acc = np.array([ax,ay,az]) - acc_bias
    gyr = np.array([gx,gy,gz]) - gyr_bias
    acc_norm = np.linalg.norm(acc)
    gyr_norm = np.linalg.norm(gyr)
    if acc_norm>ACC_MAX or acc_norm<ACC_MIN or gyr_norm>GYR_MAX:
        acc, gyr = last_acc, last_gyr
    else:
        acc = EMA_ALPHA*acc + (1-EMA_ALPHA)*last_acc
        gyr = EMA_ALPHA*gyr + (1-EMA_ALPHA)*last_gyr
        last_acc, last_gyr = acc, gyr
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyr
    return msg, last_acc, last_gyr

def process_imu(all_imu, keep_first_n=200):
    """
    Interpolate IMU to 100Hz, remove bias, smooth, ensure monotonic timestamps.
    The first `keep_first_n` IMU messages are kept raw (unfiltered) to stabilize LIO-SAM preintegration.
    """
    if len(all_imu) < 3:
        return all_imu

    all_imu.sort(key=lambda x: x[0])
    t0 = all_imu[0][0]
    t_end = all_imu[-1][0]

    # --- Compute initial bias ---
    arr = np.array([[m.linear_acceleration.x,
                     m.linear_acceleration.y,
                     m.linear_acceleration.z,
                     m.angular_velocity.x,
                     m.angular_velocity.y,
                     m.angular_velocity.z] for _, m in all_imu[:BIAS_INIT]])
    acc_bias = np.mean(arr[:, :3], axis=0) - np.array([0, 0, GRAVITY_N])
    gyr_bias = np.mean(arr[:, 3:], axis=0)

    # --- Buffers ---
    resampled = []
    last_acc = np.array([0.0, 0.0, GRAVITY_N])
    last_gyr = np.array([0.0, 0.0, 0.0])
    last_t = t0 - 1e-6
    t = t0

    # --- Step 1: write first N IMU messages exactly as they came ---
    for i, (ts, msg) in enumerate(all_imu[:keep_first_n]):
        msg.header.stamp = rospy.Time.from_sec(round(ts, 6))
        resampled.append((ts, msg))
        last_t = ts

    # --- Step 2: apply interpolation / bias correction after first N messages ---
    while t <= t_end:
        pre, post = None, None
        for i in range(len(all_imu) - 1):
            if all_imu[i][0] <= t <= all_imu[i + 1][0]:
                pre, post = all_imu[i], all_imu[i + 1]
                break

        if pre and post and post[0] - pre[0] <= MAX_GAP:
            msg_out = interp_msg(t, pre, post)
            msg_out, last_acc, last_gyr = filter_and_clip(msg_out, acc_bias, gyr_bias, last_acc, last_gyr)
        else:
            msg_out = all_imu[-1][1]

        if t <= last_t:
            t = last_t + 1e-6
        last_t = t
        msg_out.header.stamp = rospy.Time.from_sec(round(t, 6))
        msg_out.header.frame_id = "base_link"
        resampled.append((t, msg_out))
        t = round(t + 1.0 / PUB_RATE, 6)

    return resampled

# --------------------------
# Frame rename mapping
# --------------------------
frame_map = {
    "rslidar": "velodyne",
    "base": "base_link",
    "gps": "navsat_link",
}

# ==========================
# MAIN PIPELINE
# ==========================
print(f"Reading {IN_BAG} ...")
bag_in  = rosbag.Bag(IN_BAG)
bag_out = rosbag.Bag(OUT_BAG, 'w')
imu_buffer = []

for topic, msg, t in bag_in.read_messages():
    # LiDAR filtering
    if topic == '/rslidar_points':
        out_msg = filter_lidar(msg)
        # rename frame if needed
        if out_msg.header.frame_id in frame_map:
            out_msg.header.frame_id = frame_map[out_msg.header.frame_id]
        bag_out.write('/rslidar_points_dense', out_msg, t)
        bag_out.write(topic, msg, t)

    # IMU collection
    elif topic == '/imu_data':
        imu_buffer.append((t.to_sec(), msg))

    # Camera IMU (already clean, copy directly)
    elif topic == '/camera/imu':
        msg.header.frame_id = "base_link"   # unify frame name for consistency
        bag_out.write('/camera/imu', msg, t)


    # rename frames on GPS & others
    elif hasattr(msg, "header") and hasattr(msg.header, "frame_id"):
        if msg.header.frame_id in frame_map:
            msg.header.frame_id = frame_map[msg.header.frame_id]
        bag_out.write(topic, msg, t)
    else:
        bag_out.write(topic, msg, t)

bag_in.close()

print("Processing IMU (with corrected gravity and interpolation)...")
imu_resampled = process_imu(imu_buffer)
for t, msg in imu_resampled:
    # rename IMU frame if needed
    if msg.header.frame_id in frame_map:
        msg.header.frame_id = frame_map[msg.header.frame_id]
    bag_out.write('/imu_data_resampled', msg, rospy.Time.from_sec(t))

bag_out.close()
print("Saved final corrected bag at:", OUT_BAG)
