#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
import tf_conversions
import tf2_ros

if __name__ == "__main__":
    rospy.init_node("static_tf_publisher_group")
    rospy.loginfo("Publishing all static transforms (persistent)...")

    br = tf2_ros.StaticTransformBroadcaster()

    # Create both transforms
    transforms = []

    # 1. LiDAR ↔ Base
    t1 = TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base"
    t1.child_frame_id = "rslidar"
    t1.transform.translation.x = 0.15
    t1.transform.translation.y = 0.0
    t1.transform.translation.z = 0.13
    q1 = tf_conversions.transformations.quaternion_from_euler(0.0, 0.309017, 0.0)
    t1.transform.rotation.x = q1[0]
    t1.transform.rotation.y = q1[1]
    t1.transform.rotation.z = q1[2]
    t1.transform.rotation.w = q1[3]
    transforms.append(t1)

    # 2. Map ↔ Odom
    t2 = TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "map"
    t2.child_frame_id = "odom"
    t2.transform.translation.x = 0.0
    t2.transform.translation.y = 0.0
    t2.transform.translation.z = 0.0
    q2 = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    t2.transform.rotation.x = q2[0]
    t2.transform.rotation.y = q2[1]
    t2.transform.rotation.z = q2[2]
    t2.transform.rotation.w = q2[3]
    transforms.append(t2)

    # Broadcast all together (persistent)
    br.sendTransform(transforms)
    rospy.loginfo("Static TFs published: base→rslidar and map→odom")
    rospy.spin()
