#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix

# fixed transform between base and gps (your measured offsets)
X, Y, Z = -0.25, 0.0, 0.35
ROLL, PITCH, YAW = 0.0, 0.0, 0.0

def handle_gps(msg):
    t = TransformStamped()
    t.header.stamp = msg.header.stamp  # match GPS message timestamp
    t.header.frame_id = "base"
    t.child_frame_id = "gps"
    t.transform.translation.x = X
    t.transform.translation.y = Y
    t.transform.translation.z = Z
    # Identity rotation (no orientation offset)
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("gps_tf_broadcaster")
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber("/gps/fix", NavSatFix, handle_gps, queue_size=10)
    rospy.loginfo("Publishing base→gps TF with each GPS message timestamp...")
    rospy.spin()
