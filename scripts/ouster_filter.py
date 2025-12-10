#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class NanRemover:
    def __init__(self):
        # Subscribe to the new correct topic
        input_topic = "/ouster/points/corrected"
        output_topic = "/ouster/points_clean"

        self.sub = rospy.Subscriber(input_topic, PointCloud2, self.callback, queue_size=10)
        self.pub = rospy.Publisher(output_topic, PointCloud2, queue_size=10)

        rospy.loginfo("NaN remover subscribed to %s → publishing %s", input_topic, output_topic)

    def callback(self, msg):
        # Remove NaNs using skip_nans=True
        points = list(pc2.read_points(msg, skip_nans=True))

        if len(points) == 0:
            rospy.logwarn("All points were NaN — skipping frame")
            return

        cleaned = pc2.create_cloud(msg.header, msg.fields, points)
        cleaned.is_dense = True   # REQUIRED for LIO-SAM

        # Publish
        self.pub.publish(cleaned)

if __name__ == "__main__":
    rospy.init_node("nan_remover")
    NanRemover()
    rospy.spin()
