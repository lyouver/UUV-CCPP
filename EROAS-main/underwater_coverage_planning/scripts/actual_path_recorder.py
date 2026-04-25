#!/usr/bin/env python3
"""Publish the accumulated vehicle ground-truth path for RViz."""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class ActualPathRecorder:
    def __init__(self):
        self.uuv_name = rospy.get_param("~uuv_name", "rexrov")
        self.odom_topic = rospy.get_param("~odom_topic", f"/{self.uuv_name}/pose_gt")
        self.path_topic = rospy.get_param("~path_topic", f"/{self.uuv_name}/actual_path")
        self.path_frame_id = rospy.get_param("~path_frame_id", "")
        self.min_point_distance = max(0.0, float(rospy.get_param("~min_point_distance", 0.15)))
        self.max_points = max(1, int(rospy.get_param("~max_points", 50000)))

        self.path = Path()
        self.last_xyz = None
        self.pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=50)

        rospy.loginfo(
            "actual_path_recorder: odom=%s path=%s min_point_distance=%.2f max_points=%d",
            self.odom_topic,
            self.path_topic,
            self.min_point_distance,
            self.max_points,
        )

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        xyz = (float(pos.x), float(pos.y), float(pos.z))
        if self.last_xyz is not None:
            dist = math.sqrt(
                (xyz[0] - self.last_xyz[0]) ** 2
                + (xyz[1] - self.last_xyz[1]) ** 2
                + (xyz[2] - self.last_xyz[2]) ** 2
            )
            if dist < self.min_point_distance:
                return

        frame_id = self.path_frame_id or msg.header.frame_id or "world"
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = frame_id
        pose.pose = msg.pose.pose

        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = frame_id
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_points:
            self.path.poses = self.path.poses[-self.max_points :]

        self.last_xyz = xyz
        self.pub.publish(self.path)


def main():
    rospy.init_node("actual_path_recorder")
    ActualPathRecorder()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
