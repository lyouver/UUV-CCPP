#!/usr/bin/env python3
"""Publish adapter-compatible dynamic obstacle markers from Gazebo model states."""

import argparse
import math
import time

import rospy
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray


class GazeboObstacleMarkerPublisher:
    def __init__(self):
        self.obstacle_names = rospy.get_param("~obstacle_names", [])
        if isinstance(self.obstacle_names, str):
            self.obstacle_names = [x.strip() for x in self.obstacle_names.split(",") if x.strip()]
        self.obstacle_radius = float(rospy.get_param("~obstacle_radius", 2.0))
        self.dynamic_speed_threshold = float(rospy.get_param("~dynamic_speed_threshold", 0.5))
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.output_topic = rospy.get_param("~output_topic", "/onboard_detector/velocity_visualizaton")
        self.last = {}
        self.active_ids = set()
        self.pub = rospy.Publisher(self.output_topic, MarkerArray, queue_size=1)
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.cb, queue_size=1)
        rospy.loginfo(
            "chapter5 fake marker publisher: names=%s topic=%s dynamic_speed_threshold=%.2f",
            self.obstacle_names,
            self.output_topic,
            self.dynamic_speed_threshold,
        )

    def cb(self, msg):
        now = rospy.Time.now()
        markers = []
        active_ids = set()
        for marker_id, name in enumerate(self.obstacle_names):
            if name not in msg.name:
                continue
            idx = msg.name.index(name)
            pose = msg.pose[idx]
            twist = msg.twist[idx]
            vx = float(twist.linear.x)
            vy = float(twist.linear.y)
            prev = self.last.get(name)
            if abs(vx) + abs(vy) < 1e-6 and prev is not None:
                dt = max(1e-3, now.to_sec() - prev["time"])
                vx = (pose.position.x - prev["x"]) / dt
                vy = (pose.position.y - prev["y"]) / dt
            self.last[name] = {"time": now.to_sec(), "x": pose.position.x, "y": pose.position.y}
            speed_xy = math.hypot(vx, vy)
            if speed_xy < self.dynamic_speed_threshold:
                continue

            mk = Marker()
            mk.header.stamp = now
            mk.header.frame_id = self.frame_id
            mk.ns = "chapter5_dynamic_obstacles"
            mk.id = marker_id
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD
            mk.pose = pose
            diameter = max(0.2, 2.0 * self.obstacle_radius)
            mk.scale.x = diameter
            mk.scale.y = diameter
            mk.scale.z = diameter
            mk.color.a = 0.8
            mk.color.r = 1.0
            mk.color.g = 0.1
            mk.color.b = 0.1
            mk.text = "Vx={:.4f}, Vy={:.4f}".format(vx, vy)
            markers.append(mk)
            active_ids.add(marker_id)

        for marker_id in sorted(self.active_ids - active_ids):
            mk = Marker()
            mk.header.stamp = now
            mk.header.frame_id = self.frame_id
            mk.ns = "chapter5_dynamic_obstacles"
            mk.id = marker_id
            mk.action = Marker.DELETE
            markers.append(mk)

        self.active_ids = active_ids
        self.pub.publish(MarkerArray(markers=markers))


def main():
    rospy.init_node("chapter5_fake_obstacle_markers")
    GazeboObstacleMarkerPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
