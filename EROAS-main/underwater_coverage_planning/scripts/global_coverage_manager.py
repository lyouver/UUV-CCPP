#!/usr/bin/env python3
import math
import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Time
from uuv_control_msgs.msg import Waypoint
from uuv_control_msgs.srv import InitWaypointSet, InitWaypointSetRequest


class GlobalCoverageManager:
    def __init__(self):
        self.path_topic = rospy.get_param("~path_topic", "/nav/global_path")
        self.odom_topic = rospy.get_param("~odom_topic", "/rexrov/pose_gt_ned")
        self.service_name = rospy.get_param("~start_waypoint_service", "/rexrov/start_waypoint_list")

        self.lookahead_distance = float(rospy.get_param("~lookahead_distance", 30.0))
        self.min_waypoints = int(rospy.get_param("~min_waypoints", 5))
        self.max_waypoints = int(rospy.get_param("~max_waypoints", 80))
        self.command_period = float(rospy.get_param("~command_period", 4.0))
        self.index_advance_trigger = int(rospy.get_param("~index_advance_trigger", 5))

        self.max_forward_speed = float(rospy.get_param("~max_forward_speed", 0.6))
        self.heading_offset = float(rospy.get_param("~heading_offset", 0.0))
        self.radius_of_acceptance = float(rospy.get_param("~radius_of_acceptance", 1.0))
        self.use_fixed_heading = bool(rospy.get_param("~use_fixed_heading", False))
        self.interpolator = rospy.get_param("~interpolator", "cubic")

        self.min_waypoint_separation = float(rospy.get_param("~min_waypoint_separation", 0.5))
        self.min_start_distance = float(rospy.get_param("~min_start_distance", 0.3))
        self.closest_index_window = int(rospy.get_param("~closest_index_window", 0))

        self.path_points = None
        self.path_frame = ""
        self.odom_pos = None

        self.last_sent_index = -1
        self.last_sent_time = rospy.Time(0)

        self.path_sub = rospy.Subscriber(self.path_topic, Path, self.path_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=1)

        self.client = rospy.ServiceProxy(self.service_name, InitWaypointSet)

        loop_hz = float(rospy.get_param("~loop_hz", 2.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(loop_hz, 0.1)), self.on_timer)

    def path_cb(self, msg):
        if not msg.poses:
            return
        self.path_frame = msg.header.frame_id or self.path_frame
        pts = []
        for pose in msg.poses:
            p = pose.pose.position
            pts.append((p.x, p.y, p.z))
        self.path_points = np.array(pts, dtype=np.float64)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom_pos = np.array([p.x, p.y, p.z], dtype=np.float64)

    def on_timer(self, _event):
        if self.path_points is None or self.odom_pos is None:
            return
        self._send_global_segment(force=False)

    def _send_global_segment(self, force=False):
        idx = self._closest_index(self.path_points, self.odom_pos)
        segment = self._slice_path(self.path_points, idx)
        if len(segment) < 2:
            return False
        if not force and not self._should_send(idx):
            return False
        if self._send_waypoints(segment):
            self.last_sent_index = idx
            self.last_sent_time = rospy.Time.now()
            return True
        return False

    def _closest_index(self, points, pos):
        if points is None or len(points) == 0:
            return 0
        start_idx = 0
        if self.last_sent_index >= 0:
            start_idx = int(self.last_sent_index)
        end_idx = len(points) - 1
        if self.closest_index_window > 0:
            end_idx = min(end_idx, start_idx + self.closest_index_window)
        return self._closest_index_window(points, pos, start_idx, end_idx)

    def _closest_index_window(self, points, pos, start_idx, end_idx):
        start_idx = max(0, int(start_idx))
        end_idx = min(len(points) - 1, int(end_idx))
        if end_idx < start_idx:
            return start_idx
        diffs = points[start_idx:end_idx + 1] - pos
        dists = np.linalg.norm(diffs, axis=1)
        return int(start_idx + np.argmin(dists))

    def _slice_path(self, points, start_idx):
        if start_idx < 0 or start_idx >= len(points):
            return []
        segment = [points[start_idx]]
        dist = 0.0
        i = start_idx
        while i + 1 < len(points) and (dist < self.lookahead_distance or len(segment) < self.min_waypoints):
            p0 = points[i]
            p1 = points[i + 1]
            dist += float(np.linalg.norm(p1 - p0))
            segment.append(p1)
            i += 1
            if len(segment) >= self.max_waypoints:
                break
        return segment

    def _should_send(self, idx):
        if self.last_sent_index < 0:
            return True
        if abs(idx - self.last_sent_index) >= self.index_advance_trigger:
            return True
        return False

    def _send_waypoints(self, segment):
        segment = self._filter_waypoints(segment)
        if len(segment) < 2:
            return False
        req = InitWaypointSetRequest()
        req.start_now = True
        req.start_time = Time(data=rospy.Time.now())
        req.max_forward_speed = self.max_forward_speed
        req.heading_offset = self.heading_offset
        req.interpolator = String(data=self.interpolator)

        waypoints = []
        frame = self.path_frame or "world"
        now = rospy.Time.now()
        for pt in segment:
            wp = Waypoint()
            wp.header.stamp = now
            wp.header.frame_id = frame
            wp.point.x = float(pt[0])
            wp.point.y = float(pt[1])
            wp.point.z = float(pt[2])
            wp.max_forward_speed = self.max_forward_speed
            wp.heading_offset = self.heading_offset
            wp.use_fixed_heading = self.use_fixed_heading
            wp.radius_of_acceptance = self.radius_of_acceptance
            waypoints.append(wp)

        req.waypoints = waypoints
        try:
            self.client.wait_for_service(timeout=2.0)
            resp = self.client(req)
            if not resp.success:
                rospy.logwarn("start_waypoint_list returned failure")
            return resp.success
        except rospy.ROSException:
            rospy.logwarn("start_waypoint_list service unavailable")
        except rospy.ServiceException as exc:
            rospy.logwarn("start_waypoint_list call failed: %s", exc)
        return False

    def _filter_waypoints(self, segment):
        if not segment:
            return []
        filtered = []
        min_sep = max(self.min_waypoint_separation, 1e-6)
        min_start = max(self.min_start_distance, 0.0)
        for pt in segment:
            p = np.array(pt, dtype=np.float64)
            if not filtered and self.odom_pos is not None:
                if np.linalg.norm(p - self.odom_pos) < min_start:
                    continue
            if filtered:
                if np.linalg.norm(p - filtered[-1]) < min_sep:
                    continue
            filtered.append(p)
        return filtered


def main():
    rospy.init_node("global_coverage_manager")
    GlobalCoverageManager()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
