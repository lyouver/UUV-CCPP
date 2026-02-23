#!/usr/bin/env python3
import math
import os
import re
import threading
import yaml
import numpy as np

import rospy
from std_msgs.msg import Header, Bool, String, Float64
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint, Waypoint
import tf2_ros
from tf.transformations import quaternion_from_euler, quaternion_matrix


def enu_to_ned_vec(x, y, z):
    return np.array([y, x, -z], dtype=float)


def ned_to_enu_vec(x, y, z):
    return np.array([y, x, -z], dtype=float)


class LocalAvoidancePlanner:
    def __init__(self):
        self.lock = threading.Lock()

        self.uuv_name = rospy.get_param("~uuv_name", "rexrov")
        self.inertial_frame_id = rospy.get_param("~inertial_frame_id", "world_ned")
        self.global_waypoint_file = rospy.get_param("~global_waypoint_file", "")
        self.max_forward_speed = float(rospy.get_param("~max_forward_speed", 0.6))
        self.dt = float(rospy.get_param("~dt", 0.5))
        # Keep trajectory discretization (dt) independent from planner publish period.
        # If replanning too often, controller keeps resetting to a fresh trajectory.
        self.plan_period = float(rospy.get_param("~plan_period", max(1.5, self.dt)))
        self.horizon_time = float(rospy.get_param("~horizon_time", 6.0))
        self.path_resolution = float(rospy.get_param("~path_resolution", self.max_forward_speed * self.dt))
        self.publish_global_path = bool(rospy.get_param("~publish_global_path", True))
        self.publish_local_path = bool(rospy.get_param("~publish_local_path", True))
        self.local_path_topic = rospy.get_param("~local_path_topic", "/nav/local_path")
        self.global_path_topic = rospy.get_param("~global_path_topic", "/nav/global_path")
        self.odom_topic = rospy.get_param("~odom_topic", f"/{self.uuv_name}/pose_gt_ned")
        self.model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self.perception_source = rospy.get_param("~perception_source", "lidar")
        if self.perception_source not in ("lidar", "map_manager"):
            rospy.logwarn(
                "Unsupported perception_source=%s, fallback to lidar",
                self.perception_source,
            )
            self.perception_source = "lidar"
        self.fallback_to_model_states = bool(rospy.get_param("~fallback_to_model_states", False))
        if self.fallback_to_model_states:
            rospy.logwarn("fallback_to_model_states disabled (sensor-only avoidance enabled)")
            self.fallback_to_model_states = False
        self.lidar_topic = rospy.get_param("~lidar_topic", "")
        self.map_cloud_topic = rospy.get_param("~map_cloud_topic", "")
        self.map_timeout = float(rospy.get_param("~map_timeout", 1.0))
        self.map_obstacles_max = int(rospy.get_param("~map_obstacles_max", 25))
        self.map_prefer_dynamic = bool(rospy.get_param("~map_prefer_dynamic", True))
        self.map_use_collision_service = bool(rospy.get_param("~map_use_collision_service", True))
        self.map_collision_service = rospy.get_param("~map_collision_service", "/occupancy_map/check_pos_collision")
        self.map_collision_inflated = bool(rospy.get_param("~map_collision_inflated", True))
        self.map_collision_step = float(rospy.get_param("~map_collision_step", 0.5))
        self.detector_use_velocity_track = bool(
            rospy.get_param("~detector_use_velocity_track", True)
        )
        self.detector_velocity_topic = rospy.get_param(
            "~detector_velocity_topic", "/onboard_detector/velocity_visualizaton"
        )
        self.detector_track_timeout = float(
            rospy.get_param("~detector_track_timeout", self.map_timeout)
        )
        self.detector_obstacle_radius = float(
            rospy.get_param("~detector_obstacle_radius", 2.0)
        )
        self.lidar_min_range = float(rospy.get_param("~lidar_min_range", 1.0))
        self.lidar_max_range = float(rospy.get_param("~lidar_max_range", 40.0))
        self.lidar_min_z = float(rospy.get_param("~lidar_min_z", -5.0))
        self.lidar_max_z = float(rospy.get_param("~lidar_max_z", 5.0))
        self.lidar_voxel = float(rospy.get_param("~lidar_voxel", 0.5))
        self.lidar_max_points = int(rospy.get_param("~lidar_max_points", 6000))
        self.cluster_dist = float(rospy.get_param("~cluster_dist", 1.0))
        self.cluster_min_points = int(rospy.get_param("~cluster_min_points", 12))
        self.obstacle_z_window = float(rospy.get_param("~obstacle_z_window", 3.0))
        self.reject_large_clusters = bool(rospy.get_param("~reject_large_clusters", True))
        self.track_association_dist = float(rospy.get_param("~track_association_dist", 2.5))
        self.map_track_reset_dist = float(
            rospy.get_param("~map_track_reset_dist", max(8.0, self.track_association_dist * 8.0))
        )
        self.track_timeout = float(rospy.get_param("~track_timeout", 2.0))
        self.track_vel_alpha = float(rospy.get_param("~track_vel_alpha", 0.6))
        self.use_cluster_radius = bool(rospy.get_param("~use_cluster_radius", True))
        self.obstacle_radius_min = float(rospy.get_param("~obstacle_radius_min", 0.8))
        self.obstacle_radius_max = float(rospy.get_param("~obstacle_radius_max", 3.0))
        self.max_obstacles = int(rospy.get_param("~max_obstacles", 5))
        self.use_static_obstacles = bool(rospy.get_param("~use_static_obstacles", False))
        self.min_obstacle_speed = float(rospy.get_param("~min_obstacle_speed", 0.05))
        self.min_track_age = float(rospy.get_param("~min_track_age", 0.5))
        self.static_obstacle_dist = float(rospy.get_param("~static_obstacle_dist", 3.0))
        self.debug_lidar = bool(rospy.get_param("~debug_lidar", False))
        self.obstacle_names = rospy.get_param("~obstacle_names", ["moving_sphere_obstacle"])
        self.obstacle_radius = float(rospy.get_param("~obstacle_radius", 2.0))
        self.autostart_waypoints = bool(rospy.get_param("~autostart_waypoints", True))
        self.autostart_interpolator = rospy.get_param("~autostart_interpolator", "cubic")
        self.dist_thresh_dynamic = float(rospy.get_param("~dist_thresh_dynamic", 4.0))
        self.pred_horizon = float(rospy.get_param("~pred_horizon", 4.0))
        self.weight_ref = float(rospy.get_param("~weight_ref", 1.0))
        self.weight_smooth = float(rospy.get_param("~weight_smooth", 0.2))
        self.weight_dynamic = float(rospy.get_param("~weight_dynamic", 1.0))
        self.step_size = float(rospy.get_param("~step_size", 0.3))
        self.iterations = int(rospy.get_param("~iterations", 5))
        self.smooth_alpha = float(rospy.get_param("~smooth_alpha", 0.6))
        self.rejoin_clearance = float(rospy.get_param("~rejoin_clearance", 1.0))
        self.rejoin_alpha = float(rospy.get_param("~rejoin_alpha", 0.25))
        self.limit_speed = bool(rospy.get_param("~limit_speed", True))
        self.side_bias_weight = float(rospy.get_param("~side_bias_weight", 0.8))
        self.side_lock_dist = float(rospy.get_param("~side_lock_dist", 6.0))
        self.side_release_dist = float(rospy.get_param("~side_release_dist", 8.0))
        self.use_corridor_filter = bool(rospy.get_param("~use_corridor_filter", True))
        self.forward_window = float(rospy.get_param("~forward_window", 25.0))
        self.lateral_window = float(rospy.get_param("~lateral_window", 6.0))
        self.behind_ignore = float(rospy.get_param("~behind_ignore", 3.0))
        self.ignore_z_in_dynamic_cost = bool(rospy.get_param("~ignore_z_in_dynamic_cost", True))
        self.yaw_from_path = bool(rospy.get_param("~yaw_from_path", True))
        self.plan_2d = bool(rospy.get_param("~plan_2d", True))
        self.replan_min_interval = float(rospy.get_param("~replan_min_interval", 2.0))
        self.min_avoid_hold_time = float(rospy.get_param("~min_avoid_hold_time", 3.0))
        self.force_maneuver_enable = bool(rospy.get_param("~force_maneuver_enable", True))
        self.force_maneuver_right = float(rospy.get_param("~force_maneuver_right", 5.0))
        self.force_maneuver_forward = float(rospy.get_param("~force_maneuver_forward", 10.0))
        self.force_maneuver_left = float(rospy.get_param("~force_maneuver_left", 5.0))
        self.force_maneuver_rejoin_right = float(rospy.get_param("~force_maneuver_rejoin_right", 5.0))
        self.force_maneuver_finish_tol = float(rospy.get_param("~force_maneuver_finish_tol", 1.0))
        self.force_maneuver_retrigger_cooldown = float(
            rospy.get_param("~force_maneuver_retrigger_cooldown", 8.0)
        )
        self.force_maneuver_trigger_min_forward = float(
            rospy.get_param("~force_maneuver_trigger_min_forward", 0.5)
        )
        self.dynamic_arc_enable = bool(rospy.get_param("~dynamic_arc_enable", True))
        self.dynamic_arc_speed_threshold = float(
            rospy.get_param("~dynamic_arc_speed_threshold", 0.4)
        )
        self.dynamic_arc_observe_time = float(
            rospy.get_param("~dynamic_arc_observe_time", 1.5)
        )
        self.dynamic_arc_range_default = float(
            rospy.get_param("~dynamic_arc_range_default", 2.0)
        )
        self.dynamic_arc_margin = float(rospy.get_param("~dynamic_arc_margin", 1.5))
        self.dynamic_arc_forward_margin = float(
            rospy.get_param("~dynamic_arc_forward_margin", 6.0)
        )
        self.dynamic_arc_min_offset = float(
            rospy.get_param("~dynamic_arc_min_offset", 4.0)
        )
        self.dynamic_arc_finish_tol = float(
            rospy.get_param("~dynamic_arc_finish_tol", 1.0)
        )
        self.dynamic_arc_retrigger_cooldown = float(
            rospy.get_param("~dynamic_arc_retrigger_cooldown", 6.0)
        )
        self.dynamic_arc_trigger_min_forward = float(
            rospy.get_param("~dynamic_arc_trigger_min_forward", 0.5)
        )
        self.global_rejoin_hold_time = float(rospy.get_param("~global_rejoin_hold_time", 3.0))
        self.avoidance_clearance = float(rospy.get_param("~avoidance_clearance", 1.0))
        self.avoid_longitudinal = float(rospy.get_param("~avoid_longitudinal", 10.0))
        self.avoid_lateral_max = float(rospy.get_param("~avoid_lateral_max", 6.0))
        self.offset_smooth_window = int(rospy.get_param("~offset_smooth_window", 5))
        self.z_hold_mode = rospy.get_param("~z_hold_mode", "current")
        self.publish_epsilon = float(rospy.get_param("~publish_epsilon", 0.2))
        self.publish_keepalive = float(rospy.get_param("~publish_keepalive", 4.0))

        self.robot_pos = None
        self.robot_vel = None
        self.robot_quat = None
        self.obstacle_track = None
        self.obstacle_last_seen = None
        self.prev_traj = None
        self.avoidance_side = None
        self.avoidance_active = False
        self.z_hold = None
        self.last_plan_time = 0.0
        self.last_traj = None
        self.last_publish_time = 0.0
        self.last_published_traj = None
        self.local_override_active = False
        self.avoid_hold_until = 0.0
        self.force_maneuver_active = False
        self.force_maneuver_path = None
        self.force_maneuver_goal = None
        self.force_maneuver_block_until = 0.0
        self.dynamic_arc_state = None
        self.dynamic_arc_block_until = 0.0
        self.global_rejoin_hold_until = 0.0
        self.global_progress_idx = 0

        self.tf_buffer = None
        self.tf_listener = None
        self.lidar_retry_timer = None
        self.lidar_sub = None
        self.map_retry_timer = None
        self.map_sub = None
        self.map_obstacles = []
        self.map_last_seen = None
        self.map_primary_track = None
        self.detector_sub = None
        self.detector_tracks = []
        self.detector_last_seen = None
        self.map_collision_client = None
        self._map_collision_req_cls = None
        self._map_collision_srv_cls = None
        self._last_map_collision_connect_try = 0.0
        self._autostart_waypoints_done = False
        self._autostart_retry_timer = None

        self.global_waypoints = self._load_global_waypoints(self.global_waypoint_file)
        self.global_path_dense = self._densify_path(self.global_waypoints, self.path_resolution)

        self.global_path_pub = rospy.Publisher(self.global_path_topic, Path, queue_size=1, latch=True)
        self.local_path_pub = rospy.Publisher(self.local_path_topic, Path, queue_size=1)
        self.traj_pub = rospy.Publisher(f"/{self.uuv_name}/dp_controller/input_trajectory", Trajectory, queue_size=1)
        self.mode_pub = rospy.Publisher("~planner_mode", String, queue_size=1, latch=True)
        self.avoid_active_pub = rospy.Publisher("~avoidance_active", Bool, queue_size=1, latch=True)
        self.local_publish_pub = rospy.Publisher("~local_trajectory_published", Bool, queue_size=1)
        self.global_ref_pub = rospy.Publisher("~global_reference_influence", Bool, queue_size=1, latch=True)
        self.obstacle_speed_pub = rospy.Publisher("~obstacle_speed", Float64, queue_size=1)
        self.obstacle_over_min_pub = rospy.Publisher(
            "~obstacle_speed_over_min_threshold", Bool, queue_size=1)
        self.obstacle_over_arc_pub = rospy.Publisher(
            "~obstacle_speed_over_arc_threshold", Bool, queue_size=1)
        self.min_speed_threshold_pub = rospy.Publisher(
            "~min_obstacle_speed_threshold", Float64, queue_size=1, latch=True)
        self.arc_speed_threshold_pub = rospy.Publisher(
            "~arc_speed_threshold", Float64, queue_size=1, latch=True)
        self._last_mode = ""

        self.min_speed_threshold_pub.publish(Float64(self.min_obstacle_speed))
        self.arc_speed_threshold_pub.publish(Float64(self.dynamic_arc_speed_threshold))

        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        if self.perception_source == "lidar":
            if not self.lidar_topic:
                self.lidar_topic = self._detect_lidar_topic()
            if self.lidar_topic:
                self.lidar_sub = rospy.Subscriber(self.lidar_topic, PointCloud2, self._lidar_cb, queue_size=1)
                rospy.loginfo("LiDAR topic set to %s", self.lidar_topic)
            else:
                rospy.logwarn("No LiDAR topic found yet, will keep trying")
                self.lidar_retry_timer = rospy.Timer(rospy.Duration(1.0), self._retry_lidar_sub)
        elif self.perception_source == "map_manager":
            if not self.map_cloud_topic:
                self.map_cloud_topic = self._detect_map_topic()
            if self.map_cloud_topic:
                self.map_sub = rospy.Subscriber(self.map_cloud_topic, PointCloud2, self._map_cloud_cb, queue_size=1)
                rospy.loginfo("Map cloud topic set to %s", self.map_cloud_topic)
            else:
                rospy.logwarn("No map cloud topic found yet, will keep trying")
                self.map_retry_timer = rospy.Timer(rospy.Duration(1.0), self._retry_map_sub)
            if self.detector_use_velocity_track and self.detector_velocity_topic:
                self.detector_sub = rospy.Subscriber(
                    self.detector_velocity_topic, MarkerArray, self._detector_vel_cb, queue_size=1
                )
                rospy.loginfo("Detector velocity topic set to %s", self.detector_velocity_topic)
            if self.map_use_collision_service:
                self._connect_map_collision_service()

        self.timer = rospy.Timer(rospy.Duration(self.plan_period), self._plan_cb)

        if self.publish_global_path:
            self._publish_global_path()

        if self.autostart_waypoints:
            self._autostart_waypoints_done = self._start_waypoint_tracking()
            if not self._autostart_waypoints_done:
                rospy.logwarn("Will keep retrying waypoint autostart until controller service is available")
                self._autostart_retry_timer = rospy.Timer(
                    rospy.Duration(2.0), self._retry_start_waypoint_tracking
                )

    def _retry_start_waypoint_tracking(self, _event):
        if self._autostart_waypoints_done:
            if self._autostart_retry_timer is not None:
                self._autostart_retry_timer.shutdown()
                self._autostart_retry_timer = None
            return
        self._autostart_waypoints_done = self._start_waypoint_tracking()
        if self._autostart_waypoints_done and self._autostart_retry_timer is not None:
            self._autostart_retry_timer.shutdown()
            self._autostart_retry_timer = None

    def _resume_global_tracking_if_needed(self):
        if not self.local_override_active:
            return
        if self._start_waypoint_tracking():
            self.local_override_active = False
            rospy.loginfo("local_avoidance_planner: resumed global waypoint tracking")
        else:
            rospy.logwarn_throttle(
                5.0,
                "local_avoidance_planner: failed to resume global waypoint tracking, will retry",
            )

    def _start_waypoint_tracking(self):
        # Prefer restarting global tracking from nearest forward path point.
        with self.lock:
            robot_pos = None if self.robot_pos is None else self.robot_pos.copy()
        if robot_pos is not None and self._start_waypoint_tracking_from_forward(robot_pos):
            return True

        # Fallback to loading from file if forward restart service is unavailable.
        if not self.global_waypoint_file:
            return False
        service_name = f"/{self.uuv_name}/init_waypoints_from_file"
        try:
            rospy.wait_for_service(service_name, timeout=1.0)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "Service %s not available yet: %s", service_name, e)
            return False
        try:
            from uuv_control_msgs.srv import InitWaypointsFromFile, InitWaypointsFromFileRequest
            req = InitWaypointsFromFileRequest()
            req.start_now = True
            req.filename.data = os.path.abspath(self.global_waypoint_file)
            req.interpolator.data = self.autostart_interpolator
            client = rospy.ServiceProxy(service_name, InitWaypointsFromFile)
            resp = client(req)
            if not resp.success:
                rospy.logwarn("init_waypoints_from_file failed")
                return False
            rospy.loginfo("Waypoint autostart succeeded via %s", service_name)
            return True
        except Exception as e:
            rospy.logwarn("init_waypoints_from_file call error: %s", e)
            return False

    def _nearest_forward_index(self, points, pos, start_idx=0):
        if points is None or len(points) == 0:
            return 0
        start_idx = max(0, min(int(start_idx), len(points) - 1))
        nearest = self._nearest_index(points, pos, use_2d=True)
        idx0 = max(nearest, start_idx)

        best_idx = None
        best_dist = float("inf")
        for i in range(idx0, len(points)):
            t_hat = self._path_tangent_2d(points, i)
            rel = points[i][:2] - pos[:2]
            forward = float(np.dot(rel, t_hat))
            if forward < 0.0:
                continue
            d = float(np.linalg.norm(rel))
            if d < best_dist:
                best_dist = d
                best_idx = i
        if best_idx is None:
            return idx0
        return int(best_idx)

    def _build_forward_waypoint_msgs(self, robot_pos):
        if not self.global_path_dense:
            return []
        idx = self._nearest_forward_index(self.global_path_dense, robot_pos, self.global_progress_idx)
        self.global_progress_idx = idx
        points = self.global_path_dense[idx:]
        if len(points) == 0:
            return []

        msgs = []
        stamp = rospy.Time.now()
        for p in points:
            wp = Waypoint()
            wp.header.stamp = stamp
            wp.header.frame_id = self.inertial_frame_id
            wp.point.x = float(p[0])
            wp.point.y = float(p[1])
            wp.point.z = float(p[2])
            wp.max_forward_speed = float(self.max_forward_speed)
            wp.heading_offset = 0.0
            wp.use_fixed_heading = False
            wp.radius_of_acceptance = 0.5
            msgs.append(wp)
        return msgs

    def _start_waypoint_tracking_from_forward(self, robot_pos):
        service_name = f"/{self.uuv_name}/start_waypoint_list"
        try:
            rospy.wait_for_service(service_name, timeout=0.8)
        except Exception:
            return False
        try:
            from uuv_control_msgs.srv import InitWaypointSet, InitWaypointSetRequest
            req = InitWaypointSetRequest()
            req.start_now = True
            req.max_forward_speed = float(self.max_forward_speed)
            req.heading_offset = 0.0
            req.interpolator.data = self.autostart_interpolator
            req.waypoints = self._build_forward_waypoint_msgs(robot_pos)
            if len(req.waypoints) == 0:
                return False
            client = rospy.ServiceProxy(service_name, InitWaypointSet)
            resp = client(req)
            if not resp.success:
                rospy.logwarn("start_waypoint_list failed")
                return False
            rospy.loginfo(
                "Waypoint autostart succeeded via %s (forward idx=%d, n=%d)",
                service_name, self.global_progress_idx, len(req.waypoints))
            return True
        except Exception as e:
            rospy.logwarn("start_waypoint_list call error: %s", e)
            return False

    def _load_global_waypoints(self, file_path):
        if not file_path or not os.path.isfile(file_path):
            rospy.logerr("Global waypoint file not found: %s", file_path)
            return []
        with open(file_path, "r") as f:
            data = yaml.safe_load(f) or {}

        file_frame = data.get("inertial_frame_id", self.inertial_frame_id)
        waypoints = []
        for wp in data.get("waypoints", []):
            p = wp.get("point", [0.0, 0.0, 0.0])
            p = np.array([float(p[0]), float(p[1]), float(p[2])], dtype=float)
            if file_frame != self.inertial_frame_id:
                if file_frame == "world" and self.inertial_frame_id == "world_ned":
                    p = enu_to_ned_vec(p[0], p[1], p[2])
                elif file_frame == "world_ned" and self.inertial_frame_id == "world":
                    p = ned_to_enu_vec(p[0], p[1], p[2])
            waypoints.append(p)

        if len(waypoints) == 0:
            rospy.logerr("No waypoints loaded from %s", file_path)
        return waypoints

    def _densify_path(self, points, resolution):
        if len(points) < 2:
            return points
        if resolution <= 0:
            resolution = 0.5
        dense = []
        for i in range(len(points) - 1):
            p0 = points[i]
            p1 = points[i + 1]
            dist = np.linalg.norm(p1 - p0)
            steps = max(1, int(math.ceil(dist / resolution)))
            for s in range(steps):
                t = float(s) / float(steps)
                dense.append(p0 + t * (p1 - p0))
        dense.append(points[-1])
        return dense

    def _publish_global_path(self):
        if not self.global_path_dense:
            return
        path = Path()
        path.header.frame_id = self.inertial_frame_id
        path.header.stamp = rospy.Time.now()
        for p in self.global_path_dense:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            ps.pose.position.z = p[2]
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.global_path_pub.publish(path)

    def _odom_cb(self, msg):
        with self.lock:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            vel = msg.twist.twist.linear
            self.robot_pos = np.array([pos.x, pos.y, pos.z], dtype=float)
            self.robot_vel = np.array([vel.x, vel.y, vel.z], dtype=float)
            self.robot_quat = np.array([ori.x, ori.y, ori.z, ori.w], dtype=float)
        rospy.loginfo_once("local_avoidance_planner: odom received")

    def _apply_pose_to_points(self, points, pos, quat):
        if points is None:
            return []
        if pos is None or quat is None:
            return []
        rot = quaternion_matrix([quat[0], quat[1], quat[2], quat[3]])[0:3, 0:3]
        trans_vec = np.array([pos[0], pos[1], pos[2]], dtype=float)
        out = []
        for p in points:
            out.append(rot.dot(p) + trans_vec)
        return out

    def _model_states_cb(self, msg):
        rospy.logwarn_throttle(5.0, "model_states ignored (sensor-only avoidance enabled)")
        return

    def _detect_lidar_topic(self):
        try:
            topics = rospy.get_published_topics()
        except Exception:
            return ""
        candidates = []
        for name, ttype in topics:
            if ttype != "sensor_msgs/PointCloud2":
                continue
            if "lidar" in name and "points" in name:
                candidates.append(name)
        if not candidates:
            return ""
        preferred = f"/{self.uuv_name}/lidar/points"
        if preferred in candidates:
            return preferred
        preferred_ns = f"/{self.uuv_name}/{self.uuv_name}/lidar/points"
        if preferred_ns in candidates:
            return preferred_ns
        return candidates[0]

    def _detect_map_topic(self):
        try:
            topics = rospy.get_published_topics()
        except Exception:
            return ""
        candidates = []
        for name, ttype in topics:
            if ttype != "sensor_msgs/PointCloud2":
                continue
            if ("inflated_voxel_map" in name
                    or "occupancy_map/voxel_map" in name
                    or "dynamic_map/voxel_map" in name):
                candidates.append(name)
        if not candidates:
            return ""
        preferred = "/dynamic_map/inflated_voxel_map" if self.map_prefer_dynamic else "/occupancy_map/inflated_voxel_map"
        if preferred in candidates:
            return preferred
        fallback = "/occupancy_map/inflated_voxel_map"
        if fallback in candidates:
            return fallback
        return candidates[0]

    def _retry_lidar_sub(self, _event):
        if self.lidar_sub is not None:
            return
        self.lidar_topic = self._detect_lidar_topic()
        if not self.lidar_topic:
            return
        self.lidar_sub = rospy.Subscriber(self.lidar_topic, PointCloud2, self._lidar_cb, queue_size=1)
        rospy.loginfo("LiDAR topic detected: %s", self.lidar_topic)
        if self.lidar_retry_timer is not None:
            self.lidar_retry_timer.shutdown()
            self.lidar_retry_timer = None

    def _retry_map_sub(self, _event):
        if self.map_sub is not None:
            return
        self.map_cloud_topic = self._detect_map_topic()
        if not self.map_cloud_topic:
            return
        self.map_sub = rospy.Subscriber(self.map_cloud_topic, PointCloud2, self._map_cloud_cb, queue_size=1)
        rospy.loginfo("Map cloud topic detected: %s", self.map_cloud_topic)
        if self.map_retry_timer is not None:
            self.map_retry_timer.shutdown()
            self.map_retry_timer = None

    def _connect_map_collision_service(self):
        now = rospy.Time.now().to_sec()
        if (now - self._last_map_collision_connect_try) < 2.0:
            return
        self._last_map_collision_connect_try = now

        if self.map_collision_client is not None:
            return
        try:
            from map_manager.srv import CheckPosCollision, CheckPosCollisionRequest
            self._map_collision_srv_cls = CheckPosCollision
            self._map_collision_req_cls = CheckPosCollisionRequest
        except Exception as e:
            rospy.logwarn_throttle(5.0, "map_manager CheckPosCollision type unavailable: %s", e)
            return

        service_candidates = [self.map_collision_service]
        dynamic_service = "/dynamic_map/check_pos_collision"
        occupancy_service = "/occupancy_map/check_pos_collision"
        if self.map_prefer_dynamic:
            service_candidates.extend([dynamic_service, occupancy_service])
        else:
            service_candidates.extend([occupancy_service, dynamic_service])

        used = set()
        for srv_name in service_candidates:
            if srv_name in used:
                continue
            used.add(srv_name)
            try:
                rospy.wait_for_service(srv_name, timeout=0.7)
                self.map_collision_client = rospy.ServiceProxy(
                    srv_name, self._map_collision_srv_cls, persistent=True)
                self.map_collision_service = srv_name
                rospy.loginfo("Map collision service connected: %s", srv_name)
                return
            except Exception:
                continue

        self.map_collision_client = None
        rospy.logwarn_throttle(5.0, "Map collision service not ready (tried %s)", list(used))

    def _is_map_occupied(self, point):
        if not self.map_use_collision_service:
            return False
        if self.map_collision_client is None:
            self._connect_map_collision_service()
            return False
        try:
            req = self._map_collision_req_cls()
            req.x = float(point[0])
            req.y = float(point[1])
            req.z = float(point[2])
            req.inflated = bool(self.map_collision_inflated)
            resp = self.map_collision_client(req)
            return bool(resp.occupied)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Map collision check call failed: %s", e)
            self.map_collision_client = None
            return False

    def _enforce_map_collision_free(self, ref, traj):
        if ref is None or traj is None or traj.shape[0] == 0:
            return traj
        if not self.map_use_collision_service:
            return traj
        out = traj.copy()
        blocked_count = 0
        fixed_count = 0
        step = max(0.2, self.map_collision_step)
        lateral_limit = self.avoid_lateral_max
        if self.lateral_window > 0.0:
            lateral_limit = min(lateral_limit, self.lateral_window)
        lateral_limit = max(step, lateral_limit)

        for i in range(1, out.shape[0]):
            p = out[i].copy()
            if not self._is_map_occupied(p):
                continue
            blocked_count += 1

            t_hat = self._path_tangent_2d(ref, i)
            n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
            side = self.avoidance_side
            if side is None:
                rel = out[i][:2] - ref[i][:2]
                side = np.sign(np.dot(rel, n_hat))
                if side == 0.0:
                    side = 1.0

            found = False
            k_max = int(lateral_limit / step)
            for k in range(1, k_max + 1):
                off = float(k) * step
                for s in (side, -side):
                    cand = ref[i].copy()
                    cand[0] = ref[i, 0] + s * off * n_hat[0]
                    cand[1] = ref[i, 1] + s * off * n_hat[1]
                    cand[2] = out[i, 2]
                    if not self._is_map_occupied(cand):
                        out[i] = cand
                        found = True
                        fixed_count += 1
                        break
                if found:
                    break

            if not found:
                out[i] = out[i - 1].copy()

        # If almost all forward samples are reported occupied and we cannot fix most of them,
        # map can be over-conservative (e.g. inflated terrain/unknown). Fall back to raw ref.
        n_eval = max(1, out.shape[0] - 1)
        if blocked_count > int(0.8 * n_eval) and fixed_count < int(0.2 * blocked_count):
            rospy.logwarn_throttle(
                2.0,
                "map collision filter too restrictive (blocked=%d fixed=%d), fallback to reference",
                blocked_count,
                fixed_count,
            )
            return traj.copy()
        return out

    def _transform_points(self, points, target_frame, source_frame, stamp):
        target_frame = self._normalize_frame_id(target_frame)
        source_frame = self._normalize_frame_id(source_frame)
        if target_frame == source_frame:
            return points
        if self.tf_buffer is None:
            return []
        lookup_time = stamp
        if isinstance(stamp, rospy.Time):
            if stamp.to_sec() <= 0.0:
                lookup_time = rospy.Time(0)
        else:
            lookup_time = rospy.Time(0)

        trans = None
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame, source_frame, lookup_time, rospy.Duration(0.2))
        except Exception:
            trans = None

        if trans is None:
            source_alt = self._simplify_frame_id(source_frame)
            if source_alt != source_frame:
                try:
                    trans = self.tf_buffer.lookup_transform(
                        target_frame, source_alt, lookup_time, rospy.Duration(0.2))
                    source_frame = source_alt
                except Exception:
                    trans = None

        if trans is None:
            # Common Gazebo setup: map/world frames are identical but no TF is published.
            if target_frame in ("map", "world") and source_frame in ("map", "world"):
                return points
            # Fallback: use odometry pose to map base_link points to world
            if target_frame == self._normalize_frame_id(self.inertial_frame_id):
                base_frame = self._normalize_frame_id(f"{self.uuv_name}/base_link")
                base_points = None
                if source_frame == base_frame:
                    base_points = points
                elif self.tf_buffer is not None:
                    try:
                        trans_bl = self.tf_buffer.lookup_transform(
                            base_frame, source_frame, lookup_time, rospy.Duration(0.2))
                        q_bl = trans_bl.transform.rotation
                        t_bl = trans_bl.transform.translation
                        rot_bl = quaternion_matrix([q_bl.x, q_bl.y, q_bl.z, q_bl.w])[0:3, 0:3]
                        trans_bl_vec = np.array([t_bl.x, t_bl.y, t_bl.z], dtype=float)
                        base_points = []
                        for p in points:
                            base_points.append(rot_bl.dot(p) + trans_bl_vec)
                    except Exception:
                        base_points = None
                if base_points is not None and self.robot_pos is not None and self.robot_quat is not None:
                    return self._apply_pose_to_points(base_points, self.robot_pos, self.robot_quat)
            rospy.logwarn_throttle(5.0, "TF lookup failed: %s -> %s", source_frame, target_frame)
            return []
        q = trans.transform.rotation
        t = trans.transform.translation
        rot = quaternion_matrix([q.x, q.y, q.z, q.w])[0:3, 0:3]
        trans_vec = np.array([t.x, t.y, t.z], dtype=float)
        out = []
        for p in points:
            out.append(rot.dot(p) + trans_vec)
        return out

    @staticmethod
    def _normalize_frame_id(frame_id):
        if frame_id is None:
            return ""
        return frame_id.lstrip("/")

    def _simplify_frame_id(self, frame_id):
        if not frame_id:
            return frame_id
        prefix = f"{self.uuv_name}/{self.uuv_name}/"
        if frame_id.startswith(prefix):
            return f"{self.uuv_name}/" + frame_id[len(prefix):]
        return frame_id

    def _voxel_downsample(self, points, voxel):
        if voxel <= 0.0:
            return points
        grid = {}
        for p in points:
            key = (int(math.floor(p[0] / voxel)),
                   int(math.floor(p[1] / voxel)),
                   int(math.floor(p[2] / voxel)))
            if key in grid:
                grid[key][0] += p
                grid[key][1] += 1
            else:
                grid[key] = [p.copy(), 1]
        out = []
        for s, n in grid.values():
            out.append(s / float(n))
        return out

    def _cluster_points(self, points):
        if not points:
            return []
        cell = max(1e-3, self.cluster_dist)
        grid = {}
        coords = []
        for i, p in enumerate(points):
            key = (int(math.floor(p[0] / cell)),
                   int(math.floor(p[1] / cell)),
                   int(math.floor(p[2] / cell)))
            coords.append(key)
            grid.setdefault(key, []).append(i)

        visited = [False] * len(points)
        clusters = []
        neighbor_offsets = [(dx, dy, dz)
                            for dx in (-1, 0, 1)
                            for dy in (-1, 0, 1)
                            for dz in (-1, 0, 1)]

        for i in range(len(points)):
            if visited[i]:
                continue
            queue = [i]
            visited[i] = True
            cluster = [i]
            while queue:
                idx = queue.pop()
                key = coords[idx]
                for dx, dy, dz in neighbor_offsets:
                    nkey = (key[0] + dx, key[1] + dy, key[2] + dz)
                    for j in grid.get(nkey, []):
                        if visited[j]:
                            continue
                        if np.linalg.norm(points[j] - points[idx]) <= self.cluster_dist:
                            visited[j] = True
                            queue.append(j)
                            cluster.append(j)
            if len(cluster) >= self.cluster_min_points:
                clusters.append(cluster)
        return clusters

    def _update_tracks(self, clusters, points, stamp, robot_pos=None):
        if robot_pos is None:
            robot_pos = np.zeros(3, dtype=float)

        detections = []
        for cl in clusters:
            pts = np.array([points[i] for i in cl], dtype=float)
            if pts.size == 0:
                continue
            centroid = pts.mean(axis=0)
            # Use horizontal footprint radius for marine obstacles; tall objects
            # (e.g., long cylinders) should not be rejected due to Z extent.
            radius = (
                float(np.max(np.linalg.norm(pts[:, :2] - centroid[:2], axis=1)))
                if pts.shape[0] > 1 else self.obstacle_radius
            )
            if self.reject_large_clusters and radius > self.obstacle_radius_max:
                continue
            detections.append((centroid, radius))

        if not detections:
            if self.obstacle_last_seen is not None and self.obstacle_track is not None:
                if (stamp - self.obstacle_last_seen) > self.track_timeout:
                    self.obstacle_track = None
                    self.obstacle_last_seen = None
            return

        def det_key(det):
            pos = det[0]
            diff = pos - robot_pos
            if self.ignore_z_in_dynamic_cost:
                diff[2] = 0.0
            return np.linalg.norm(diff)

        pos, radius = min(detections, key=det_key)
        radius = float(np.clip(radius if self.use_cluster_radius else self.obstacle_radius,
                                self.obstacle_radius_min, self.obstacle_radius_max))

        if self.obstacle_track is None:
            vel = np.zeros(3, dtype=float)
            first_stamp = stamp
        else:
            prev = self.obstacle_track
            dist = np.linalg.norm((pos - prev["pos"])[:2])
            if dist > self.track_association_dist:
                vel = np.zeros(3, dtype=float)
                first_stamp = stamp
            else:
                dt = max(1e-3, stamp - prev["stamp"])
                vel_meas = (pos - prev["pos"]) / dt
                vel = self.track_vel_alpha * vel_meas + (1.0 - self.track_vel_alpha) * prev["vel"]
                first_stamp = prev.get("first_stamp", stamp)

        self.obstacle_track = {
            "pos": pos,
            "vel": vel,
            "radius": radius,
            "stamp": stamp,
            "first_stamp": first_stamp,
        }
        self.obstacle_last_seen = stamp

    def _get_active_obstacle(self, robot_pos, now):
        tr = self.obstacle_track
        if tr is None:
            return None
        if (now - tr["stamp"]) > self.track_timeout:
            return None
        pos = tr["pos"]
        vel = tr["vel"]
        radius = tr["radius"]
        speed = float(np.linalg.norm(vel))
        age = now - tr.get("first_stamp", tr["stamp"])
        if not self.use_static_obstacles:
            if speed < self.min_obstacle_speed and age >= self.min_track_age:
                if robot_pos is None:
                    return None
                diff = pos - robot_pos
                if self.ignore_z_in_dynamic_cost:
                    diff[2] = 0.0
                dist = np.linalg.norm(diff)
                if dist > self.static_obstacle_dist:
                    return None
        return {"pos": pos, "vel": vel, "radius": radius, "stamp": tr["stamp"]}

    def _debug_obstacle_state(self, now, robot_pos):
        tr = self.obstacle_track
        if tr is None:
            rospy.loginfo_throttle(1.0, "obstacle: none")
            return
        pos = tr["pos"]
        vel = tr["vel"]
        speed = float(np.linalg.norm(vel))
        age = now - tr.get("first_stamp", tr["stamp"])
        dynamic = speed >= self.min_obstacle_speed
        rospy.loginfo_throttle(
            1.0,
            "obstacle pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) speed=%.2f age=%.2f dynamic=%s",
            pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            speed, age, "true" if dynamic else "false",
        )

    def _lidar_cb(self, msg):
        # Extract and filter points in sensor frame
        pts = []
        count = 0
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            r = math.sqrt(x * x + y * y + z * z)
            if r < self.lidar_min_range or r > self.lidar_max_range:
                continue
            if z < self.lidar_min_z or z > self.lidar_max_z:
                continue
            pts.append(np.array([x, y, z], dtype=float))
            count += 1
            if self.lidar_max_points > 0 and count >= self.lidar_max_points:
                break

        if not pts:
            return

        pts = self._voxel_downsample(pts, self.lidar_voxel)
        pts_tf = self._transform_points(pts, self.inertial_frame_id, msg.header.frame_id, msg.header.stamp)
        used_frame = self.inertial_frame_id
        if not pts_tf and self.inertial_frame_id == "world_ned":
            # Fallback to world (ENU) if world_ned TF is unavailable
            pts_tf = self._transform_points(pts, "world", msg.header.frame_id, msg.header.stamp)
            used_frame = "world"
        if not pts_tf:
            return

        if used_frame == "world" and self.inertial_frame_id == "world_ned":
            pts_tf = [enu_to_ned_vec(p[0], p[1], p[2]) for p in pts_tf]

        if self.obstacle_z_window > 0.0:
            with self.lock:
                robot_pos = None if self.robot_pos is None else self.robot_pos.copy()
            if robot_pos is not None:
                z0 = robot_pos[2]
                pts_tf = [p for p in pts_tf if abs(p[2] - z0) <= self.obstacle_z_window]

        if not pts_tf:
            return

        clusters = self._cluster_points(pts_tf)
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.Time.now().to_sec()
        with self.lock:
            robot_pos = None if self.robot_pos is None else self.robot_pos.copy()
        self._update_tracks(clusters, pts_tf, stamp, robot_pos=robot_pos)

        if self.debug_lidar:
            rospy.loginfo_throttle(
                1.0,
                "LiDAR points=%d clusters=%d obstacle=%s",
                len(pts_tf), len(clusters),
                "yes" if self.obstacle_track is not None else "no")

    def _map_cloud_cb(self, msg):
        pts = []
        count = 0
        # Map clouds may already be in a world/map frame. In that case,
        # origin-range filtering would incorrectly drop far-away points.
        use_origin_range = not (
            msg.header.frame_id in ("map", "world")
            and self.inertial_frame_id in ("map", "world")
        )
        with self.lock:
            robot_pos = None if self.robot_pos is None else self.robot_pos.copy()
        local_r = None
        if robot_pos is not None:
            # Keep a broader local capture radius for dynamic clouds so lateral
            # movers are not dropped before clustering.
            local_r = max(self.forward_window, self.lateral_window, self.avoid_longitudinal, 10.0)
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            if use_origin_range:
                r = math.sqrt(x * x + y * y + z * z)
                if r < self.lidar_min_range or r > self.lidar_max_range:
                    continue
            if robot_pos is not None and msg.header.frame_id in ("map", "world"):
                dxy = math.hypot(x - robot_pos[0], y - robot_pos[1])
                if local_r is not None and dxy > local_r:
                    continue
                if self.obstacle_z_window > 0.0 and abs(z - robot_pos[2]) > self.obstacle_z_window:
                    continue
            pts.append(np.array([x, y, z], dtype=float))
            count += 1
            if self.lidar_max_points > 0 and count >= self.lidar_max_points:
                break

        if not pts:
            return

        pts = self._voxel_downsample(pts, self.lidar_voxel)
        pts_tf = self._transform_points(pts, self.inertial_frame_id, msg.header.frame_id, msg.header.stamp)
        used_frame = self.inertial_frame_id
        if not pts_tf and self.inertial_frame_id == "world_ned":
            pts_tf = self._transform_points(pts, "world", msg.header.frame_id, msg.header.stamp)
            used_frame = "world"
        if not pts_tf:
            return

        if used_frame == "world" and self.inertial_frame_id == "world_ned":
            pts_tf = [enu_to_ned_vec(p[0], p[1], p[2]) for p in pts_tf]

        if robot_pos is not None and msg.header.frame_id not in ("map", "world"):
            z0 = robot_pos[2]
            local_r = max(self.forward_window, self.lateral_window, self.avoid_longitudinal, 10.0)
            filtered = []
            for p in pts_tf:
                dxy = np.linalg.norm((p - robot_pos)[:2])
                if dxy > local_r:
                    continue
                if self.obstacle_z_window > 0.0 and abs(p[2] - z0) > self.obstacle_z_window:
                    continue
                filtered.append(p)
            pts_tf = filtered

        if not pts_tf:
            return

        clusters = self._cluster_points(pts_tf)
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.Time.now().to_sec()

        obstacles = []
        for cl in clusters:
            pts_arr = np.array([pts_tf[i] for i in cl], dtype=float)
            if pts_arr.size == 0:
                continue
            centroid = pts_arr.mean(axis=0)
            # Use XY radius to avoid rejecting vertically long objects.
            radius = (
                float(np.max(np.linalg.norm(pts_arr[:, :2] - centroid[:2], axis=1)))
                if pts_arr.shape[0] > 1 else self.obstacle_radius
            )
            if self.reject_large_clusters and radius > self.obstacle_radius_max:
                continue
            radius = float(np.clip(radius, self.obstacle_radius_min, self.obstacle_radius_max))
            obstacles.append(
                {
                    "pos": centroid,
                    "vel": np.zeros(3, dtype=float),
                    "radius": radius,
                    "stamp": stamp,
                }
            )

        if robot_pos is not None:
            obstacles.sort(key=lambda o: np.linalg.norm((o["pos"] - robot_pos)[:2]))
        if self.map_obstacles_max > 0:
            obstacles = obstacles[: self.map_obstacles_max]

        self._update_map_primary_track(obstacles, stamp, robot_pos)

        with self.lock:
            self.map_obstacles = obstacles
            self.map_last_seen = stamp

        if self.debug_lidar:
            rospy.loginfo_throttle(
                1.0,
                "Map cloud points=%d clusters=%d kept=%d",
                len(pts_tf), len(clusters), len(obstacles))

    def _parse_detector_velocity(self, text):
        if not text:
            return 0.0, 0.0
        m = re.search(
            r"Vx\s*=?\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,?\s*Vy\s*=?\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)",
            text,
        )
        if m is not None:
            return float(m.group(1)), float(m.group(2))
        nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", text)
        if len(nums) >= 2:
            return float(nums[0]), float(nums[1])
        return 0.0, 0.0

    def _detector_vel_cb(self, msg):
        tracks = []
        now = rospy.Time.now().to_sec()
        stamp = now
        for mk in msg.markers:
            if mk.header.stamp.to_sec() > 0.0:
                stamp = mk.header.stamp.to_sec()
            vx, vy = self._parse_detector_velocity(mk.text)
            pos = np.array(
                [mk.pose.position.x, mk.pose.position.y, mk.pose.position.z], dtype=float
            )
            vel = np.array([vx, vy, 0.0], dtype=float)
            src_frame = self._normalize_frame_id(mk.header.frame_id)
            if src_frame in ("map", "world") and self.inertial_frame_id == "world_ned":
                pos = enu_to_ned_vec(pos[0], pos[1], pos[2])
                vel = enu_to_ned_vec(vel[0], vel[1], vel[2])
            track = {
                "pos": pos,
                "vel": vel,
                "radius": float(self.detector_obstacle_radius),
                "stamp": stamp,
                "first_stamp": stamp,
            }
            tracks.append(track)

        with self.lock:
            self.detector_tracks = tracks
            self.detector_last_seen = stamp if len(tracks) > 0 else now

    def _get_active_detector_obstacle(self, robot_pos, ref, now):
        if not self.detector_use_velocity_track:
            return None
        with self.lock:
            tracks = [
                {
                    "pos": t["pos"].copy(),
                    "vel": t["vel"].copy(),
                    "radius": float(t["radius"]),
                    "stamp": float(t["stamp"]),
                    "first_stamp": float(t.get("first_stamp", t["stamp"])),
                }
                for t in self.detector_tracks
            ]
            last_seen = self.detector_last_seen
        if len(tracks) == 0:
            return None
        if last_seen is None or (now - last_seen) > self.detector_track_timeout:
            return None
        if ref is None or len(ref) < 2:
            return None

        idx = self._nearest_index(ref, robot_pos, use_2d=True)
        t_hat = self._path_tangent_2d(ref, idx)
        n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
        candidates = []
        for tr in tracks:
            speed = float(np.linalg.norm(tr["vel"][:2]))
            age = now - tr.get("first_stamp", tr["stamp"])
            if (
                not self.use_static_obstacles
                and speed < self.min_obstacle_speed
                and age >= self.min_track_age
            ):
                diff = tr["pos"] - robot_pos
                if self.ignore_z_in_dynamic_cost:
                    diff[2] = 0.0
                if np.linalg.norm(diff) > self.static_obstacle_dist:
                    continue

            rel = tr["pos"][:2] - robot_pos[:2]
            forward = float(np.dot(rel, t_hat))
            lateral = float(np.dot(rel, n_hat))
            if forward < -self.behind_ignore:
                continue
            if self.forward_window > 0.0 and forward > self.forward_window:
                continue
            if self.lateral_window > 0.0 and abs(lateral) > (self.lateral_window + tr["radius"]):
                continue
            clearance_margin = np.linalg.norm(rel) - tr["radius"] - self.avoidance_clearance
            candidates.append((clearance_margin, abs(forward), tr))

        if len(candidates) == 0:
            return None
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][2]

    def _update_map_primary_track(self, obstacles, stamp, robot_pos):
        with self.lock:
            prev = None if self.map_primary_track is None else dict(self.map_primary_track)

        if not obstacles:
            if prev is not None and (stamp - prev["stamp"]) > self.track_timeout:
                with self.lock:
                    self.map_primary_track = None
            return

        chosen = None
        best_dist = float("inf")
        if prev is not None:
            dt_pred = max(1e-3, stamp - prev["stamp"])
            pred = prev["pos"] + prev["vel"] * dt_pred
            for o in obstacles:
                d = float(np.linalg.norm((o["pos"] - pred)[:2]))
                if d < best_dist:
                    best_dist = d
                    chosen = o

        if chosen is None:
            if robot_pos is not None:
                chosen = min(
                    obstacles,
                    key=lambda o: float(np.linalg.norm((o["pos"] - robot_pos)[:2])),
                )
            else:
                chosen = obstacles[0]

        pos = chosen["pos"].copy()
        radius = float(chosen["radius"])
        if prev is None:
            vel = np.zeros(3, dtype=float)
            first_stamp = stamp
        else:
            dt = max(1e-3, stamp - prev["stamp"])
            if best_dist > self.map_track_reset_dist:
                # Large jump means obstacle identity likely changed; reset speed.
                vel = np.zeros(3, dtype=float)
                first_stamp = stamp
            else:
                vel_meas = (pos - prev["pos"]) / dt
                vel = self.track_vel_alpha * vel_meas + (1.0 - self.track_vel_alpha) * prev["vel"]
                first_stamp = prev.get("first_stamp", stamp)

        track = {
            "pos": pos,
            "vel": vel,
            "radius": radius,
            "stamp": stamp,
            "first_stamp": first_stamp,
        }
        with self.lock:
            self.map_primary_track = track
        if self.debug_lidar:
            rospy.loginfo_throttle(
                1.0,
                "map primary track speed=%.2f d=%.2f",
                float(np.linalg.norm(track["vel"][:2])),
                float(best_dist if prev is not None else 0.0),
            )

    def _get_active_map_obstacle(self, robot_pos, ref, now):
        with self.lock:
            obs = list(self.map_obstacles)
            last_seen = self.map_last_seen
            primary_track = None if self.map_primary_track is None else {
                "pos": self.map_primary_track["pos"].copy(),
                "vel": self.map_primary_track["vel"].copy(),
                "radius": float(self.map_primary_track["radius"]),
                "stamp": float(self.map_primary_track["stamp"]),
                "first_stamp": float(self.map_primary_track.get("first_stamp", self.map_primary_track["stamp"])),
            }
        if last_seen is None or (now - last_seen) > self.map_timeout:
            return None
        if ref is None or len(ref) < 2:
            return None
        if not obs:
            if primary_track is not None and (now - primary_track["stamp"]) <= self.map_timeout:
                return {
                    "pos": primary_track["pos"].copy(),
                    "vel": primary_track["vel"].copy(),
                    "radius": float(primary_track["radius"]),
                    "stamp": float(primary_track["stamp"]),
                    "first_stamp": float(primary_track["first_stamp"]),
                }
            return None

        idx = self._nearest_index(ref, robot_pos, use_2d=True)
        t_hat = self._path_tangent_2d(ref, idx)
        n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
        candidates = []
        for o in obs:
            rel = o["pos"][:2] - robot_pos[:2]
            forward = float(np.dot(rel, t_hat))
            lateral = float(np.dot(rel, n_hat))
            if forward < -self.behind_ignore:
                continue
            if self.forward_window > 0.0 and forward > self.forward_window:
                continue
            if self.lateral_window > 0.0 and abs(lateral) > (self.lateral_window + o["radius"]):
                continue
            d = float(np.linalg.norm(rel)) - o["radius"]
            # Rank by risk: smaller clearance margin is more dangerous.
            # Secondary keys keep behavior deterministic and forward-looking.
            clearance_margin = d - self.avoidance_clearance
            candidates.append((clearance_margin, abs(forward), d, o))

        if not candidates:
            # Corridor gating can be too strict near sharp heading changes.
            # Fallback to nearest obstacle in XY so dynamic obstacles are still
            # considered by the avoider.
            nearest = min(
                obs,
                key=lambda o: float(np.linalg.norm((o["pos"] - robot_pos)[:2]))
            )
            rospy.logwarn_throttle(
                2.0,
                "map obstacle fallback: no corridor candidate, using nearest obstacle",
            )
            selected = nearest
        else:
            candidates.sort(key=lambda item: (item[0], item[1], item[2]))
            selected = candidates[0][3]

        out = {
            "pos": selected["pos"].copy(),
            "vel": np.zeros(3, dtype=float),
            "radius": float(selected["radius"]),
            "stamp": float(selected["stamp"]),
        }
        if primary_track is not None and (now - primary_track["stamp"]) <= self.map_timeout:
            out["vel"] = primary_track["vel"].copy()
            out["first_stamp"] = float(primary_track["first_stamp"])
        return out

    def _nearest_index(self, points, pos, use_2d=False):
        if points is None or len(points) == 0:
            return 0
        if use_2d:
            dists = [np.linalg.norm(p[:2] - pos[:2]) for p in points]
        else:
            dists = [np.linalg.norm(p - pos) for p in points]
        return int(np.argmin(dists))

    def _build_reference_traj(self, num_points):
        if not self.global_path_dense or self.robot_pos is None:
            return None
        idx = self._nearest_forward_index(self.global_path_dense, self.robot_pos, self.global_progress_idx)
        self.global_progress_idx = idx
        ref = []
        ref.append(self.robot_pos.copy())

        # Approach the nearest global point with bounded step size to avoid large jumps
        target = self.global_path_dense[idx].copy()
        step = max(1e-3, self.path_resolution)
        dist = np.linalg.norm(target - self.robot_pos)
        if dist > 1e-6:
            steps = int(math.ceil(dist / step))
            for s in range(1, steps + 1):
                p = self.robot_pos + (float(s) / float(steps)) * (target - self.robot_pos)
                ref.append(p)
                if len(ref) >= num_points:
                    return np.array(ref[:num_points], dtype=float)

        # Then continue along the global path
        for j in range(idx + 1, len(self.global_path_dense)):
            ref.append(self.global_path_dense[j].copy())
            if len(ref) >= num_points:
                break

        # Pad with the last point if still short
        while len(ref) < num_points:
            ref.append(ref[-1].copy())

        return np.array(ref, dtype=float)

    def _dynamic_cost_grad(self, p, t, obstacles):
        grad = np.zeros(3, dtype=float)
        if not obstacles:
            return grad

        a = 3.0 * self.dist_thresh_dynamic
        b = -3.0 * (self.dist_thresh_dynamic ** 2)
        if self.pred_horizon > 0.0 and t > self.pred_horizon:
            t = self.pred_horizon
        for (pos, vel, radius) in obstacles:
            obs_pos = pos + vel * t
            diff = p - obs_pos
            if self.ignore_z_in_dynamic_cost:
                diff[2] = 0.0
            norm = np.linalg.norm(diff)
            if norm < 1e-6:
                continue
            dist = norm - radius
            dist_err = self.dist_thresh_dynamic - dist
            if dist_err <= 0:
                continue
            unit = diff / norm
            if dist_err <= self.dist_thresh_dynamic:
                grad += -3.0 * (dist_err ** 2) * unit
            else:
                grad += -(2.0 * a * dist_err + b) * unit
        return grad

    def _path_tangent(self, ref, i):
        if ref is None or len(ref) < 2:
            return np.array([1.0, 0.0, 0.0], dtype=float)
        if i < len(ref) - 1:
            d = ref[i + 1] - ref[i]
        else:
            d = ref[i] - ref[i - 1]
        d[2] = 0.0
        n = np.linalg.norm(d)
        if n < 1e-6:
            return np.array([1.0, 0.0, 0.0], dtype=float)
        return d / n

    def _path_tangent_2d(self, ref, i):
        if ref is None or len(ref) < 2:
            return np.array([1.0, 0.0], dtype=float)
        if i < len(ref) - 1:
            d = ref[i + 1][:2] - ref[i][:2]
        else:
            d = ref[i][:2] - ref[i - 1][:2]
        n = np.linalg.norm(d)
        if n < 1e-6:
            return np.array([1.0, 0.0], dtype=float)
        return d / n

    def _update_avoidance_side(self, ref, obstacle, robot_pos):
        if ref is None or len(ref) < 2 or obstacle is None or robot_pos is None:
            self.avoidance_side = None
            return
        idx = self._nearest_index(ref, robot_pos, use_2d=True)
        ref_p = ref[idx]
        rel = obstacle["pos"][:2] - ref_p[:2]
        dist = float(np.linalg.norm(rel))
        if self.avoidance_side is None:
            if dist <= self.side_lock_dist:
                t_hat = self._path_tangent_2d(ref, idx)
                n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
                sign = np.sign(np.dot(rel, n_hat))
                if sign == 0.0:
                    sign = 1.0
                self.avoidance_side = -sign
        else:
            if dist >= self.side_release_dist:
                self.avoidance_side = None

    def _predict_collision_2d(self, robot_pos, robot_vel, obstacle, horizon):
        if obstacle is None:
            return None, None
        r = obstacle["pos"][:2] - robot_pos[:2]
        v_rel = robot_vel[:2] - obstacle["vel"][:2]
        v2 = float(np.dot(v_rel, v_rel))
        if v2 < 1e-6:
            t = 0.0
        else:
            t = -float(np.dot(r, v_rel)) / v2
            t = max(0.0, min(horizon, t))
        closest = r + v_rel * t
        d_min = float(np.linalg.norm(closest))
        return d_min, t

    def _smooth_offsets(self, offsets):
        window = max(1, int(self.offset_smooth_window))
        if window <= 1:
            return offsets
        half = window // 2
        out = []
        n = len(offsets)
        for i in range(n):
            a = max(0, i - half)
            b = min(n, i + half + 1)
            out.append(float(sum(offsets[a:b])) / float(b - a))
        return out

    def _apply_speed_limit(self, traj):
        if not self.limit_speed or self.max_forward_speed <= 0.0:
            return traj
        max_step = self.max_forward_speed * self.dt
        n = traj.shape[0]
        for i in range(1, n):
            delta = traj[i] - traj[i - 1]
            delta[2] = 0.0
            d = np.linalg.norm(delta[:2])
            if d > max_step and d > 1e-6:
                step = (delta[:2] / d) * max_step
                traj[i, 0] = traj[i - 1, 0] + step[0]
                traj[i, 1] = traj[i - 1, 1] + step[1]
        return traj

    def _reuse_prev_traj(self, prev_traj, robot_pos, n_pts):
        if prev_traj is None or prev_traj.shape[0] < 2:
            return None
        idx = self._nearest_index(prev_traj, robot_pos, use_2d=True)
        sub = prev_traj[idx:]
        if sub.shape[0] >= n_pts:
            traj = sub[:n_pts].copy()
        else:
            pad = np.repeat(sub[-1:].copy(), n_pts - sub.shape[0], axis=0)
            traj = np.vstack([sub, pad])
        traj[0, 0:2] = robot_pos[:2]
        return traj

    def _build_hold_traj(self, hold_pos, n_pts):
        p = np.array(hold_pos, dtype=float)
        traj = np.repeat(p.reshape(1, 3), n_pts, axis=0)
        if self.z_hold_mode == "current" and self.z_hold is not None:
            traj[:, 2] = self.z_hold
        return traj

    def _can_start_dynamic_arc(self, ref, robot_pos, robot_vel, obstacle, now):
        if not self.dynamic_arc_enable:
            return False
        if obstacle is None or ref is None or len(ref) < 2 or robot_pos is None:
            return False
        if now < self.global_rejoin_hold_until or now < self.dynamic_arc_block_until:
            return False

        obs_vel = obstacle.get("vel", np.zeros(3, dtype=float))
        speed = float(np.linalg.norm(obs_vel[:2]))
        if speed < self.dynamic_arc_speed_threshold:
            return False

        idx = self._nearest_index(ref, robot_pos, use_2d=True)
        t_hat = self._path_tangent_2d(ref, idx)
        rel = obstacle["pos"][:2] - robot_pos[:2]
        forward = float(np.dot(rel, t_hat))
        if forward < self.dynamic_arc_trigger_min_forward:
            return False

        horizon = max(self.horizon_time, self.pred_horizon) if self.pred_horizon > 0.0 else self.horizon_time
        d_min, _ = self._predict_collision_2d(robot_pos, robot_vel, obstacle, horizon)
        r_safe = obstacle["radius"] + self.avoidance_clearance
        trigger_dist = max(r_safe, self.dist_thresh_dynamic)
        return d_min is not None and d_min < trigger_dist

    def _start_dynamic_arc_observe(self, ref, robot_pos, obstacle, now):
        if ref is None or len(ref) < 2 or robot_pos is None or obstacle is None:
            return False
        idx = self._nearest_index(ref, robot_pos, use_2d=True)
        t_hat = self._path_tangent_2d(ref, idx)
        n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
        anchor = ref[idx].copy()
        rel = obstacle["pos"][:2] - anchor[:2]
        lat = float(np.dot(rel, n_hat))
        fwd = float(np.dot(rel, t_hat))
        self.dynamic_arc_state = {
            "mode": "observe",
            "start_time": now,
            "observe_until": now + max(0.1, self.dynamic_arc_observe_time),
            "anchor": anchor.copy(),
            "t_hat": t_hat.copy(),
            "n_hat": n_hat.copy(),
            "lat_min": lat,
            "lat_max": lat,
            "forward_max": fwd,
            "radius_max": float(obstacle["radius"]),
            "hold_pos": robot_pos.copy(),
        }
        self.local_override_active = True
        self.avoidance_active = True
        if self.z_hold_mode == "current":
            self.z_hold = robot_pos[2]
        rospy.loginfo(
            "local_avoidance_planner: dynamic arc observe started (speed=%.2f, observe=%.2fs)",
            float(np.linalg.norm(obstacle.get("vel", np.zeros(3, dtype=float))[:2])),
            self.dynamic_arc_observe_time,
        )
        return True

    def _update_dynamic_arc_observe(self, obstacle):
        st = self.dynamic_arc_state
        if st is None or st.get("mode") != "observe":
            return
        if obstacle is None:
            return
        rel = obstacle["pos"][:2] - st["anchor"][:2]
        lat = float(np.dot(rel, st["n_hat"]))
        fwd = float(np.dot(rel, st["t_hat"]))
        st["lat_min"] = min(st["lat_min"], lat)
        st["lat_max"] = max(st["lat_max"], lat)
        st["forward_max"] = max(st["forward_max"], fwd)
        st["radius_max"] = max(st["radius_max"], float(obstacle["radius"]))

    def _pick_arc_rejoin_point(self, robot_pos, t_hat, min_forward):
        if not self.global_path_dense:
            return None
        idx0 = self._nearest_forward_index(self.global_path_dense, robot_pos, self.global_progress_idx)
        for i in range(idx0, len(self.global_path_dense)):
            p = self.global_path_dense[i]
            fwd = float(np.dot((p[:2] - robot_pos[:2]), t_hat))
            if fwd >= min_forward:
                self.global_progress_idx = i
                return p.copy()
        self.global_progress_idx = len(self.global_path_dense) - 1
        return self.global_path_dense[-1].copy()

    def _start_dynamic_arc_execute(self, robot_pos):
        st = self.dynamic_arc_state
        if st is None or st.get("mode") != "observe":
            return False
        t_hat = st["t_hat"]
        n_hat = st["n_hat"]
        lat_min = float(st["lat_min"])
        lat_max = float(st["lat_max"])
        radius_max = float(st["radius_max"])

        # If observation window is short/partial, keep a default moving-range bound.
        lat_min = min(lat_min, -self.dynamic_arc_range_default)
        lat_max = max(lat_max, self.dynamic_arc_range_default)

        clearance = self.avoidance_clearance + self.dynamic_arc_margin
        need_left = max(0.0, lat_max + radius_max + clearance)
        need_right = max(0.0, -lat_min + radius_max + clearance)

        if need_left <= need_right:
            side = 1.0
            offset_mag = need_left
        else:
            side = -1.0
            offset_mag = need_right
        offset_mag = max(self.dynamic_arc_min_offset, offset_mag)

        obs_forward = max(st["forward_max"], self.dynamic_arc_trigger_min_forward)
        min_forward = obs_forward + radius_max + self.dynamic_arc_forward_margin
        rejoin = self._pick_arc_rejoin_point(robot_pos, t_hat, min_forward)
        if rejoin is None:
            return False

        p0 = robot_pos.copy()
        p2 = rejoin.copy()
        if self.z_hold_mode == "current" and self.z_hold is not None:
            p2[2] = self.z_hold

        ctrl_forward = max(4.0, 0.6 * min_forward)
        pm = p0.copy()
        pm[0] = p0[0] + t_hat[0] * ctrl_forward + n_hat[0] * (side * offset_mag)
        pm[1] = p0[1] + t_hat[1] * ctrl_forward + n_hat[1] * (side * offset_mag)
        pm[2] = p0[2]

        sample_n = max(20, int(max(8.0, min_forward) / max(self.path_resolution, 0.2)))
        curve = []
        for k in range(sample_n):
            u = float(k) / float(max(1, sample_n - 1))
            p = ((1.0 - u) ** 2) * p0 + (2.0 * (1.0 - u) * u) * pm + (u ** 2) * p2
            curve.append(p)
        dense = self._densify_path(curve, self.path_resolution)
        if len(dense) < 2:
            return False

        st["mode"] = "execute"
        st["path"] = np.array(dense, dtype=float)
        st["goal"] = st["path"][-1].copy()
        st["side"] = side
        st["offset_mag"] = offset_mag
        st["rejoin_forward"] = min_forward
        rospy.loginfo(
            "local_avoidance_planner: dynamic arc execute started (side=%s, offset=%.2f, forward=%.2f)",
            "left" if side > 0.0 else "right",
            offset_mag,
            min_forward,
        )
        return True

    def _get_dynamic_arc_traj(self, robot_pos, n_pts):
        st = self.dynamic_arc_state
        if st is None or st.get("mode") != "execute":
            return None, True
        path = st.get("path", None)
        goal = st.get("goal", None)
        if path is None or path.shape[0] < 2:
            return None, True

        idx = self._nearest_index(path, robot_pos, use_2d=True)
        done = False
        if idx >= path.shape[0] - 1:
            done = True
        elif goal is not None:
            if np.linalg.norm((robot_pos - goal)[:2]) <= self.dynamic_arc_finish_tol:
                done = True
        if done:
            return None, True

        sub = path[idx:]
        if sub.shape[0] >= n_pts:
            traj = sub[:n_pts].copy()
        else:
            pad = np.repeat(sub[-1:].copy(), n_pts - sub.shape[0], axis=0)
            traj = np.vstack([sub, pad])
        traj[0, 0:2] = robot_pos[:2]
        return traj, False

    def _clear_dynamic_arc(self, now=None, completed=False):
        self.dynamic_arc_state = None
        if completed:
            if now is not None:
                self.dynamic_arc_block_until = now + self.dynamic_arc_retrigger_cooldown
                self.global_rejoin_hold_until = now + self.global_rejoin_hold_time
            rospy.loginfo("local_avoidance_planner: dynamic arc completed, returning to global tracking")

    def _start_force_maneuver(self, ref, robot_pos):
        if ref is None or len(ref) < 2 or robot_pos is None:
            return False
        idx = self._nearest_index(ref, robot_pos, use_2d=True)
        t_hat = self._path_tangent_2d(ref, idx)
        n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
        right_hat = -n_hat

        p0 = robot_pos.copy()
        p1 = p0.copy()
        p1[0] += right_hat[0] * self.force_maneuver_right
        p1[1] += right_hat[1] * self.force_maneuver_right
        p2 = p1.copy()
        p2[0] += t_hat[0] * self.force_maneuver_forward
        p2[1] += t_hat[1] * self.force_maneuver_forward
        p3 = p2.copy()
        p3[0] += n_hat[0] * self.force_maneuver_left
        p3[1] += n_hat[1] * self.force_maneuver_left
        p4 = p3.copy()
        p4[0] += right_hat[0] * self.force_maneuver_rejoin_right
        p4[1] += right_hat[1] * self.force_maneuver_rejoin_right

        dense = self._densify_path([p0, p1, p2, p3, p4], self.path_resolution)
        if len(dense) < 2:
            return False
        self.force_maneuver_path = np.array(dense, dtype=float)
        self.force_maneuver_goal = self.force_maneuver_path[-1].copy()
        self.force_maneuver_active = True
        self.local_override_active = True
        self.avoidance_active = True
        if self.z_hold_mode == "current":
            self.z_hold = robot_pos[2]
        rospy.loginfo(
            "local_avoidance_planner: force maneuver started (right=%.1f, forward=%.1f, left=%.1f, rejoin_right=%.1f)",
            self.force_maneuver_right,
            self.force_maneuver_forward,
            self.force_maneuver_left,
            self.force_maneuver_rejoin_right,
        )
        return True

    def _can_start_force_maneuver(self, ref, robot_pos, obstacle, now):
        if not self.force_maneuver_enable:
            return False
        if obstacle is None or ref is None or len(ref) < 2 or robot_pos is None:
            return False
        if now < self.global_rejoin_hold_until:
            return False
        if now < self.force_maneuver_block_until:
            return False
        idx = self._nearest_index(ref, robot_pos, use_2d=True)
        t_hat = self._path_tangent_2d(ref, idx)
        rel = obstacle["pos"][:2] - robot_pos[:2]
        forward = float(np.dot(rel, t_hat))
        return forward >= self.force_maneuver_trigger_min_forward

    def _get_force_maneuver_traj(self, robot_pos, n_pts):
        if not self.force_maneuver_active or self.force_maneuver_path is None:
            return None, True
        path = self.force_maneuver_path
        idx = self._nearest_index(path, robot_pos, use_2d=True)
        done = False
        if idx >= path.shape[0] - 1:
            done = True
        elif self.force_maneuver_goal is not None:
            if np.linalg.norm((robot_pos - self.force_maneuver_goal)[:2]) <= self.force_maneuver_finish_tol:
                done = True
        if done:
            return None, True

        sub = path[idx:]
        if sub.shape[0] >= n_pts:
            traj = sub[:n_pts].copy()
        else:
            pad = np.repeat(sub[-1:].copy(), n_pts - sub.shape[0], axis=0)
            traj = np.vstack([sub, pad])
        traj[0, 0:2] = robot_pos[:2]
        return traj, False

    def _clear_force_maneuver(self, now=None, completed=False):
        self.force_maneuver_active = False
        self.force_maneuver_path = None
        self.force_maneuver_goal = None
        if completed:
            if now is not None:
                self.force_maneuver_block_until = now + self.force_maneuver_retrigger_cooldown
                self.global_rejoin_hold_until = now + self.global_rejoin_hold_time
            rospy.loginfo("local_avoidance_planner: force maneuver completed, returning to global tracking")

    def _plan_local_traj_2d(self, ref, obstacle, robot_pos):
        traj = ref.copy()
        n = traj.shape[0]
        if obstacle is None or robot_pos is None:
            return traj

        self._update_avoidance_side(ref, obstacle, robot_pos)
        if self.avoidance_side is None:
            return traj

        long_range = self.avoid_longitudinal if self.avoid_longitudinal > 0.0 else self.forward_window
        long_range = max(long_range, 1e-3)
        offsets = [0.0] * n
        for i in range(n):
            t = i * self.dt
            if self.pred_horizon > 0.0:
                t = min(t, self.pred_horizon)
            obs_pos = obstacle["pos"][:2] + obstacle["vel"][:2] * t
            ref_xy = ref[i][:2]
            t_hat = self._path_tangent_2d(ref, i)
            n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
            rel = obs_pos - ref_xy
            forward = float(np.dot(rel, t_hat))
            lateral = float(np.dot(rel, n_hat))
            if forward < -self.behind_ignore or forward > long_range:
                continue
            if self.lateral_window > 0.0 and abs(lateral) > self.lateral_window:
                continue
            needed = (obstacle["radius"] + self.avoidance_clearance) - abs(lateral)
            if needed <= 0.0:
                continue
            weight = 1.0 - min(1.0, max(0.0, forward) / long_range)
            offset = self.avoidance_side * min(self.avoid_lateral_max, needed) * weight
            offsets[i] = offset

        offsets = self._smooth_offsets(offsets)
        for i in range(n):
            t_hat = self._path_tangent_2d(ref, i)
            n_hat = np.array([-t_hat[1], t_hat[0]], dtype=float)
            traj[i, 0] = ref[i, 0] + offsets[i] * n_hat[0]
            traj[i, 1] = ref[i, 1] + offsets[i] * n_hat[1]

        if self.prev_traj is not None and self.prev_traj.shape == traj.shape:
            a = max(0.0, min(1.0, self.smooth_alpha))
            traj = a * self.prev_traj + (1.0 - a) * traj

        if self.perception_source == "map_manager":
            traj = self._enforce_map_collision_free(ref, traj)
        traj = self._apply_speed_limit(traj)
        self.prev_traj = traj.copy()
        return traj

    def _traj_to_msg(self, traj):
        msg = Trajectory()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.inertial_frame_id
        now = rospy.Time.now()

        n = traj.shape[0]
        for i in range(n):
            p = traj[i]
            tp = TrajectoryPoint()
            tp.header.stamp = now + rospy.Duration.from_sec(i * self.dt)
            tp.pose.position.x = p[0]
            tp.pose.position.y = p[1]
            tp.pose.position.z = p[2]

            yaw = 0.0
            if self.yaw_from_path and i < n - 1:
                d = traj[i + 1] - traj[i]
                yaw = math.atan2(d[1], d[0])
            q = quaternion_from_euler(0.0, 0.0, yaw)
            tp.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

            # Finite difference velocity
            if i < n - 1:
                v = (traj[i + 1] - traj[i]) / self.dt
            else:
                v = (traj[i] - traj[i - 1]) / self.dt if i > 0 else np.zeros(3)
            tp.velocity.linear = Vector3(v[0], v[1], v[2])
            tp.velocity.angular = Vector3(0.0, 0.0, 0.0)
            tp.acceleration.linear = Vector3(0.0, 0.0, 0.0)
            tp.acceleration.angular = Vector3(0.0, 0.0, 0.0)
            msg.points.append(tp)
        return msg

    def _should_publish_traj(self, traj, now, force=False):
        if force:
            return True
        if traj is None or traj.shape[0] == 0:
            return False
        if self.last_published_traj is None:
            return True
        if (now - self.last_publish_time) >= self.publish_keepalive:
            return True
        prev = self.last_published_traj
        if prev.shape != traj.shape:
            return True
        diff_xy = np.linalg.norm((traj - prev)[:, :2], axis=1)
        if float(np.max(diff_xy)) > self.publish_epsilon:
            return True
        return False

    def _publish_local_path(self, traj):
        path = Path()
        path.header.frame_id = self.inertial_frame_id
        path.header.stamp = rospy.Time.now()
        for p in traj:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            ps.pose.position.z = p[2]
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.local_path_pub.publish(path)

    def _publish_runtime_state(self, mode, avoid_active, local_published):
        self.mode_pub.publish(String(mode))
        self.avoid_active_pub.publish(Bool(bool(avoid_active)))
        self.local_publish_pub.publish(Bool(bool(local_published)))
        # This planner always builds local trajectories from global path reference.
        self.global_ref_pub.publish(Bool(mode != "GLOBAL_PASS_THROUGH"))
        if mode != self._last_mode:
            rospy.loginfo(
                "local_avoidance_planner state: mode=%s avoid=%s local_publish=%s global_ref=%s",
                mode,
                "yes" if avoid_active else "no",
                "yes" if local_published else "no",
                "yes" if mode != "GLOBAL_PASS_THROUGH" else "no",
            )
            self._last_mode = mode

    def _publish_obstacle_speed_diag(self, obstacle):
        speed = 0.0
        if obstacle is not None:
            speed = float(np.linalg.norm(obstacle.get("vel", np.zeros(3, dtype=float))[:2]))
        over_min = speed >= self.min_obstacle_speed
        over_arc = speed >= self.dynamic_arc_speed_threshold
        self.obstacle_speed_pub.publish(Float64(speed))
        self.obstacle_over_min_pub.publish(Bool(over_min))
        self.obstacle_over_arc_pub.publish(Bool(over_arc))
        rospy.loginfo_throttle(
            0.5,
            "obstacle speed=%.2f m/s (>=min %.2f: %s, >=arc %.2f: %s)",
            speed,
            self.min_obstacle_speed,
            "yes" if over_min else "no",
            self.dynamic_arc_speed_threshold,
            "yes" if over_arc else "no",
        )

    def _get_map_primary_track_copy(self):
        with self.lock:
            if self.map_primary_track is None:
                return None
            return {
                "pos": self.map_primary_track["pos"].copy(),
                "vel": self.map_primary_track["vel"].copy(),
                "radius": float(self.map_primary_track["radius"]),
                "stamp": float(self.map_primary_track["stamp"]),
                "first_stamp": float(self.map_primary_track.get("first_stamp", self.map_primary_track["stamp"])),
            }

    def _plan_cb(self, _event):
        with self.lock:
            if self.robot_pos is None:
                rospy.logwarn_throttle(
                    2.0,
                    "local_avoidance_planner: no odom received on %s, skipping plan",
                    self.odom_topic,
                )
                return
            robot_pos = self.robot_pos.copy()
            robot_vel = self.robot_vel.copy() if self.robot_vel is not None else np.zeros(3, dtype=float)

        now = rospy.Time.now().to_sec()
        n_pts = max(4, int(self.horizon_time / self.dt))
        ref = self._build_reference_traj(n_pts)
        if ref is None:
            rospy.logwarn_throttle(
                2.0,
                "local_avoidance_planner: global path empty or no waypoints loaded (%s)",
                self.global_waypoint_file,
            )
            return

        # After a local avoidance sequence, force global pass-through for a short hold.
        if now < self.global_rejoin_hold_until:
            self._resume_global_tracking_if_needed()
            rospy.loginfo_throttle(
                1.0,
                "local_avoidance_planner: global rejoin hold active (%.2fs left)",
                self.global_rejoin_hold_until - now,
            )
            self._publish_runtime_state("GLOBAL_REJOIN_HOLD", False, False)
            return

        map_obstacle = None
        detector_obstacle = None
        lidar_obstacle = None
        source_obstacle = None
        if self.perception_source == "map_manager":
            detector_obstacle = self._get_active_detector_obstacle(robot_pos, ref, now)
            map_obstacle = self._get_active_map_obstacle(robot_pos, ref, now)
            primary_track = self._get_map_primary_track_copy()
            source_obstacle = (
                detector_obstacle
                if detector_obstacle is not None
                else (primary_track if primary_track is not None else map_obstacle)
            )
            self._publish_obstacle_speed_diag(source_obstacle)
        else:
            self._debug_obstacle_state(now, robot_pos)
            lidar_obstacle = self._get_active_obstacle(robot_pos, now)
            source_obstacle = lidar_obstacle
            self._publish_obstacle_speed_diag(source_obstacle)

        # Dynamic arc mode:
        # 1) detect high-speed moving obstacle, 2) stop and observe its sweep range,
        # 3) generate an arc path to a forward point on global trajectory.
        if self.dynamic_arc_enable:
            if self.dynamic_arc_state is None:
                if self._can_start_dynamic_arc(ref, robot_pos, robot_vel, source_obstacle, now):
                    if self._start_dynamic_arc_observe(ref, robot_pos, source_obstacle, now):
                        traj = self._build_hold_traj(
                            self.dynamic_arc_state["hold_pos"], n_pts)
                        self.prev_traj = traj.copy()
                        self.last_traj = traj.copy()
                        self.last_plan_time = now
                        should_publish = self._should_publish_traj(traj, now, force=True)
                        if should_publish:
                            msg = self._traj_to_msg(traj)
                            self.traj_pub.publish(msg)
                            if self.publish_local_path:
                                self._publish_local_path(traj)
                            self.last_published_traj = traj.copy()
                            self.last_publish_time = now
                        self._publish_runtime_state("LOCAL_ARC_OBSERVE", True, should_publish)
                        return
            else:
                arc_mode = self.dynamic_arc_state.get("mode")
                if arc_mode == "observe":
                    self._update_dynamic_arc_observe(source_obstacle)
                    if now >= self.dynamic_arc_state.get("observe_until", now):
                        if not self._start_dynamic_arc_execute(robot_pos):
                            rospy.logwarn("local_avoidance_planner: dynamic arc build failed, fallback to normal planning")
                            self._clear_dynamic_arc(now=now, completed=False)
                            self.avoidance_active = False
                            self.z_hold = None
                            self.local_override_active = False
                        else:
                            arc_mode = "execute"
                    if self.dynamic_arc_state is not None and self.dynamic_arc_state.get("mode") == "observe":
                        traj = self._build_hold_traj(self.dynamic_arc_state["hold_pos"], n_pts)
                        self.prev_traj = traj.copy()
                        self.last_traj = traj.copy()
                        self.last_plan_time = now
                        should_publish = self._should_publish_traj(traj, now, force=True)
                        if should_publish:
                            msg = self._traj_to_msg(traj)
                            self.traj_pub.publish(msg)
                            if self.publish_local_path:
                                self._publish_local_path(traj)
                            self.last_published_traj = traj.copy()
                            self.last_publish_time = now
                        self._publish_runtime_state("LOCAL_ARC_OBSERVE", True, should_publish)
                        return

                if self.dynamic_arc_state is not None and self.dynamic_arc_state.get("mode") == "execute":
                    traj, done = self._get_dynamic_arc_traj(robot_pos, n_pts)
                    if done or traj is None:
                        self._clear_dynamic_arc(now=now, completed=True)
                        self.avoidance_active = False
                        self.avoidance_side = None
                        self.z_hold = None
                        self.last_traj = None
                        self.avoid_hold_until = 0.0
                        self._resume_global_tracking_if_needed()
                        self._publish_runtime_state("GLOBAL_REJOIN_HOLD", False, False)
                        return
                    if self.limit_speed:
                        traj = self._apply_speed_limit(traj)
                    if self.z_hold_mode == "current" and self.z_hold is not None:
                        traj[:, 2] = self.z_hold
                    self.prev_traj = traj.copy()
                    self.last_traj = traj.copy()
                    self.last_plan_time = now
                    self.local_override_active = True
                    should_publish = self._should_publish_traj(traj, now, force=True)
                    if should_publish:
                        msg = self._traj_to_msg(traj)
                        self.traj_pub.publish(msg)
                        if self.publish_local_path:
                            self._publish_local_path(traj)
                        self.last_published_traj = traj.copy()
                        self.last_publish_time = now
                    self._publish_runtime_state("LOCAL_ARC_EXECUTE", True, should_publish)
                    return

        # Forced local maneuver has highest priority once triggered:
        # right -> forward -> left must finish before returning to global pass-through.
        if self.force_maneuver_enable and self.force_maneuver_active:
            traj, done = self._get_force_maneuver_traj(robot_pos, n_pts)
            if done or traj is None:
                self._clear_force_maneuver(now=now, completed=True)
                self.avoidance_active = False
                self.avoidance_side = None
                self.z_hold = None
                self.last_traj = None
                self.avoid_hold_until = 0.0
            else:
                if self.limit_speed:
                    traj = self._apply_speed_limit(traj)
                if self.z_hold_mode == "current" and self.z_hold is not None:
                    traj[:, 2] = self.z_hold
                self.prev_traj = traj.copy()
                self.last_traj = traj.copy()
                self.last_plan_time = now
                should_publish = self._should_publish_traj(traj, now, force=True)
                if should_publish:
                    msg = self._traj_to_msg(traj)
                    self.traj_pub.publish(msg)
                    if self.publish_local_path:
                        self._publish_local_path(traj)
                    self.last_published_traj = traj.copy()
                    self.last_publish_time = now
                self._publish_runtime_state("LOCAL_FORCED_MANEUVER", True, should_publish)
                return

        # CERLAB map_manager-first mode:
        # use occupancy collision checks for static map AND dynamic obstacles
        # from onboard_detector (via map_cloud_topic).
        if self.perception_source == "map_manager" and self.map_use_collision_service:
            obstacle = (
                detector_obstacle
                if detector_obstacle is not None
                else (
                    map_obstacle
                    if map_obstacle is not None
                    else self._get_active_map_obstacle(robot_pos, ref, now)
                )
            )
            need_avoid = False
            if obstacle is not None:
                horizon = max(self.horizon_time, self.pred_horizon) if self.pred_horizon > 0.0 else self.horizon_time
                d_min, _ = self._predict_collision_2d(robot_pos, robot_vel, obstacle, horizon)
                r_safe = obstacle["radius"] + self.avoidance_clearance
                trigger_dist = max(r_safe, self.dist_thresh_dynamic)
                if d_min is not None and d_min < trigger_dist:
                    need_avoid = True
                elif self.avoidance_active:
                    dist = np.linalg.norm((obstacle["pos"] - robot_pos)[:2]) - obstacle["radius"]
                    if dist < self.side_release_dist:
                        need_avoid = True

            if need_avoid:
                if not self.avoidance_active:
                    self.avoidance_active = True
                    if self.z_hold_mode == "current":
                        self.z_hold = robot_pos[2]
                if self._can_start_force_maneuver(ref, robot_pos, obstacle, now):
                    started = self._start_force_maneuver(ref, robot_pos)
                    if started:
                        traj, done = self._get_force_maneuver_traj(robot_pos, n_pts)
                        if done or traj is None:
                            traj = ref.copy()
                            self._clear_force_maneuver(now=now, completed=False)
                        else:
                            if self.limit_speed:
                                traj = self._apply_speed_limit(traj)
                            if self.z_hold_mode == "current" and self.z_hold is not None:
                                traj[:, 2] = self.z_hold
                            self.prev_traj = traj.copy()
                            self.last_traj = traj.copy()
                            self.last_plan_time = now
                            should_publish = self._should_publish_traj(traj, now, force=True)
                            if should_publish:
                                msg = self._traj_to_msg(traj)
                                self.traj_pub.publish(msg)
                                if self.publish_local_path:
                                    self._publish_local_path(traj)
                                self.last_published_traj = traj.copy()
                                self.last_publish_time = now
                            self._publish_runtime_state("LOCAL_FORCED_MANEUVER", True, should_publish)
                            return
                    traj = self._plan_local_traj_2d(ref, obstacle, robot_pos)
                elif obstacle is None and self.last_traj is not None:
                    traj = self._reuse_prev_traj(self.last_traj, robot_pos, n_pts)
                    if traj is None:
                        traj = ref.copy()
                else:
                    traj = self._plan_local_traj_2d(ref, obstacle, robot_pos)
            else:
                self.avoidance_active = False
                self.avoidance_side = None
                self.z_hold = None
                traj = ref.copy()

            traj = self._enforce_map_collision_free(ref, traj)
            if self.limit_speed:
                traj = self._apply_speed_limit(traj)

            if self.avoidance_active and self.z_hold is not None:
                traj[:, 2] = self.z_hold
            else:
                traj[:, 2] = ref[:, 2]

            self.prev_traj = traj.copy()
            self.last_traj = traj.copy()
            self.last_plan_time = now

            should_publish = self._should_publish_traj(
                traj, now, force=bool(self.avoidance_active))
            if should_publish:
                msg = self._traj_to_msg(traj)
                self.traj_pub.publish(msg)
                if self.publish_local_path:
                    self._publish_local_path(traj)
                self.last_published_traj = traj.copy()
                self.last_publish_time = now
            rospy.loginfo_throttle(
                2.0,
                "local_avoidance_planner(map_manager): %s traj pts=%d avoid=%s",
                "published" if should_publish else "keep",
                traj.shape[0],
                "yes" if self.avoidance_active else "no",
            )
            mode = "LOCAL_AVOIDANCE" if self.avoidance_active else "LOCAL_REFERENCE_FOLLOW"
            self._publish_runtime_state(mode, self.avoidance_active, should_publish)
            return

        if self.perception_source == "map_manager":
            obstacle = (
                detector_obstacle
                if detector_obstacle is not None
                else (
                    map_obstacle
                    if map_obstacle is not None
                    else self._get_active_map_obstacle(robot_pos, ref, now)
                )
            )
            if self.debug_lidar:
                if obstacle is None:
                    rospy.loginfo_throttle(1.0, "map_manager obstacle=no")
                else:
                    speed = float(np.linalg.norm(obstacle.get("vel", np.zeros(3, dtype=float))[:2]))
                    rospy.loginfo_throttle(
                        1.0,
                        "map_manager obstacle=yes speed=%.2f",
                        speed,
                    )
        else:
            obstacle = lidar_obstacle if lidar_obstacle is not None else self._get_active_obstacle(robot_pos, now)

        need_avoid = False
        if obstacle is not None:
            horizon = max(self.horizon_time, self.pred_horizon) if self.pred_horizon > 0.0 else self.horizon_time
            d_min, _ = self._predict_collision_2d(robot_pos, robot_vel, obstacle, horizon)
            r_safe = obstacle["radius"] + self.avoidance_clearance
            if d_min is not None and d_min < r_safe:
                need_avoid = True
            elif self.avoidance_active:
                dist = np.linalg.norm((obstacle["pos"] - robot_pos)[:2]) - obstacle["radius"]
                if dist < self.side_release_dist:
                    need_avoid = True

        if self.avoidance_active and now < self.avoid_hold_until:
            need_avoid = True

        if need_avoid:
            if not self.avoidance_active:
                self.avoidance_active = True
                if self.z_hold_mode == "current":
                    self.z_hold = robot_pos[2]
            self.avoid_hold_until = max(self.avoid_hold_until, now + self.min_avoid_hold_time)
        else:
            self.avoidance_active = False
            self.avoidance_side = None
            self.z_hold = None
            self.last_traj = None
            self.avoid_hold_until = 0.0

        if need_avoid and (not self.force_maneuver_active) and self._can_start_force_maneuver(ref, robot_pos, obstacle, now):
            self._start_force_maneuver(ref, robot_pos)
        if self.force_maneuver_enable and self.force_maneuver_active:
            traj, done = self._get_force_maneuver_traj(robot_pos, n_pts)
            if done or traj is None:
                self._clear_force_maneuver(now=now, completed=True)
                need_avoid = False
                self.avoidance_active = False
                self.avoidance_side = None
                self.z_hold = None
                self.last_traj = None
                self.avoid_hold_until = 0.0
            else:
                if self.limit_speed:
                    traj = self._apply_speed_limit(traj)
                if self.z_hold_mode == "current" and self.z_hold is not None:
                    traj[:, 2] = self.z_hold
                self.prev_traj = traj.copy()
                self.last_traj = traj.copy()
                self.last_plan_time = now
                self.local_override_active = True
                should_publish = self._should_publish_traj(traj, now, force=True)
                if should_publish:
                    msg = self._traj_to_msg(traj)
                    self.traj_pub.publish(msg)
                    if self.publish_local_path:
                        self._publish_local_path(traj)
                    self.last_published_traj = traj.copy()
                    self.last_publish_time = now
                self._publish_runtime_state("LOCAL_FORCED_MANEUVER", True, should_publish)
                return

        # In map_manager mode (without service-based collision projection), let
        # the global waypoint tracker run when there is no active obstacle.
        if self.perception_source == "map_manager" and not self.map_use_collision_service and not need_avoid:
            self._resume_global_tracking_if_needed()
            rospy.loginfo_throttle(
                2.0,
                "local_avoidance_planner(map_manager): obstacle=no, pass-through global tracking",
            )
            self._publish_runtime_state("GLOBAL_PASS_THROUGH", False, False)
            return

        if need_avoid:
            if (now - self.last_plan_time) >= self.replan_min_interval or self.last_traj is None:
                if obstacle is None and self.last_traj is not None:
                    traj = self._reuse_prev_traj(self.last_traj, robot_pos, n_pts)
                    if traj is None:
                        traj = ref.copy()
                else:
                    traj = self._plan_local_traj_2d(ref, obstacle, robot_pos)
                self.last_plan_time = now
                self.last_traj = traj.copy()
                self.local_override_active = True
            else:
                traj = self._reuse_prev_traj(self.last_traj, robot_pos, n_pts)
                if traj is None:
                    if obstacle is None and self.last_traj is not None:
                        traj = self.last_traj.copy()
                    else:
                        traj = self._plan_local_traj_2d(ref, obstacle, robot_pos)
                    self.last_plan_time = now
                    self.last_traj = traj.copy()
                    self.local_override_active = True
        else:
            traj = ref.copy()
            if self.perception_source == "map_manager":
                traj = self._enforce_map_collision_free(ref, traj)
            if self.limit_speed:
                traj = self._apply_speed_limit(traj)
            self.prev_traj = traj.copy()

        if self.avoidance_active and self.z_hold is not None:
            traj[:, 2] = self.z_hold
        else:
            traj[:, 2] = ref[:, 2]

        should_publish = self._should_publish_traj(
            traj, now, force=bool(self.avoidance_active))
        if should_publish:
            msg = self._traj_to_msg(traj)
            self.traj_pub.publish(msg)
            if self.publish_local_path:
                self._publish_local_path(traj)
            self.last_published_traj = traj.copy()
            self.last_publish_time = now
        rospy.loginfo_throttle(
            2.0,
            "local_avoidance_planner: %s traj pts=%d avoid=%s",
            "published" if should_publish else "keep",
            traj.shape[0],
            "yes" if self.avoidance_active else "no",
        )
        mode = "LOCAL_AVOIDANCE" if self.avoidance_active else "LOCAL_REFERENCE_FOLLOW"
        self._publish_runtime_state(mode, self.avoidance_active, should_publish)


def main():
    rospy.init_node("local_avoidance_planner")
    LocalAvoidancePlanner()
    rospy.spin()


if __name__ == "__main__":
    main()
