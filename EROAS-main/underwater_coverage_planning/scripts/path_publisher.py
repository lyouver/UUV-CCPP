#!/usr/bin/env python3
import os
import glob
import yaml
import rospy
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def ned_to_enu(x_n, y_n, z_n):
    return y_n, x_n, -z_n


def enu_to_ned(x_e, y_e, z_e):
    return y_e, x_e, -z_e


def load_waypoints(waypoint_file):
    with open(waypoint_file, "r") as f:
        data = yaml.safe_load(f) or {}
    frame_id = data.get("inertial_frame_id", "")
    points = []
    for wp in data.get("waypoints", []):
        pt = wp.get("point", [])
        if len(pt) >= 3:
            points.append((float(pt[0]), float(pt[1]), float(pt[2])))
    return frame_id, points


def find_latest_waypoints(guiji_dir):
    pattern = os.path.join(guiji_dir, "*_ros_waypoints.yaml")
    files = glob.glob(pattern)
    if not files:
        return ""
    files.sort(key=os.path.getmtime, reverse=True)
    return files[0]


def convert_points(points, input_frame, output_frame):
    if input_frame == output_frame:
        return points
    converted = []
    if input_frame == "world_ned" and output_frame == "world":
        for x, y, z in points:
            converted.append(ned_to_enu(x, y, z))
        return converted
    if input_frame == "world" and output_frame == "world_ned":
        for x, y, z in points:
            converted.append(enu_to_ned(x, y, z))
        return converted
    return points


def build_path(points, frame_id):
    path = Path()
    path.header.frame_id = frame_id
    for x, y, z in points:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def main():
    rospy.init_node("global_path_publisher")

    path_topic = rospy.get_param("~path_topic", "/nav/global_path")
    waypoint_file = rospy.get_param("~waypoint_file", "")
    guiji_dir = rospy.get_param("~guiji_dir", "")
    auto_latest = rospy.get_param("~auto_latest", True)
    publish_rate = float(rospy.get_param("~publish_rate", 0.0))
    downsample_step = int(rospy.get_param("~downsample_step", 1))

    output_frame = rospy.get_param("~output_frame", "world_ned")
    input_frame_param = rospy.get_param("~input_frame", "")

    if not waypoint_file:
        if not guiji_dir:
            rospack = rospkg.RosPack()
            guiji_dir = os.path.join(rospack.get_path("underwater_coverage_planning"), "scripts", "guiji")
        if auto_latest:
            waypoint_file = find_latest_waypoints(guiji_dir)

    if not waypoint_file or not os.path.isfile(waypoint_file):
        rospy.logerr("No valid waypoint_file found. Set ~waypoint_file or enable ~auto_latest.")
        return

    yaml_frame, points = load_waypoints(waypoint_file)
    if downsample_step > 1:
        points = points[::downsample_step]

    input_frame = input_frame_param or yaml_frame or output_frame
    points = convert_points(points, input_frame, output_frame)

    if not points:
        rospy.logerr("Waypoint file has no points: %s", waypoint_file)
        return

    path_msg = build_path(points, output_frame)
    pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)

    rospy.loginfo("Publishing global path: %s (points=%d, frame=%s)", waypoint_file, len(points), output_frame)
    rate = rospy.Rate(publish_rate) if publish_rate > 0 else None
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        path_msg.header.stamp = now
        for pose in path_msg.poses:
            pose.header.stamp = now
        pub.publish(path_msg)
        if publish_rate <= 0:
            break
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
