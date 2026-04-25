#!/usr/bin/env python3
"""Extract table 5.5 and figure source data from chapter-5 online runs."""

import argparse
import csv
import math
import os
import re
import sys
from bisect import bisect_left

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from sip_coverage.metrics import ensure_dir, write_csv


ROBOT_RADIUS_DEFAULT = 0.6


def _log_time(line):
    m = re.search(r"\[\s*(?:INFO|WARN|ERROR|DEBUG)\]\s*\[([0-9.]+)\]", line)
    if not m:
        return None
    try:
        return float(m.group(1))
    except ValueError:
        return None


def _parse_mode_log(path):
    timeline = []
    transitions = []
    solve_times = []
    if not os.path.isfile(path):
        return timeline, transitions, solve_times
    state_re = re.compile(
        r"mode=(\w+).*?obs=(\d+).*?moving_obs=(\d+).*?clearance_xy=([-0-9.]+).*?dist_to_global=([-0-9.]+).*?hold_left=([-0-9.]+)"
    )
    trans_re = re.compile(r"mode (\w+) -> (\w+) \((.*?)\)")
    solve_re = re.compile(r"local_mpc_solve_time_sec=([0-9.eE+-]+)")
    with open(path, "r", errors="ignore") as f:
        for line in f:
            t = _log_time(line)
            sm = state_re.search(line)
            if sm:
                timeline.append(
                    {
                        "time": float(t if t is not None else len(timeline)),
                        "mode": sm.group(1),
                        "obs": int(sm.group(2)),
                        "moving_obs": int(sm.group(3)),
                        "clearance_xy": float(sm.group(4)),
                        "dist_to_global": float(sm.group(5)),
                        "hold_left": float(sm.group(6)),
                    }
                )
            tm = trans_re.search(line)
            if tm:
                transitions.append(
                    {
                        "time": float(t if t is not None else len(transitions)),
                        "from_mode": tm.group(1),
                        "to_mode": tm.group(2),
                        "reason": tm.group(3),
                    }
                )
            pm = solve_re.search(line)
            if pm:
                solve_times.append(
                    {
                        "time": float(t if t is not None else len(solve_times)),
                        "local_mpc_solve_time_sec": float(pm.group(1)),
                    }
                )
    return timeline, transitions, solve_times


def _parse_waypoint_file(path):
    if not path or not os.path.isfile(path):
        return []
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    points = []
    for wp in data.get("waypoints", []):
        p = wp.get("point")
        if isinstance(p, list) and len(p) >= 3:
            points.append((float(p[0]), float(p[1]), float(p[2])))
    return points


def _point_segment_distance_xy(p, a, b):
    px, py = p[0], p[1]
    ax, ay = a[0], a[1]
    bx, by = b[0], b[1]
    dx = bx - ax
    dy = by - ay
    den = dx * dx + dy * dy
    if den <= 1e-12:
        return math.hypot(px - ax, py - ay)
    u = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / den))
    qx = ax + u * dx
    qy = ay + u * dy
    return math.hypot(px - qx, py - qy)


def _distance_to_path_xy(p, path):
    if len(path) < 2:
        return 0.0
    return min(_point_segment_distance_xy(p, path[i - 1], path[i]) for i in range(1, len(path)))


def _read_bag_data(bag_path, obstacle_names):
    try:
        import rosbag
    except Exception as exc:
        return [], [], f"rosbag_unavailable:{exc}"
    if not os.path.isfile(bag_path):
        return [], [], "bag_missing"
    robot = []
    obstacles = []
    try:
        bag = rosbag.Bag(bag_path)
        for topic, msg, t in bag.read_messages(topics=["/rexrov/pose_gt", "/gazebo/model_states"]):
            stamp = float(t.to_sec())
            if topic == "/rexrov/pose_gt":
                p = msg.pose.pose.position
                robot.append({"time": stamp, "x": float(p.x), "y": float(p.y), "z": float(p.z)})
            elif topic == "/gazebo/model_states":
                for name in obstacle_names:
                    if name not in msg.name:
                        continue
                    idx = msg.name.index(name)
                    p = msg.pose[idx].position
                    obstacles.append(
                        {
                            "time": stamp,
                            "name": name,
                            "x": float(p.x),
                            "y": float(p.y),
                            "z": float(p.z),
                        }
                    )
        bag.close()
    except Exception as exc:
        return robot, obstacles, f"bag_read_error:{exc}"
    return robot, obstacles, ""


def _nearest_obstacle_clearance(robot, obstacles, obstacle_radius, robot_radius):
    if not robot or not obstacles:
        return 0.0
    by_name = {}
    for obs in obstacles:
        by_name.setdefault(obs["name"], []).append(obs)
    for vals in by_name.values():
        vals.sort(key=lambda r: r["time"])
    min_clear = float("inf")
    for pose in robot:
        for vals in by_name.values():
            times = [v["time"] for v in vals]
            idx = bisect_left(times, pose["time"])
            candidates = []
            if idx < len(vals):
                candidates.append(vals[idx])
            if idx > 0:
                candidates.append(vals[idx - 1])
            for obs in candidates:
                d = math.sqrt((pose["x"] - obs["x"]) ** 2 + (pose["y"] - obs["y"]) ** 2 + (pose["z"] - obs["z"]) ** 2)
                min_clear = min(min_clear, d - obstacle_radius - robot_radius)
    return 0.0 if not math.isfinite(min_clear) else float(min_clear)


def _rejoin_time(transitions):
    last_local_exit = None
    last_rejoin = 0.0
    for tr in transitions:
        if tr["from_mode"] == "LOCAL_MPC" and tr["to_mode"] == "REJOIN_HOLD":
            last_local_exit = tr["time"]
        elif tr["to_mode"] == "GLOBAL_PASS" and last_local_exit is not None:
            last_rejoin = max(0.0, tr["time"] - last_local_exit)
            last_local_exit = None
    return float(last_rejoin)


def _extract_run(run_dir):
    scenario_path = os.path.join(run_dir, "scenario.yaml")
    scenario = {}
    if os.path.isfile(scenario_path):
        with open(scenario_path, "r") as f:
            scenario = yaml.safe_load(f) or {}
    profile = scenario.get("obstacle_profile") or {}
    obstacle_names = profile.get("obstacle_names") or ["moving_sphere_obstacle_1"]
    obstacle_radius = float(profile.get("obstacle_radius", 2.0))
    robot_radius = float((scenario.get("mpc_overrides") or {}).get("robot_collision_radius", ROBOT_RADIUS_DEFAULT))
    timeline, transitions, solve_times = _parse_mode_log(os.path.join(run_dir, "adapter_and_launch.log"))
    robot, obstacles, bag_error = _read_bag_data(os.path.join(run_dir, "run.bag"), obstacle_names)
    global_path = _parse_waypoint_file(scenario.get("waypoint_file", ""))
    max_dev = max((_distance_to_path_xy((p["x"], p["y"], p["z"]), global_path) for p in robot), default=0.0)
    min_clear = _nearest_obstacle_clearance(robot, obstacles, obstacle_radius, robot_radius)

    fig_root = ensure_dir(os.path.join(run_dir, "figures"))
    write_csv(os.path.join(fig_root, "fig_5_4_obstacle_tracks.csv"), obstacles)
    write_csv(os.path.join(fig_root, "fig_5_5_robot_trajectory.csv"), robot)
    write_csv(os.path.join(fig_root, "fig_5_5_local_mpc_solve_times.csv"), solve_times)
    write_csv(os.path.join(fig_root, "fig_5_6_mode_timeline.csv"), timeline)
    write_csv(os.path.join(fig_root, "fig_5_6_mode_transitions.csv"), transitions)

    result = "success"
    if bag_error:
        result = bag_error
    elif min_clear < 0.0:
        result = "collision_risk"
    elif not any(tr["to_mode"] == "LOCAL_MPC" for tr in transitions):
        result = "no_local_mpc_trigger"

    return {
        "scenario_name": scenario.get("scenario_name", os.path.basename(os.path.dirname(run_dir))),
        "run_dir": run_dir,
        "min_obstacle_clearance_m": float(min_clear),
        "local_planning_avg_time_sec": float(
            sum(r["local_mpc_solve_time_sec"] for r in solve_times) / len(solve_times)
            if solve_times
            else 0.0
        ),
        "max_path_deviation_m": float(max_dev),
        "rejoin_time_sec": _rejoin_time(transitions),
        "mode_switch_count": int(len(transitions)),
        "execution_result": result,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", default="/home/tb/dave_ws/chapter5_outputs/online_batch")
    parser.add_argument("--output-dir", default="", help="Defaults to INPUT_DIR/tables")
    parser.add_argument("--write-table-5-5", action="store_true", help="Write formal table_5_5.csv")
    args = parser.parse_args()

    rows = []
    for root, dirs, files in os.walk(args.input_dir):
        if "scenario.yaml" in files:
            rows.append(_extract_run(root))
    if not rows:
        raise RuntimeError(f"No online runs found under {args.input_dir}")
    out_dir = args.output_dir or os.path.join(args.input_dir, "tables")
    metric_file = "table_5_5.csv" if args.write_table_5_5 else "online_metrics_raw.csv"
    write_csv(
        os.path.join(out_dir, metric_file),
        rows,
        [
            "scenario_name",
            "run_dir",
            "min_obstacle_clearance_m",
            "local_planning_avg_time_sec",
            "max_path_deviation_m",
            "rejoin_time_sec",
            "mode_switch_count",
            "execution_result",
        ],
    )
    print(f"online metrics: {os.path.join(out_dir, metric_file)}")
    if not args.write_table_5_5:
        print("table_5_5.csv was not written; pass --write-table-5-5 when this thesis table is needed.")


if __name__ == "__main__":
    main()
