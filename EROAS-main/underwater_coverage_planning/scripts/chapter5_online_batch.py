#!/usr/bin/env python3
"""Run chapter-5 online Gazebo batches and collect rosbag/log files."""

import argparse
import os
import signal
import subprocess
import sys
import time

import yaml


DEFAULT_MPC_PARAM_FILE = "/home/tb/dave_ws/src/EROAS-main/example/src/config/local_mpc_adapter.yaml"

TOPICS = [
    "/rexrov/pose_gt",
    "/gazebo/model_states",
    "/onboard_detector/velocity_visualizaton",
    "/nav/global_path",
    "/nav/local_path",
    "/rexrov/dp_controller/input_trajectory",
    "/rosout",
]


def _load_scenarios(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    scenarios = data.get("scenarios", [])
    if not isinstance(scenarios, list):
        raise ValueError("Scenario YAML must contain a 'scenarios' list")
    return scenarios


def _terminate(proc, timeout=10.0):
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()


def _roslaunch_cmd(scenario):
    cmd = [
        "roslaunch",
        scenario["world_launch"],
        "gui:=false",
        "controller_gui:=false",
        "show_main_rviz:=false",
        "show_detector_rviz:=false",
        "show_world_terrain_in_rviz:=false",
        "global_waypoints_file:=" + scenario["waypoint_file"],
    ]
    if scenario.get("_local_mpc_param_file"):
        cmd.append("local_mpc_param_file:=" + scenario["_local_mpc_param_file"])
    return cmd


def _fake_marker_cmd(scenario):
    profile = scenario.get("obstacle_profile") or {}
    names = ",".join(profile.get("obstacle_names") or [])
    radius = profile.get("obstacle_radius", 2.0)
    dynamic_speed_threshold = profile.get("dynamic_speed_threshold", 0.5)
    return [
        "rosrun",
        "underwater_coverage_planning",
        "chapter5_fake_obstacle_markers.py",
        f"_obstacle_names:={names}",
        f"_obstacle_radius:={radius}",
        f"_dynamic_speed_threshold:={dynamic_speed_threshold}",
    ]


def _write_run_mpc_params(scenario, run_dir):
    with open(DEFAULT_MPC_PARAM_FILE, "r") as f:
        params = yaml.safe_load(f) or {}
    overrides = dict(scenario.get("mpc_overrides") or {})
    for key, value in overrides.items():
        if key == "dynamic_safety_dist":
            params["mpc_planner/dynamic_safety_dist"] = value
        elif key == "static_safety_dist":
            params["mpc_planner/static_safety_dist"] = value
        elif key == "robot_collision_radius":
            params[key] = value
            params["occupancy_map/collision_check_radius"] = value
        else:
            params[key] = value
    param_file = os.path.join(run_dir, "local_mpc_adapter.yaml")
    with open(param_file, "w") as f:
        yaml.safe_dump(params, f, allow_unicode=True, sort_keys=False)
    return param_file


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenarios",
        default="/home/tb/dave_ws/src/EROAS-main/underwater_coverage_planning/config/chapter5_online_scenarios.yaml",
    )
    parser.add_argument("--output-dir", default="/home/tb/dave_ws/chapter5_outputs/online_batch")
    parser.add_argument("--only", default="", help="Run one scenario by scenario_name")
    parser.add_argument("--repeat", type=int, default=None, help="Override repeat count")
    parser.add_argument("--duration-sec", type=float, default=None, help="Override run duration")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    out_root = os.path.abspath(args.output_dir)
    os.makedirs(out_root, exist_ok=True)
    with open(os.path.join(out_root, "README.md"), "w") as f:
        f.write(
            "# Chapter 5 Online Batch Output\n\n"
            "- Each `SCENARIO/run_XX` directory contains `scenario.yaml`, `commands.txt`, `adapter_and_launch.log`, and `run.bag` when not using `--dry-run`.\n"
            "- Run `chapter5_extract_online_metrics.py --input-dir OUTPUT_DIR` after the batch to generate raw online metrics and figure source CSV files.\n"
            "- The formal `table_5_5.csv` is only written when passing `--write-table-5-5` to the extractor.\n"
            "- `mpc_overrides` in the scenario YAML is recorded as experiment metadata; the current launch file exposes only its declared roslaunch args.\n"
        )
    scenarios = _load_scenarios(args.scenarios)
    if args.only:
        scenarios = [s for s in scenarios if s.get("scenario_name") == args.only]
    if not scenarios:
        raise RuntimeError("No online scenarios selected")

    for scenario in scenarios:
        repeat = int(args.repeat if args.repeat is not None else scenario.get("repeat", 1))
        duration = float(args.duration_sec if args.duration_sec is not None else scenario.get("duration_sec", 60))
        for r in range(1, repeat + 1):
            run_dir = os.path.join(out_root, scenario["scenario_name"], f"run_{r:02d}")
            os.makedirs(run_dir, exist_ok=True)
            launch_log = open(os.path.join(run_dir, "adapter_and_launch.log"), "w")
            bag_path = os.path.join(run_dir, "run.bag")
            run_scenario = dict(scenario)
            run_scenario["_local_mpc_param_file"] = _write_run_mpc_params(run_scenario, run_dir)
            launch_cmd = _roslaunch_cmd(run_scenario)
            marker_cmd = _fake_marker_cmd(run_scenario)
            bag_cmd = ["rosbag", "record", "-O", bag_path] + TOPICS
            with open(os.path.join(run_dir, "scenario.yaml"), "w") as f:
                yaml.safe_dump(run_scenario, f, allow_unicode=True, sort_keys=False)
            with open(os.path.join(run_dir, "commands.txt"), "w") as f:
                f.write(" ".join(launch_cmd) + "\n")
                f.write(" ".join(marker_cmd) + "\n")
                f.write(" ".join(bag_cmd) + "\n")

            if args.dry_run:
                print(f"[dry-run] {scenario['scenario_name']} run {r}: {run_dir}")
                continue

            print(f"Starting {scenario['scenario_name']} run {r}/{repeat} for {duration:.1f}s")
            launch_proc = subprocess.Popen(launch_cmd, stdout=launch_log, stderr=subprocess.STDOUT)
            time.sleep(12.0)
            marker_proc = subprocess.Popen(marker_cmd, stdout=launch_log, stderr=subprocess.STDOUT)
            time.sleep(2.0)
            bag_proc = subprocess.Popen(bag_cmd, stdout=launch_log, stderr=subprocess.STDOUT)
            try:
                time.sleep(duration)
            finally:
                _terminate(bag_proc)
                _terminate(marker_proc)
                _terminate(launch_proc)
                launch_log.close()
            print(f"Finished: {run_dir}")

    print(f"Online batch outputs: {out_root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
