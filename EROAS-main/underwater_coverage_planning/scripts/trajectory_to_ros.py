#!/usr/bin/env python3
"""
轨迹加载器生成器（Waypoints-only）
直接从 *_ros_waypoints.yaml 生成 *_loader.launch，调用官方服务加载航点
"""

import os
from datetime import datetime

def _find_package_root(start_dir: str) -> str:
	cur = os.path.abspath(start_dir)
	while True:
		if os.path.isfile(os.path.join(cur, 'package.xml')):
			return cur
		parent = os.path.dirname(cur)
		if parent == cur:
			return ''
		cur = parent


def create_loader_launch_file(waypoint_file: str, launch_file: str):
	print(f"创建loader launch文件: {launch_file}")
	# 相对路径（相对于 underwater_coverage_planning 包）
	pkg_root = _find_package_root(os.path.dirname(__file__))
	if pkg_root:
		rel_wp_path = os.path.relpath(waypoint_file, start=pkg_root)
		if rel_wp_path.startswith('..'):
			waypoint_default = os.path.abspath(waypoint_file)
		else:
			waypoint_default = f"$(find underwater_coverage_planning)/{rel_wp_path}"
	else:
		waypoint_default = os.path.abspath(waypoint_file)
	launch_content = f'''<?xml version="1.0"?>
<launch>
	<arg name="uuv_name" default="rexrov"/>
	<arg name="waypoint_file" default="{waypoint_default}"/>
	<arg name="max_forward_speed" default="40.0"/>
	<arg name="use_fixed_heading" default="false"/>
	<arg name="downsample_step" default="1"/>
	<arg name="interpolator" default="cubic"/>

	<node name="trajectory_to_waypoints_loader"
	      pkg="underwater_coverage_planning"
	      type="trajectory_to_waypoints_loader.py"
	      output="screen">
		<param name="uuv_name" value="$(arg uuv_name)"/>
		<param name="trajectory_file" value=""/>
		<param name="waypoint_file" value="$(arg waypoint_file)"/>
		<param name="max_forward_speed" value="$(arg max_forward_speed)"/>
		<param name="use_fixed_heading" value="$(arg use_fixed_heading)"/>
		<param name="downsample_step" value="$(arg downsample_step)"/>
		<param name="interpolator" value="$(arg interpolator)"/>
	</node>
</launch>
'''
	with open(launch_file, 'w') as f:
		f.write(launch_content)
	print("Loader Launch文件创建完成")


def main():
	print("=== Waypoints-only Loader 生成器 ===")
	output_dir = os.path.join(os.path.dirname(__file__), 'guiji')
	if not os.path.exists(output_dir):
		os.makedirs(output_dir)
		print(f"创建输出目录: {output_dir}")

	guiji_dir = os.path.join(os.path.dirname(__file__), 'guiji')
	if not os.path.exists(guiji_dir):
		print("未找到 guiji 目录，请先运行 interactive_coverage_planner.py 生成 *_ros_waypoints.yaml")
		return

	waypoints_files = [os.path.join(guiji_dir, f) for f in os.listdir(guiji_dir) if f.endswith('_ros_waypoints.yaml')]
	if not waypoints_files:
		print("未找到 *_ros_waypoints.yaml，请先运行 interactive_coverage_planner.py")
		return
	waypoints_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)
	waypoint_file = waypoints_files[0]
	print(f"发现 Waypoints 文件：{waypoint_file}")

	base = os.path.basename(waypoint_file).replace('_ros_waypoints.yaml', '')
	loader_launch_file = os.path.join(output_dir, f"{base}_loader.launch")
	create_loader_launch_file(waypoint_file, loader_launch_file)

	print("\n=== 生成完成（Waypoints-only）===")
	print(f"Loader launch:   {loader_launch_file}")
	print("用法：")
	print("1. 启动环境与控制器:")
	print("   roslaunch underwater_coverage_planning compact_terrain_rexrov_with_trajectory.launch")
	print("2. 加载航点(官方服务):")
	print(f"   roslaunch underwater_coverage_planning {os.path.relpath(loader_launch_file, start=os.path.join(os.path.dirname(__file__), '../../..'))}")

if __name__ == "__main__":
	main()
