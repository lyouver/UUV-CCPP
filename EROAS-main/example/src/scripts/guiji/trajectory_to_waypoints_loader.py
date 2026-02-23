#!/usr/bin/env python3
import os
import sys
import yaml
import rospy
import numpy as np
from std_msgs.msg import String, Time
from uuv_control_msgs.srv import InitWaypointsFromFile, InitWaypointsFromFileRequest


def enu_to_ned_pos(x, y, z):
	return y, x, -z


def main():
	rospy.init_node('trajectory_to_waypoints_loader')
	# Params
	uuv_name = rospy.get_param('~uuv_name', 'rexrov')
	trajectory_file = rospy.get_param('~trajectory_file', '')
	waypoint_file = rospy.get_param('~waypoint_file', '')
	default_speed = float(rospy.get_param('~max_forward_speed', 0.5))
	use_fixed_heading = bool(rospy.get_param('~use_fixed_heading', False))
	downsample_step = int(rospy.get_param('~downsample_step', 1))
	interpolator = rospy.get_param('~interpolator', 'cubic')

	# If waypoint_file provided and exists, use it directly
	if waypoint_file and os.path.isfile(waypoint_file):
		rospy.loginfo('Using provided waypoints file: %s', waypoint_file)
		# Validate content and enforce NED depth (z >= 0)
		with open(waypoint_file, 'r') as f:
			wps_yaml = yaml.safe_load(f) or {}
			if 'waypoints' in wps_yaml:
				changed = False
				for wp in wps_yaml['waypoints']:
					if len(wp.get('point', [])) >= 3 and wp['point'][2] < 0:
						wp['point'][2] = abs(wp['point'][2])
						changed = True
				if wps_yaml.get('inertial_frame_id', '') != 'world_ned':
					wps_yaml['inertial_frame_id'] = 'world_ned'
					changed = True
				if changed:
					with open(waypoint_file, 'w') as wf:
						yaml.safe_dump(wps_yaml, wf, default_flow_style=False)
	else:
		# Fallback: must have a trajectory_file to convert
		if not trajectory_file or not os.path.isfile(trajectory_file):
			rospy.logerr('No valid waypoint_file or trajectory_file provided!')
			sys.exit(1)
		# Determine output waypoint file path if not provided
		if waypoint_file == '':
			base, _ = os.path.splitext(trajectory_file)
			waypoint_file = base + '_waypoints.yaml'
		# Load trajectory YAML
		with open(trajectory_file, 'r') as f:
			traj = yaml.safe_load(f)
		# Determine input frame
		input_frame = 'world'
		if 'header' in traj and isinstance(traj['header'], dict):
			input_frame = traj['header'].get('frame_id', 'world')
		rospy.loginfo('Input trajectory frame_id: %s', input_frame)
		# Convert to waypoint list in world_ned (NED)
		wps = []
		points = traj.get('points', [])
		if downsample_step <= 0:
			downsample_step = 1
		for idx in range(0, len(points), downsample_step):
			p = points[idx]
			pos = p.get('positions', [0, 0, 0, 0, 0, 0])
			x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
			if input_frame == 'world':
				x, y, z = enu_to_ned_pos(x, y, z)
			# Ensure underwater depth in NED (z >= 0)
			if z < 0:
				z = abs(z)
			wp = {
				'point': [float(x), float(y), float(z)],
				'max_forward_speed': float(default_speed),
				'heading': 0.0,
				'use_fixed_heading': bool(use_fixed_heading)
			}
			wps.append(wp)
		waypoints_yaml = {
			'inertial_frame_id': 'world_ned',
			'waypoints': wps
		}
		with open(waypoint_file, 'w') as f:
			yaml.safe_dump(waypoints_yaml, f, default_flow_style=False)
		rospy.loginfo('Saved waypoints to %s (count=%d)', waypoint_file, len(wps))

	# Call init_waypoints_from_file service
	service_name = f'/{uuv_name}/init_waypoints_from_file'
	try:
		rospy.wait_for_service(service_name, timeout=10.0)
	except Exception as e:
		rospy.logerr('Service unavailable: %s, err=%s', service_name, e)
		sys.exit(2)

	try:
		client = rospy.ServiceProxy(service_name, InitWaypointsFromFile)
		req = InitWaypointsFromFileRequest()
		req.start_now = True
		req.filename = String(data=os.path.abspath(waypoint_file))
		req.interpolator = String(data=interpolator)
		resp = client(req)
		if not resp.success:
			rospy.logerr('init_waypoints_from_file failed')
			sys.exit(3)
		rospy.loginfo('Waypoints loaded and tracking started via %s', service_name)
	except Exception as e:
		rospy.logerr('Service call failed: %s', e)
		sys.exit(4)

	rospy.loginfo('Done.')


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass 