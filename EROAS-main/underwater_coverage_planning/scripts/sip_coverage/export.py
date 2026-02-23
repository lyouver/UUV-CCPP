import math
from typing import List, Tuple

import yaml

from .models import Viewpoint


def build_ordered_waypoints(viewpoints: List[Viewpoint], order: List[int], cfg):
    ordered = [viewpoints[i] for i in order]
    if not ordered:
        return []

    waypoints = []
    t = 0.0
    prev_heading = 0.0

    for i, vp in enumerate(ordered):
        if i + 1 < len(ordered):
            nxt = ordered[i + 1]
            dx = float(nxt.position[0] - vp.position[0])
            dy = float(nxt.position[1] - vp.position[1])
            dz = float(nxt.position[2] - vp.position[2])
            heading = math.atan2(dy, dx)
            dist_xy = math.hypot(dx, dy)
            t_xy = dist_xy / max(float(cfg.max_forward_speed), 1e-3)
            t_z = abs(dz) / max(float(cfg.max_vertical_speed), 1e-3)
            t_turn = abs((heading - prev_heading + math.pi) % (2.0 * math.pi) - math.pi) / max(float(cfg.max_yaw_rate), 1e-3)
            dt = max(t_xy, t_z) + t_turn
        else:
            heading = prev_heading
            dt = 0.0

        waypoints.append(
            {
                "x": float(vp.position[0]),
                "y": float(vp.position[1]),
                "z": float(vp.position[2]),
                "yaw": float(heading),
                "timestamp": float(t),
            }
        )
        t += dt
        prev_heading = heading

    return waypoints


def save_ros_waypoints_yaml(base_path: str, waypoints, cfg) -> str:
    ros_wps = []
    for wp in waypoints:
        ros_wps.append(
            {
                "point": [float(wp["x"]), float(wp["y"]), float(wp["z"])],
                "max_forward_speed": float(cfg.max_forward_speed),
                "heading": 0.0,
                "use_fixed_heading": False,
            }
        )

    data = {
        "inertial_frame_id": "world",
        "waypoints": ros_wps,
    }

    outfile = f"{base_path}_ros_waypoints.yaml"
    with open(outfile, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False)

    return outfile


def tour_path_length(viewpoints: List[Viewpoint], order: List[int]) -> float:
    if len(order) < 2:
        return 0.0

    total = 0.0
    for i in range(1, len(order)):
        a = viewpoints[order[i - 1]].position
        b = viewpoints[order[i]].position
        total += float(math.sqrt(((b - a) ** 2).sum()))

    # Closed tour length report
    a = viewpoints[order[-1]].position
    b = viewpoints[order[0]].position
    total += float(math.sqrt(((b - a) ** 2).sum()))
    return total
