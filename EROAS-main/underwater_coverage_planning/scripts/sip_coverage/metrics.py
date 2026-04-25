import csv
import json
import math
import os
from typing import Dict, Iterable, List, Sequence

import numpy as np

from .models import Viewpoint


def ensure_dir(path: str) -> str:
    os.makedirs(path, exist_ok=True)
    return path


def write_json(path: str, data) -> str:
    ensure_dir(os.path.dirname(path))
    with open(path, "w") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    return path


def write_csv(path: str, rows: List[Dict], fieldnames: Sequence[str] = None) -> str:
    ensure_dir(os.path.dirname(path))
    if fieldnames is None:
        keys = []
        for row in rows:
            for k in row.keys():
                if k not in keys:
                    keys.append(k)
        fieldnames = keys
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(fieldnames))
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path


def path_length_open(points: Sequence[np.ndarray]) -> float:
    if len(points) < 2:
        return 0.0
    return float(sum(np.linalg.norm(points[i] - points[i - 1]) for i in range(1, len(points))))


def segment_lengths(points: Sequence[np.ndarray]) -> List[float]:
    return [float(np.linalg.norm(points[i] - points[i - 1])) for i in range(1, len(points))]


def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return v.copy()
    return v / n


def _target_face_ids(terrain, cfg) -> np.ndarray:
    c = terrain.face_centers
    mask = (
        (c[:, 0] >= float(cfg.x_min))
        & (c[:, 0] <= float(cfg.x_max))
        & (c[:, 1] >= float(cfg.y_min))
        & (c[:, 1] <= float(cfg.y_max))
    )
    return np.where(mask)[0]


def _line_of_sight_heightmap(terrain, p0: np.ndarray, p1: np.ndarray, samples: int = 24) -> bool:
    # Skip the final surface contact interval, otherwise every valid observation
    # would be rejected because the ray terminates on the terrain.
    for a in np.linspace(0.0, 0.95, samples):
        p = (1.0 - float(a)) * p0 + float(a) * p1
        if not terrain.is_inside_xy(float(p[0]), float(p[1])):
            return False
        terrain_h = terrain.query_height(float(p[0]), float(p[1]))
        if float(p[2]) < terrain_h + 0.05:
            return False
    return True


def _in_camera_fov(vp: Viewpoint, target: np.ndarray, horizontal_deg: float, vertical_deg: float) -> bool:
    forward = _normalize(vp.center - vp.position)
    direction = _normalize(target - vp.position)
    if float(np.dot(forward, direction)) <= 1e-6:
        return False

    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    right = np.cross(forward, world_up)
    if float(np.linalg.norm(right)) < 1e-6:
        right = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    right = _normalize(right)
    up = _normalize(np.cross(right, forward))

    x = float(np.dot(direction, forward))
    h = math.degrees(math.atan2(float(np.dot(direction, right)), x))
    v = math.degrees(math.atan2(float(np.dot(direction, up)), x))
    return abs(h) <= horizontal_deg * 0.5 and abs(v) <= vertical_deg * 0.5


def visible_face_ids(terrain, viewpoints: List[Viewpoint], cfg) -> List[int]:
    min_range = float(getattr(cfg, "sensor_min_range", 0.2))
    max_range = float(getattr(cfg, "sensor_max_range", 15.0))
    horizontal_fov = float(getattr(cfg, "sensor_horizontal_fov_deg", 87.0))
    vertical_fov = float(getattr(cfg, "sensor_vertical_fov_deg", 58.0))
    max_incidence = float(getattr(cfg, "sensor_max_incidence_deg", 65.0))
    min_normal_dot = math.cos(math.radians(max_incidence))

    visible = set()
    for face_id in _target_face_ids(terrain, cfg).tolist():
        center = terrain.face_centers[face_id]
        normal = _normalize(terrain.face_normals[face_id])
        for vp in viewpoints:
            ray_surface_to_view = vp.position - center
            dist = float(np.linalg.norm(ray_surface_to_view))
            if dist < min_range or dist > max_range:
                continue
            if abs(float(np.dot(_normalize(ray_surface_to_view), normal))) < min_normal_dot:
                continue
            if not _in_camera_fov(vp, center, horizontal_fov, vertical_fov):
                continue
            if not _line_of_sight_heightmap(terrain, vp.position, center):
                continue
            visible.add(int(face_id))
            break
    return sorted(visible)


def build_table_5_2_row(
    terrain,
    viewpoints: List[Viewpoint],
    order: List[int],
    cfg,
    total_duration_sec: float,
) -> Dict:
    ordered = [viewpoints[i] for i in order]
    points = [vp.position for vp in ordered]
    segs = segment_lengths(points)
    closed_len = path_length_open(points)
    if len(points) > 1:
        closed_len += float(np.linalg.norm(points[0] - points[-1]))
    target_faces = _target_face_ids(terrain, cfg)
    visible = visible_face_ids(terrain, ordered, cfg)
    target_count = int(len(target_faces))
    covered_count = int(len(visible))
    coverage_rate = float(covered_count / target_count) if target_count else 0.0
    return {
        "candidate_viewpoints": int(len(viewpoints)),
        "final_waypoints": int(len(order)),
        "target_coverage_cells": target_count,
        "covered_cells": covered_count,
        "uncovered_cells": int(target_count - covered_count),
        "coverage_rate": coverage_rate,
        "open_path_length_m": path_length_open(points),
        "closed_path_length_m": float(closed_len),
        "mean_segment_length_m": float(np.mean(segs)) if segs else 0.0,
        "max_segment_length_m": float(np.max(segs)) if segs else 0.0,
        "generation_time_sec": float(total_duration_sec),
    }


def build_table_5_3_rows(stage_records: Iterable[Dict]) -> List[Dict]:
    wanted = ["initial_order", "post_lkh", "post_2opt", "final_resample"]
    by_stage = {}
    for rec in stage_records:
        stage = rec.get("stage")
        if stage in wanted:
            if stage == "final_resample" or stage not in by_stage:
                by_stage[stage] = rec
    base = by_stage.get("initial_order", {})
    base_cost = float(base.get("tour_cost", 0.0) or 0.0)
    labels = {
        "initial_order": "初始SIP路径",
        "post_lkh": "LKH访问顺序优化后",
        "post_2opt": "LKH + 2-opt局部修正后",
        "final_resample": "LKH + 2-opt + 迭代重采样后",
    }
    rows = []
    for stage in wanted:
        rec = by_stage.get(stage)
        if rec is None:
            continue
        cost = float(rec["tour_cost"])
        improvement = 0.0 if base_cost <= 0.0 else (base_cost - cost) / base_cost
        rows.append(
            {
                "planning_stage": labels[stage],
                "stage": stage,
                "path_length_m": float(rec.get("path_length_open_m", 0.0)),
                "tour_cost": cost,
                "compute_time_sec": float(rec.get("duration_sec", 0.0)),
                "relative_improvement": float(improvement),
                "two_opt_swaps": int(rec.get("two_opt_swaps", 0)),
                "relocate_moves": int(rec.get("relocate_moves", 0)),
            }
        )
    return rows


def build_table_5_3_iteration_rows(iteration_records: Iterable[Dict]) -> List[Dict]:
    rows = []
    first_cost = None
    for rec in iteration_records:
        cost = float(rec.get("tour_cost", 0.0))
        if first_cost is None:
            first_cost = cost
        improvement = 0.0 if not first_cost or first_cost <= 0.0 else (first_cost - cost) / first_cost
        rows.append(
            {
                "iteration": int(rec.get("iteration", len(rows) + 1)),
                "solver": rec.get("solver", ""),
                "path_length_m": float(rec.get("path_length_open_m", 0.0)),
                "tour_cost": cost,
                "compute_time_sec": float(rec.get("duration_sec", 0.0)),
                "relative_improvement_vs_iter1": float(improvement),
                "feasible_edges": int(rec.get("feasible_edges", 0)),
                "total_edges": int(rec.get("total_edges", 0)),
            }
        )
    return rows


def save_fig_5_1_data(output_dir: str, terrain, viewpoints: List[Viewpoint], order: List[int]) -> None:
    fig_dir = ensure_dir(os.path.join(output_dir, "figures", "fig_5_1"))
    write_csv(
        os.path.join(fig_dir, "terrain_vertices.csv"),
        [
            {"vertex_id": i, "x": float(v[0]), "y": float(v[1]), "z": float(v[2])}
            for i, v in enumerate(terrain.vertices)
        ],
    )
    write_csv(
        os.path.join(fig_dir, "terrain_faces.csv"),
        [
            {"face_id": i, "v0": int(f[0]), "v1": int(f[1]), "v2": int(f[2])}
            for i, f in enumerate(terrain.faces)
        ],
    )
    write_csv(
        os.path.join(fig_dir, "tour_waypoints.csv"),
        [
            {
                "seq": seq,
                "viewpoint_index": int(idx),
                "x": float(viewpoints[idx].position[0]),
                "y": float(viewpoints[idx].position[1]),
                "z": float(viewpoints[idx].position[2]),
                "face_index": int(viewpoints[idx].face_index),
            }
            for seq, idx in enumerate(order)
        ],
    )


def save_stage_snapshots(output_dir: str, snapshots: Dict[str, Dict]) -> None:
    snap_dir = ensure_dir(os.path.join(output_dir, "stage_snapshots"))
    for name, data in snapshots.items():
        write_json(os.path.join(snap_dir, f"{name}.json"), data)
