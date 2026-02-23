import math
from typing import List, Optional

import numpy as np

from .models import Viewpoint


def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return v.copy()
    return v / n


def _tangent_basis(normal: np.ndarray):
    n = _normalize(normal)
    ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(float(np.dot(n, ref))) > 0.95:
        ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    t1 = _normalize(np.cross(n, ref))
    t2 = _normalize(np.cross(n, t1))
    return t1, t2


def _make_viewpoint(
    face_index: int,
    center: np.ndarray,
    normal: np.ndarray,
    standoff: float,
    lateral_offset: np.ndarray,
    cfg,
    terrain,
) -> Optional[Viewpoint]:
    n = _normalize(normal)
    preferred_clearance = max(float(cfg.terrain_clearance), float(getattr(cfg, "viewpoint_preferred_clearance", cfg.terrain_clearance)))
    # Try both sides of the surface normal and keep the safer one.
    candidates = []
    for sgn in (1.0, -1.0):
        p = center + sgn * n * standoff + lateral_offset
        p = p.astype(np.float64)
        if not terrain.is_inside_xy(float(p[0]), float(p[1])):
            continue
        p[2] = np.clip(p[2], cfg.z_min, cfg.z_max)
        # Push up to maintain preferred viewpoint clearance if needed.
        terrain_h = terrain.query_height(float(p[0]), float(p[1]))
        min_allowed_z = terrain_h + preferred_clearance
        if p[2] < min_allowed_z:
            p[2] = min_allowed_z
        if p[2] > cfg.z_max:
            continue
        clearance = float(p[2] - terrain_h)
        if clearance + 1e-9 < preferred_clearance:
            continue
        # Heading should face the inspected surface center.
        heading = math.atan2(float(center[1] - p[1]), float(center[0] - p[0]))
        candidates.append((clearance, p, heading))

    if not candidates:
        return None

    candidates.sort(key=lambda x: x[0], reverse=True)
    _, best_p, best_h = candidates[0]
    return Viewpoint(face_index=face_index, center=center.copy(), normal=n.copy(), position=best_p, heading=best_h, standoff=standoff)


def sample_initial_viewpoints(terrain, cfg) -> List[Viewpoint]:
    centers = terrain.face_centers
    normals = terrain.face_normals

    in_bounds = (
        (centers[:, 0] >= cfg.x_min)
        & (centers[:, 0] <= cfg.x_max)
        & (centers[:, 1] >= cfg.y_min)
        & (centers[:, 1] <= cfg.y_max)
    )

    center_ids = np.where(in_bounds)[0]
    if center_ids.size == 0:
        raise RuntimeError("No mesh faces inside configured XY bounds")

    stride = max(1, int(cfg.viewpoint_stride))
    center_ids = center_ids[::stride]

    if cfg.max_viewpoints and center_ids.size > cfg.max_viewpoints:
        idx = np.linspace(0, center_ids.size - 1, int(cfg.max_viewpoints), dtype=int)
        center_ids = center_ids[idx]

    selected: List[Viewpoint] = []
    min_spacing = float(cfg.min_viewpoint_spacing)

    for fi in center_ids.tolist():
        vp = _make_viewpoint(
            face_index=fi,
            center=centers[fi],
            normal=normals[fi],
            standoff=float(cfg.base_standoff_distance),
            lateral_offset=np.zeros(3, dtype=np.float64),
            cfg=cfg,
            terrain=terrain,
        )
        if vp is None:
            continue

        if selected and min_spacing > 0.0:
            d = [np.linalg.norm(vp.position - s.position) for s in selected]
            if float(np.min(d)) < min_spacing:
                continue

        selected.append(vp)

    if not selected:
        raise RuntimeError("Viewpoint sampling returned no feasible viewpoints")

    return selected


def generate_resample_candidates(vp: Viewpoint, cfg, terrain) -> List[Viewpoint]:
    candidates: List[Viewpoint] = []
    t1, t2 = _tangent_basis(vp.normal)

    base = float(vp.standoff)
    ds = float(cfg.candidate_standoff_delta)
    standoff_values = [max(0.5, base - ds), base, base + ds]

    jitter = float(cfg.candidate_lateral_jitter)
    lateral_offsets = [
        np.zeros(3, dtype=np.float64),
        t1 * jitter,
        -t1 * jitter,
        t2 * jitter,
        -t2 * jitter,
    ]

    for s in standoff_values:
        for lateral in lateral_offsets:
            cand = _make_viewpoint(
                face_index=vp.face_index,
                center=vp.center,
                normal=vp.normal,
                standoff=s,
                lateral_offset=lateral,
                cfg=cfg,
                terrain=terrain,
            )
            if cand is not None:
                candidates.append(cand)

    return candidates
