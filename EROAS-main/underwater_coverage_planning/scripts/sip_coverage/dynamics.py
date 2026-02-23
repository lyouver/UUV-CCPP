import math
from typing import List, Tuple

import numpy as np

from .models import EdgeMetrics, Viewpoint

class UUVTransitionModel:
    """Geometry-based transition model with terrain-only feasibility checks."""

    def __init__(self, cfg):
        _ = cfg

    def transition_metrics(self, a: Viewpoint, b: Viewpoint, terrain, cfg) -> EdgeMetrics:
        d = b.position - a.position
        dx, dy, dz = float(d[0]), float(d[1]), float(d[2])
        euclidean = math.sqrt(dx * dx + dy * dy + dz * dz)

        if euclidean < 1e-6:
            return EdgeMetrics(feasible=False, travel_time=math.inf, turn_time=math.inf, cost=math.inf, min_clearance=-math.inf)

        edge_ok, min_clear = terrain.edge_min_clearance(a.position, b.position, step=float(cfg.edge_sample_step))
        if not edge_ok:
            return EdgeMetrics(feasible=False, travel_time=math.inf, turn_time=0.0, cost=math.inf, min_clearance=-math.inf)

        if min_clear < float(cfg.terrain_clearance):
            return EdgeMetrics(feasible=False, travel_time=math.inf, turn_time=0.0, cost=math.inf, min_clearance=min_clear)

        return EdgeMetrics(
            feasible=True,
            travel_time=euclidean,
            turn_time=0.0,
            cost=euclidean,
            min_clearance=min_clear,
        )


def build_directed_cost_matrix(
    viewpoints: List[Viewpoint],
    transition_model: UUVTransitionModel,
    terrain,
    cfg,
) -> Tuple[np.ndarray, List[List[EdgeMetrics]]]:
    n = len(viewpoints)
    if n < 2:
        raise RuntimeError("Need at least two viewpoints for tour optimization")

    cost = np.full((n, n), float(cfg.fallback_large_cost), dtype=np.float64)
    metrics: List[List[EdgeMetrics]] = [
        [EdgeMetrics(False, math.inf, math.inf, math.inf, -math.inf) for _ in range(n)] for _ in range(n)
    ]

    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            m = transition_model.transition_metrics(viewpoints[i], viewpoints[j], terrain, cfg)
            metrics[i][j] = m
            if m.feasible and np.isfinite(m.cost):
                cost[i, j] = m.cost

    return cost, metrics
