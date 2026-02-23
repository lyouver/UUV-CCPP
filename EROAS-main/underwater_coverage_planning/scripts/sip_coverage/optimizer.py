from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from .dynamics import UUVTransitionModel, build_directed_cost_matrix
from .models import Viewpoint
from .tour_solver import solve_tour
from .viewpoints import generate_resample_candidates


@dataclass
class PlanResult:
    viewpoints: List[Viewpoint]
    order: List[int]
    cost: float
    solver_name: str
    feasible_edges: int
    total_edges: int


def _clone_viewpoint(v: Viewpoint) -> Viewpoint:
    return Viewpoint(
        face_index=int(v.face_index),
        center=v.center.copy(),
        normal=v.normal.copy(),
        position=v.position.copy(),
        heading=float(v.heading),
        standoff=float(v.standoff),
    )


def _count_feasible_on_tour(order: List[int], metrics) -> Tuple[int, int]:
    if not order:
        return 0, 0
    n = len(order)
    good = 0
    for i in range(n):
        a = order[i]
        b = order[(i + 1) % n]
        if metrics[a][b].feasible:
            good += 1
    return good, n


def _resample_once(
    viewpoints: List[Viewpoint],
    order: List[int],
    model: UUVTransitionModel,
    terrain,
    cfg,
) -> List[Viewpoint]:
    new_vps = [_clone_viewpoint(v) for v in viewpoints]
    n = len(order)

    for k, idx in enumerate(order):
        prev_idx = order[(k - 1) % n]
        next_idx = order[(k + 1) % n]

        base_vp = new_vps[idx]
        base_prev = model.transition_metrics(new_vps[prev_idx], base_vp, terrain, cfg)
        base_next = model.transition_metrics(base_vp, new_vps[next_idx], terrain, cfg)
        if base_prev.feasible and base_next.feasible:
            best_cost = base_prev.cost + base_next.cost
            best_vp = base_vp
        else:
            best_cost = float(cfg.fallback_large_cost)
            best_vp = base_vp

        for cand in generate_resample_candidates(base_vp, cfg, terrain):
            prev_m = model.transition_metrics(new_vps[prev_idx], cand, terrain, cfg)
            next_m = model.transition_metrics(cand, new_vps[next_idx], terrain, cfg)
            if not (prev_m.feasible and next_m.feasible):
                continue
            local = prev_m.cost + next_m.cost
            if local + 1e-9 < best_cost:
                best_cost = local
                best_vp = cand

        new_vps[idx] = best_vp

    return new_vps


def optimize_coverage(viewpoints: List[Viewpoint], terrain, cfg) -> PlanResult:
    if len(viewpoints) < 2:
        raise RuntimeError("Need at least two viewpoints")

    model = UUVTransitionModel(cfg)
    current = [_clone_viewpoint(v) for v in viewpoints]

    best_result = None

    num_iters = max(int(cfg.resample_iterations), 1)
    for it in range(num_iters):
        cost_matrix, metrics = build_directed_cost_matrix(current, model, terrain, cfg)
        order, tour_cost, solver_name = solve_tour(cost_matrix, cfg)
        feasible_edges, total_edges = _count_feasible_on_tour(order, metrics)

        print(
            f"Iteration {it + 1}/{num_iters}: solver={solver_name}, "
            f"tour_cost={tour_cost:.2f}, feasible_edges={feasible_edges}/{total_edges}"
        )

        result = PlanResult(
            viewpoints=[_clone_viewpoint(v) for v in current],
            order=order[:],
            cost=float(tour_cost),
            solver_name=solver_name,
            feasible_edges=feasible_edges,
            total_edges=total_edges,
        )

        if best_result is None or result.cost < best_result.cost:
            best_result = result

        if it == num_iters - 1:
            break

        current = _resample_once(current, order, model, terrain, cfg)

    return best_result
