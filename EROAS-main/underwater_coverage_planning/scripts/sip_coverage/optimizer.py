from dataclasses import dataclass
import time
from typing import Dict, List, Tuple

import numpy as np

from .dynamics import UUVTransitionModel, build_directed_cost_matrix
from .export import tour_path_length
from .models import Viewpoint
from .tour_solver import solve_tour_with_stages
from .viewpoints import generate_resample_candidates


@dataclass
class PlanResult:
    viewpoints: List[Viewpoint]
    order: List[int]
    cost: float
    solver_name: str
    feasible_edges: int
    total_edges: int
    stage_records: List[Dict]
    iteration_records: List[Dict]
    stage_snapshots: Dict[str, Dict]


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
    all_stage_records: List[Dict] = []
    iteration_records: List[Dict] = []
    stage_snapshots: Dict[str, Dict] = {}

    num_iters = max(int(cfg.resample_iterations), 1)
    for it in range(num_iters):
        iter_t0 = time.perf_counter()
        cost_matrix, metrics = build_directed_cost_matrix(current, model, terrain, cfg)
        order, tour_cost, solver_name, solver_stages = solve_tour_with_stages(cost_matrix, cfg)
        feasible_edges, total_edges = _count_feasible_on_tour(order, metrics)
        iter_duration = time.perf_counter() - iter_t0

        print(
            f"Iteration {it + 1}/{num_iters}: solver={solver_name}, "
            f"tour_cost={tour_cost:.2f}, feasible_edges={feasible_edges}/{total_edges}"
        )

        for stage in solver_stages:
            stage_order = stage["order"]
            rec = {
                "iteration": it + 1,
                "stage": stage["stage"],
                "solver": stage["solver"],
                "tour_cost": float(stage["cost"]),
                "duration_sec": float(stage["duration_sec"]),
                "two_opt_swaps": int(stage.get("two_opt_swaps", 0)),
                "relocate_moves": int(stage.get("relocate_moves", 0)),
                "path_length_closed_m": float(tour_path_length(current, stage_order)),
                "path_length_open_m": float(_open_path_length(current, stage_order)),
                "waypoints": len(stage_order),
            }
            all_stage_records.append(rec)
            if it == 0 and stage["stage"] not in stage_snapshots:
                stage_snapshots[stage["stage"]] = _make_snapshot(current, stage_order, rec)

        iter_rec = {
            "iteration": it + 1,
            "solver": solver_name,
            "tour_cost": float(tour_cost),
            "duration_sec": float(iter_duration),
            "feasible_edges": int(feasible_edges),
            "total_edges": int(total_edges),
            "path_length_closed_m": float(tour_path_length(current, order)),
            "path_length_open_m": float(_open_path_length(current, order)),
        }
        iteration_records.append(iter_rec)

        result = PlanResult(
            viewpoints=[_clone_viewpoint(v) for v in current],
            order=order[:],
            cost=float(tour_cost),
            solver_name=solver_name,
            feasible_edges=feasible_edges,
            total_edges=total_edges,
            stage_records=all_stage_records[:],
            iteration_records=iteration_records[:],
            stage_snapshots=stage_snapshots.copy(),
        )

        if best_result is None or result.cost < best_result.cost:
            best_result = result

        if it == num_iters - 1:
            break

        current = _resample_once(current, order, model, terrain, cfg)

    if best_result is not None:
        final_rec = {
            "iteration": len(iteration_records),
            "stage": "final_resample",
            "solver": best_result.solver_name,
            "tour_cost": float(best_result.cost),
            "duration_sec": 0.0,
            "two_opt_swaps": 0,
            "relocate_moves": 0,
            "path_length_closed_m": float(tour_path_length(best_result.viewpoints, best_result.order)),
            "path_length_open_m": float(_open_path_length(best_result.viewpoints, best_result.order)),
            "waypoints": len(best_result.order),
        }
        best_result.stage_records = all_stage_records[:] + [final_rec]
        best_result.iteration_records = iteration_records[:]
        best_result.stage_snapshots = stage_snapshots.copy()
        best_result.stage_snapshots["final_resample"] = _make_snapshot(best_result.viewpoints, best_result.order, final_rec)

    return best_result


def _open_path_length(viewpoints: List[Viewpoint], order: List[int]) -> float:
    if len(order) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(order)):
        a = viewpoints[order[i - 1]].position
        b = viewpoints[order[i]].position
        total += float(np.linalg.norm(b - a))
    return total


def _make_snapshot(viewpoints: List[Viewpoint], order: List[int], record: Dict) -> Dict:
    return {
        "stage": record["stage"],
        "iteration": int(record["iteration"]),
        "solver": record["solver"],
        "tour_cost": float(record["tour_cost"]),
        "path_length_open_m": float(record["path_length_open_m"]),
        "path_length_closed_m": float(record["path_length_closed_m"]),
        "order": [int(i) for i in order],
        "viewpoints": [
            {
                "index": int(i),
                "face_index": int(viewpoints[i].face_index),
                "center": [float(x) for x in viewpoints[i].center.tolist()],
                "normal": [float(x) for x in viewpoints[i].normal.tolist()],
                "position": [float(x) for x in viewpoints[i].position.tolist()],
                "heading": float(viewpoints[i].heading),
                "standoff": float(viewpoints[i].standoff),
            }
            for i in order
        ],
    }
