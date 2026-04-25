import os
import subprocess
import tempfile
import time
from typing import Dict, List, Optional, Tuple

import numpy as np


def _resolve_lkh_exec() -> Optional[str]:
    candidates = [
        os.environ.get("LKH_EXEC", ""),
        "/home/tb/dave_ws/src/LKH/LKH",
        "/home/tb/dave_ws/src/EROAS-main/LKH/LKH",
    ]
    for p in candidates:
        if p and os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    return None


def _route_cost(cost: np.ndarray, order: List[int]) -> float:
    total = 0.0
    n = len(order)
    for i in range(n - 1):
        total += float(cost[order[i], order[i + 1]])
    total += float(cost[order[-1], order[0]])
    return total


def _path_cost_open(cost: np.ndarray, order: List[int]) -> float:
    total = 0.0
    n = len(order)
    for i in range(n - 1):
        total += float(cost[order[i], order[i + 1]])
    return total


def _rotate_to_open_tour(cost: np.ndarray, order: List[int]) -> List[int]:
    """Rotate cycle so the largest edge is the implicit closure edge."""
    n = len(order)
    if n < 3:
        return order
    edge_costs = [float(cost[order[i], order[(i + 1) % n]]) for i in range(n)]
    k = int(np.argmax(edge_costs))
    return order[(k + 1) :] + order[: (k + 1)]


def _nearest_neighbor_directed(cost: np.ndarray, start: int = 0) -> List[int]:
    n = cost.shape[0]
    remaining = set(range(n))
    current = int(start)
    order = [current]
    remaining.remove(current)

    while remaining:
        next_idx = min(remaining, key=lambda j: float(cost[current, j]))
        order.append(next_idx)
        remaining.remove(next_idx)
        current = next_idx
    return order


def _path_max_segment(cost: np.ndarray, order: List[int], open_tour: bool) -> float:
    if len(order) < 2:
        return 0.0
    edges = [float(cost[order[i], order[i + 1]]) for i in range(len(order) - 1)]
    if not open_tour:
        edges.append(float(cost[order[-1], order[0]]))
    return max(edges) if edges else 0.0


def _candidate_better(
    cand_cost: float,
    cand_max_seg: float,
    best_cost: float,
    best_max_seg: float,
    tol: float = 1e-9,
) -> bool:
    if cand_cost + tol < best_cost:
        return True
    if abs(cand_cost - best_cost) <= tol and cand_max_seg + tol < best_max_seg:
        return True
    return False


def _two_opt_directed_with_stats(
    cost: np.ndarray,
    order: List[int],
    *,
    open_tour: bool,
    max_passes: int = 10,
) -> Tuple[List[int], Dict[str, int]]:
    n = len(order)
    if n < 4:
        return order, {"passes": 0, "swaps": 0}

    best = order[:]
    objective = _path_cost_open if open_tour else _route_cost
    best_cost = float(objective(cost, best))
    best_max_seg = _path_max_segment(cost, best, open_tour)

    passes = 0
    swaps = 0
    improved = True
    while improved and passes < max_passes:
        passes += 1
        improved = False
        i_start = 0 if open_tour else 1
        j_stop = n if open_tour else (n - 1)
        for i in range(i_start, n - 2):
            for j in range(i + 1, j_stop):
                cand = best[:i] + list(reversed(best[i : j + 1])) + best[j + 1 :]
                c = float(objective(cost, cand))
                c_max_seg = _path_max_segment(cost, cand, open_tour)
                if _candidate_better(c, c_max_seg, best_cost, best_max_seg):
                    best = cand
                    best_cost = c
                    best_max_seg = c_max_seg
                    improved = True
                    swaps += 1
    return best, {"passes": passes, "swaps": swaps}


def _relocate_directed_with_stats(
    cost: np.ndarray,
    order: List[int],
    *,
    open_tour: bool,
    max_passes: int = 6,
) -> Tuple[List[int], Dict[str, int]]:
    n = len(order)
    if n < 4:
        return order, {"passes": 0, "moves": 0}

    objective = _path_cost_open if open_tour else _route_cost
    best = order[:]
    best_cost = float(objective(cost, best))
    best_max_seg = _path_max_segment(cost, best, open_tour)
    passes = 0
    moves = 0
    improved = True
    while improved and passes < max_passes:
        passes += 1
        improved = False
        remove_start = 0 if open_tour else 1
        for i in range(remove_start, n):
            node = best[i]
            remaining = best[:i] + best[i + 1 :]
            insert_start = 0 if open_tour else 1
            for j in range(insert_start, len(remaining) + 1):
                if j == i:
                    continue
                cand = remaining[:j] + [node] + remaining[j:]
                c = float(objective(cost, cand))
                c_max_seg = _path_max_segment(cost, cand, open_tour)
                if _candidate_better(c, c_max_seg, best_cost, best_max_seg):
                    best = cand
                    best_cost = c
                    best_max_seg = c_max_seg
                    improved = True
                    moves += 1
                    break
            if improved:
                break
    return best, {"passes": passes, "moves": moves}


def _write_atsp_problem(path: str, int_cost: np.ndarray) -> None:
    n = int_cost.shape[0]
    with open(path, "w") as f:
        f.write("NAME: sip_uuv_coverage\n")
        f.write("TYPE: ATSP\n")
        f.write(f"DIMENSION: {n}\n")
        f.write("EDGE_WEIGHT_TYPE: EXPLICIT\n")
        f.write("EDGE_WEIGHT_FORMAT: FULL_MATRIX\n")
        f.write("EDGE_WEIGHT_SECTION\n")
        for i in range(n):
            f.write(" ".join(str(int(x)) for x in int_cost[i].tolist()) + "\n")
        f.write("EOF\n")


def _parse_lkh_tour(path: str, n: int) -> Optional[List[int]]:
    if not os.path.isfile(path):
        return None

    sol = []
    in_section = False
    with open(path, "r") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            if s == "TOUR_SECTION":
                in_section = True
                continue
            if not in_section:
                continue
            if s.startswith("-1") or s == "EOF":
                break
            try:
                sol.append(int(s))
            except ValueError:
                continue

    if not sol:
        return None

    # LKH is 1-based. For ATSP, the output may contain extra transformed nodes;
    # keep only valid original node ids and de-duplicate while preserving order.
    order = []
    seen = set()
    for x in sol:
        if 1 <= x <= n:
            idx = x - 1
            if idx not in seen:
                seen.add(idx)
                order.append(idx)
        if len(order) == n:
            break

    if len(order) != n:
        return None

    # Rotate so the sequence starts from node 0 (deterministic output).
    if 0 in order:
        k = order.index(0)
        order = order[k:] + order[:k]
    return order


def _stage_cost(cost: np.ndarray, order: List[int], cfg) -> float:
    if bool(getattr(cfg, "open_tour", False)):
        return _path_cost_open(cost, order)
    return _route_cost(cost, order)


def _stage_order(cost: np.ndarray, order: List[int], cfg) -> List[int]:
    if bool(getattr(cfg, "open_tour", False)):
        return _rotate_to_open_tour(cost, order)
    return order[:]


def solve_tour_with_stages(cost: np.ndarray, cfg) -> Tuple[List[int], float, str, List[Dict]]:
    n = cost.shape[0]
    if n < 2:
        raise RuntimeError("Tour solver requires at least 2 nodes")

    finite = np.where(np.isfinite(cost), cost, float(cfg.fallback_large_cost))
    finite = np.clip(finite, 0.0, float(cfg.fallback_large_cost))

    stages: List[Dict] = []
    t0 = time.perf_counter()
    order = _nearest_neighbor_directed(finite, start=0)
    stage_order = _stage_order(finite, order, cfg)
    stages.append(
        {
            "stage": "initial_order",
            "solver": "nearest",
            "cost": float(_stage_cost(finite, stage_order, cfg)),
            "duration_sec": float(time.perf_counter() - t0),
            "two_opt_swaps": 0,
            "order": stage_order[:],
        }
    )

    solver_name = "nearest"

    if cfg.use_lkh:
        lkh_exec = _resolve_lkh_exec()
        if lkh_exec is None:
            raise RuntimeError("LKH is required (`use_lkh: true`) but executable was not found.")

        t0 = time.perf_counter()
        scale = max(int(cfg.lkh_scale), 1)
        int_cost = np.rint(finite * scale).astype(np.int64)
        np.fill_diagonal(int_cost, int(float(cfg.fallback_large_cost) * scale))

        with tempfile.TemporaryDirectory(prefix="sip_uuv_lkh_") as td:
            tsp_path = os.path.join(td, "problem.atsp")
            par_path = os.path.join(td, "problem.par")
            tour_path = os.path.join(td, "output.tour")
            _write_atsp_problem(tsp_path, int_cost)

            with open(par_path, "w") as f:
                f.write(f"PROBLEM_FILE = {tsp_path}\n")
                f.write(f"OUTPUT_TOUR_FILE = {tour_path}\n")
                f.write(f"RUNS = {max(int(cfg.lkh_runs), 1)}\n")

            subprocess.run([lkh_exec, par_path], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            order = _parse_lkh_tour(tour_path, n)
            if order is None:
                raise RuntimeError("LKH did not return a valid ATSP tour.")
            solver_name = "lkh-atsp"
        stage_order = _stage_order(finite, order, cfg)
        stages.append(
            {
                "stage": "post_lkh",
                "solver": solver_name,
                "cost": float(_stage_cost(finite, stage_order, cfg)),
                "duration_sec": float(time.perf_counter() - t0),
                "two_opt_swaps": 0,
                "order": stage_order[:],
            }
        )

    if cfg.use_two_opt:
        t0 = time.perf_counter()
        if bool(getattr(cfg, "open_tour", False)):
            order = _rotate_to_open_tour(finite, order)
        order, two_opt_stats = _two_opt_directed_with_stats(
            finite,
            order,
            open_tour=bool(getattr(cfg, "open_tour", False)),
        )
        order, relocate_stats = _relocate_directed_with_stats(
            finite,
            order,
            open_tour=bool(getattr(cfg, "open_tour", False)),
        )
        stage_order = _stage_order(finite, order, cfg)
        stages.append(
            {
                "stage": "post_2opt",
                "solver": f"{solver_name}+2opt",
                "cost": float(_stage_cost(finite, stage_order, cfg)),
                "duration_sec": float(time.perf_counter() - t0),
                "two_opt_swaps": int(two_opt_stats["swaps"]),
                "relocate_moves": int(relocate_stats["moves"]),
                "order": stage_order[:],
            }
        )
        solver_name = f"{solver_name}+2opt"

    if bool(getattr(cfg, "open_tour", False)):
        order = _rotate_to_open_tour(finite, order)
        total = _path_cost_open(finite, order)
    else:
        total = _route_cost(finite, order)
    return order, total, solver_name, stages


def solve_tour(cost: np.ndarray, cfg) -> Tuple[List[int], float, str]:
    order, total, solver_name, _ = solve_tour_with_stages(cost, cfg)
    return order, total, solver_name
