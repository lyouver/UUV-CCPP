import os
import subprocess
import tempfile
from typing import List, Optional, Tuple

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


def _two_opt_directed(cost: np.ndarray, order: List[int], max_passes: int = 6) -> List[int]:
    n = len(order)
    if n < 4:
        return order

    best = order[:]
    best_cost = _route_cost(cost, best)

    passes = 0
    improved = True
    while improved and passes < max_passes:
        passes += 1
        improved = False
        for i in range(1, n - 2):
            for j in range(i + 1, n - 1):
                cand = best[:i] + list(reversed(best[i : j + 1])) + best[j + 1 :]
                c = _route_cost(cost, cand)
                if c + 1e-9 < best_cost:
                    best = cand
                    best_cost = c
                    improved = True
    return best


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


def solve_tour(cost: np.ndarray, cfg) -> Tuple[List[int], float, str]:
    n = cost.shape[0]
    if n < 2:
        raise RuntimeError("Tour solver requires at least 2 nodes")

    finite = np.where(np.isfinite(cost), cost, float(cfg.fallback_large_cost))
    finite = np.clip(finite, 0.0, float(cfg.fallback_large_cost))

    order = None
    solver_name = "nearest+2opt"

    if cfg.use_lkh:
        lkh_exec = _resolve_lkh_exec()
        if lkh_exec is None:
            raise RuntimeError("LKH is required (`use_lkh: true`) but executable was not found.")

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

    if order is None:
        order = _nearest_neighbor_directed(finite, start=0)

    if cfg.use_two_opt:
        order = _two_opt_directed(finite, order)

    if bool(getattr(cfg, "open_tour", False)):
        order = _rotate_to_open_tour(finite, order)
        total = _path_cost_open(finite, order)
    else:
        total = _route_cost(finite, order)
    return order, total, solver_name
