#!/usr/bin/env python3
"""Generate stage-wise coverage visualization HTML files."""

import argparse
import json
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from sip_coverage.models import Viewpoint
from sip_coverage.terrain import load_terrain_model, resolve_heightmap_path
from sip_coverage.visualization import save_plan_visualization
from sip_coverage.config import PlannerConfig


STAGE_LABELS = {
    "initial_order": "初始SIP路径",
    "post_lkh": "LKH访问顺序优化后",
    "post_2opt": "LKH + 2-opt局部修正后",
    "final_resample": "LKH + 2-opt + 迭代重采样后",
}


def _load_snapshot(path: str):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _rebuild_viewpoints(snapshot):
    raw = snapshot.get("viewpoints", [])
    if not raw:
        raise RuntimeError(f"Snapshot {snapshot.get('stage')} has no viewpoints")
    max_idx = max(int(vp["index"]) for vp in raw)
    viewpoints = [None] * (max_idx + 1)
    for vp in raw:
        idx = int(vp["index"])
        viewpoints[idx] = Viewpoint(
            face_index=int(vp["face_index"]),
            center=np.array(vp["center"], dtype=np.float64),
            normal=np.array(vp["normal"], dtype=np.float64),
            position=np.array(vp["position"], dtype=np.float64),
            heading=float(vp["heading"]),
            standoff=float(vp["standoff"]),
        )
    missing = [i for i, vp in enumerate(viewpoints) if vp is None]
    if missing:
        raise RuntimeError(f"Snapshot {snapshot.get('stage')} misses viewpoint indices: {missing[:10]}")
    return viewpoints


def _safe_name(stage: str) -> str:
    return stage.replace("/", "_").replace(" ", "_")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="/home/tb/dave_ws/src/EROAS-main/underwater_coverage_planning/config/sip_uuv_planner.yaml",
    )
    parser.add_argument(
        "--input-dir",
        default="/home/tb/dave_ws/chapter5_outputs/revised_global/stage_snapshots",
        help="Directory containing exported stage snapshots",
    )
    parser.add_argument(
        "--output-dir",
        default="/home/tb/dave_ws/chapter5_outputs/revised_global/stage_visualizations",
        help="Directory for generated HTML files",
    )
    parser.add_argument("--heightmap", default="", help="Optional terrain DAE override")
    parser.add_argument("--no-open", action="store_true", help="Do not open generated HTML files")
    args = parser.parse_args()

    cfg = PlannerConfig.from_yaml(args.config)
    terrain = load_terrain_model(resolve_heightmap_path(args.heightmap or None), cfg)
    input_dir = os.path.abspath(args.input_dir)
    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    wanted = ["initial_order", "post_lkh", "post_2opt", "final_resample"]
    generated = []
    for stage in wanted:
        path = os.path.join(input_dir, f"{stage}.json")
        if not os.path.isfile(path):
            continue
        snapshot = _load_snapshot(path)
        viewpoints = _rebuild_viewpoints(snapshot)
        order = [int(i) for i in snapshot["order"]]
        title = STAGE_LABELS.get(stage, stage)
        html_path = os.path.join(output_dir, f"coverage_visualization_{_safe_name(stage)}.html")
        save_plan_visualization(
            terrain,
            viewpoints,
            order,
            html_path,
            auto_open=not bool(args.no_open),
        )
        generated.append((stage, title, html_path))

    if not generated:
        raise RuntimeError(f"No stage snapshots found in {input_dir}")

    readme_path = os.path.join(output_dir, "README.md")
    with open(readme_path, "w", encoding="utf-8") as f:
        f.write("# Stage HTML Visualizations\n\n")
        for stage, title, html_path in generated:
            f.write(f"- `{stage}`: {title} -> `{html_path}`\n")

    for _, title, html_path in generated:
        print(f"{title}: {html_path}")


if __name__ == "__main__":
    raise SystemExit(main())
