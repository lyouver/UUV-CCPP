#!/usr/bin/env python3
"""Run chapter-5 offline global coverage experiments and aggregate tables."""

import argparse
import copy
import csv
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from sip_coverage import AdvancedSIPCoveragePlanner, PlannerConfig
from sip_coverage.metrics import write_csv


def _read_single_csv(path):
    with open(path, "r", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError(f"No rows in {path}")
    return rows[0]


def _variant_configs(base_cfg):
    variants = [
        ("initial_order", "仅初始视点顺序", False, False, 1, "作为未优化基准"),
        ("post_lkh", "加入LKH访问顺序优化", True, False, 1, "验证全局访问顺序优化作用"),
        ("post_2opt", "加入2-opt局部修正", True, True, 1, "验证局部连接修正作用"),
        (
            "full_method",
            "完整方法",
            True,
            True,
            max(int(base_cfg.resample_iterations), 1),
            "验证串行优化链综合效果",
        ),
    ]
    for key, label, use_lkh, use_two_opt, iterations, role in variants:
        cfg = copy.deepcopy(base_cfg)
        cfg.use_lkh = bool(use_lkh)
        cfg.use_two_opt = bool(use_two_opt)
        cfg.resample_iterations = int(iterations)
        yield key, label, cfg, role


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="Planner YAML config")
    parser.add_argument("--heightmap", default="", help="Terrain DAE path")
    parser.add_argument("--output-dir", default="/home/tb/dave_ws/chapter5_outputs/global_batch")
    parser.add_argument("--max-viewpoints", type=int, default=None)
    parser.add_argument("--iterations", type=int, default=None)
    args = parser.parse_args()

    cfg = PlannerConfig.from_yaml(args.config) if args.config else PlannerConfig()
    if args.max_viewpoints is not None:
        cfg.max_viewpoints = int(args.max_viewpoints)
    if args.iterations is not None:
        cfg.resample_iterations = int(args.iterations)
    cfg.auto_open_html = False

    out_root = os.path.abspath(args.output_dir)
    os.makedirs(out_root, exist_ok=True)

    table_5_4_rows = []
    for key, label, run_cfg, role in _variant_configs(cfg):
        run_dir = os.path.join(out_root, "runs", key)
        run_cfg.output_dir = os.path.join(run_dir, "waypoints")
        planner = AdvancedSIPCoveragePlanner(cfg=run_cfg, heightmap_path=(args.heightmap or None))
        outputs = planner.run(export_run_dir=run_dir, export_stages=True)
        row_5_2 = _read_single_csv(outputs["table_5_2"])
        table_5_4_rows.append(
            {
                "comparison": label,
                "variant": key,
                "coverage_rate": row_5_2["coverage_rate"],
                "path_length_m": row_5_2["open_path_length_m"],
                "max_segment_length_m": row_5_2["max_segment_length_m"],
                "planning_time_sec": row_5_2["generation_time_sec"],
                "conclusion_role": role,
            }
        )

    tables_dir = os.path.join(out_root, "tables")
    write_csv(
        os.path.join(tables_dir, "table_5_4.csv"),
        table_5_4_rows,
        [
            "comparison",
            "variant",
            "coverage_rate",
            "path_length_m",
            "max_segment_length_m",
            "planning_time_sec",
            "conclusion_role",
        ],
    )
    with open(os.path.join(out_root, "README.md"), "w") as f:
        f.write(
            "# Chapter 5 Global Batch Output\n\n"
            "- `tables/table_5_4.csv`: global planning ablation table.\n"
            "- `runs/*/tables/table_5_2.csv`: per-variant coverage statistics.\n"
            "- `runs/*/tables/table_5_3.csv`: per-variant optimizer stage statistics.\n"
            "- `runs/*/figures`: source data for chapter-5 figures.\n"
        )
    print(f"Global chapter-5 batch complete: {out_root}")
    print(f"table_5_4: {os.path.join(tables_dir, 'table_5_4.csv')}")


if __name__ == "__main__":
    main()
