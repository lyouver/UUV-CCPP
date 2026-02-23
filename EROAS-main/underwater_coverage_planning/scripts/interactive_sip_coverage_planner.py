#!/usr/bin/env python3
"""Entry script for SIP-style UUV coverage planning."""

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SEARCH_DIRS = [SCRIPT_DIR]

try:
    import rospkg

    pkg_dir = rospkg.RosPack().get_path("underwater_coverage_planning")
    SEARCH_DIRS.append(os.path.join(pkg_dir, "scripts"))
except Exception:
    pass

for d in SEARCH_DIRS:
    if d and os.path.isdir(d) and d not in sys.path:
        sys.path.insert(0, d)

from sip_coverage import AdvancedSIPCoveragePlanner, PlannerConfig


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="SIP-style iterative coverage planner (UUV-adapted)")
    p.add_argument("--config", default="", help="Optional YAML config file")
    p.add_argument("--heightmap", default="", help="Path to terrain DAE file")

    p.add_argument("--max-viewpoints", type=int, default=None, help="Cap sampled viewpoints")
    p.add_argument("--iterations", type=int, default=None, help="Iterative resampling rounds")

    p.add_argument("--max-forward-speed", type=float, default=None, help="UUV max forward speed (m/s)")
    p.add_argument("--max-vertical-speed", type=float, default=None, help="UUV max vertical speed (m/s)")
    p.add_argument("--max-yaw-rate", type=float, default=None, help="UUV max yaw rate (rad/s)")
    p.add_argument("--max-pitch-deg", type=float, default=None, help="Max climb/sink pitch (deg)")
    p.add_argument("--terrain-clearance", type=float, default=None, help="Minimum terrain clearance (m)")

    p.add_argument("--no-lkh", action="store_true", help="Disable LKH, force heuristic solver")
    p.add_argument("--no-open", action="store_true", help="Do not auto-open HTML")
    return p


def _load_config(args) -> PlannerConfig:
    if args.config:
        cfg = PlannerConfig.from_yaml(args.config)
    else:
        cfg = PlannerConfig()

    cfg.merge_cli_overrides(
        max_viewpoints=args.max_viewpoints,
        resample_iterations=args.iterations,
        use_lkh=(False if args.no_lkh else None),
        auto_open_html=(False if args.no_open else None),
    )

    if args.max_forward_speed is not None:
        cfg.max_forward_speed = float(args.max_forward_speed)
    if args.max_vertical_speed is not None:
        cfg.max_vertical_speed = float(args.max_vertical_speed)
    if args.max_yaw_rate is not None:
        cfg.max_yaw_rate = float(args.max_yaw_rate)
    if args.max_pitch_deg is not None:
        cfg.max_pitch_deg = float(args.max_pitch_deg)
    if args.terrain_clearance is not None:
        cfg.terrain_clearance = float(args.terrain_clearance)

    return cfg


def main() -> int:
    print("=== SIP-style Interactive Coverage Planner (UUV) ===")
    parser = _build_arg_parser()
    args = parser.parse_args()

    try:
        cfg = _load_config(args)
        planner = AdvancedSIPCoveragePlanner(cfg=cfg, heightmap_path=(args.heightmap or None))
        planner.run()
        return 0
    except Exception as exc:
        print(f"Error: {exc}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
