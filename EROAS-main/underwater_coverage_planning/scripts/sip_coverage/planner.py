import os
import time
from datetime import datetime
from typing import Dict, Optional

from .config import PlannerConfig
from .experiment_export import export_plan_run
from .export import build_ordered_waypoints, save_ros_waypoints_yaml, tour_path_length
from .optimizer import optimize_coverage
from .terrain import load_terrain_model, resolve_heightmap_path
from .viewpoints import sample_initial_viewpoints
from .visualization import save_plan_visualization


class AdvancedSIPCoveragePlanner:
    """SIP-style global coverage planner adapted for UUV constraints."""

    def __init__(self, cfg: PlannerConfig, heightmap_path: Optional[str] = None):
        self.cfg = cfg
        self.heightmap_path = resolve_heightmap_path(heightmap_path)

        script_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        self.output_dir = self.cfg.ensure_output_dir(script_dir)

    def run(self, export_run_dir: Optional[str] = None, export_stages: bool = False) -> Dict[str, str]:
        total_t0 = time.perf_counter()
        print("Loading terrain:", self.heightmap_path)
        terrain = load_terrain_model(self.heightmap_path, self.cfg)
        print(f"Terrain loaded: vertices={len(terrain.vertices)}, faces={len(terrain.faces)}, bounds={terrain.bounds}")

        print("Sampling initial viewpoints...")
        viewpoints = sample_initial_viewpoints(terrain, self.cfg)
        print(f"Initial viewpoints: {len(viewpoints)}")

        print("Running SIP-style iterative optimization...")
        result = optimize_coverage(viewpoints, terrain, self.cfg)

        ordered_waypoints = build_ordered_waypoints(result.viewpoints, result.order, self.cfg)
        path_len = tour_path_length(result.viewpoints, result.order)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self.output_dir, f"trajectory_{stamp}")
        yaml_file = save_ros_waypoints_yaml(base, ordered_waypoints, self.cfg)

        html_file = os.path.join(self.output_dir, f"coverage_visualization_{stamp}.html")
        save_plan_visualization(
            terrain,
            result.viewpoints,
            result.order,
            html_file,
            auto_open=bool(self.cfg.auto_open_html),
        )

        print("Optimization summary:")
        print(f"  solver: {result.solver_name}")
        print(f"  best_cost: {result.cost:.2f}")
        print(f"  path_length(closed): {path_len:.2f} m")
        print(f"  feasible_edges: {result.feasible_edges}/{result.total_edges}")
        print(f"  waypoints: {len(ordered_waypoints)}")
        print(f"Waypoints saved to: {yaml_file}")
        print(f"Visualization saved to: {html_file}")

        outputs = {
            "yaml": yaml_file,
            "html": html_file,
            "solver": result.solver_name,
        }

        if export_run_dir:
            exported = export_plan_run(
                export_dir=os.path.abspath(export_run_dir),
                terrain=terrain,
                result=result,
                cfg=self.cfg,
                yaml_file=yaml_file,
                html_file=html_file,
                total_duration_sec=float(time.perf_counter() - total_t0),
                export_stages=bool(export_stages),
            )
            outputs.update(exported)
            print(f"Experiment export saved to: {os.path.abspath(export_run_dir)}")

        return outputs
