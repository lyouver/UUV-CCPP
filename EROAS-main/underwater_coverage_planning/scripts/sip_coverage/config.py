from dataclasses import dataclass, fields
from typing import Any, Dict, Optional
import os
import yaml


@dataclass
class PlannerConfig:
    # Coverage bounds.
    x_min: float = -65.0
    x_max: float = 57.0
    y_min: float = -55.0
    y_max: float = 50.0
    z_min: float = -25.0
    z_max: float = 2.0

    # Mesh crop bounds (context region loaded from terrain).
    crop_x_min: float = -65.0
    crop_x_max: float = 65.0
    crop_y_min: float = -60.0
    crop_y_max: float = 60.0

    # Terrain loading/scaling.
    mesh_scale_factor: float = 50.0
    mesh_z_offset: float = -25.0
    terrain_grid_resolution: float = 1.0

    # Viewpoint sampling.
    base_standoff_distance: float = 10.0
    viewpoint_stride: int = 10
    max_viewpoints: int = 80
    min_viewpoint_spacing: float = 6.0
    viewpoint_preferred_clearance: float = 0.8

    # SIP-like iterative resampling.
    resample_iterations: int = 3
    candidate_standoff_delta: float = 1.0
    candidate_lateral_jitter: float = 1.0

    # Solver settings.
    use_lkh: bool = True
    use_two_opt: bool = True
    open_tour: bool = True
    lkh_runs: int = 1
    lkh_scale: int = 1000
    fallback_large_cost: float = 1e6

    # UUV dynamics (adapted from aerial assumptions to underwater vehicle constraints).
    max_forward_speed: float = 0.8
    max_vertical_speed: float = 0.4
    max_yaw_rate: float = 0.35
    max_pitch_deg: float = 45.0
    yaw_cost_weight: float = 4.0

    # Feasibility checks against terrain.
    terrain_clearance: float = 0.3
    edge_sample_step: float = 2.0

    # Coverage visibility metrics for chapter-5 experiments.
    sensor_min_range: float = 0.2
    sensor_max_range: float = 15.0
    sensor_horizontal_fov_deg: float = 87.0
    sensor_vertical_fov_deg: float = 58.0
    sensor_max_incidence_deg: float = 65.0

    # Output.
    output_dir: Optional[str] = None
    auto_open_html: bool = True

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PlannerConfig":
        known = {f.name for f in fields(cls)}
        kwargs = {k: v for k, v in data.items() if k in known}
        return cls(**kwargs)

    @classmethod
    def from_yaml(cls, path: str) -> "PlannerConfig":
        with open(path, "r") as f:
            raw = yaml.safe_load(f) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"Config file must contain a mapping, got: {type(raw)}")
        return cls.from_dict(raw)

    def merge_cli_overrides(
        self,
        *,
        max_viewpoints: Optional[int] = None,
        resample_iterations: Optional[int] = None,
        use_lkh: Optional[bool] = None,
        auto_open_html: Optional[bool] = None,
    ) -> "PlannerConfig":
        if max_viewpoints is not None:
            self.max_viewpoints = int(max_viewpoints)
        if resample_iterations is not None:
            self.resample_iterations = int(resample_iterations)
        if use_lkh is not None:
            self.use_lkh = bool(use_lkh)
        if auto_open_html is not None:
            self.auto_open_html = bool(auto_open_html)
        return self

    def ensure_output_dir(self, script_dir: str) -> str:
        out = self.output_dir or os.path.join(script_dir, "guiji")
        out = os.path.abspath(out)
        os.makedirs(out, exist_ok=True)
        self.output_dir = out
        return out
