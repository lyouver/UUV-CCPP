from dataclasses import dataclass
import numpy as np


@dataclass
class Viewpoint:
    """Candidate inspection viewpoint anchored to one mesh face."""

    face_index: int
    center: np.ndarray
    normal: np.ndarray
    position: np.ndarray
    heading: float
    standoff: float


@dataclass
class EdgeMetrics:
    """Feasibility and timing metrics for a transition edge."""

    feasible: bool
    travel_time: float
    turn_time: float
    cost: float
    min_clearance: float
