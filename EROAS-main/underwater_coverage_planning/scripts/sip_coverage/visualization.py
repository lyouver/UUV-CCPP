from typing import List

import numpy as np
import plotly.graph_objects as go
import plotly.offline as pyo

from .models import Viewpoint


def save_plan_visualization(terrain, viewpoints: List[Viewpoint], order: List[int], html_path: str, auto_open: bool = True) -> None:
    fig = go.Figure()

    mesh = terrain.mesh
    fig.add_trace(
        go.Mesh3d(
            x=mesh.vertices[:, 0],
            y=mesh.vertices[:, 1],
            z=mesh.vertices[:, 2],
            i=mesh.faces[:, 0],
            j=mesh.faces[:, 1],
            k=mesh.faces[:, 2],
            intensity=mesh.vertices[:, 2],
            colorscale="Viridis",
            opacity=1.0,
            name="Terrain",
            showscale=True,
        )
    )

    if order:
        path_pts = np.array([viewpoints[i].position for i in order] + [viewpoints[order[0]].position])
        fig.add_trace(
            go.Scatter3d(
                x=path_pts[:, 0],
                y=path_pts[:, 1],
                z=path_pts[:, 2],
                mode="lines+markers",
                line=dict(color="orange", width=5),
                marker=dict(size=3, color="orange"),
                name="Inspection tour",
            )
        )

    fig.update_layout(
        title="SIP-style UUV Coverage Plan",
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="data",
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.0)),
        ),
        width=1400,
        height=900,
        margin=dict(l=0, r=0, t=50, b=0),
    )

    pyo.plot(fig, filename=html_path, auto_open=auto_open)
