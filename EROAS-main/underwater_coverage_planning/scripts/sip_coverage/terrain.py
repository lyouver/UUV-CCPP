import os
from typing import Optional, Tuple

import numpy as np
import trimesh

try:
    import rospkg
except ImportError:
    rospkg = None


class TerrainModel:
    def __init__(self, mesh: trimesh.Trimesh, grid_resolution: float):
        self.mesh = mesh
        self.vertices = np.asarray(mesh.vertices)
        self.faces = np.asarray(mesh.faces)
        self.face_centers = np.asarray(mesh.triangles_center)
        self.face_normals = np.asarray(mesh.face_normals)
        self.bounds = np.asarray(mesh.bounds)

        self.grid_resolution = max(float(grid_resolution), 0.2)
        self._build_height_grid()

    def _build_height_grid(self) -> None:
        x_min = float(self.vertices[:, 0].min())
        x_max = float(self.vertices[:, 0].max())
        y_min = float(self.vertices[:, 1].min())
        y_max = float(self.vertices[:, 1].max())

        nx = int(np.ceil((x_max - x_min) / self.grid_resolution)) + 1
        ny = int(np.ceil((y_max - y_min) / self.grid_resolution)) + 1

        self.grid_x_min = x_min
        self.grid_y_min = y_min
        self.grid_x_max = x_max
        self.grid_y_max = y_max
        self.grid_nx = max(nx, 2)
        self.grid_ny = max(ny, 2)

        # Rasterize each triangle onto the XY grid to avoid severe holes caused by vertex-only binning.
        h = np.full((self.grid_nx, self.grid_ny), -np.inf, dtype=np.float64)
        for f in self.faces:
            v0 = self.vertices[f[0]]
            v1 = self.vertices[f[1]]
            v2 = self.vertices[f[2]]

            x0, y0 = float(v0[0]), float(v0[1])
            x1, y1 = float(v1[0]), float(v1[1])
            x2, y2 = float(v2[0]), float(v2[1])

            den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
            if abs(den) < 1e-12:
                continue

            tri_x_min = min(x0, x1, x2)
            tri_x_max = max(x0, x1, x2)
            tri_y_min = min(y0, y1, y2)
            tri_y_max = max(y0, y1, y2)

            ix0 = max(0, int(np.floor((tri_x_min - x_min) / self.grid_resolution)) - 1)
            ix1 = min(self.grid_nx - 1, int(np.ceil((tri_x_max - x_min) / self.grid_resolution)) + 1)
            iy0 = max(0, int(np.floor((tri_y_min - y_min) / self.grid_resolution)) - 1)
            iy1 = min(self.grid_ny - 1, int(np.ceil((tri_y_max - y_min) / self.grid_resolution)) + 1)

            xs = x_min + np.arange(ix0, ix1 + 1, dtype=np.float64) * self.grid_resolution
            ys = y_min + np.arange(iy0, iy1 + 1, dtype=np.float64) * self.grid_resolution
            xx, yy = np.meshgrid(xs, ys, indexing="ij")

            a = ((y1 - y2) * (xx - x2) + (x2 - x1) * (yy - y2)) / den
            b = ((y2 - y0) * (xx - x2) + (x0 - x2) * (yy - y2)) / den
            c = 1.0 - a - b

            inside = (a >= -1e-8) & (b >= -1e-8) & (c >= -1e-8)
            if not np.any(inside):
                continue

            zz = a * float(v0[2]) + b * float(v1[2]) + c * float(v2[2])
            block = h[ix0 : ix1 + 1, iy0 : iy1 + 1]
            block[inside] = np.maximum(block[inside], zz[inside])
            h[ix0 : ix1 + 1, iy0 : iy1 + 1] = block

        self.height_valid = np.isfinite(h)
        if not np.any(self.height_valid):
            raise RuntimeError("Failed to rasterize terrain surface onto height grid")

        self.height_z_max = float(np.max(h[self.height_valid]))
        self.height_grid = h

    def is_inside_xy(self, x: float, y: float) -> bool:
        return (self.grid_x_min <= x <= self.grid_x_max) and (self.grid_y_min <= y <= self.grid_y_max)

    def query_height(self, x: float, y: float) -> float:
        """Return approximate terrain height z at (x, y) via bilinear interpolation."""
        if not self.is_inside_xy(x, y):
            return self.height_z_max

        fx = (x - self.grid_x_min) / self.grid_resolution
        fy = (y - self.grid_y_min) / self.grid_resolution

        x0 = int(np.floor(fx))
        y0 = int(np.floor(fy))
        x1 = min(x0 + 1, self.grid_nx - 1)
        y1 = min(y0 + 1, self.grid_ny - 1)

        wx = float(fx - x0)
        wy = float(fy - y0)

        h00 = self.height_grid[x0, y0]
        h10 = self.height_grid[x1, y0]
        h01 = self.height_grid[x0, y1]
        h11 = self.height_grid[x1, y1]
        v00 = self.height_valid[x0, y0]
        v10 = self.height_valid[x1, y0]
        v01 = self.height_valid[x0, y1]
        v11 = self.height_valid[x1, y1]

        if not (v00 and v10 and v01 and v11):
            # Conservative fallback near sparse cells: use local maximum known surface height.
            for r in range(1, 7):
                xa = max(0, x0 - r)
                xb = min(self.grid_nx - 1, x1 + r)
                ya = max(0, y0 - r)
                yb = min(self.grid_ny - 1, y1 + r)
                local_valid = self.height_valid[xa : xb + 1, ya : yb + 1]
                if np.any(local_valid):
                    local_h = self.height_grid[xa : xb + 1, ya : yb + 1]
                    return float(np.max(local_h[local_valid]))
            return self.height_z_max

        h0 = (1.0 - wx) * h00 + wx * h10
        h1 = (1.0 - wx) * h01 + wx * h11
        return float((1.0 - wy) * h0 + wy * h1)

    def clearance(self, p: np.ndarray) -> float:
        return float(p[2] - self.query_height(float(p[0]), float(p[1])))

    def edge_min_clearance(self, p0: np.ndarray, p1: np.ndarray, step: float) -> Tuple[bool, float]:
        dist = float(np.linalg.norm(p1 - p0))
        if dist < 1e-6:
            c = self.clearance(p0)
            return True, c

        n = max(2, int(np.ceil(dist / max(step, 0.2))) + 1)
        min_clear = np.inf
        for i in range(n):
            a = i / float(n - 1)
            p = (1.0 - a) * p0 + a * p1
            if not self.is_inside_xy(float(p[0]), float(p[1])):
                return False, -np.inf
            c = self.clearance(p)
            if c < min_clear:
                min_clear = c
        return True, float(min_clear)


def resolve_heightmap_path(explicit_path: Optional[str] = None) -> str:
    candidates = []
    if explicit_path:
        candidates.append(explicit_path)

    candidates.extend(
        [
            "/home/tb/dave_ws/src/EROAS-main/uuv_simulator/uuv_gazebo_worlds/models/sand_heightmap/meshes/heightmap.dae",
            "/home/tb/dave_ws/src/uuv_simulator/uuv_gazebo_worlds/models/sand_heightmap/meshes/heightmap.dae",
        ]
    )

    if rospkg is not None:
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path("uuv_gazebo_worlds")
            candidates.append(os.path.join(pkg_path, "models/sand_heightmap/meshes/heightmap.dae"))
        except Exception:
            pass

    for p in candidates:
        if p and os.path.isfile(p):
            return p
    raise FileNotFoundError("Could not locate heightmap.dae (set --heightmap if needed).")


def load_terrain_model(dae_path: str, cfg) -> TerrainModel:
    if not os.path.isfile(dae_path):
        raise FileNotFoundError(f"Terrain file does not exist: {dae_path}")

    scene = trimesh.load(dae_path, process=False)
    if isinstance(scene, trimesh.Scene):
        if len(scene.geometry) == 0:
            raise ValueError("DAE file has no geometry")
        mesh = list(scene.geometry.values())[0]
    else:
        mesh = scene

    # Keep legacy scaling convention so existing bounds keep working.
    mesh.vertices *= float(cfg.mesh_scale_factor)
    mesh.vertices[:, 2] += float(cfg.mesh_z_offset)

    triangles = mesh.triangles
    centroids = triangles.mean(axis=1)
    mask = (
        (centroids[:, 0] >= cfg.crop_x_min)
        & (centroids[:, 0] <= cfg.crop_x_max)
        & (centroids[:, 1] >= cfg.crop_y_min)
        & (centroids[:, 1] <= cfg.crop_y_max)
    )
    if mask.any():
        mesh.update_faces(mask)
        mesh.remove_unreferenced_vertices()

    return TerrainModel(mesh=mesh, grid_resolution=cfg.terrain_grid_resolution)
