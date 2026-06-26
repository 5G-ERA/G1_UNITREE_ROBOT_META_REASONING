# Summit XL — ground-truth maps for the G1 project

High-quality maps captured with the **Summit XL** (3D RSLIDAR, ROS 2 Humble, slam_toolbox)
to use as a **ground-truth reference** for the G1 meta-reasoning work (overlay G1 runs, validate
path planning, score localization confidence). Frame: `robot_map`.

> Note: a Summit map is **not** a drop-in for the G1's native relocalization (different sensor /
> height / format). It's a reference, aligned to the G1 frame with a one-time rigid transform.

## Files

| File | What it is |
|------|------------|
| `summit_map.pcd` | Dense 3D point-cloud map, 930k pts, 5 cm voxels (PCL `.pcd`). Open in CloudCompare / Open3D / RViz. |
| `summit_map.json` | Same cloud downsampled to `[x,y,z]` list — same schema as the G1 `dataset/map_full.json`. |
| `summit_map_3d.html` | Standalone interactive 3D viewer (Three.js, 140k pts). Open in a browser; drag/zoom/pan. |
| `summit_map_2d.png` | Static top-down (colour=height) + wall-slice floor plan. |
| `summit_occupancy.png` | Wall-occupancy raster from the 3D cloud (torso band). |
| `rbk_2026_06_26_16_22_47.yaml` | slam_toolbox 2D occupancy map metadata (res 0.05 m, origin [-11, -6.62]). |
| `rbk_2026_06_26_16_22_47_nav.yaml` | Same, navigation (inflated) variant. |
| `EXPERIMENT_resolution.txt` | Resolution tag (medium = 0.05 m/cell). |
| `summit_pcd_mapper.py` | The capture tool: accumulates `/robot/top_laser/point_cloud` into `robot_map` → `.pcd`+`.json`. |

## Missing / to add
- **`rbk_2026_06_26_16_22_47.png`** (and `_nav.png`): the actual 2D occupancy **images** referenced by
  the `.yaml`. The `.yaml` is here but not the `.png`. Add them for a georeferenced floor plan.

## How the maps were made
1. On the robot: `bash map_res.sh medium` (slam_toolbox, 0.05 m/cell).
2. On a PC (`ROS_DOMAIN_ID=39`, `RMW=rmw_cyclonedds_cpp`): `python3 summit_pcd_mapper.py`, drive
   slowly with the gamepad closing loops, `Ctrl+C` → `summit_map.pcd` + `summit_map.json`.
3. 2D occupancy saved with `bash save_res.sh` → `config/maps/rbk_<date>/` (`.png` + `.yaml`).

## Map frame
`robot_map`, resolution 0.05 m. 2D origin (bottom-left of the image) = `[-11, -6.62] m`.
3D cloud bbox ≈ x[-12.2, 6.7], y[-8.6, 10.6], z[-0.1, 2.9] — consistent with the 2D origin.
