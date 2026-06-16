# Repository Structure Audit

This document records the intended public structure for the project.

## Keep as Source

```text
sau_sift_nav/
live_sift_nav.py
sift_dashboard_launcher.py
dashboard/src/
dashboard/public/
dashboard/package.json
dashboard/package-lock.json
dashboard/index.html
dashboard/vite.config.js
QGIS/SAU CAMPUS/SAU_campus.qgz
scripts/
tools/
vision_debug/
examples/logs/
VISION_NAV.md
chapter4_outputs/
figures/
legacy/
```

## Generated or Local-Only

These should stay out of normal Git history:

```text
.venv/
dashboard/node_modules/
dashboard/dist/
map_1x/
map_1_2x/
map_1_4x/
map_3_4x/
QGIS/SAU CAMPUS/output/
data/*/frames/
tuning_plots*/
*.tif
*.tiff
*.pkl
*.npz
*.log
```

## Needs Curation Before Public Release

- Root-level CSV files should be moved into an examples or report-data folder
  only if they are intentionally part of the thesis evidence.
- Legacy scripts such as `match_frame_to_tiles.py` and
  `match_frame_multiscale.py` live under `legacy/` and should be described as
  early prototypes.
- Large report or design files should be attached to a GitHub release or stored
  externally instead of being committed directly.

## Notes

The current development history contains large files. For a clean GitHub
publication, prefer a fresh public history, an orphan branch, or a curated
export rather than pushing the existing full history.
