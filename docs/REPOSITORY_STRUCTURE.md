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
examples/experimental_logs/
VISION_NAV.md
chapter4_outputs/
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

## Public Release Notes

- This public thesis repository is a curated export of the development
  workspace.
- Curated thesis accuracy logs live under `examples/logs/`.
- Selected raw integration logs live under `examples/experimental_logs/`.
- Legacy scripts live under `legacy/` and are retained as early prototypes.
- Large report or design files should be attached to a GitHub release or stored
  externally instead of being committed directly.

## Notes

The original development workspace contained large generated files and local
experiment outputs. This repository keeps a clean public history and excludes
those generated assets through `.gitignore`.
