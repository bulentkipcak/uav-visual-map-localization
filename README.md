# UAV Visual Map Localization

Visual map localization for UAVs using image-to-map matching in a Gazebo and
ArduPilot SITL environment.

This repository contains the thesis-version implementation of a SIFT-based
visual localization pipeline. A downward-looking simulated camera frame is
matched against a georeferenced reference map, and the estimated camera
position is compared with simulation ground truth.

## Scope

The main scope of this version is:

- reference-map patch generation,
- offline SIFT feature extraction,
- live camera-frame matching,
- RANSAC homography estimation,
- local North/East position estimation,
- Gazebo ground-truth comparison,
- dashboard-based monitoring and configuration,
- CSV logging and report-oriented analysis.

MAVLink `VISION_POSITION_ESTIMATE` and EKF3 external-navigation experiments are
included as experimental integration work. They should not be interpreted as a
fully validated flight-control solution in this thesis version.

## Repository Layout

```text
sau_sift_nav/              Main Python package
live_sift_nav.py           Live localization entry point
sift_dashboard_launcher.py Configurable dashboard launcher
dashboard/                 React/Vite dashboard source
QGIS/SAU CAMPUS/           Lightweight QGIS project file; raster outputs excluded
scripts/                   Dataset and SIFT database build utilities
tools/                     Offline analysis and report-figure utilities
vision_debug/              MAVLink/EKF/Gazebo diagnostic tools and notes
examples/logs/             Curated thesis accuracy logs
examples/experimental_logs/Selected raw experimental integration logs
chapter4_outputs/          Selected report figures
legacy/                    Early prototype scripts
```

Large map assets, generated tile databases, raw video frames, virtual
environments, and local build outputs are intentionally excluded from Git.

## Python Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Gazebo Python bindings are environment-specific and are not installed from
`requirements.txt`. They must be available from the local Gazebo installation
when Gazebo ground-truth features are used.

## Dashboard Setup

The dashboard uses the current Vite/React toolchain and requires Node.js
20.19.0 or newer.

```bash
cd dashboard
npm install
npm run build
cd ..
```

The configurable launcher can then serve the built dashboard:

```bash
python3 sift_dashboard_launcher.py --host 127.0.0.1 --port 8099
```

## Typical Observe Run

The exact command depends on the local SITL, Gazebo, camera, and map asset
setup. A typical observation run uses the SIFT master-map source, receives the
camera stream, logs CSV output, and keeps VPE transmission disabled:

```bash
python3 live_sift_nav.py \
  --map-source sift-master \
  --host 127.0.0.1 \
  --port 8080 \
  --mavlink udp:127.0.0.1:14550 \
  --no-send-vision \
  --telemetry-seed-source global \
  --search-radius-m 150 \
  --max-search-radius-m 300 \
  --max-tiles-per-scale 9 \
  --gazebo-truth \
  --log-csv sift_observe.csv
```

## Data Assets

The repository does not include the full reference-map images, generated patch
images, or SIFT descriptor databases because they are large generated assets.
The lightweight QGIS project is included for reproducibility, but its raster
layers depend on local/generated assets. Use the scripts under `scripts/` to
rebuild the SIFT assets from the local master map.

Important generated asset groups:

- georeferenced master map,
- overlapping reference-map patches,
- SIFT descriptor databases,
- legacy multi-scale tile databases.

## Thesis Version Status

This version is intended to document the bachelor thesis implementation and
simulation evaluation. Future work may add alternative feature extractors,
learning-based matching, improved filtering, or tighter sensor-fusion support.

See `docs/THESIS_VERSION.md` and `docs/REPOSITORY_STRUCTURE.md` for publishing
notes.

## License and Citation

The source code is released under the MIT License. If you use this work in an
academic context, please cite the repository using `CITATION.cff`.
