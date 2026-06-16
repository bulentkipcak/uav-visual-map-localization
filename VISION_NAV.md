# Vision Navigation Notes

This document summarizes the live SIFT navigation path and the experimental
MAVLink/EKF integration notes. The validated thesis scope is visual
localization and Gazebo ground-truth comparison. MAVLink
`VISION_POSITION_ESTIMATE` and EKF3 source switching are kept as experimental
integration work, not as a fully validated flight-control result.

For the main thesis workflow, prefer the observe command in `README.md` with
`--no-send-vision`.

## Runtime Architecture

- `live_sift_nav.py` is the CLI entry point.
- `sau_sift_nav/geometry.py` owns map pixel, Gazebo world, and local NED
  conversion.
- `sau_sift_nav/patches.py` loads the SIFT master-map patch database.
- `sau_sift_nav/tiles.py` keeps the legacy tile path for old experiments.
- `sau_sift_nav/localizer.py` performs ROI-limited SIFT matching with RANSAC
  homography.
- `sau_sift_nav/state.py` stores the latest frame, telemetry, visual fix,
  search radius, and reset counter.
- `sau_sift_nav/mavlink_io.py` reads telemetry and can send
  `VISION_POSITION_ESTIMATE`; `VISION_SPEED_ESTIMATE` is optional.
- `sau_sift_nav/video.py` reads the Gazebo UDP H264 camera stream.
- `sau_sift_nav/web.py` serves the legacy dashboard.
- `sau_sift_nav/launcher_api.py` serves the configurable dashboard launcher.

## SIFT Master-Map Mode

The thesis version uses the SIFT master-map path:

```bash
--map-source sift-master
```

The generated patch database is expected under:

```text
QGIS/SAU CAMPUS/output/SIFT/patches/
```

Generated raster assets and descriptor databases are intentionally excluded from
Git because they are large derived files. Rebuild them locally with:

```bash
python3 scripts/build_sift_master_patches.py
```

Verified master-map geometry:

```text
image = 12288 x 12288 px
world extent = 1500 m x 1500 m
x = [-750, 750]
y = [-750, 750]
pixel_size = 0.1220703125 m/px
patch grid = 11 x 11
patch count = 121
patch size = 2048 px = 250 m
step = 1024 px = 125 m
overlap = 1024 px = 125 m
```

Runtime candidate selection uses a bounded local search around the current
search hint. With a fresh hint, `--max-tiles-per-scale 9` corresponds to a
3x3 patch window. Larger windows are useful for uncertain relocalization but
increase processing time.

## Search Strategy

The live path avoids full-map matching whenever possible:

1. Use the latest accepted visual fix when available.
2. Otherwise seed the search region from telemetry or GPS-derived local
   position.
3. Search nearby patches first.
4. Stop early when the inlier count is strong enough.
5. Grow the search radius after misses.
6. Reset the search radius after an accepted match.

This implements a practical prior-and-measurement workflow: vehicle state gives
a search prior, then image matching supplies the visual measurement.

## Recommended Observation Run

Use this mode for thesis-style accuracy logging. It does not publish VPE into
the autopilot:

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

## Experimental VPE Output

VPE output is disabled by default in the public thesis configuration. Enable it
only for controlled integration tests:

```bash
--send-vision
```

When enabled, `VISION_POSITION_ESTIMATE` is sent in the local NED frame:

- `x`: north in meters
- `y`: east in meters
- `z`: selected by `--vision-z-source zero|telemetry`
- `roll/pitch/yaw`: selected by `--vision-attitude-source telemetry|zero`
- `covariance[0]`: NaN, meaning unknown covariance
- `reset_counter`: increments when the visual estimate is reset

`VISION_SPEED_ESTIMATE` is not sent unless `--send-vision-speed` is passed.
Zero speed is useful only for hover/debug isolation tests; it should not be
treated as a validated motion solution.

`--vision-publish-mode rate` republishes a valid recent estimate at
`--vision-rate-hz`. `--vision-publish-mode fix` sends once per accepted visual
fix and can drop below ArduPilot's VisualOdom health rate if SIFT is slow.

## EKF Source-Set Experiments

The `vision_debug/` tools include EKF source-set switching, dummy vision, Gazebo
truth bridge, and MAVLink doctor utilities. These files are useful for repeating
the integration experiments, but they should be presented as experimental work
and limitations in the thesis version.

Do not describe EKF source switching as a validated mission-control result for
this release. The supported thesis result is SIFT-based visual position
estimation evaluated against Gazebo ground truth.
