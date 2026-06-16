# Live SIFT Vision Navigation

## Runtime architecture

- `live_sift_nav.py` is only the CLI entrypoint.
- `sau_sift_nav/geometry.py` owns map pixel, Gazebo world, and local NED conversion.
- `sau_sift_nav/tiles.py` loads tile metadata and lazily caches SIFT descriptors.
- `sau_sift_nav/patches.py` loads the new georeferenced SIFT master-map patch database.
- `sau_sift_nav/localizer.py` performs ROI-limited SIFT matching with RANSAC homography.
- `sau_sift_nav/state.py` keeps the latest frame, telemetry, visual fix, search radius, and reset counter.
- `sau_sift_nav/mavlink_io.py` reads telemetry and sends `VISION_POSITION_ESTIMATE`; `VISION_SPEED_ESTIMATE` is optional.
- `sau_sift_nav/video.py` reads the Gazebo UDP H264 camera stream.
- `sau_sift_nav/web.py` serves the dashboard.

## Current development decision

VPE-only source-set-2 testing showed that MAVLink/EKF message format is not the
main blocker anymore. The active design is:

- Send `VISION_POSITION_ESTIMATE` only.
- Fill VPE `x/y` from visual navigation.
- Send VPE `z=0`.
- Send VPE `roll/pitch/yaw=0` during the active isolation tests.
- Keep altitude on barometer with `EK3_SRC2_POSZ=1`.
- Keep yaw on compass with `EK3_SRC2_YAW=1`.
- Keep velocity fusion disabled with `EK3_SRC2_VELXY=0`.

The next implementation step is a temporal visual estimator/filter between raw
SIFT and VPE publishing:

```text
raw SIFT fix -> visual estimator/filter -> filtered VPE x/y
```

The latest plan and baseline commands are documented in
`vision_debug/SIFT_NEXT_STEPS.md`.

## New SIFT master-map mode

The legacy tile DB is still available. The new master-map DB is selected at
runtime with:

```bash
--map-source sift-master
```

Master input:

```text
QGIS/SAU CAMPUS/output/SIFT/SIFT_master.png
```

Generated DB:

```text
QGIS/SAU CAMPUS/output/SIFT/patches/
```

Generation command:

```bash
python3 scripts/build_sift_master_patches.py
```

Verified geometry:

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
```

Runtime candidate selection uses the same matcher interface as the legacy tile
path. With a fresh search hint, use:

```text
--max-tiles-per-scale 9   # 3x3 patch window
--max-tiles-per-scale 25  # 5x5 uncertain window
```

Full 121-patch matching is possible when there is no hint, but it is a slow
relocalization/debug path, not the normal live path.

## Search strategy

The slow path is full-map SIFT. The live path uses a bounded local search:

1. Use the last visual fix if available.
2. Propagate it with `LOCAL_POSITION_NED.vx/vy`.
3. Before the first visual fix, seed the ROI from telemetry/GPS local position.
4. Search only nearby tiles, sorted by distance from the predicted pixel.
5. Stop early when a strong inlier count is reached.
6. On a miss, grow the search radius until the maximum radius is reached.
7. On a match, reset the search radius.

This is the practical version of a Bayesian/Monte-Carlo localization idea: maintain a prior from motion, then use image matching as the measurement update. We use a single Gaussian-like ROI instead of particles because the simulator is smooth and the vehicle cannot teleport.

## MAVLink vision output

The app sends `VISION_POSITION_ESTIMATE` at `--vision-rate-hz` even though SIFT runs more slowly. Between recent SIFT fixes, it republishes the latest visual position. The default `--vision-max-age-sec 3.0` gives the current SIFT path enough room for roughly 1.3 s updates, but still stops publishing when the visual estimate is stale. The sent frame is local NED:

- `x`: north in meters
- `y`: east in meters
- `z`: current local down if available
- `roll/pitch/yaw`: current vehicle attitude if available
- `covariance[0]`: NaN, meaning unknown covariance
- `reset_counter`: increments when the visual estimate jumps by more than the reset threshold

The current default is VPE-only: `VISION_SPEED_ESTIMATE` is not sent unless
`--send-vision-speed` is passed. This matches the active test goal: feed EKF only
ExternalNav XY position, while altitude, yaw, acceleration, and velocity come from
the vehicle's normal sources.

If a later comparison needs VSE, enable it explicitly with `--send-vision-speed`.
`--vision-speed-source zero` is hover/debug only; during motion it can conflict with
changing VPE positions if EKF is also fusing `EK3_SRC*_VELXY=6`. `visual` should be
tested only after SIFT fixes are fresh and stable enough.

VPE roll/pitch/yaw can be selected with `--vision-attitude-source telemetry|zero`.
The default `telemetry` copies current ArduPilot attitude into the required VPE fields.
Use `zero` only for isolation tests that prove EKF source selection is ignoring VPE
attitude as expected.

VPE publishing can be selected with `--vision-publish-mode rate|fix`. `rate` is the
normal ExternalNav-friendly mode: it republishes the latest valid pose at
`--vision-rate-hz`. `fix` is experimental: it sends exactly once per newly accepted
SIFT fix, using the raw fix position instead of age-predicted repeats. This can help
isolate whether repeated stale measurements are increasing EKF variance, but it may
drop below ArduPilot's VisualOdom health rate if SIFT is slow.

For the setup you described, let ArduPilot use ExternalNav only for horizontal position, while barometer and compass stay responsible for altitude and heading.

## Suggested run command

```bash
python3 live_sift_nav.py \
  --host 127.0.0.1 \
  --port 8080 \
  --scales '1/4x' \
  --mavlink tcp:127.0.0.1:5763 \
  --vision-mavlink tcp:127.0.0.1:5762 \
  --vision-rate-hz 10 \
  --vision-max-age-sec 3.0 \
  --search-radius-m 120 \
  --max-search-radius-m 700 \
  --max-tiles-per-scale 10 \
  --early-stop-inliers 140
```

To also send the EKF source parameters from the script:

```bash
python3 live_sift_nav.py --configure-ekf
```

Parameters sent by `--configure-ekf`:

```text
VISO_TYPE=1
VISO_POS_M_NSE=2.0
VISO_VEL_M_NSE=2.0
AHRS_EKF_TYPE=3
EK3_ENABLE=1
EK2_ENABLE=0
EK3_SRC1_POSXY=6
EK3_SRC1_VELXY=0
EK3_SRC1_POSZ=1
EK3_SRC1_VELZ=0
EK3_SRC1_YAW=1
EK3_SRC_OPTIONS=0
```

Use `SIM_GPS_DISABLE=1` only after the dashboard shows `Vision TX: sending`.

The dashboard may also show `REJECTED`. That means SIFT found a homography, but it was not safe enough to feed to EKF. Current publish gates are:

- `--min-nav-inliers 80`
- `--max-nav-error-m 15`
- `--max-nav-jump-m 35`

When GPS is still available, `--max-nav-error-m` is the most important guard: a visual fix far away from the current local NED estimate is treated as a false tile match and is not published.
