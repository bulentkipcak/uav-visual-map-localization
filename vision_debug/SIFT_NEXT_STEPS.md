# SIFT Next Steps

This note captures the current decision point after the VPE-only source-set-2
tests.

## Current Conclusion

The MAVLink/EKF side is no longer the primary blocker.

Confirmed:

```text
VISION_POSITION_ESTIMATE reaches ArduPilot.
VPE-only XY works with EK3_SRC2_POSXY=6 and EK3_SRC2_VELXY=0.
VISION_SPEED_ESTIMATE is not required for the active design.
VPE z can be sent as 0 because EK3_SRC2_POSZ=1 keeps altitude on barometer.
VPE roll/pitch/yaw can be sent as 0 because EK3_SRC2_YAW=1 keeps yaw on compass.
ExternalNav flags usually stay 831 when fresh VPE continues.
```

Remaining blocker:

```text
Raw SIFT position fixes still contain jitter/outliers around 3-5 m during motion.
Those raw jumps eventually increase EKF variance or make EKF lose horizontal
ExternalNav trust.
```

So the next main task is not more MAVLink parameter work. It is:

```text
raw SIFT fix -> visual temporal estimator/filter -> VPE publisher
```

## Master Map Patch Pipeline

The new SIFT master-map path is now available alongside the legacy tile path.
It does not delete or replace the old `map_1x`, `map_1_2x`, `map_1_4x`
database.

Input:

```text
QGIS/SAU CAMPUS/output/SIFT/SIFT_master.png
QGIS/SAU CAMPUS/output/SIFT/SIFT_master.png.aux.xml
```

The PNG aux file confirms this geotransform:

```text
x_min=-750
y_max=750
pixel_size=0.1220703125 m/px
y pixel axis is image-down, world-y is north/up
```

Generated outputs:

```text
QGIS/SAU CAMPUS/output/SIFT/patches/patch_r00_c00.png ... patch_r10_c10.png
QGIS/SAU CAMPUS/output/SIFT/patches/patch_metadata.json
QGIS/SAU CAMPUS/output/SIFT/patches/patch_metadata.csv
QGIS/SAU CAMPUS/output/SIFT/patches/sift/*.npz
QGIS/SAU CAMPUS/output/SIFT/patches/sift_database_manifest.json
QGIS/SAU CAMPUS/output/SIFT/patches/sift_database.pkl
```

`patches/` is ignored by git because it is generated and currently about 1 GB.
Keep the builder script tracked; regenerate the database when the master map or
SIFT parameters change.

Generation command:

```bash
python3 scripts/build_sift_master_patches.py
```

Verified generation result:

```text
master_size = 12288 x 12288
patch_size = 2048 px
step = 1024 px
grid = 11 x 11
patch_count = 121
total_keypoints = 181539
nfeatures = 1500
first patch world extent = x[-750,-500], y[500,750]
last patch world extent = x[500,750], y[-750,-500]
```

Runtime one-frame smoke test:

```bash
python3 live_sift_nav.py \
  --map-source sift-master \
  --no-mavlink \
  --once \
  --image frame_80m.jpg \
  --hint-north 0 \
  --hint-east 0 \
  --search-radius-m 150 \
  --max-tiles-per-scale 9 \
  --resize-w 512 \
  --min-inliers 20 \
  --early-stop-inliers 80
```

This confirmed that the runtime can load the new patch DB and search a 3x3
candidate set. The smoke test produced an OK match in 9 considered patches.

For live testing with the master DB, start with a 3x3 ROI:

```bash
python3 live_sift_nav.py \
  --map-source sift-master \
  --host 127.0.0.1 \
  --port 8080 \
  --mavlink udp:127.0.0.1:14550 \
  --send-vision \
  --no-send-vision-speed \
  --vision-publish-mode fix \
  --vision-rate-hz 10 \
  --vision-max-age-sec 3.0 \
  --vision-z-source zero \
  --vision-attitude-source zero \
  --no-telemetry-position-gate \
  --telemetry-seed-source global \
  --telemetry-max-age-sec 1.0 \
  --search-radius-m 150 \
  --max-search-radius-m 300 \
  --search-nav-max-age-sec 0 \
  --max-tiles-per-scale 9 \
  --early-stop-inliers 160 \
  --min-nav-inliers 120 \
  --max-nav-jump-m 0 \
  --log-csv live_sift_master_vpe.csv
```

If the 3x3 ROI is too slow, try `--max-tiles-per-scale 4` or keep `9` but reduce
`--resize-w`. If relocalization is needed, use `--max-tiles-per-scale 25` for a
5x5 window or remove the hint/seed for a full 121-patch search only as a slow
debug mode.

First live observe result with `live_sift_master_observe.csv`:

```text
raw SIFT OK = 144 / 144 over 40 s
raw fix rate = 3.6 Hz
tile stayed on patch_r04_c05.png
duration p95 = 0.229 s
raw calc_error p95 = 1.91 m
raw calc_error max = 13.30 m, from the first low-inlier global initialization
```

Next master-map observe command should lower the nav inlier gate:

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
  --resize-w 512 \
  --early-stop-inliers 90 \
  --min-nav-inliers 60 \
  --max-nav-jump-m 5 \
  --log-csv live_sift_master_observe_inl60.csv
```

That follow-up produced:

```text
duration = 124 s
raw SIFT OK = 447 / 447
nav accepted = 347
accepted rate = 2.80 Hz
accepted calc_error p95 = 2.08 m
accepted calc_error max = 2.74 m
accepted errors over 3 m = 0
```

Important camera finding:

```text
Current Gazebo gimbal camera is 640x480 with horizontal_fov=2.0 rad (114.6 deg).
The intended SIFT-master baseline was 1024x1024 with 65 deg FOV.
```

So before the next EKF source-set-2 test, either:

```text
1. Update Gazebo camera to 1024x1024 and horizontal_fov=1.134464 rad, then rebuild
   observe statistics.
2. Keep the current camera and explicitly treat this as a 640x480 / 114.6 deg
   baseline.
```

Option 1 is cleaner because its ground footprint and image resolution match the
master-map/patch design assumptions.

Camera model update applied:

```text
file = <ardupilot_gazebo>/models/gimbal_small_3d/model.sdf
backup = <ardupilot_gazebo>/models/gimbal_small_3d/model.sdf.codexbak
horizontal_fov = 1.134464 rad
image = 1024 x 1024
```

Expected footprint:

```text
80 m altitude  -> 101.9 m footprint, 0.0995 m/px camera ground sample
100 m altitude -> 127.4 m footprint, 0.1244 m/px camera ground sample
```

Gazebo must be restarted for this SDF model change to take effect.

First 100 m observe after restart:

```text
CSV = live_sift_master_1024_fov65_observe.csv
duration = 136 s
raw SIFT OK = 490 / 490
nav accepted = 400
accepted rate = 2.94 Hz
accepted calc_error median = 0.74 m
accepted calc_error p95 = 2.08 m
accepted calc_error max = 3.12 m
good_count median = 181
```

Compared with the previous wide camera, the new camera improved median accuracy
and good match count. The rate is still below the 4 Hz target because the current
observe run used `--interval-sec 0.25` and `--min-nav-inliers 60`.

Next observe speed test:

```bash
python3 live_sift_nav.py \
  --map-source sift-master \
  --host 127.0.0.1 \
  --port 8080 \
  --mavlink udp:127.0.0.1:14550 \
  --no-send-vision \
  --telemetry-seed-source global \
  --interval-sec 0.15 \
  --search-radius-m 150 \
  --max-search-radius-m 300 \
  --max-tiles-per-scale 9 \
  --resize-w 512 \
  --early-stop-inliers 80 \
  --min-nav-inliers 50 \
  --max-nav-jump-m 5 \
  --log-csv live_sift_master_1024_fov65_fast_observe.csv
```

Native 512 camera test setup:

```text
file = <ardupilot_gazebo>/models/gimbal_small_3d/model.sdf
backup = <ardupilot_gazebo>/models/gimbal_small_3d/model.sdf.codexbak_1024_fov65
horizontal_fov = 1.134464 rad
image = 512 x 512
```

Expected footprint is still the same because FOV did not change:

```text
80 m altitude  -> 101.9 m footprint, 0.1991 m/px camera ground sample
100 m altitude -> 127.4 m footprint, 0.2489 m/px camera ground sample
```

For this test, do not pass `--resize-w`; Gazebo already produces the intended
512 px frame.

Native 512 observe before disabling zoom:

```text
CSV = live_sift_master_512_fov65_observe.csv
accepted rate = 4.21 Hz
accepted calc_error p95 = 2.54 m
accepted calc_error max = 3.24 m
```

Dashboard footprint issue:

```text
The cyan camera footprint appeared larger than the yellow 250 m patch.
At 100 m and 65 deg, expected footprint is about 127 m, so the cyan polygon should
be about half the patch width.
```

Cause:

```text
CameraZoomPlugin hard-codes refHfov=2.0 and goalHfov=2.0, so it can force the
camera back to 2.0 rad even if model.sdf says 1.134464 rad.
2.0 rad at 100 m = about 311 m footprint, larger than the 250 m patch.
```

Action applied:

```text
CameraZoomPlugin block disabled in gimbal_small_3d/model.sdf.
Backup: <ardupilot_gazebo>/models/gimbal_small_3d/model.sdf.codexbak_zoom_plugin
```

Diagnostics added for the next run:

```text
CSV fields: frame_quad_width_m, frame_quad_height_m, frame_quad_area_m2
Dashboard log: camera footprint: W x H m
```

No-zoom observe result:

```text
CSV = live_sift_master_512_fov65_nozoom_observe.csv
duration = 266 s
accepted rate = 5.35 Hz
accepted calc_error median = 0.70 m
accepted calc_error p95 = 1.93 m
accepted calc_error max = 2.84 m
accepted errors over 3 m = 0
duration median = 0.095 s
duration p95 = 0.206 s
```

Footprint check:

```text
frame_quad_width_m mean = 126.76 m
frame_quad_height_m mean = 126.72 m
```

This matches the expected 100 m / 65 deg footprint of about 127 m, so the
CameraZoomPlugin change fixed the cyan footprint being larger than the yellow
250 m patch.

## Camera Geometry Test Plan

Purpose:

```text
Pick a camera resolution, FOV, and flight altitude that gives stable SIFT
localization at 4 Hz or faster while keeping the camera ground sample distance
close to the master map ground sample distance.
```

Master map ground sample distance:

```text
master map size = 1500 m / 12288 px
master map GSD  = 0.1220703125 m/px
patch size      = 2048 px = 250 m
patch step      = 1024 px = 125 m
```

Current fixed assumptions:

```text
gimbal pitch = -90 deg, nadir/down-looking
CameraZoomPlugin = disabled
do not use --resize-w for native camera tests
```

Decision criteria:

```text
accepted rate >= 4 Hz
duration p95 <= 0.25 s
accepted calc_error p95 <= 2.0 m
accepted calc_error max <= 3.0 m
accepted errors over 3 m = 0
low_inliers reject rate is low
frame_quad_width_m / frame_quad_height_m matches expected footprint
```

Candidate geometry table:

```text
Test  Altitude  FOV   Image      Footprint  Camera GSD   Ratio to map GSD
R     100 m     65    512 px     127.4 m    0.2489 m/px  2.04x   done
S1    100 m     65    1024 px    127.4 m    0.1244 m/px  1.02x   main candidate
S2    100 m     65    768 px     127.4 m    0.1659 m/px  1.36x   middle candidate
S3    120 m     55    1024 px    124.9 m    0.1220 m/px  1.00x   narrow-FOV candidate
S4    80 m      75    1024 px    122.8 m    0.1199 m/px  0.98x   low-alt candidate
```

Interpretation:

```text
S1 is the theoretical best match for the current 100 m flight profile because
its camera GSD almost exactly matches the master map GSD.

R is already proven stable and fast, but it throws away detail because 512 px at
100 m / 65 deg is about 2x coarser than the master map.

S2 is the fallback if S1 is too slow.

S3 may reduce perspective/distortion effects because it uses a narrower FOV,
but it requires 120 m flight altitude.

S4 is only needed if the final mission must stay near 80 m.
```

Gazebo camera values for each test:

```text
file = <ardupilot_gazebo>/models/gimbal_small_3d/model.sdf

65 deg FOV = 1.134464 rad
55 deg FOV = 0.959931 rad
75 deg FOV = 1.308997 rad
```

After changing camera values, restart Gazebo/SITL before logging a CSV.

Common observe command template:

```bash
python3 live_sift_nav.py \
  --map-source sift-master \
  --host 127.0.0.1 \
  --port 8080 \
  --mavlink udp:127.0.0.1:14550 \
  --no-send-vision \
  --telemetry-seed-source global \
  --interval-sec 0.15 \
  --search-radius-m 150 \
  --max-search-radius-m 300 \
  --max-tiles-per-scale 9 \
  --early-stop-inliers 80 \
  --min-nav-inliers 50 \
  --max-nav-jump-m 5 \
  --log-csv CSV_NAME_HERE.csv
```

CSV summary command:

```bash
python3 scripts/summarize_sift_csv.py CSV_NAME_HERE.csv
```

### Test S1 - 100 m / 65 deg / 1024 px

Expected:

```text
footprint ~= 127.4 m
camera GSD ~= 0.1244 m/px
frame_quad_width_m and frame_quad_height_m should be near 127 m
```

CSV:

```text
live_sift_master_100m_fov65_1024.csv
```

Observed:

```text
accepted rate = 5.37 Hz
nav rejected = 0
duration median = 0.179 s
duration p95 = 0.254 s
accepted calc_error median = 0.64 m
accepted calc_error p95 = 2.07 m
accepted calc_error max = 2.94 m
accepted errors over 3 m = 0
inliers median = 102
frame_quad_width_m mean = 123.87 m
frame_quad_height_m mean = 123.76 m
```

Decision:

```text
S1 is viable and has excellent inlier quality, but it narrowly misses the strict
p95 duration and p95 error targets. Run S2 next to test whether 768 px gives a
better speed/accuracy balance.
```

### Test S2 - 100 m / 65 deg / 768 px

Expected:

```text
footprint ~= 127.4 m
camera GSD ~= 0.1659 m/px
frame_quad_width_m and frame_quad_height_m should still be near 127 m
```

CSV:

```text
live_sift_master_100m_fov65_768.csv
```

Observed:

```text
accepted rate = 5.56 Hz
nav rejected = 0
duration median = 0.123 s
duration p95 = 0.204 s
duration p99 = 0.221 s
accepted calc_error median = 0.62 m
accepted calc_error p95 = 1.96 m
accepted calc_error max = 2.86 m
accepted errors over 3 m = 0
inliers median = 99
frame_quad_width_m mean = 124.65 m
frame_quad_height_m mean = 124.68 m
```

Decision:

```text
S2 passes the observe-only criteria and is currently the best 100 m / 65 deg
balance candidate.

One first-frame global relocalization scanned 121 patches and took 2.203 s.
Steady-state after that used vision_predicted search over 9 patches, with max
duration 0.285 s.
```

### Test S3 - 120 m / 55 deg / 1024 px

Expected:

```text
footprint ~= 124.9 m
camera GSD ~= 0.1220 m/px
frame_quad_width_m and frame_quad_height_m should be near 125 m
```

CSV:

```text
live_sift_master_120m_fov55_1024.csv
```

Observed:

```text
accepted rate = 4.70 Hz
nav rejected = 0
duration median = 0.199 s
duration p95 = 0.264 s
accepted calc_error median = 0.93 m
accepted calc_error p95 = 2.71 m
accepted calc_error max = 3.50 m
accepted errors over 3 m = 9
inliers median = 94
frame_quad_width_m mean = 121.51 m
frame_quad_height_m mean = 121.31 m
```

Decision:

```text
S3 is not the default candidate. It stays above 4 Hz and has no rejects, but it
is worse than S2 in both error and duration.
```

### Test S4 - 80 m / 75 deg / 1024 px

Expected:

```text
footprint ~= 122.8 m
camera GSD ~= 0.1199 m/px
frame_quad_width_m and frame_quad_height_m should be near 123 m
```

CSV:

```text
live_sift_master_80m_fov75_1024.csv
```

Observed:

```text
accepted rate = 5.18 Hz
nav rejected = 0
duration median = 0.178 s
duration p95 = 0.263 s
accepted calc_error median = 0.71 m
accepted calc_error p95 = 2.19 m
accepted calc_error max = 3.57 m
accepted errors over 3 m = 3
inliers median = 90
frame_quad_width_m mean = 122.29 m
frame_quad_height_m mean = 122.08 m
```

Decision:

```text
S4 confirms that 80 m / 75 deg gives the expected ~123 m footprint, but the
wider FOV likely increases perspective/distortion pressure. It is usable, but
it does not beat S2 and has accepted errors over 3 m.
```

## Camera Geometry Decision

Default candidate:

```text
100 m altitude
65 deg horizontal FOV = 1.134464 rad
768 x 768 camera image
```

Why:

```text
S2 passed all observe-only criteria:

accepted rate = 5.56 Hz
duration p95 = 0.204 s
accepted calc_error p95 = 1.96 m
accepted calc_error max = 2.86 m
accepted errors over 3 m = 0
nav rejected = 0
```

Camera test ranking:

```text
1. S2, 100 m / 65 deg / 768 px   - best speed/accuracy balance
2. S1, 100 m / 65 deg / 1024 px  - strong, slightly slower
3. S4, 80 m / 75 deg / 1024 px   - usable, but has >3 m outliers
4. S3, 120 m / 55 deg / 1024 px  - stable, but worse error
```

Next recommended test:

```text
Run a longer S2 validation, still observe-only, while moving around the map.
Use the same 100 m / 65 deg / 768 px camera setup and log at least 5-10 minutes.
If that stays clean, use S2 for the first master-map VPE publish test.
```

Long S2 validation:

```text
CSV = live_sift_master_100m_fov65_768_long.csv
duration = 301 s
accepted rate = 5.89 Hz
nav rejected = 0
duration median = 0.109 s
duration p95 = 0.177 s
accepted calc_error median = 0.75 m
accepted calc_error p95 = 1.78 m
accepted calc_error p99 = 2.20 m
accepted calc_error max = 3.21 m
accepted errors over 3 m = 2 / 1773
frame_quad_width_m mean = 124.07 m
frame_quad_height_m mean = 124.01 m
```

Decision update:

```text
S2 remains the default geometry. The long run is clean enough for the next VPE
publish test, but keep a 3 m max-nav-error gate because two rare accepted fixes
were slightly over 3 m.
```

Long 1024 validation:

```text
CSV = live_sift_master_100m_fov65_1024_long.csv
duration = 299 s
accepted rate = 5.76 Hz
nav rejected = 0
duration median = 0.166 s
duration p95 = 0.227 s
accepted calc_error median = 0.85 m
accepted calc_error p95 = 2.06 m
accepted calc_error p99 = 2.61 m
accepted calc_error max = 3.14 m
accepted errors over 3 m = 4 / 1722
frame_quad_width_m mean = 123.91 m
frame_quad_height_m mean = 123.90 m
```

Final camera geometry decision:

```text
Use 100 m / 65 deg / 768 px as the default.

1024 px does not improve the long-run result. It gives slightly more inliers,
but it is slower and has worse p95 error than 768 px.
```

## First Master-Map VPE Test

Use the final camera geometry:

```text
100 m altitude
65 deg horizontal FOV = 1.134464 rad
768 x 768 camera image
CameraZoomPlugin disabled
```

VPE packet policy:

```text
VISION_POSITION_ESTIMATE only
x/y = SIFT N/E estimate
z = 0
roll/pitch/yaw = 0
covariance[0] = NaN
VISION_SPEED_ESTIMATE disabled
```

Source-set policy:

```text
source set 1 = GPS takeoff / GPS navigation
source set 2 = ExternalNav XY only
GPS remains enabled for recovery
do not set SIM_GPS_DISABLE=1 during this first test
```

### Phase 1 - Publish SIFT VPE while still on GPS source set

Start at 100 m hover on source set 1. Run SIFT with telemetry gate enabled,
because GPS/local telemetry is still an independent reference in this phase:

```bash
python3 live_sift_nav.py \
  --map-source sift-master \
  --host 127.0.0.1 \
  --port 8080 \
  --mavlink udp:127.0.0.1:14550 \
  --send-vision \
  --no-send-vision-speed \
  --vision-publish-mode fix \
  --vision-max-age-sec 1.0 \
  --vision-z-source zero \
  --vision-attitude-source zero \
  --telemetry-seed-source global \
  --telemetry-max-age-sec 1.0 \
  --interval-sec 0.15 \
  --search-radius-m 150 \
  --max-search-radius-m 300 \
  --max-tiles-per-scale 9 \
  --early-stop-inliers 80 \
  --min-nav-inliers 50 \
  --max-nav-error-m 3.0 \
  --max-nav-jump-m 5 \
  --log-csv live_sift_master_vpe_gps_gate.csv
```

Expected:

```text
Vision TX = sending in fix mode
sent_count increases only when a new visual fix is accepted
actual VPE rate should be near accepted SIFT rate, about 5-6 Hz
nav_status mostly OK
no EKF variance
vehicle remains on source set 1 / GPS
```

Observed:

```text
CSV = live_sift_master_vpe_gps_gate_fix.csv
duration = 113 s
accepted / VPE fix rate = 5.89 Hz
nav rejected = 0
duration p95 = 0.176 s
accepted calc_error p95 = 1.66 m
accepted calc_error max = 2.80 m
accepted errors over 3 m = 0
EKF flags = 831
pos_horiz_variance = 0.0023
```

Decision:

```text
Phase 1 passed. SIFT VPE in fix mode is healthy while source set 1/GPS remains
active. Proceed to Phase 2 source set 2 switch test.
```

### Phase 2 - Source set 2 switch test

Stop the phase 1 process. Restart SIFT VPE with telemetry position gate disabled,
because after switching to source set 2 the autopilot telemetry is no longer an
independent truth reference:

```bash
python3 live_sift_nav.py \
  --map-source sift-master \
  --host 127.0.0.1 \
  --port 8080 \
  --mavlink udp:127.0.0.1:14550 \
  --send-vision \
  --no-send-vision-speed \
  --vision-publish-mode fix \
  --vision-max-age-sec 1.0 \
  --vision-z-source zero \
  --vision-attitude-source zero \
  --telemetry-seed-source global \
  --telemetry-max-age-sec 1.0 \
  --no-telemetry-position-gate \
  --interval-sec 0.15 \
  --search-radius-m 150 \
  --max-search-radius-m 300 \
  --max-tiles-per-scale 9 \
  --early-stop-inliers 80 \
  --min-nav-inliers 50 \
  --max-nav-jump-m 5 \
  --log-csv live_sift_master_vpe_src2.csv
```

After the dashboard shows fresh fixes and `Vision TX = sending`, switch with
rollback protection:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  safe-switch-source-set \
  --to-set 2 \
  --rollback-set 1 \
  --monitor-sec 60 \
  --max-pos-horiz-variance 2.0 \
  --max-velocity-variance 0.7
```

If phase 2 stays stable for 60 s, test a small guided movement at low speed.

Observed first Phase 2 attempt:

```text
CSV = live_sift_master_vpe_src2.csv
vehicle mode during switch = AUTO mission
vehicle speed during switch = roughly 3-10 m/s
switch to set 2 = accepted
rollback reason = pos_horiz_variance=2.4473 > 2.0000
rollback to set 1 = accepted
post-rollback EKF flags = 831
post-rollback pos_horiz_variance = 0.0466
```

Decision:

```text
Do not treat this as a failed hover source-set-2 test. It was too aggressive:
the vehicle was moving fast in AUTO while receiving VPE-only position fixes with
no VISION_SPEED_ESTIMATE.

Repeat Phase 2 in GUIDED hover at 100 m. If stable for 60 s, command only a
small low-speed movement.
```

Recommended repeat command:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  safe-switch-source-set \
  --to-set 2 \
  --rollback-set 1 \
  --monitor-sec 60 \
  --max-pos-horiz-variance 2.0 \
  --max-velocity-variance 0.7
```

Strict hover attempt:

```text
CSV = live_sift_master_vpe_src2_strict.csv
duration = 55 s
accepted rate = 4.95 Hz
rejected = 16, prediction_jump_gate only
accepted calc_error p95 = 1.85 m
accepted calc_error max = 5.57 m
post-test EKF flags = 831
post-test pos_horiz_variance = 0.0053
```

Decision:

```text
The stricter SIFT gates helped, but source set 2 still did not hold long enough.
The next variable is EKF trust in ExternalNav. VISO_POS_M_NSE=2.0 is probably
too confident for this SIFT stream during source set 2.
```

Next test:

```text
Set VISO_POS_M_NSE=4.0, keep the same strict SIFT command, then repeat the
GUIDED hover source-set-2 switch. If it holds, try VISO_POS_M_NSE=3.0 later.
```

Observed with VISO_POS_M_NSE=4.0:

```text
CSV = live_sift_master_vpe_src2_noise4.csv
duration = 76 s
accepted rate = 4.05 Hz
rejected = 34, prediction_jump_gate only
accepted calc_error max = 6.26 m where telemetry comparison was available
post-test EKF flags = 831
post-test pos_horiz_variance = 0.0041
```

Decision:

```text
VISO_POS_M_NSE=4.0 did not solve the source-set-2 hold problem. Stop tuning only
EKF noise. The next useful work is:

1. Add independent Gazebo ground-truth logging to source-set-2 SIFT tests.
2. Add a temporal/innovation filter before publishing SIFT VPE to EKF.
```

## Best Master-Map Observe Baseline

Use this as the current SIFT master-map baseline before enabling VPE:

```bash
python3 live_sift_nav.py \
  --map-source sift-master \
  --host 127.0.0.1 \
  --port 8080 \
  --mavlink udp:127.0.0.1:14550 \
  --no-send-vision \
  --telemetry-seed-source global \
  --interval-sec 0.15 \
  --search-radius-m 150 \
  --max-search-radius-m 300 \
  --max-tiles-per-scale 9 \
  --early-stop-inliers 80 \
  --min-nav-inliers 50 \
  --max-nav-jump-m 5 \
  --log-csv live_sift_master_512_fov65_nozoom_observe.csv
```

## Best Live Baseline So Far

Run `live_sift_nav.py` with this baseline before further changes:

```bash
python3 live_sift_nav.py \
  --host 127.0.0.1 \
  --port 8080 \
  --scales '1/4x' \
  --mavlink udp:127.0.0.1:14550 \
  --send-vision \
  --no-send-vision-speed \
  --vision-publish-mode fix \
  --vision-rate-hz 10 \
  --vision-max-age-sec 3.0 \
  --vision-z-source zero \
  --vision-attitude-source zero \
  --no-telemetry-position-gate \
  --telemetry-seed-source global \
  --telemetry-max-age-sec 1.0 \
  --search-radius-m 90 \
  --max-search-radius-m 130 \
  --search-nav-max-age-sec 0 \
  --max-tiles-per-scale 1 \
  --early-stop-inliers 160 \
  --min-nav-inliers 140 \
  --max-nav-jump-m 0 \
  --log-csv live_sift_vpe_z0_1tile_inl140_nojump.csv
```

Safe switch debug command:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  safe-switch-source-set \
  --to-set 2 \
  --rollback-set 1 \
  --monitor-sec 0 \
  --max-pos-horiz-variance 2.0 \
  --max-velocity-variance 0.7
```

## Best Test Result So Far

From the latest segment of `live_sift_vpe_z0_1tile_inl140_nojump.csv`:

```text
duration = 150 s
accepted fixes = 428
accepted fix rate = 2.85 Hz
rejected fixes = 120
reject reasons = low_inliers only
accepted duration mean = 0.20 s
accepted duration p95 = 0.23 s
max gap between accepted fixes = 3 s
last accepted fix was at the end of the log
```

But:

```text
accepted calc_error p95 = about 4.1 m
accepted calc_error max = about 5.7 m
```

This means latency and starvation improved, but raw SIFT jitter/outliers remain.

## Next Implementation

Add a temporal estimator layer, likely in:

```text
sau_sift_nav/estimator.py
```

The estimator should receive accepted raw SIFT fixes and produce a filtered visual
pose for VPE.

Suggested behavior:

```text
1. Keep the last filtered north/east position.
2. Estimate visual velocity from recent filtered fixes.
3. For each new raw SIFT fix, compute dt and expected position.
4. Reject physically implausible jumps based on speed and elapsed time.
5. Down-weight low-quality fixes based on inliers, good_count, tile count, and age.
6. Smooth accepted fixes with EMA or a small median/EMA hybrid.
7. Publish only the filtered visual pose, not the raw SIFT fix.
8. Mark estimator status in dashboard and CSV.
```

First simple version:

```text
filtered = alpha * raw + (1 - alpha) * predicted
alpha depends on quality:
  high inliers and low jump -> alpha around 0.4-0.7
  weak fix -> alpha around 0.1-0.3
  implausible jump -> reject
```

Start with conservative limits:

```text
max_visual_speed_mps = 5.0
max_jump_extra_m = 1.5
ema_alpha_good = 0.45
ema_alpha_weak = 0.20
min_filter_inliers = 140
```

## Success Criteria

Before returning to GPS-denied tests, target:

```text
fresh filtered VPE >= 3 Hz, ideally 4-5 Hz
accepted duration p95 < 0.30 s
filtered error p95 < 2 m during slow Go To
no VPE starvation tail
EKF flags remain 831
no EKF variance / LAND during source set 2
```

## Important Notes

Do not re-enable telemetry position gate after source set 2:

```text
LOCAL_POSITION_NED is no longer independent once EKF fuses our VPE.
```

For GPS-on evaluation only, keep:

```text
--telemetry-seed-source global
```

This gives dashboard/CSV truth while the acceptance gate remains off.

## Implemented First Filter Pass

The first estimator pass is implemented inside `sau_sift_nav/state.py` rather
than a separate `estimator.py` file so that the change stays small:

```text
nav_filter_alpha:
  1.0 = old raw SIFT behavior
  <1.0 = EMA toward the new SIFT fix from the previous visual prediction

max_nav_step_speed_mps + max_nav_step_slack_m:
  reject a new SIFT fix if it implies an implausible visual-only step from the
  previous accepted visual estimate.
```

Also added:

```text
--gazebo-truth
```

This subscribes to Gazebo pose info and logs independent truth into the
dashboard/CSV. It does not publish anything to ArduPilot.

### Test VPE-5 - Source set 2 with filter + Gazebo truth

Start live SIFT first while still on source set 1 / GPS:

```bash
python3 live_sift_nav.py \
  --map-source sift-master \
  --host 127.0.0.1 \
  --port 8080 \
  --mavlink udp:127.0.0.1:14550 \
  --send-vision \
  --no-send-vision-speed \
  --vision-publish-mode fix \
  --vision-max-age-sec 1.0 \
  --vision-z-source zero \
  --vision-attitude-source zero \
  --telemetry-seed-source global \
  --telemetry-max-age-sec 1.0 \
  --no-telemetry-position-gate \
  --gazebo-truth \
  --gz-topic /world/iris_runway/pose/info \
  --gz-model iris_with_gimbal \
  --gazebo-truth-bootstrap telemetry \
  --interval-sec 0.15 \
  --search-radius-m 150 \
  --max-search-radius-m 220 \
  --max-tiles-per-scale 9 \
  --early-stop-inliers 180 \
  --min-nav-inliers 80 \
  --max-nav-jump-m 0 \
  --nav-filter-alpha 0.35 \
  --nav-filter-reset-residual-m 5.0 \
  --max-nav-step-speed-mps 3.0 \
  --max-nav-step-slack-m 1.0 \
  --log-csv live_sift_master_vpe_src2_filter_gt_3.csv
```

Expected before switching:

```text
dashboard shows gazebo truth = receiving
CSV truth_source = gazebo
gazebo_error_m is close to the visual error seen on the dashboard
Vision TX = sending fix count increases only on new accepted fixes
```

Then switch with rollback guard:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  safe-switch-source-set \
  --to-set 2 \
  --rollback-set 1 \
  --monitor-sec 90 \
  --max-pos-horiz-variance 2.0 \
  --max-velocity-variance 0.7
```

After the run:

```bash
python3 scripts/summarize_sift_csv.py live_sift_master_vpe_src2_filter_gt_3.csv
```

What to look for:

```text
truth_source should mostly be gazebo
gazebo_error_m p95 should stay under about 2 m in hover / slow Go To
nav_filter_residual_m should reveal rejected or smoothed SIFT jumps
reject_reason should mostly be empty or visual_step_speed_gate
EKF flags should remain 831 without EKF variance / LAND
```

VPE-6 showed that `prediction_jump_gate` is harmful in source set 2 because it
uses AP local state, which is no longer independent. It also showed that
fallback/relocalization must reset the visual filter instead of blending against
stale visual prediction. Both changes are reflected in the command above.

### Test VPE-8 - Source switch with frozen telemetry alignment

VPE-7/VPE-8 analysis showed an important source-switch issue: even while the
vehicle is not commanded to move, switching to EKF source set 2 can make it
start drifting. The likely cause is that the raw SIFT/map pose is not exactly in
the same XY frame as ArduPilot's source-set-1 GPS/local estimate at the switch
instant. A stable 1-2 m offset is enough for the controller to chase the wrong
hold point.

Use `--vision-align-source telemetry` while still on source set 1. The first
valid visual fix is compared against the fresh telemetry seed, then that offset
is frozen and applied only to the outgoing `VISION_POSITION_ESTIMATE` x/y. The
SIFT search itself remains in the map/Gazebo frame.

```bash
python3 live_sift_nav.py \
  --map-source sift-master \
  --host 127.0.0.1 \
  --port 8080 \
  --mavlink udp:127.0.0.1:14550 \
  --send-vision \
  --no-send-vision-speed \
  --vision-publish-mode fix \
  --vision-max-age-sec 1.0 \
  --vision-z-source zero \
  --vision-attitude-source zero \
  --vision-align-source telemetry \
  --vision-align-max-age-sec 1.0 \
  --telemetry-seed-source global \
  --telemetry-max-age-sec 1.0 \
  --no-telemetry-position-gate \
  --gazebo-truth \
  --gz-topic /world/iris_runway/pose/info \
  --gz-model iris_with_gimbal \
  --gazebo-truth-bootstrap map-origin \
  --interval-sec 0.15 \
  --search-radius-m 150 \
  --max-search-radius-m 220 \
  --max-tiles-per-scale 9 \
  --early-stop-inliers 180 \
  --min-nav-inliers 80 \
  --max-nav-jump-m 0 \
  --nav-filter-alpha 0.35 \
  --nav-filter-reset-residual-m 5.0 \
  --max-nav-step-speed-mps 3.0 \
  --max-nav-step-slack-m 1.0 \
  --log-csv live_sift_master_vpe_src2_aligned.csv
```

Expected before switching:

```text
vision align line appears on the dashboard
vision_align_offset_north_m/east_m are non-empty in CSV
vision_tx_north/east is nav_north/east plus that frozen offset
```

Then switch with the same rollback guard. Success means the vehicle stays in
hover after source set 2 instead of moving by itself. Only after this is stable
should `VISO_DELAY_MS` be swept for dynamic lag.
