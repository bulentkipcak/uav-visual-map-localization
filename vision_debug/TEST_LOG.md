# Vision ExternalNav Test Log

Date: 2026-05-31

## Active Direction - 2026-06-01

Current priority changed to VPE-only XY fusion.

Goal:

```text
Feed only ExternalNav x/y position to EKF.
Do not feed VISION_SPEED_ESTIMATE.
Let altitude come from barometer, yaw from compass, and vehicle dynamics from onboard IMU/accel.
```

Active source-set target:

```text
source set 1 = GPS for takeoff
source set 2 = ExternalNav position-only

EK3_SRC1_POSXY=3
EK3_SRC1_VELXY=3
EK3_SRC1_POSZ=1
EK3_SRC1_VELZ=3
EK3_SRC1_YAW=1

EK3_SRC2_POSXY=6
EK3_SRC2_VELXY=0
EK3_SRC2_POSZ=1
EK3_SRC2_VELZ=0
EK3_SRC2_YAW=1
```

Immediate test plan:

1. Use `mavlink_doctor.py set-takeoff-switch-params` with the new VPE-only default.
2. Restart SITL.
3. Take off with GPS/source set 1 to 80 m.
4. Run `gazebo_truth_bridge.py --no-send-speed --rate 4` so only VPE is published.
5. Switch to source set 2 with `safe-switch-source-set`.
6. Hover, then use `setspeed 2` and a small map Go To.
7. Record whether mode stays GUIDED and EKF variance stays low.
8. Repeat the Gazebo truth run with `--attitude-source zero` to isolate whether VPE roll/pitch/yaw fields matter.

Pass result means:

```text
Gazebo truth VPE-only is enough for navigation.
Next bottleneck is SIFT freshness/quality, not VSE.
```

Fail result means:

```text
Position-only ExternalNav is not enough in this SITL/ArduPilot setup, or VPE timing/source params still need work.
Then compare against VPE+VSE by setting EK3_SRC2_VELXY=6 and running gazebo truth with --send-speed.
```

After VPE-only is understood, start a separate SIFT optimization track:

```text
new map -> tile generation -> descriptor cache -> offline replay benchmark -> ROI/motion-model search -> live integration
target fresh accepted SIFT fixes at 4-5 Hz, not just MAVLink TX at 4 Hz
```

## Current Findings

### VPE-only Gazebo truth source set 2 - PASS

Date: 2026-06-01

Result:

```text
Gazebo ground truth VPE-only bridge was running.
EKF source set 2 was selected.
Vehicle held hover with light oscillation and accepted lateral movement.
No EKF failsafe or LAND was observed during the reported test.
```

Confirmed params:

```text
EK3_SRC1_POSXY=3
EK3_SRC1_VELXY=3
EK3_SRC1_POSZ=1
EK3_SRC1_VELZ=3
EK3_SRC1_YAW=1

EK3_SRC2_POSXY=6
EK3_SRC2_VELXY=0
EK3_SRC2_POSZ=1
EK3_SRC2_VELZ=0
EK3_SRC2_YAW=1
```

Status sample while moving in GUIDED:

```text
mode=GUIDED
LOCAL_POSITION_NED x=81.407 y=-7.709 z=-80.161 vx=0.849 vy=1.854 vz=-0.001
GLOBAL_POSITION_INT lat=40.7439133 lon=30.3312134 rel_alt=80.00 hdg=66.62
VFR_HUD heading=66 groundspeed=2.04 alt=293.16
EKF_STATUS_REPORT flags=831:ATTITUDE,VEL_HORIZ,VEL_VERT,POS_HORIZ_REL,POS_HORIZ_ABS,POS_VERT_ABS,PRED_POS_HORIZ_REL,PRED_POS_HORIZ_ABS
velocity_variance=0.0080
pos_horiz_variance=0.0144
pos_vert_variance=0.0085
compass_variance=0.0148
```

Interpretation:

```text
Position-only ExternalNav XY is sufficient for this SITL navigation test.
VISION_SPEED_ESTIMATE is not required for the Gazebo truth baseline.
The next bottleneck is SIFT estimate freshness/quality, not EKF's ability to use VPE-only XY.
```

### VPE-only zero attitude isolation - PASS

Date: 2026-06-01

Purpose:

```text
Check whether VPE roll/pitch/yaw fields affect EKF/source-set-2 behavior when
EK3_SRC2_YAW=1 and only ExternalNav XY is selected.
```

Command shape:

```bash
python3 vision_debug/gazebo_truth_bridge.py \
  --mavlink udp:127.0.0.1:14550 \
  --gz-topic /world/iris_runway/pose/info \
  --model iris_with_gimbal \
  --axis enu \
  --rate 4 \
  --no-send-speed \
  --speed-source zero \
  --attitude-source zero \
  --duration 0
```

Reported result:

```text
No problem observed.
Vehicle behavior remained normal after switching to source set 2.
```

Interpretation:

```text
With the active params, VPE roll/pitch/yaw are not required for this navigation path.
This supports the intended design: SIFT only needs to provide x/y; altitude, yaw,
attitude, velocity, and acceleration can remain on ArduPilot's normal sensor sources.
```

### Telemetry freshness guard added

Date: 2026-06-01

Observation:

```text
After switching to source set 2, LOCAL_POSITION_NED may stop arriving fresh on the
live/bridge process even though ArduPilot can still have a healthy EKF local position.
Also, source-set-2 LOCAL_POSITION_NED is no longer independent truth because it is
affected by the VPE stream we publish.
```

Code change:

```text
MAVLink telemetry submessages now carry receive timestamps.
live_sift_nav.py has --telemetry-max-age-sec, default 1.0.
Telemetry older than that is ignored for search seed, telemetry gate, local velocity,
VPE down fill, and truth/error display.
Dashboard now shows local/gps age and stale/fresh status.
CSV truth/error uses the selected fresh telemetry seed instead of stale LOCAL_POSITION_NED.
```

Expected effect:

```text
If LOCAL_POSITION_NED freezes after source set 2, SIFT will not keep using that old
local value as a gate/seed/velocity reference. With GPS available, use
--telemetry-seed-source global. In GPS-denied tests, disable telemetry gate and rely on
visual prediction, inlier checks, jump gate, and estimate age.
```

### SIFT VPE-only source set 2 - variance rollback

Date: 2026-06-01

Setup:

```text
live_sift_nav.py was publishing VPE-only SIFT estimates.
VISION_SPEED_ESTIMATE was not used.
EKF source set 2 was selected with EK3_SRC2_POSXY=6 and EK3_SRC2_VELXY=0.
```

Safe-switch result:

```text
flags stayed 831 during the run.
Rollback was triggered by watchdog threshold, not by missing horizontal flags:
ROLLBACK_REASON pos_horiz_variance=0.3608>0.3500
FAIL flags=831 vel_var=0.017 pos_var=0.361
```

Latest CSV snapshot from `live_sift_vpe_only.csv`:

```text
rows = 135
OK rows = 135
accepted mean_error = 1.42 m
accepted p95_error = 2.613 m
accepted max_error = 2.885 m
match duration mean = 1.284 s
match duration p95 = 1.531 s
mean inliers = 130.3
```

Interpretation:

```text
This is no longer an EKF/VPE-format blocker. The EKF accepts VPE-only and keeps
horizontal aiding flags, but current SIFT fixes are too slow/noisy for a strict
0.35 pos_horiz_variance watchdog. The next main work is SIFT quality/rate and
gating/ROI optimization.
```

Follow-up experiment added:

```text
live_sift_nav.py --vision-publish-mode fix
```

This sends VPE once per newly accepted SIFT fix instead of republishing the same latest
fix at a fixed rate. Purpose: isolate whether repeated stale SIFT measurements are
driving EKF position variance. Risk: if fresh SIFT fixes are below ArduPilot's healthy
ExternalNav rate, EKF may stop aiding.

### SIFT VPE-only fix publish mode - 30 s PASS

Date: 2026-06-01

Command shape:

```bash
python3 live_sift_nav.py \
  --send-vision \
  --no-send-vision-speed \
  --vision-publish-mode fix \
  --vision-rate-hz 10 \
  --vision-attitude-source zero \
  --telemetry-seed-source global \
  --telemetry-max-age-sec 1.0 \
  --max-tiles-per-scale 6 \
  --early-stop-inliers 160 \
  --min-nav-inliers 100 \
  --max-nav-error-m 2.0 \
  --max-nav-jump-m 5.0 \
  --log-csv live_sift_vpe_fixmode.csv
```

Safe-switch result:

```text
safe-switch-source-set --monitor-sec 30 --max-pos-horiz-variance 1.0 --max-velocity-variance 0.7
SAFE_SWITCH_OK
flags stayed 831
pos_horiz_variance stayed below 1.0 during the 30 s monitor
```

Recent CSV window:

```text
last ~27 s rows = 51
accepted fixes = 27
accepted fix rate ~= 1.0 Hz
all match row rate ~= 1.89 Hz
rejected = 24
rejected reasons: telemetry_position_gate=19, low_inliers=5
duration mean = 0.591 s
duration p95 = 1.618 s
```

Interpretation:

```text
Fix-mode is a useful isolation result: repeating the latest SIFT estimate at fixed
rate likely contributed to EKF position variance. Publishing only new accepted fixes
kept EKF healthy for 30 s, even though the fresh accepted SIFT rate is only about 1 Hz.

This is not the final solution. It proves the EKF/VPE path can tolerate this stream in
the short term, but SIFT still needs rate/latency/ROI optimization to reach reliable
4-5 Hz fresh accepted fixes.
```

### SIFT VPE-only fix mode without telemetry gate - variance rollback

Date: 2026-06-01

Setup:

```text
--vision-publish-mode fix
--no-send-vision-speed
--vision-attitude-source zero
--no-telemetry-position-gate
--min-nav-inliers 100
--max-nav-jump-m 5.0
```

Safe-switch result:

```text
EKF flags stayed 831.
Rollback was triggered by pos_horiz_variance, not by lost aiding:
ROLLBACK_REASON pos_horiz_variance=1.0505>1.0000
```

Recent CSV window:

```text
last 60 s:
rows = 139
OK = 137
REJECTED = 2
accepted fix rate ~= 2.28 Hz
duration mean = 0.425 s
duration p95 = 1.372 s
```

Important caveat:

```text
This run omitted --telemetry-seed-source global, so after source set 2 the dashboard/CSV
truth can fall back to LOCAL_POSITION_NED, which is not independent. Repeat evaluation
runs with --telemetry-seed-source global even if --no-telemetry-position-gate is active.
```

Interpretation:

```text
No-gate fix-mode is closer to GPS-denied operation, but current SIFT still causes EKF
position variance growth during movement. This supports moving to SIFT optimization:
better ROI, faster fresh fixes, stricter ambiguity handling, and more stable estimates.
```

### MAVLink / EKF path

- `VISION_POSITION_ESTIMATE` reaches ArduPilot.
- `VISION_SPEED_ESTIMATE` reaches ArduPilot.
- `MAV_CMD_SET_EKF_SOURCE_SET` works:

```text
COMMAND_ACK command=42007 result=0:MAV_RESULT_ACCEPTED
```

- EKF3 accepts ExternalNav as a source when data is continuous.

### Previous source set setup

Earlier source-set layout used VPE+VSE on source set 2:

```text
SIM_GPS_DISABLE=0
VISO_TYPE=1
VISO_POS_M_NSE=2.0
VISO_VEL_M_NSE=2.0
EK3_SRC1_POSXY=3
EK3_SRC1_VELXY=3
EK3_SRC1_POSZ=1
EK3_SRC1_VELZ=3
EK3_SRC1_YAW=1
EK3_SRC2_POSXY=6
EK3_SRC2_VELXY=6
EK3_SRC2_POSZ=1
EK3_SRC2_VELZ=0
EK3_SRC2_YAW=1
EK3_SRC_OPTIONS=0
```

This was useful to prove that ExternalNav fusion can work, but it is not the active
first target anymore. The active VPE-only target keeps `EK3_SRC2_VELXY=0`.

### Gazebo ground truth bridge

Gazebo ground truth bridge was stable in hover and at 4 Hz with the earlier VPE+VSE setup.

Historical command used:

```bash
python3 vision_debug/gazebo_truth_bridge.py \
  --mavlink udp:127.0.0.1:14550 \
  --gz-topic /world/iris_runway/pose/info \
  --model iris_with_gimbal \
  --axis enu \
  --rate 4 \
  --send-speed \
  --speed-source gz \
  --duration 0
```

The next test repeats this with `--no-send-speed` and `EK3_SRC2_VELXY=0`.

Observed healthy EKF state:

```text
EKF_STATUS_REPORT flags=831
velocity_variance low
pos_horiz_variance low
mode GUIDED
```

### Axis mapping

The correct Gazebo world XY to ArduPilot local NED mapping is currently:

```text
north = gazebo_y
east  = gazebo_x
```

Bridge option:

```text
--axis enu
```

Evidence from bootstrap:

```text
gz=(-12.195,-32.499,100.623)
local=(-32.510,-12.204,-99.969)
```

### Motion behavior

During movement, `err=(north,east,down)` grows, but after reaching the target it decreases again. This looks like timing / controller lag / EKF lag rather than a fixed axis error.

Representative pattern:

```text
err grows during motion
err returns close to zero after settling
EKF remains healthy with ground truth bridge
```

Current conclusion:

```text
ExternalNav fusion works with Gazebo ground truth.
4 Hz ExternalNav is acceptable in this SITL setup.
The remaining problem is likely SIFT estimate quality: jitter, bias, wrong tile/homography, timestamp age, or NED conversion.
```

## Next Tests

### Test A - SIFT observe-only against GPS/local NED

Purpose: Check SIFT estimate error without feeding SIFT into EKF.

Setup:

1. Start Gazebo and SITL cleanly.
2. Run `set-takeoff-switch-params`.
3. Restart SITL.
4. Use dummy vision only for pre-arm.
5. Take off with GPS/source set 1 to 80 m.
6. Stop dummy vision.

Run live SIFT without publishing vision:

```bash
python3 live_sift_nav.py \
  --host 127.0.0.1 \
  --port 8080 \
  --scales '1/4x' \
  --mavlink udp:127.0.0.1:14550 \
  --no-send-vision \
  --vision-rate-hz 4 \
  --vision-max-age-sec 3.0 \
  --vision-speed-source zero \
  --search-radius-m 120 \
  --max-search-radius-m 700 \
  --max-tiles-per-scale 10 \
  --early-stop-inliers 140 \
  --log-csv live_sift_observe.csv
```

Watch:

```text
Tahmin N/E
Gercek N/E
Hata
Tile
Inliers
REJECTED reason, if any
```

Pass condition:

```text
Hover error stays low, ideally < 3 m.
No large jumps between neighboring frames.
Tile remains plausible.
```

Fail clues:

```text
error > 10 m while hover -> map/NED bias or wrong homography
error jumps suddenly -> false tile or unstable homography
inliers high but error high -> map transform/georeference issue
```

Result on 2026-05-31:

```text
file=live_sift_observe.csv
OK rows=916
mean_error=0.199 m
min_error=0.007 m
max_error=0.568 m
mean_inliers=179.1
mean_duration=0.201 s
err_gt3m=0
err_gt5m=0
err_gt10m=0
tile=tile_r001_c003_x2304_y768.jpg for all OK rows
```

Dashboard sample:

```text
Tahmin N/E = 0.02 / -0.17 m
Gercek N/E = -0.02 / 0.02 m
Hata = 0.20 m
Inliers = 207 / good 331
Sure = 0.160 s
Video = streaming
MAVLink = receiving
```

Conclusion:

```text
PASS. Hovering SIFT estimate is accurate and stable when observed against GPS/local NED.
```

### Test B - SIFT observe-only while moving slowly

Purpose: Check whether SIFT remains stable during motion.

Keep Test A running with `--no-send-vision`. Give a small guided target:

```text
5 m to 10 m movement only
slow / gentle movement
```

Watch `live_sift_observe.csv` and dashboard.

Pass condition:

```text
error may grow during motion but settles back below ~3 m after stop.
no large false-tile jumps.
```

Result snapshot from `live_sift_observe.csv` after moving left/right:

```text
time range = 2026-05-31T22:25:48 .. 2026-05-31T22:37:04
overall OK rows = 1799
overall mean_error = 0.331 m
overall median_error = 0.236 m
overall max_error = 7.166 m

motion segment after initial hover:
rows = 883
mean_error = 0.467 m
median_error = 0.500 m
max_error = 7.166 m
err_gt1m = 28
err_gt3m = 4
err_gt5m = 2

after 22:32 movement:
rows = 531
mean_error = 0.608 m
median_error = 0.545 m
max_error = 7.166 m
err_gt1m = 19
err_gt3m = 4
err_gt5m = 2
```

Worst rows:

```text
22:32:27 err=7.166 m inliers=138 duration=1.244 s tiles=9/9
22:32:25 err=5.543 m inliers=134 duration=1.183 s tiles=9/9
22:32:24 err=3.717 m inliers=132 duration=1.302 s tiles=9/9
22:32:11 err=3.020 m inliers=130 duration=1.165 s tiles=8/8 tile_r001_c004
```

Conclusion:

```text
CONDITIONAL PASS.
SIFT does not get lost and the estimate settles after movement.
There are short transient spikes during motion, so raw SIFT should not be fused yet.
Next publish test must use tighter gates before switching EKF source set 2.
```

### Test C - SIFT publish while staying on GPS source set

Purpose: Check whether SIFT VPE/VSE stream itself is continuous and sane without switching EKF to ExternalNav.

Run live SIFT with publishing enabled, but keep source set 1:

```bash
python3 live_sift_nav.py \
  --host 127.0.0.1 \
  --port 8080 \
  --scales '1/4x' \
  --mavlink udp:127.0.0.1:14550 \
  --vision-rate-hz 4 \
  --vision-max-age-sec 3.0 \
  --vision-speed-source zero \
  --search-radius-m 120 \
  --max-search-radius-m 700 \
  --max-tiles-per-scale 10 \
  --early-stop-inliers 140 \
  --min-nav-inliers 130 \
  --max-nav-error-m 2.0 \
  --max-nav-jump-m 10.0 \
  --log-csv live_sift_publish_gps.csv
```

Keep EKF source set 1:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  switch-source-set \
  --set 1
```

Pass condition:

```text
Vision TX sending @ 4Hz
error stable
no stale_estimate
no big nav jumps
```

Result snapshot from `live_sift_publish_gps.csv`:

```text
time range = 2026-05-31T22:44:54 .. 2026-05-31T22:48:28
raw OK matches = 300
NO_MATCH = 1
accepted nav estimates = 114
rejected nav estimates = 186

accepted mean_error = 0.591 m
accepted median_error = 0.570 m
accepted max_error = 1.909 m
accepted err_gt2m = 0
accepted err_gt3m = 0

rejected mean_calc_error = 1.541 m
rejected max_calc_error = 9.625 m
rejected err_gt3m = 36
rejected err_gt5m = 18
```

Interpretation:

```text
Gate behavior PASS: bad visual fixes were rejected before becoming nav estimates.
Recovery behavior FAIL: accepted stream stopped after 22:46:14, search radius grew to 700 m,
and the matcher kept scanning wide regions even though GPS/local seed was still available.
```

Fix added after this test:

```text
1. Stale visual nav search hints now fall back to telemetry seed after --search-nav-max-age-sec.
2. telemetry_position_gate and prediction_jump_gate rejections reset search radius to base radius.
3. CSV logs now include calc_error_m, nav_status, reject_reason, reject_error_m, reject_jump_m.
```

### Test C2 - SIFT publish recovery check after gating fix

Purpose: Confirm that rejected matches no longer push search to max radius and that Vision TX keeps a continuous accepted stream.

Run with a new CSV file:

```bash
python3 live_sift_nav.py \
  --host 127.0.0.1 \
  --port 8080 \
  --scales '1/4x' \
  --mavlink udp:127.0.0.1:14550 \
  --vision-rate-hz 4 \
  --vision-max-age-sec 3.0 \
  --vision-speed-source zero \
  --search-radius-m 120 \
  --max-search-radius-m 250 \
  --search-nav-max-age-sec 2.0 \
  --max-tiles-per-scale 10 \
  --early-stop-inliers 140 \
  --min-nav-inliers 130 \
  --max-nav-error-m 2.0 \
  --max-nav-jump-m 10.0 \
  --log-csv live_sift_publish_gps_recovery.csv
```

Pass condition:

```text
nav_status alternates OK/REJECTED occasionally, but OK does not disappear for long.
search_radius_m mostly stays near 120 m.
accepted calc_error_m remains below 2 m.
Vision TX stays sending @ 4Hz.
```

Result snapshot from `live_sift_publish_gps_recovery.csv`:

```text
time range = 2026-05-31T22:53:14 .. 2026-05-31T22:55:44
raw OK matches = 240
accepted nav estimates = 46
rejected nav estimates = 194

accepted mean_error = 0.462 m
accepted median_error = 0.423 m
accepted max_error = 0.970 m
accepted err_gt1m = 0

rejected reasons:
telemetry_position_gate = 66
low_inliers = 46
prediction_jump_gate = 82

search_source:
vision_predicted = 46
local_ned_seed = 193
global = 1

search_radius_m:
120 m = 148 rows
125 m = 44 rows
250 m = 24 rows
216 m = 21 rows
```

Interpretation:

```text
PARTIAL PASS.
The 700 m runaway is fixed and accepted estimates are very clean.
However, after visual nav goes stale, prediction_jump_gate still compares new local-seed
matches against the old visual nav estimate and rejects many otherwise good fixes.
Example: calc_error_m < 2 m but prediction_jump_gate rejects 82 rows.
```

Second fix added after this test:

```text
prediction_jump_gate is now skipped when the previous visual nav estimate is older than
--search-nav-max-age-sec.
CSV now includes reject_nav_age_sec for jump-gate debugging.
```

### Test C3 - SIFT publish recovery check after stale jump-gate fix

Run the same scenario with a new CSV:

```bash
python3 live_sift_nav.py \
  --host 127.0.0.1 \
  --port 8080 \
  --scales '1/4x' \
  --mavlink udp:127.0.0.1:14550 \
  --vision-rate-hz 4 \
  --vision-max-age-sec 3.0 \
  --vision-speed-source zero \
  --search-radius-m 120 \
  --max-search-radius-m 250 \
  --search-nav-max-age-sec 2.0 \
  --max-tiles-per-scale 10 \
  --early-stop-inliers 140 \
  --min-nav-inliers 130 \
  --max-nav-error-m 2.0 \
  --max-nav-jump-m 10.0 \
  --log-csv live_sift_publish_gps_recovery2.csv
```

Expected improvement:

```text
prediction_jump_gate should be rare and should show low reject_nav_age_sec.
Good local-seed fixes with calc_error_m < 2 m should become accepted again.
```

Result snapshot from `live_sift_publish_gps_recovery2.csv`:

```text
time range = 2026-05-31T23:00:18 .. 2026-05-31T23:02:16
raw OK matches = 165
accepted nav estimates = 57
rejected nav estimates = 108

accepted mean_error = 1.345 m
accepted median_error = 1.260 m
accepted max_error = 1.979 m
accepted err_gt2m = 0

rejected reasons:
low_inliers = 57
telemetry_position_gate = 51
prediction_jump_gate = 0

search_source:
vision_predicted = 83
local_ned_seed = 81
global = 1
```

Interpretation:

```text
PASS for stale jump-gate fix: prediction_jump_gate disappeared.
PARTIAL PASS for publish quality: accepted fixes stay under the 2 m gate, but the mean
accepted error rose to 1.345 m during faster movement. This is probably motion/latency,
so guided speed must be limited before trying EKF source set 2.
```

Guided speed note:

```text
SITL params read back:
WPNAV_SPEED = 200 cm/s
WPNAV_ACCEL = 100 cm/s/s
GUID_OPTIONS = 0
LOIT_SPEED = 1250 cm/s
```

With `GUID_OPTIONS=0`, Guided position control may keep the speed limit that was active
when the Guided position submode started. For MAVProxy map Go To, use one of:

```text
setspeed 2
```

or set Guided to use WPNav for position control:

```text
param set GUID_OPTIONS 64
param set WPNAV_SPEED 200
param set WPNAV_ACCEL 100
mode GUIDED
```

### Test C4 - SIFT publish at 2 m/s Guided speed

Purpose: Re-run publish quality after limiting MAVProxy Guided map Go To with `setspeed 2`.

Command used:

```bash
python3 live_sift_nav.py \
  --host 127.0.0.1 \
  --port 8080 \
  --scales '1/4x' \
  --mavlink udp:127.0.0.1:14550 \
  --vision-rate-hz 4 \
  --vision-max-age-sec 3.0 \
  --vision-speed-source zero \
  --search-radius-m 120 \
  --max-search-radius-m 250 \
  --search-nav-max-age-sec 2.0 \
  --max-tiles-per-scale 10 \
  --early-stop-inliers 140 \
  --min-nav-inliers 130 \
  --max-nav-error-m 2.0 \
  --max-nav-jump-m 10.0 \
  --log-csv live_sift_2ms.csv
```

MAVProxy:

```text
setspeed 2
```

Result snapshot from `live_sift_2ms.csv`:

```text
time range = 2026-05-31T23:14:28 .. 2026-05-31T23:17:03
raw OK matches = 221
accepted nav estimates = 176
rejected nav estimates = 45

accepted mean_error = 0.746 m
accepted median_error = 0.685 m
accepted p95_error = 1.555 m
accepted max_error = 1.983 m
accepted err_gt2m = 0

rejected reasons:
low_inliers = 39
telemetry_position_gate = 6
prediction_jump_gate = 0

after 23:15:30:
accepted rows = 172
accepted mean_error = 0.723 m
accepted max_error = 1.909 m
```

Conclusion:

```text
PASS for source-set switch precondition.
At 2 m/s, accepted SIFT estimates remain under the 2 m gate and are usually below 1 m.
Next step can be guarded EKF source-set switch to ExternalNav while keeping setspeed 2.
```

### Source set 2 telemetry gate note

Important observation:

```text
After switching to EKF source set 2, LOCAL_POSITION_NED is no longer an independent
GPS/local reference. It is affected by the vision estimates that this app publishes.
If telemetry_position_gate rejects new SIFT fixes, the app keeps publishing the previous
vision estimate, EKF LOCAL_POSITION_NED can stay near that previous point, and the
dashboard blue telemetry marker appears stuck. This creates a self-locking rejection loop.
```

Code support added:

```text
--no-telemetry-position-gate
--telemetry-seed-source auto|local|global
```

Use `--telemetry-seed-source global` while GPS is still enabled during source-set-2
debugging. This keeps the dashboard blue marker and `telemetry_position_gate` tied to
`GLOBAL_POSITION_INT` instead of EKF `LOCAL_POSITION_NED`, which is no longer independent
after switching to ExternalNav.

Use `--no-telemetry-position-gate` only for real GPS-denied tests after SIFT quality and
rate are good enough. Keep other gates active:

```text
min_nav_inliers
prediction_jump_gate
vision max age
safe-switch-source-set rollback
```

Recommended source-set-2 live command while GPS is still available:

```bash
python3 live_sift_nav.py \
  --host 127.0.0.1 \
  --port 8080 \
  --scales '1/4x' \
  --mavlink udp:127.0.0.1:14550 \
  --vision-rate-hz 4 \
  --vision-max-age-sec 3.0 \
  --vision-speed-source zero \
  --search-radius-m 120 \
  --max-search-radius-m 250 \
  --search-nav-max-age-sec 2.0 \
  --max-tiles-per-scale 10 \
  --early-stop-inliers 140 \
  --min-nav-inliers 120 \
  --telemetry-seed-source global \
  --max-nav-error-m 2.0 \
  --max-nav-jump-m 5.0 \
  --log-csv live_sift_src2_globalgate.csv
```

MAVProxy before and during the switch:

```text
setspeed 2
```

### Test D - SIFT guarded source switch

Purpose: Use SIFT as ExternalNav only after Test A-C look stable.

Run:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  safe-switch-source-set \
  --to-set 2 \
  --rollback-set 1 \
  --monitor-sec 0 \
  --max-velocity-variance 1.0 \
  --max-pos-horiz-variance 1.5
```

Only run this if:

```text
Vision TX sending
SIFT error low
no large jumps for at least 20 s
vehicle hovering or moving very slowly
```

Pass condition:

```text
No ROLLBACK
No EKF variance
Mode remains GUIDED
Position remains controlled
```

### Test E - Velocity isolation if SIFT switch fails

If Test D fails but Test A-C look good, repeat with velocity influence reduced:

1. Keep `--vision-speed-source zero`.
2. Try source set 2 with `EK3_SRC2_VELXY=0`.

Expected interpretation:

```text
position-only stable -> velocity fusion issue
position-only unstable -> SIFT position/timing issue
```

### Test F - VPE fix mode with telemetry gate disabled

Command under test:

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
  --vision-attitude-source zero \
  --no-telemetry-position-gate \
  --telemetry-max-age-sec 1.0 \
  --search-radius-m 120 \
  --max-search-radius-m 250 \
  --search-nav-max-age-sec 2.0 \
  --max-tiles-per-scale 6 \
  --early-stop-inliers 160 \
  --min-nav-inliers 100 \
  --max-nav-error-m 2.0 \
  --max-nav-jump-m 5.0 \
  --log-csv live_sift_vpe_fixmode.csv
```

Safe-switch result:

```text
EKF flags stayed 831.
Velocity variance stayed low at 0.007.
Rollback reason was pos_horiz_variance=1.0505>1.0000.
```

Interpretation:

```text
This is not a VPE health/message-format failure. ArduPilot keeps accepting
ExternalNav position. The remaining issue is horizontal position variance growth,
most likely caused by SIFT estimate jitter/fix cadence during motion.
```

Note:

```text
For GPS-on evaluation, add --telemetry-seed-source global so dashboard truth and
CSV error remain independent of EKF local position after switching to source set 2.
```

Follow-up change:

```text
live_sift_nav now has --vision-z-source zero|telemetry and the default is zero.
This keeps SIFT VPE aligned with the manual VPE-only tests:
x/y from SIFT, z=0, roll/pitch/yaw=0 when --vision-attitude-source zero is used.

Because EK3_SRC2_POSZ=1, EKF source set 2 should keep using barometer for vertical
position; the VPE z field is still present because the MAVLink message requires it.

--no-telemetry-position-gate is the right mode after switching to source set 2,
because LOCAL_POSITION_NED is no longer an independent truth source. For GPS-on
evaluation, --telemetry-seed-source global can still be used for dashboard/CSV truth
and search seeding while the acceptance gate remains off.
```

### Test G - SIFT VPE z=0, telemetry gate off

Safe-switch output:

```text
EKF flags stayed 831 throughout the run.
Velocity variance stayed low at 0.008.
Rollback reason was pos_horiz_variance=1.7334>1.5000.
The vehicle reached roughly 3.8 m/s local velocity during the run.
```

CSV summary from `live_sift_vpe_z0_nogate_global.csv`:

```text
rows = 156 over 132 s
accepted fixes = 155
accepted fix rate = 1.17 Hz
rejected fixes = 1 prediction_jump_gate
calc_error mean = 0.85 m
calc_error p95 = 1.94 m
calc_error max = 6.29 m
match duration mean = 0.85 s
match duration p95 = 1.55 s
inliers mean = 156.9
```

Interpretation:

```text
VPE message format, z=0, rpy=0, and VSE-off mode are accepted by EKF.
The remaining blocker is not message validity; it is SIFT fix cadence and jitter
during motion. 1.17 Hz is below the desired 4-5 Hz target.
```

### Test H - `setspeed 1` longer hold, eventual ExternalNav timeout

Safe-switch result:

```text
Vehicle stayed on source set 2 longer than the 2 m/s test.
EKF flags stayed 831 for most of the run.
Console showed EKF lane switch / GPS glitch messages around rollback.
Final rollback reason was flags_missing=792, current flags=37.
```

CSV evidence from `live_sift_vpe_z0_nogate_global.csv`:

```text
rows = 308
accepted fixes = 307
rejected fixes = 1
duration mean = 0.80 s
duration p95 = 1.53 s
duration max = 12.88 s
calc_error mean = 0.86 m
calc_error p95 = 1.86 m
calc_error max = 15.58 m
inliers mean = 160.8
tiles_scanned p95 = 6
tiles_scanned max = 40
```

Critical event:

```text
13:18:39 -> 13:18:52: 13 s gap between accepted fixes.
The 13:18:52 fix used search_source=global, scanned 40 tiles, took 12.88 s,
and had 15.58 m error.
```

Interpretation:

```text
At low speed the system lasts longer, but a single slow/global fallback can stop
fresh VPE updates long enough for EKF to lose ExternalNav horizontal position
flags. SIFT needs bounded-latency matching and/or a continuous estimator layer
before source set 2 can be robust.
```

### Test I - No global fallback, tighter tile cap

Settings changed:

```text
--search-nav-max-age-sec 0
--max-search-radius-m 180
--max-tiles-per-scale 3
```

Safe-switch result:

```text
System behaved much better than the global fallback run.
No 12 s matching stall occurred.
EKF flags stayed 831 until rollback.
Rollback reason was pos_horiz_variance=1.5672>1.5000.
```

CSV summary from `live_sift_vpe_z0_nogate_no_global_fallback.csv`:

```text
rows = 220 over 90 s
accepted fixes = 220
accepted fix rate = 2.44 Hz
rejected fixes = 0
duration mean = 0.40 s
duration p95 = 0.77 s
duration max = 0.89 s
calc_error mean = 0.82 m
calc_error median = 0.34 m
calc_error p95 = 3.03 m
calc_error max = 4.42 m
inliers mean = 164.4
tiles_scanned mean = 1.59
tiles_scanned p95 = 3
```

Quality correlation:

```text
tiles_scanned=1 fixes were very clean: roughly 0.3-0.5 m error.
tiles_scanned=3 fixes carried most of the 3-4 m error.
inliers >= 150 were also much cleaner than lower-inlier fixes.
```

Interpretation:

```text
The bounded-latency search improved the system substantially. Remaining oscillation
looks like accepting weaker multi-tile SIFT fixes, not a MAVLink/VPE issue.
Next quick test should tighten SIFT acceptance before deeper optimization.
```

### Test J - Strict one-tile gate

Settings changed:

```text
--search-radius-m 90
--max-search-radius-m 130
--max-tiles-per-scale 1
--min-nav-inliers 150
--max-nav-jump-m 2.0
```

Safe-switch result:

```text
Vehicle behavior was visibly better.
Accepted fixes were very clean.
EKF flags stayed 831 for a long section, then dropped to 37 after VPE starvation.
Final rollback reason was flags_missing=792, current flags=37.
```

CSV summary from `live_sift_vpe_z0_strict_1tile.csv`:

```text
rows = 352 over 100 s
accepted fixes = 210
accepted fix rate = 2.10 Hz overall
rejected fixes = 142
reject reasons = low_inliers 133, prediction_jump_gate 9
accepted duration mean = 0.27 s
accepted duration p95 = 0.33 s
accepted calc_error mean = 0.61 m
accepted calc_error p95 = 0.75 m
accepted calc_error max = 0.81 m
accepted inliers mean = 163.7
tiles_scanned = 1 for all fixes
```

Critical event:

```text
Last accepted fix was at 13:29:49.
The log continued until 13:30:21, so there was roughly 32 s without an accepted
VPE update. This explains the eventual ExternalNav flag loss.
```

Threshold sweep:

```text
min_inliers 150 -> very clean, but can starve VPE
min_inliers 140 -> estimated 2.70 Hz accepted rate, p95 error around 1.45 m
min_inliers 130 -> estimated 3.18 Hz accepted rate, p95 error around 2.94 m
```

Interpretation:

```text
One-tile strict matching is the best behavior so far, but the 150-inlier gate is
too strict for continuous EKF feeding. The next compromise test should keep the
one-tile bounded latency, lower min_nav_inliers to about 140, and disable or relax
the prediction jump gate because stale nav caused recovery rejections.
```

### Test K - One tile, inliers 140, jump gate disabled

Settings changed:

```text
--max-tiles-per-scale 1
--min-nav-inliers 140
--max-nav-jump-m 0
```

Safe-switch result:

```text
This was visibly better than the stricter 150-inlier run.
EKF flags stayed 831 through the run.
Final rollback reason was pos_horiz_variance=1.5542>1.5000.
This rollback was caused by the debug guard threshold, not by ExternalNav flag loss.
```

CSV summary from `live_sift_vpe_z0_1tile_inl140_nojump.csv`:

```text
rows = 1068 over 307 s
accepted fixes = 756
accepted fix rate = 2.46 Hz
rejected fixes = 312
reject reasons = low_inliers 312
accepted duration mean = 0.27 s
accepted duration p95 = 0.34 s
accepted calc_error mean = 1.06 m
accepted calc_error median = 0.76 m
accepted calc_error p95 = 3.24 m
accepted calc_error max = 4.32 m
tiles_scanned = 1 for all fixes
last accepted fix was at the end of the log, so no VPE starvation tail occurred
```

Threshold sweep on this run:

```text
min_inliers 140 -> 2.46 Hz, p95 error 3.24 m
min_inliers 145 -> 1.97 Hz, p95 error 3.20 m
min_inliers 150 -> 1.48 Hz, p95 error 3.21 m
min_inliers 160 -> 0.65 Hz, p95 error 2.91 m
```

Interpretation:

```text
Increasing the inlier threshold does not remove the remaining outliers enough to
justify the lower fix rate. The 1-tile / 140-inlier / no-jump-gate setup is the
best live tuning so far. Remaining improvement should come from a visual temporal
filter/estimator or SIFT pipeline optimization, not just static thresholds.
```

### Test L - Same live tuning, safe-switch variance guard 2.0

Safe-switch command changed:

```text
--max-pos-horiz-variance 2.0
```

Safe-switch result:

```text
The vehicle stayed on source set 2 longer than the 1.5 guard run.
EKF flags stayed 831 for most of the run.
Near the end, EKF lane switched and horizontal position flags dropped to 37.
Final rollback reason was flags_missing=792, current flags=37.
```

Latest CSV segment from `live_sift_vpe_z0_1tile_inl140_nojump.csv`:

```text
segment time = 13:42:19 -> 13:44:49
duration = 150 s
rows = 548
accepted fixes = 428
accepted fix rate = 2.85 Hz
rejected fixes = 120
reject reasons = low_inliers 120
accepted duration mean = 0.20 s
accepted duration p95 = 0.23 s
accepted calc_error mean = 1.17 m
accepted calc_error median = 0.69 m
accepted calc_error p95 = 4.11 m
accepted calc_error max = 5.72 m
max gap between accepted fixes = 3 s
last accepted fix was at the end of the log
```

Interpretation:

```text
The latency/fix-rate side is now much better; VPE starvation is not the main issue
in this run. The remaining failure is EKF losing trust after enough raw SIFT
position jitter/outliers and aggressive motion. This confirms the next step should
be a temporal visual estimator/filter before publishing VPE, not more static
inlier threshold tuning.
```

## Current Decision Summary

Status:

```text
VPE-only XY path is validated enough to stop focusing on MAVLink format.
Best live tuning so far is 1-tile / min_nav_inliers=140 / no jump gate.
Latency is acceptable for the next stage, but raw SIFT outliers remain.
```

Active baseline:

```text
--vision-publish-mode fix
--no-send-vision-speed
--vision-z-source zero
--vision-attitude-source zero
--no-telemetry-position-gate
--search-radius-m 90
--max-search-radius-m 130
--search-nav-max-age-sec 0
--max-tiles-per-scale 1
--min-nav-inliers 140
--max-nav-jump-m 0
```

Next work:

```text
Implement a temporal visual estimator/filter:
raw SIFT fix -> filtered visual pose -> VPE.
```

Details are in:

```text
vision_debug/SIFT_NEXT_STEPS.md
```

### Test M - New SIFT master-map patch DB

Input files:

```text
QGIS/SAU CAMPUS/output/SIFT/SIFT_master.png
QGIS/SAU CAMPUS/output/SIFT/SIFT_master.png.aux.xml
```

The aux geotransform matches the intended local map:

```text
x_min = -750
y_max = 750
pixel_size = 0.1220703125 m/px
image row increases downward, world-y/north decreases downward
```

Build command:

```bash
python3 scripts/build_sift_master_patches.py
```

Build result:

```text
master_size = 12288 x 12288
grid = 11 x 11
patch_count = 121
patch_size = 2048 px
step = 1024 px
total_keypoints = 181539
elapsed = 167.1 s
```

Output files:

```text
QGIS/SAU CAMPUS/output/SIFT/patches/patch_metadata.json
QGIS/SAU CAMPUS/output/SIFT/patches/patch_metadata.csv
QGIS/SAU CAMPUS/output/SIFT/patches/sift/*.npz
QGIS/SAU CAMPUS/output/SIFT/patches/sift_database_manifest.json
QGIS/SAU CAMPUS/output/SIFT/patches/sift_database.pkl
```

Metadata verification:

```text
patch_metadata patches = 121
patch PNG count = 121
patch NPZ count = 121
patch_r00_c00 world extent = x[-750,-500], y[500,750]
patch_r10_c10 world extent = x[500,750], y[-750,-500]
```

Runtime smoke test:

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

Smoke result:

```text
status = OK
scale_name = sift_master
tile_img = patch_r05_c06.png
tiles_considered = 9
tiles_scanned = 9
good_count = 75
inliers = 42
duration = 0.159 s
pred_ned ~= north 22.6 m, east 251.2 m
```

Interpretation:

```text
The new DB generation and runtime loading path works. Accuracy of this single
smoke match is not a navigation conclusion because frame_80m.jpg is only a quick
offline compatibility frame. The next real benchmark should use live Gazebo video
with GPS truth logging, 3x3 patch ROI, and VPE disabled first.
```

### Test N - Live master-map observe, VPE disabled

Command:

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
  --log-csv live_sift_master_observe.csv
```

CSV result:

```text
duration = 40 s
rows = 144
raw SIFT OK = 144 / 144
raw fix rate = 3.6 Hz
nav accepted = 14
nav rejected = 130
reject reasons = low_inliers 126, prediction_jump_gate 4
tile = patch_r04_c05.png for every row
tiles_considered median = 9
duration mean = 0.216 s
duration p95 = 0.229 s
raw calc_error mean = 1.06 m
raw calc_error median = 0.87 m
raw calc_error p95 = 1.91 m
raw calc_error max = 13.30 m
```

Threshold sweep on this run:

```text
min_inliers 50 -> 3.60 Hz, p95 error 1.91 m, max error 13.30 m
min_inliers 60 -> 3.25 Hz, p95 error 1.91 m, max error 2.83 m
min_inliers 65 -> 2.70 Hz, p95 error 1.94 m, max error 2.83 m
min_inliers 70 -> 2.08 Hz, p95 error 1.94 m, max error 2.83 m
min_inliers 75 -> 1.02 Hz, p95 error 1.80 m, max error 2.47 m
min_inliers 80 -> 0.45 Hz, p95 error 1.80 m, max error 1.94 m
```

Interpretation:

```text
The new master-map DB is substantially more accurate in this short live observe
run than the old tile baseline. The old default min_nav_inliers=80 is too strict
for the resized master-map frame; it accepts only 14 fixes despite low raw error.
min_nav_inliers=60 is the best next test point because it removes the single bad
global initialization outlier while keeping about 3.25 Hz.

early_stop_inliers=160 is unrealistic in this setup because observed inliers
mostly sit around 60-90. Use early_stop_inliers around 90 for the next master-map
tests.
```

### Test O - Live master-map observe with min_nav_inliers=60

Command:

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

CSV result:

```text
duration = 124 s
rows = 447
raw SIFT OK = 447 / 447
nav accepted = 347
nav rejected = 100
accepted rate = 2.80 Hz
reject reasons = low_inliers 99, prediction_jump_gate 1
duration mean = 0.192 s
duration p95 = 0.224 s
accepted duration p95 = 0.233 s
accepted calc_error mean = 1.03 m
accepted calc_error median = 0.96 m
accepted calc_error p95 = 2.08 m
accepted calc_error max = 2.74 m
accepted errors over 3 m = 0
```

Tile distribution:

```text
patch_r05_c04 = 116
patch_r05_c05 = 102
patch_r04_c05 = 101
patch_r06_c06 = 60
patch_r04_c04 = 32
others = nearby overlap patches only
```

Camera config observed in Gazebo model:

```text
model = <ardupilot_gazebo>/models/gimbal_small_3d/model.sdf
camera image = 640 x 480
horizontal_fov = 2.0 rad = 114.6 deg
update_rate = 10 Hz
sensor pose = 0 0 0 -1.57 -1.57 0
```

Interpretation:

```text
The master-map SIFT path is now accurate enough for observation. The current
camera is not the target baseline used when designing the 250 m patch / 127 m
footprint assumption. At 80-100 m altitude, 114.6 deg FOV and 640 px width gives
much lower ground resolution and a much wider footprint than the intended
1024x1024, 65 deg camera.

Before feeding source set 2 with this master-map path, update the Gazebo camera
to the intended baseline or record that the actual test baseline is 640x480 /
114.6 deg. The current wide camera still works, but it likely caps inlier quality
and accepted rate.
```

### Camera Update - Master-map baseline

Updated Gazebo model:

```text
<ardupilot_gazebo>/models/gimbal_small_3d/model.sdf
```

Change:

```diff
- horizontal_fov = 2.0 rad
- width/height = 640 x 480
+ horizontal_fov = 1.134464 rad
+ width/height = 1024 x 1024
```

Backup:

```text
<ardupilot_gazebo>/models/gimbal_small_3d/model.sdf.codexbak
```

Expected camera footprint:

```text
80 m altitude  = 101.9 m wide
100 m altitude = 127.4 m wide
```

This matches the intended 100 m / 65 degree master-map SIFT baseline. Gazebo must
be restarted before the change is active.

### Test P - 100 m observe after 1024x1024 / 65 deg camera update

Command:

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
  --log-csv live_sift_master_1024_fov65_observe.csv
```

CSV result:

```text
duration = 136 s
rows = 490
raw SIFT OK = 490 / 490
nav accepted = 400
nav rejected = 90
accepted rate = 2.94 Hz
reject reasons = low_inliers 90
duration mean = 0.181 s
duration median = 0.147 s
duration p95 = 0.285 s
accepted duration p95 = 0.290 s
raw calc_error mean = 0.91 m
raw calc_error median = 0.76 m
raw calc_error p95 = 2.05 m
accepted calc_error mean = 0.90 m
accepted calc_error median = 0.74 m
accepted calc_error p95 = 2.08 m
accepted calc_error max = 3.12 m
accepted errors over 2 m = 25
accepted errors over 3 m = 3
accepted errors over 5 m = 0
```

Compared with the 640x480 / 114.6 deg camera:

```text
accepted rate improved: 2.80 Hz -> 2.94 Hz
raw error mean improved: 1.08 m -> 0.91 m
raw error median improved: 0.99 m -> 0.76 m
good_count median improved: 158 -> 181
tiles_considered stayed fixed at 9, no full-map fallback
```

Threshold sweep on this run:

```text
min_inliers 45 -> 3.60 Hz, p95 error 2.05 m, max error 3.12 m
min_inliers 50 -> 3.54 Hz, p95 error 2.05 m, max error 3.12 m
min_inliers 55 -> 3.29 Hz, p95 error 2.05 m, max error 3.12 m
min_inliers 60 -> 2.94 Hz, p95 error 2.08 m, max error 3.12 m
min_inliers 65 -> 2.32 Hz, p95 error 2.08 m, max error 3.01 m
min_inliers 70 -> 1.74 Hz, p95 error 2.12 m, max error 3.01 m
```

Interpretation:

```text
The camera update helped. The 1024x1024 / 65 deg baseline has better median
accuracy and more good matches. The current 60-inlier gate is safe but slightly
too strict for rate. A 50-inlier gate keeps the same p95 error while raising
accepted fixes to about 3.5 Hz.

Next speed test should reduce matcher interval and use a lower gate:
--interval-sec 0.15
--min-nav-inliers 50
--early-stop-inliers 80
```

### Camera Update - Native 512x512 test

Updated Gazebo model:

```text
<ardupilot_gazebo>/models/gimbal_small_3d/model.sdf
```

Change from the 1024x1024 / 65 deg baseline:

```diff
  horizontal_fov = 1.134464 rad
- width/height = 1024 x 1024
+ width/height = 512 x 512
```

Backup:

```text
<ardupilot_gazebo>/models/gimbal_small_3d/model.sdf.codexbak_1024_fov65
```

Expected footprint:

```text
80 m altitude  = 101.9 m wide
100 m altitude = 127.4 m wide
```

Only image resolution changed; FOV and therefore physical footprint stayed fixed.
The next observe command should omit `--resize-w`.

### Test Q - Native 512 observe before disabling CameraZoomPlugin

CSV:

```text
live_sift_master_512_fov65_observe.csv
```

Result:

```text
duration = 115 s
rows = 493
raw SIFT OK = 493 / 493
nav accepted = 484
nav rejected = 9
accepted rate = 4.21 Hz
duration median = 0.224 s
duration p95 = 0.267 s
accepted calc_error mean = 1.16 m
accepted calc_error median = 0.94 m
accepted calc_error p95 = 2.54 m
accepted calc_error max = 3.24 m
accepted errors over 3 m = 3
```

Visual issue observed:

```text
Dashboard camera footprint polygon appeared larger than the yellow 250 m patch.
At 100 m altitude and 65 deg FOV this should not happen; expected footprint is
about 127 m, roughly half the patch width.
```

Cause found:

```text
CameraZoomPlugin.cc has refHfov=2.0 and goalHfov=2.0 hard-coded.
The plugin can drive the camera HFOV back to 2.0 rad even when model.sdf says
1.134464 rad.

2.0 rad at 100 m gives about 311 m footprint, which is larger than the 250 m
patch. This matches the dashboard symptom.
```

Action:

```text
Disabled CameraZoomPlugin in:
<ardupilot_gazebo>/models/gimbal_small_3d/model.sdf

Backup:
<ardupilot_gazebo>/models/gimbal_small_3d/model.sdf.codexbak_zoom_plugin
```

Runtime diagnostics added:

```text
frame_quad_width_m
frame_quad_height_m
frame_quad_area_m2
```

These are now written to new CSV logs and shown in the dashboard log as
`camera footprint: W x H m`.

### Test R - Native 512 observe after disabling CameraZoomPlugin

CSV:

```text
live_sift_master_512_fov65_nozoom_observe.csv
```

Setup:

```text
camera image = 512 x 512
camera horizontal_fov = 1.134464 rad, about 65 deg
CameraZoomPlugin = disabled
map source = sift-master
vision TX = disabled, observe-only
```

Result:

```text
duration = 266 s
rows = 1440
raw SIFT OK = 1440 / 1440
nav accepted = 1422
nav rejected = 18
reject reason = low_inliers only
accepted rate = 5.35 Hz
duration median = 0.095 s
duration p95 = 0.206 s
duration max = 0.374 s
accepted calc_error mean = 0.84 m
accepted calc_error median = 0.70 m
accepted calc_error p95 = 1.93 m
accepted calc_error max = 2.84 m
accepted errors over 3 m = 0
```

Footprint diagnostic:

```text
frame_quad_width_m mean = 126.76 m
frame_quad_height_m mean = 126.72 m
frame_quad_width_m p95 = 135.94 m
frame_quad_height_m p95 = 136.24 m
```

Conclusion:

```text
The camera footprint now matches the expected 100 m / 65 deg footprint
of about 127 m. The cyan camera polygon should be smaller than the yellow
250 m patch, roughly half the patch width.

This confirms that the oversized footprint was caused by CameraZoomPlugin
forcing HFOV back toward 2.0 rad.
```

### Planned camera geometry tests

Goal:

```text
Choose a camera geometry that keeps SIFT localization above 4 Hz and keeps the
camera ground sample distance close to the 0.1220703125 m/px master map GSD.
```

Success criteria:

```text
accepted rate >= 4 Hz
duration p95 <= 0.25 s
accepted calc_error p95 <= 2.0 m
accepted calc_error max <= 3.0 m
accepted errors over 3 m = 0
frame_quad size matches expected footprint
```

Test queue:

```text
S1 = 100 m / 65 deg / 1024 px, CSV live_sift_master_100m_fov65_1024.csv
S2 = 100 m / 65 deg / 768 px,  CSV live_sift_master_100m_fov65_768.csv
S3 = 120 m / 55 deg / 1024 px, CSV live_sift_master_120m_fov55_1024.csv
S4 = 80 m / 75 deg / 1024 px,  CSV live_sift_master_80m_fov75_1024.csv
```

Use this command after every run:

```bash
python3 scripts/summarize_sift_csv.py CSV_NAME_HERE.csv
```

### Test S1 - 100 m / 65 deg / 1024 px

CSV:

```text
live_sift_master_100m_fov65_1024.csv
```

Setup:

```text
camera image = 1024 x 1024
camera horizontal_fov = 1.134464 rad, about 65 deg
CameraZoomPlugin = disabled
map source = sift-master
vision TX = disabled, observe-only
```

Result:

```text
duration = 146 s
rows = 784
raw SIFT OK = 784 / 784
nav accepted = 784
nav rejected = 0
accepted rate = 5.37 Hz
duration median = 0.179 s
duration p95 = 0.254 s
duration max = 0.478 s
accepted calc_error mean = 0.85 m
accepted calc_error median = 0.64 m
accepted calc_error p95 = 2.07 m
accepted calc_error max = 2.94 m
accepted errors over 3 m = 0
```

Match and footprint diagnostics:

```text
inliers median = 102
inliers p95 = 166
good_count median = 157.5
tiles_scanned median = 1
tiles_scanned p95 = 3
frame_quad_width_m mean = 123.87 m
frame_quad_height_m mean = 123.76 m
```

Conclusion:

```text
S1 is a strong candidate: no rejects, 5.37 Hz, and no accepted errors over 3 m.
It narrowly misses the strict p95 targets:

duration p95 target <= 0.25 s, measured 0.254 s
error p95 target <= 2.0 m, measured 2.07 m

Next test should be S2, 100 m / 65 deg / 768 px. It may keep most of the S1
accuracy while reducing CPU cost.
```

### Test S2 - 100 m / 65 deg / 768 px

CSV:

```text
live_sift_master_100m_fov65_768.csv
```

Setup:

```text
camera image = 768 x 768
camera horizontal_fov = 1.134464 rad, about 65 deg
CameraZoomPlugin = disabled
map source = sift-master
vision TX = disabled, observe-only
```

Result:

```text
duration = 84 s
rows = 467
raw SIFT OK = 467 / 467
nav accepted = 467
nav rejected = 0
accepted rate = 5.56 Hz
duration median = 0.123 s
duration p95 = 0.204 s
duration p99 = 0.221 s
duration max = 2.203 s, first global relocalization only
steady-state duration max = 0.285 s
accepted calc_error mean = 0.81 m
accepted calc_error median = 0.62 m
accepted calc_error p95 = 1.96 m
accepted calc_error max = 2.86 m
accepted errors over 3 m = 0
```

Match and footprint diagnostics:

```text
inliers median = 99
inliers p95 = 145
good_count median = 155
tiles_scanned median = 1
tiles_scanned p95 = 9
frame_quad_width_m mean = 124.65 m
frame_quad_height_m mean = 124.68 m
```

Note:

```text
One first-frame global search scanned all 121 patches and took 2.203 s.
After that, search_source stayed vision_predicted, tiles_considered stayed 9,
and steady-state max duration was 0.285 s.
```

Conclusion:

```text
S2 passes the current observe-only criteria:

accepted rate >= 4 Hz, measured 5.56 Hz
duration p95 <= 0.25 s, measured 0.204 s
error p95 <= 2.0 m, measured 1.96 m
error max <= 3.0 m, measured 2.86 m
accepted errors over 3 m = 0

S2 is now the best 100 m / 65 deg balance candidate.
Next test should be S3, 120 m / 55 deg / 1024 px, if 120 m flight is acceptable.
```

### Test S3 - 120 m / 55 deg / 1024 px

CSV:

```text
live_sift_master_120m_fov55_1024.csv
```

Setup:

```text
camera image = 1024 x 1024
camera horizontal_fov = 0.959931 rad, about 55 deg
CameraZoomPlugin = disabled
map source = sift-master
vision TX = disabled, observe-only
```

Result:

```text
duration = 96 s
rows = 451
raw SIFT OK = 451 / 451
nav accepted = 451
nav rejected = 0
accepted rate = 4.70 Hz
duration median = 0.199 s
duration p95 = 0.264 s
duration p99 = 0.287 s
duration max = 1.186 s, first global relocalization only
steady-state duration max = 0.302 s
accepted calc_error mean = 1.18 m
accepted calc_error median = 0.93 m
accepted calc_error p95 = 2.71 m
accepted calc_error max = 3.50 m
accepted errors over 3 m = 9
```

Match and footprint diagnostics:

```text
inliers median = 94
inliers p95 = 155
good_count median = 147
tiles_scanned median = 1
tiles_scanned p95 = 9
frame_quad_width_m mean = 121.51 m
frame_quad_height_m mean = 121.31 m
```

Conclusion:

```text
S3 is stable in the sense that it has no rejects and remains above 4 Hz, but it
does not pass the observe-only criteria:

duration p95 target <= 0.25 s, measured 0.264 s
error p95 target <= 2.0 m, measured 2.71 m
error max target <= 3.0 m, measured 3.50 m
accepted errors over 3 m target = 0, measured 9

S3 is worse than S2. Do not use S3 as the default geometry unless the mission
requires 120 m altitude and a narrower FOV.
```

### Test S4 - 80 m / 75 deg / 1024 px

CSV:

```text
live_sift_master_80m_fov75_1024.csv
```

Setup:

```text
camera image = 1024 x 1024
camera horizontal_fov = 1.308997 rad, about 75 deg
CameraZoomPlugin = disabled
map source = sift-master
vision TX = disabled, observe-only
```

Result:

```text
duration = 91 s
rows = 471
raw SIFT OK = 471 / 471
nav accepted = 471
nav rejected = 0
accepted rate = 5.18 Hz
duration median = 0.178 s
duration p95 = 0.263 s
duration p99 = 0.310 s
duration max = 1.051 s, first global relocalization only
steady-state duration max = 0.351 s
accepted calc_error mean = 0.90 m
accepted calc_error median = 0.71 m
accepted calc_error p95 = 2.19 m
accepted calc_error max = 3.57 m
accepted errors over 3 m = 3
```

Match and footprint diagnostics:

```text
inliers median = 90
inliers p95 = 134
good_count median = 161
tiles_scanned median = 1
tiles_scanned p95 = 9
frame_quad_width_m mean = 122.29 m
frame_quad_height_m mean = 122.08 m
```

Conclusion:

```text
S4 confirms the expected footprint and GSD at 80 m, but it does not beat S2.
It misses the strict criteria:

duration p95 target <= 0.25 s, measured 0.263 s
error p95 target <= 2.0 m, measured 2.19 m
error max target <= 3.0 m, measured 3.57 m
accepted errors over 3 m target = 0, measured 3

The likely tradeoff is that 75 deg FOV gives the correct GSD at 80 m but adds
more perspective/distortion pressure than 65 deg. S2 remains the default
geometry candidate.
```

### Camera geometry decision

Current best:

```text
S2 = 100 m / 65 deg / 768 px
```

Reason:

```text
accepted rate = 5.56 Hz
duration p95 = 0.204 s
accepted calc_error p95 = 1.96 m
accepted calc_error max = 2.86 m
accepted errors over 3 m = 0
nav rejected = 0
```

Ranking from the observe-only tests:

```text
1. S2, 100 m / 65 deg / 768 px   - best speed/accuracy balance
2. S1, 100 m / 65 deg / 1024 px  - strong, but slightly slower
3. S4, 80 m / 75 deg / 1024 px   - usable, but has >3 m outliers
4. S3, 120 m / 55 deg / 1024 px  - stable but clearly worse error
```

### Test S2-long - 100 m / 65 deg / 768 px, 5 min observe

CSV:

```text
live_sift_master_100m_fov65_768_long.csv
```

Setup:

```text
camera image = 768 x 768
camera horizontal_fov = 1.134464 rad, about 65 deg
CameraZoomPlugin = disabled
map source = sift-master
vision TX = disabled, observe-only
```

Result:

```text
duration = 301 s
rows = 1773
raw SIFT OK = 1773 / 1773
nav accepted = 1773
nav rejected = 0
accepted rate = 5.89 Hz
duration median = 0.109 s
duration p95 = 0.177 s
duration p99 = 0.214 s
duration max = 0.292 s
accepted calc_error mean = 0.83 m
accepted calc_error median = 0.75 m
accepted calc_error p95 = 1.78 m
accepted calc_error p99 = 2.20 m
accepted calc_error max = 3.21 m
accepted errors over 2 m = 42
accepted errors over 3 m = 2
```

Match and footprint diagnostics:

```text
inliers median = 93
inliers p95 = 138
good_count median = 139
tiles_scanned median = 1
tiles_scanned p95 = 9
frame_quad_width_m mean = 124.07 m
frame_quad_height_m mean = 124.01 m
```

Outliers:

```text
Only 2 / 1773 accepted fixes were over 3 m:

frame 397: 3.21 m, inliers 98, tiles_scanned 1
frame 1427: 3.08 m, inliers 136, tiles_scanned 1
```

Conclusion:

```text
The longer run confirms S2 as the best current geometry. P95 error and P95
duration are comfortably inside target, with zero rejects. Two rare >3 m
outliers remain, so VPE publish tests should keep max_nav_error_m near 3.0 m
or add a short smoothing/innovation gate before feeding EKF.
```

### Test S1-long - 100 m / 65 deg / 1024 px, 5 min observe

CSV:

```text
live_sift_master_100m_fov65_1024_long.csv
```

Setup:

```text
camera image = 1024 x 1024
camera horizontal_fov = 1.134464 rad, about 65 deg
CameraZoomPlugin = disabled
map source = sift-master
vision TX = disabled, observe-only
```

Result:

```text
duration = 299 s
rows = 1722
raw SIFT OK = 1722 / 1722
nav accepted = 1722
nav rejected = 0
accepted rate = 5.76 Hz
duration median = 0.166 s
duration p95 = 0.227 s
duration p99 = 0.247 s
duration max = 0.839 s, first global relocalization only
steady-state duration max = 0.270 s
accepted calc_error mean = 0.95 m
accepted calc_error median = 0.85 m
accepted calc_error p95 = 2.06 m
accepted calc_error p99 = 2.61 m
accepted calc_error max = 3.14 m
accepted errors over 2 m = 110
accepted errors over 3 m = 4
```

Match and footprint diagnostics:

```text
inliers median = 95
inliers p95 = 147
good_count median = 146
tiles_scanned median = 1
tiles_scanned p95 = 9
frame_quad_width_m mean = 123.91 m
frame_quad_height_m mean = 123.90 m
```

Comparison with S2-long:

```text
S2-long, 768 px:
duration p95 = 0.177 s
calc_error p95 = 1.78 m
calc_error max = 3.21 m
errors over 3 m = 2 / 1773

S1-long, 1024 px:
duration p95 = 0.227 s
calc_error p95 = 2.06 m
calc_error max = 3.14 m
errors over 3 m = 4 / 1722
```

Conclusion:

```text
1024 px does not improve the long-run localization result. It gives slightly
more inliers, but it is slower and has worse p95 error than 768 px. Keep S2 as
the default geometry.
```

### Test VPE-1 - SIFT VPE publish on GPS source set, fix mode

CSV:

```text
live_sift_master_vpe_gps_gate_fix.csv
```

Setup:

```text
camera = 100 m / 65 deg / 768 px
source set = 1, GPS
SIM_GPS_DISABLE = 0
vision publish mode = fix
VISION_POSITION_ESTIMATE only
VISION_SPEED_ESTIMATE disabled
z = 0
roll/pitch/yaw = 0
telemetry position gate = enabled, max_nav_error_m = 3.0
```

Result:

```text
duration = 113 s
rows = 666
raw SIFT OK = 666 / 666
nav accepted = 666
nav rejected = 0
accepted / VPE fix rate = 5.89 Hz
duration median = 0.109 s
duration p95 = 0.176 s
duration max = 0.224 s
accepted calc_error mean = 0.61 m
accepted calc_error median = 0.44 m
accepted calc_error p95 = 1.66 m
accepted calc_error p99 = 2.26 m
accepted calc_error max = 2.80 m
accepted errors over 2 m = 13
accepted errors over 3 m = 0
```

Match and footprint diagnostics:

```text
inliers median = 89
inliers p95 = 141
good_count median = 143
tiles_scanned median = 1
tiles_scanned p95 = 9
frame_quad_width_m mean = 125.09 m
frame_quad_height_m mean = 124.82 m
```

MAVLink status after the run:

```text
mode = AUTO
EK3_SRC1_POSXY = 3
EK3_SRC1_VELXY = 3
EK3_SRC2_POSXY = 6
EK3_SRC2_VELXY = 0
SIM_GPS_DISABLE = 0
EKF flags = 831
velocity_variance = 0.0218
pos_horiz_variance = 0.0023
```

Conclusion:

```text
SIFT VPE publish in fix mode is healthy while the vehicle remains on GPS source
set 1. The stream produced about 5.9 Hz fresh VPE fixes and did not disturb EKF.
Next test can switch to source set 2 with telemetry position gate disabled and
safe rollback enabled.
```

### Test VPE-2 - Source set 2 switch during AUTO mission

CSV:

```text
live_sift_master_vpe_src2.csv
```

Switch command:

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

Setup:

```text
camera = 100 m / 65 deg / 768 px
vision publish mode = fix
VISION_POSITION_ESTIMATE only
VISION_SPEED_ESTIMATE disabled
telemetry position gate = disabled
vehicle mode during switch = AUTO mission
vehicle speed during switch = roughly 3-10 m/s
```

Switch result:

```text
PRE mode=AUTO local=(61.49,-71.55,-99.87) vel=(3.12,3.52,-0.01)
SWITCH set=2 accepted
EKF flags stayed 831 during the initial monitor
pos_horiz_variance climbed while the vehicle was moving fast
rollback reason = pos_horiz_variance=2.4473 > 2.0000
ROLLBACK set=1 accepted
```

MAVLink status after rollback:

```text
mode = GUIDED
EKF flags = 831
velocity_variance = 0.0301
pos_horiz_variance = 0.0466
SIM_GPS_DISABLE = 0
source set 1 GPS and source set 2 ExternalNav params remain configured
```

Final CSV summary:

```text
duration = 323 s
rows = 1627
nav accepted = 1523
nav rejected = 67
NO_MATCH = 37
accepted rate = 4.72 Hz
rejected reasons = low_inliers 51, prediction_jump_gate 16
accepted duration p95 = 0.199 s
accepted calc_error median = 1.11 m
accepted calc_error p95 = 4.44 m
accepted calc_error max = 7.29 m
accepted errors over 3 m = 110
```

Conclusion:

```text
This was not a clean first source-set-2 validation because the switch happened
while the aircraft was actively flying an AUTO mission at high horizontal speed.
The rollback worked and recovered to source set 1/GPS.

The next source-set-2 test should be done at 100 m hover or very low speed,
not during AUTO waypoint motion. Keep rollback enabled, switch in GUIDED hover,
hold for 60 s, then try a small 1 m/s movement.
```

### Test VPE-3 - Source set 2 hover, stricter SIFT gates

CSV:

```text
live_sift_master_vpe_src2_strict.csv
```

Setup:

```text
camera = 100 m / 65 deg / 768 px
vehicle mode = GUIDED hover
vision publish mode = fix
VISION_POSITION_ESTIMATE only
VISION_SPEED_ESTIMATE disabled
telemetry position gate = disabled
early_stop_inliers = 180
min_nav_inliers = 80
max_nav_jump_m = 2.0
max_search_radius_m = 220
```

Result:

```text
duration = 55 s
rows = 288
nav accepted = 272
nav rejected = 16
accepted rate = 4.95 Hz
rejected reason = prediction_jump_gate only
accepted duration p95 = 0.235 s
accepted calc_error median = 1.60 m
accepted calc_error p95 = 1.85 m
accepted calc_error max = 5.57 m
accepted errors over 3 m = 1
```

Important sequence:

```text
For about 45 s, accepted fixes stayed around 1.5-1.9 m from telemetry.
Then frame 352 was accepted with 5.57 m error.
After that, prediction_jump_gate rejected 16 fixes as the visual estimate
returned toward the previous trajectory.
```

Post-test MAVLink status:

```text
mode = GUIDED
EKF flags = 831
velocity_variance = 0.0086
pos_horiz_variance = 0.0053
groundspeed = 0.02
SIM_GPS_DISABLE = 0
```

Conclusion:

```text
The stricter SIFT gates improved the visual stream a lot compared with VPE-2,
but source set 2 still did not hold long enough. The failure is no longer a
large low-inlier localization collapse; it is an EKF/measurement-consistency
problem with occasional accepted visual innovations.

Next test should reduce EKF trust in ExternalNav by increasing VISO_POS_M_NSE
before switching to source set 2.
```

### Test VPE-4 - Source set 2 hover, VISO_POS_M_NSE=4

CSV:

```text
live_sift_master_vpe_src2_noise4.csv
```

Setup:

```text
camera = 100 m / 65 deg / 768 px
vehicle mode = GUIDED hover
vision publish mode = fix
VISION_POSITION_ESTIMATE only
VISION_SPEED_ESTIMATE disabled
telemetry position gate = disabled
early_stop_inliers = 180
min_nav_inliers = 80
max_nav_jump_m = 2.0
max_search_radius_m = 220
VISO_POS_M_NSE = 4.0
```

Result:

```text
duration = 76 s
rows = 342
nav accepted = 308
nav rejected = 34
accepted rate = 4.05 Hz
rejected reason = prediction_jump_gate only
accepted duration p95 = 0.263 s
accepted calc_error median = 0.41 m where telemetry comparison was available
accepted calc_error p95 = 5.79 m where telemetry comparison was available
accepted calc_error max = 6.26 m
accepted errors over 3 m = 15
```

Important sequence:

```text
First 55-60 s looked clean where telemetry comparison was available.
Then the visual estimate started walking ahead in patch_r03_c04.
prediction_jump_gate rejected 34 fixes, but accepted fixes later still reached
about 4-6 m discrepancy.
```

Post-test MAVLink status:

```text
VISO_POS_M_NSE = 4
mode = GUIDED
EKF flags = 831
velocity_variance = 0.0122
pos_horiz_variance = 0.0041
groundspeed = 0.02
```

Conclusion:

```text
Increasing VISO_POS_M_NSE to 4.0 did not solve the source-set-2 hold problem.
The next step should not be another EKF noise-only tweak. The source-set-2 tests
need an independent Gazebo ground-truth monitor/logger, and the SIFT VPE stream
needs an additional temporal/innovation filter before accepted visual fixes are
published to EKF.
```

### Test VPE-5 preparation - Filter + Gazebo truth

Implemented first filter/truth pass:

```text
sau_sift_nav/gazebo_truth.py:
  subscribes to Gazebo pose info and logs independent truth to state/CSV.

sau_sift_nav/state.py:
  adds nav_filter_alpha, max_nav_step_speed_mps, max_nav_step_slack_m.
  keeps raw SIFT north/east and filtered nav north/east separately.

sau_sift_nav/logging.py:
  adds truth_source, gazebo_error_m, nav_raw_*, nav_filter_residual_m.
```

The MAVLink payload policy is unchanged:

```text
VISION_POSITION_ESTIMATE only
x/y from filtered SIFT
z = 0
roll/pitch/yaw = 0
covariance[0] = NaN
VISION_SPEED_ESTIMATE disabled
```

### Test VPE-5 first run - Filter active, truth missing

CSV:

```text
live_sift_master_vpe_src2_filter_gt.csv
```

Result:

```text
duration = 38 s
rows = 176
accepted = 174
rejected = 2
accepted rate = 4.58 Hz
reject reason = visual_step_speed_gate only
duration p95 = 0.255 s
nav_filter_residual_m median = 0.126 m
nav_filter_residual_m p95 = 0.405 m
nav_filter_residual_m max = 1.454 m
truth_source = empty for all rows
```

Interpretation:

```text
The filter behaved as intended: it held publish rate above 4 Hz, smoothed raw
SIFT motion, and rejected two visual-only step spikes.

The run cannot be used to judge real localization error because Gazebo truth did
not enter the CSV. The first row still did a full 121-patch global search, then
normal tracking stayed at 9 patches.

The biggest raw/filtered disturbance happened around 23:15:58-23:16:02. The
filter limited the largest filtered step to about 0.40 m and rejected two frames,
but without independent truth we cannot tell if this was real motion or visual
drift.
```

Follow-up:

```text
CSV logging now includes truth_status, truth_age_sec, truth_pose_age_sec, and
truth_gz_x/y/z so the next run can diagnose whether Gazebo truth is receiving,
waiting for pose, or waiting for telemetry seed.
```

### Test VPE-6 - Filter + Gazebo truth receiving

CSV:

```text
live_sift_master_vpe_src2_filter_gt_2.csv
```

Result:

```text
duration = 68 s
rows = 298
truth_source = gazebo for all rows
truth_status = receiving for all rows
accepted = 219
rejected = 79
accepted rate = 3.22 Hz
row rate = 4.38 Hz
reject reasons:
  prediction_jump_gate = 74
  visual_step_speed_gate = 3
  low_inliers = 2
gazebo_error_m median = 0.380 m
gazebo_error_m p95 = 3.169 m
gazebo_error_m max = 4.396 m
accepted gazebo_error_m p95 = 3.624 m
nav_filter_residual_m median = 0.182 m
nav_filter_residual_m p95 = 19.258 m
nav_filter_residual_m max = 25.686 m
```

Important observations:

```text
Gazebo truth is now working.

The visual match itself is often good. Near the end, raw SIFT fixes were within
0.03-0.78 m of Gazebo truth, but many were rejected because the internal
filtered nav estimate had already been pulled far away.

The main failure is filter recovery after reject/fallback:
  row 229: search_source=global, raw error=2.30 m, but filtered nav jumps to
           about (8.09, 11.86) because the filter blends against an old visual
           prediction with residual 11.31 m.
  row 279: search_source=global_position_seed, raw error=0.13 m, but filtered
           nav jumps to about (8.38, 16.78) because residual is 25.69 m.

This proves a code bug: after fallback/relocalization, the visual filter must
reset to the raw SIFT fix instead of EMA-blending with a stale prediction.
```

Code action:

```text
Added nav_filter_reset_residual_m.
When search_source is not vision_predicted, or the raw-vs-predicted residual is
above the reset threshold, the visual filter now resets to raw SIFT and
increments reset_counter.

Next test should disable AP-local based prediction_jump_gate with
--max-nav-jump-m 0 because LOCAL_POSITION_NED is not independent in source set 2.
Use visual_step_speed_gate as the physical consistency gate instead.
```

### Test VPE-7 - prediction_jump disabled, reset filter active

CSV:

```text
live_sift_master_vpe_src2_filter_gt_3.csv
```

Result:

```text
duration = 63 s
rows = 285
truth_source = gazebo for all rows
truth_status = receiving for all rows
accepted = 242
rejected = 43
accepted rate = 3.84 Hz
row rate = 4.52 Hz
reject reasons:
  visual_step_speed_gate = 24
  low_inliers = 19
gazebo_error_m median = 0.757 m
gazebo_error_m p95 = 5.648 m
gazebo_error_m max = 6.474 m
accepted gazebo_error_m p95 = 5.203 m
nav_filter_reset = 3 rows
```

Interpretation:

```text
Source-set-2 local position must not be treated as independent truth. The
correct truth channel for these tests is Gazebo truth.

Disabling prediction_jump_gate removed the AP-local dependent rejects. However,
raw SIFT and filtered VPE still lag behind Gazebo during faster motion. The
first clean hover/slow segment stayed near 0.2-0.4 m error, then as motion
increased the error rose above 3-6 m.

The next issue is timing/latency, not local telemetry. The VPE timestamp should
represent the camera frame measurement time, not the send time. Code now sends
the frame capture timestamp in VISION_POSITION_ESTIMATE.usec.

For ArduPilot parameters, VISO_DELAY_MS is relevant now because SIFT processing
takes about 0.18-0.27 s. VISO_POS_M_NSE is also relevant, but it should be used
to reduce EKF trust after the visual stream timing is corrected, not to hide
large SIFT lag.
```

### Code action - VPE output alignment

Added optional frozen VPE alignment:

```text
--vision-align-source telemetry
--vision-align-max-age-sec 1.0
```

When enabled, the first accepted visual fix is compared against the fresh
telemetry seed while source set 1 is still active. The resulting XY offset is
frozen and applied to outgoing `VISION_POSITION_ESTIMATE` x/y only. The SIFT
search and Gazebo-error logging remain in map coordinates.

Why:

```text
Source set 2 makes ArduPilot use ExternalNav XY. If the first VPE is offset
from the GPS/local source-1 frame, the vehicle can begin moving even with no
new Go To command. The frozen alignment test checks whether that source-switch
frame mismatch is the main hover drift cause.
```

New CSV columns:

```text
vision_tx_north_m
vision_tx_east_m
vision_tx_map_north_m
vision_tx_map_east_m
vision_align_source
vision_align_seed_source
vision_align_offset_north_m
vision_align_offset_east_m
vision_align_age_sec
```

### Test VPE-8A - local alignment, alpha 0.8, waited before source switch

CSV:

```text
live_sift_master_vpe_src2_aligned_local_alpha08_waited.csv
```

Configuration highlights:

```text
--vision-align-source telemetry
--telemetry-seed-source local
--nav-filter-alpha 0.8
--min-nav-inliers 70
--max-nav-step-speed-mps 0
```

Result:

```text
duration = 52 s
rows = 223
accepted = 223
rejected = 0
accepted rate = 4.29 Hz
alignment source = local_ned_seed
alignment offset = N -0.176 m / E +0.153 m
SIFT inliers median = 103
match duration median = 0.212 s
Gazebo error median = 1.47 m, p95 = 4.63 m, max = 5.67 m
```

Interpretation:

```text
The local-frame alignment locks quickly and remains fixed. During the initial
hover window, VPE-to-Gazebo error is about 0.08-0.25 m, so the source-switch
offset problem is mostly resolved.

After source set 2, the vehicle still starts moving by itself around row 72
(08:21:48). Once it moves, visual fixes remain accepted and inliers stay high,
but the pose lags Gazebo truth by 3-5 m during the faster segment. This points
to latency / EKF trust / controller feedback rather than a bad match or reject
gate problem.
```

Next isolation:

```text
1. Test VISO_DELAY_MS around 200 ms because SIFT processing median is ~0.21 s
   and p95 is ~0.31 s.
2. Test VISO_POS_M_NSE around 4-5 m to reduce EKF over-trust in delayed/noisy
   SIFT fixes.
3. Keep local alignment; it is now the better alignment source for source-set
   switching.
```

### Test VPE-8B - VISO delay/noise sweep

CSV files:

```text
live_sift_master_vpe_src2_aligned_local_alpha08_delay200_noise4.csv
live_sift_master_vpe_src2_aligned_local_alpha08_noise4_delay0.csv
live_sift_master_vpe_src2_aligned_local_alpha08_noise6_delay0.csv
```

Result summary:

```text
delay=200, noise=4:
  duration = 68 s
  accepted rate = 5.78 Hz
  path = 100.5 m
  median Gazebo error = 2.58 m
  p95 Gazebo error = 5.48 m
  conclusion: worse than delay=0; do not use 200 ms for now.

delay=0, noise=4:
  first 52 s path = 7.2 m
  first 52 s median Gazebo error = 1.15 m
  first 52 s p95 Gazebo error = 1.83 m
  full run path = 207.6 m after long runaway
  conclusion: best sustained setting so far, but still runs away eventually.

delay=0, noise=6:
  first 150 rows hover error median about 0.26-0.31 m
  after row ~156, vehicle starts moving
  first 52 s path = 49.7 m
  p95 Gazebo error = 8.40 m
  conclusion: too loose / less useful than noise=4 for sustained source-set-2 hold.
```

Current best EKF parameter direction:

```text
VISO_DELAY_MS = 0
VISO_POS_M_NSE = 4
```

Open issue:

```text
The source switch no longer fails because of initial alignment; local alignment
is working. The remaining failure is long-term source-set-2 control stability.
When the vehicle starts moving, visual pose stays accepted but lags and the
control loop can amplify the error. Next isolation should test whether adding
velocity aiding from visual finite differences, or changing EK3_SRC2_VELXY away
from 0, helps EKF hold XY without relying only on delayed position fixes.
```

### Test GT-4HZ - Gazebo truth VPE-only at fixed 4 Hz

Log:

```text
gazebo_truth_vpe4hz.log
```

Command:

```bash
python3 vision_debug/gazebo_truth_bridge.py \
  --mavlink udp:127.0.0.1:14550 \
  --gz-topic /world/iris_runway/pose/info \
  --model iris_with_gimbal \
  --axis enu \
  --rate 4 \
  --stream-hz 10 \
  --no-send-speed \
  --speed-source zero \
  --attitude-source zero \
  --duration 0
```

Result:

```text
VPE-only, no VSE.
sent counter reached 236, approximately 59 s at 4 Hz.
EKF flags stayed 831 for all printed rows.
velocity variance stayed around 0.009-0.010.
position variance stayed around 0.005-0.014.
XY error mean = 0.04 m, max = 0.085 m.
Gazebo truth pose drift over the run was about 0.07 m.
No EKF variance / LAND / stopped aiding messages in the bridge log.
```

Interpretation:

```text
ArduPilot/EKF3 can hold source set 2 with fixed 4 Hz VPE-only XY when the
position measurement is effectively perfect Gazebo truth. Therefore the SIFT
runaway is not caused simply by 4 Hz VPE-only being unsupported. The remaining
problem is the SIFT measurement stream's latency/noise/jitter/occasional bias
and the control loop response to those errors.
```

### Next Test SIFT-4HZ-SENDTIME - Fixed-rate filtered SIFT VPE

Purpose:

```text
Bring the SIFT VPE stream closer to the successful Gazebo-truth bridge test.
Instead of sending one MAVLink VPE only when a new SIFT fix arrives, publish a
4 Hz VPE stream from the filtered visual estimate. Timestamp the outgoing VPE
at send time, because rate mode predicts the filtered pose forward to "now".

This still sends VISION_POSITION_ESTIMATE only:
  x/y = aligned filtered SIFT estimate
  z = 0
  roll/pitch/yaw = 0
  covariance[0] = NaN
  no VISION_SPEED_ESTIMATE
```

Expected comparison:

```text
If this improves source-set-2 hold, the variable/fix-only timing and stale frame
timestamps were part of the control problem. If it still runs away while Gazebo
truth VPE at 4 Hz is stable, the next layer should be a stronger visual
measurement filter or outlier/latency compensation before publishing VPE.
```

Result:

```text
CSV = live_sift_master_vpe_rate4_sendtime_filter.csv
duration = 66 s
row rate = 4.18 Hz
accepted rate = 4.11 Hz
accepted = 271
rejected = 5
rejects = 4 visual_step_speed_gate, 1 low_inliers
VPE timestamp source = send for all rows

Gazebo error:
  mean = 0.44 m
  median = 0.23 m
  p95 = 1.68 m
  p99 = 2.33 m
  max = 2.44 m

Accepted fixes:
  accepted_err_over_2m = 8
  accepted_err_over_3m = 0
  accepted_err_over_5m = 0

Matcher:
  mean duration = 0.239 s
  median duration = 0.232 s
  p95 duration = 0.297 s
  inliers median = 98
```

Interpretation:

```text
This is a clear improvement over the previous SIFT source-set-2 runs. Fixed-rate
4 Hz VPE, send-time timestamps, visual prediction, and a tighter step-speed gate
kept the run bounded for this 66 s hover test. There was a brief error rise
around rows 200-249, but the visual_step_speed_gate caught the worst jump and
the estimate recovered instead of running away.

Current SIFT baseline candidate:
  vision-publish-mode = rate
  vision-rate-hz = 4
  vision-timestamp-source = send
  vision-speed-source = visual
  no VISION_SPEED_ESTIMATE
  z/roll/pitch/yaw = 0
  nav-filter-alpha = 0.45
  max-nav-step-speed-mps = 2.0
  max-nav-step-slack-m = 0.8
  VISO_DELAY_MS = 0
  VISO_POS_M_NSE = 4
```

### Test SIFT-AB - send-time vs frame-time VPE timestamp

CSV files:

```text
live_sift_ab_sendtime_60s.csv
live_sift_ab2_sendtime_60s.csv
```

Important note:

```text
The CSV column is the source of truth for this comparison:
  live_sift_ab_sendtime_60s.csv     -> vision_timestamp_source = send
  live_sift_ab2_sendtime_60s.csv    -> vision_timestamp_source = frame

The two runs were not equal length:
  send run  = 174 rows, about 50 s
  frame run = 94 rows, about 27 s
```

Full-run result:

```text
send:
  accepted = 171
  rejected = 3 visual_step_speed_gate
  median Gazebo error = 0.29 m
  p95 Gazebo error = 2.01 m
  max Gazebo error = 4.01 m
  accepted_err_over_3m = 0

frame:
  accepted = 90
  rejected = 4 visual_step_speed_gate
  median Gazebo error = 0.47 m
  p95 Gazebo error = 1.53 m
  max Gazebo error = 2.15 m
  accepted_err_over_3m = 0
```

Equal-row comparison:

```text
send first 94 rows:
  median Gazebo error = 0.19 m
  p95 Gazebo error = 0.42 m
  max Gazebo error = 0.66 m
  rejected = 0

frame all 94 rows:
  median Gazebo error = 0.47 m
  p95 Gazebo error = 1.44 m
  max Gazebo error = 2.15 m
  rejected = 4
```

Interpretation:

```text
With the current rate-mode + visual-prediction publisher, send-time timestamping
is more consistent. Frame-time timestamping describes the original camera frame,
but the published x/y has already been filtered/predicted toward send time.
That mismatch likely explains the worse A/B result.

Keep the baseline at:
  vision-publish-mode = rate
  vision-rate-hz = 4
  vision-timestamp-source = send
```

### Test SIFT-OBS-MOVE - GPS/source1 motion observe

CSV:

```text
sift_observe_100m_move_1ms_90s.csv
```

Context:

```text
The vehicle accidentally moved fast at the beginning of the test, then returned
at about 1 m/s near the end. This turned out to be useful because it exposed the
speed-dependent SIFT lag. VPE was disabled, source set stayed GPS/source1, and
SIFT was compared against independent Gazebo truth.
```

Overall:

```text
rows = 338
duration = about 87 s
row rate = 3.89 Hz
all SIFT matches returned OK
all nav updates were intentionally rejected by min_nav_inliers=9999
Gazebo error median = 0.396 m
Gazebo error p95 = 2.89 m
Gazebo error max = 3.57 m
median processing duration = 0.240 s
```

Speed-dependent error:

```text
0.5-2 m/s:
  median error = about 0.36-0.37 m
  along-track error median = about -0.35 m
  estimated lag = about 0.44-0.48 s

2-5 m/s:
  median error = about 0.93-1.04 m
  along-track error median = about -0.89 to -0.99 m
  estimated lag = about 0.23 s

>=5 m/s:
  median speed = about 7.7 m/s
  median error = about 2.35 m
  along-track error median = about -2.35 m
  cross-track error median = about 0.06-0.07 m
  estimated lag = about 0.31 s
```

Interpretation:

```text
The moving error is mostly along-track and negative, which means the SIFT pose is
behind the vehicle rather than laterally wrong. Cross-track error stays small.
This points to measurement latency / publish latency, not a map/tile mismatch.

The observed delay is around 0.25-0.45 s, consistent with SIFT processing time
and rate-mode timing. At 7-8 m/s this becomes roughly 2-3 m of position error.
At 1 m/s it becomes roughly 0.35-0.45 m.

Next direction: keep send-time VPE, but explicitly compensate visual estimate
forward by a tunable latency horizon, or publish a delayed/capture-time pose
without prediction. Since send-time + prediction already worked better in A/B,
the likely next practical knob is a configurable extra prediction time around
0.25-0.35 s, plus keeping commanded speed near 1 m/s until the estimator is
validated.
```

### Test SIFT-SRC2-EXTRA025 - Source set 2 with 0.25 s extra prediction

CSV:

```text
live_sift_src2_extra025_1ms_goto.csv
```

MAVLink monitor:

```text
Source set 2 was accepted, then after roughly 10 s the safe-switch monitor
rolled back because EKF flags dropped horizontal position aiding:
  flags_missing = POS_HORIZ_REL, POS_HORIZ_ABS, PRED_POS_HORIZ_REL, PRED_POS_HORIZ_ABS
  current flags = ATTITUDE, VEL_VERT, POS_VERT_ABS
```

CSV summary:

```text
rows = 163
duration = 53 s
accepted nav fixes = 104
rejected nav fixes = 59
reject reason = low_inliers
accepted rate = about 1.96 Hz
median SIFT/Gazebo error = 0.123 m
p95 SIFT/Gazebo error = 0.420 m
max SIFT/Gazebo error = 0.703 m
median processing duration = 0.259 s
median VPE prediction horizon = 0.827 s
p95 VPE prediction horizon = 1.189 s
```

Interpretation:

```text
The raw SIFT measurement was good during this run; bad map matching was not the
cause of rollback. The likely problem is stream health/continuity and possibly
over-prediction:

1. min_nav_inliers=85 was too strict for this hover location. Many good SIFT
   fixes had 70-84 inliers and were rejected, dropping accepted fix rate to
   about 2 Hz.

2. Because accepted fixes were intermittent, VPE had to reuse/predict older nav
   estimates. With extra_predict=0.25, the actual prediction horizon often
   reached 0.8-1.2 s, which is too much for hover and increases TX path jitter.

3. The AP rollback showed loss of horizontal aiding flags, not large position
   variance. This is consistent with external-nav health/continuity problems.
```

Next setting to try:

```text
Back off extra prediction for now and restore stream continuity:
  vision-extra-predict-sec = 0.0
  min-nav-inliers = 70 or 75
  vision-max-age-sec = 2.0
  nav-filter-alpha = 0.25 or 0.35 for hover stability
```

### Test SIFT-SRC2-CONTINUOUS - inlier gate relaxed, continuous VPE

CSV files:

```text
live_sift_src2_continuous_alpha025_inl70.csv
live_sift_src2_continuous_alpha045_inl70.csv
```

alpha=0.25 result:

```text
rows = 365
duration = 92 s
accepted = 357/365
accepted rate = 3.88 Hz
VPE status = sending almost all run
EKF flags stayed 831
rollback reason = pos_horiz_variance 0.3666 > 0.35

first 60 rows:
  TX error p95 = 0.225 m

last 60 rows:
  raw SIFT median error = 0.755 m
  nav median error = 0.794 m
  TX median error = 0.855 m
  TX p95 error = 3.036 m
```

alpha=0.45 result:

```text
rows = 130
duration = 39 s
accepted = 123/130
accepted rate = 3.15 Hz
VPE status = sending almost all run
EKF flags stayed 831
rollback reason = pos_horiz_variance 0.3697 > 0.35

first 60 rows:
  TX error p95 = 0.233 m

last 60 rows:
  raw SIFT median error = 0.417 m
  nav median error = 0.535 m
  TX median error = 0.828 m
  TX p95 error = 2.572 m
```

Interpretation:

```text
Relaxing min_nav_inliers to 70 fixed the previous external-nav continuity
problem. The VPE stream stayed mostly continuous and EKF flags remained healthy.
The remaining failure is EKF variance growth from hover drift / SIFT jitter.

alpha=0.45 did not fix it; it failed sooner and produced a more aggressive TX
path. alpha=0.25 lasted longer and is the better local baseline. The next knob
should not be more filter alpha. Since the EKF is still overreacting to the
visual position stream, increase external-nav position measurement noise.

Next setting:
  VISO_POS_M_NSE = 6
  vision-extra-predict-sec = 0
  min-nav-inliers = 70
  nav-filter-alpha = 0.25
```

### Test SIFT-SRC2-NOISE6 - Higher VISO_POS_M_NSE

CSV:

```text
live_sift_src2_noise6_alpha025_inl70.csv
```

Result:

```text
VISO_POS_M_NSE = 6
rows = 171
duration = 50 s
accepted = 163/171
accepted rate = 3.26 Hz
VPE status = sending almost all run
EKF flags stayed 831
rollback reason = pos_horiz_variance 0.3549 > 0.35
```

Interpretation:

```text
Increasing VISO_POS_M_NSE to 6 did not solve the hover/source-set-2 instability.
The important new finding is around rows 138-142: visual_step_speed_gate rejected
several SIFT fixes while the raw SIFT/Gazebo error was still reasonable. During
those rejects, the nav estimate and outgoing VPE lagged badly behind Gazebo
truth; vision_tx_age_sec rose to about 1.9 s and TX error exceeded 5 m.

This means the visual_step_speed_gate can freeze the estimator exactly when the
vehicle starts drifting. The gate was intended to reject outliers, but in closed
loop source-set-2 it can reject real motion and make the EKF/control loop worse.
```

Next setting:

```text
Disable visual_step_speed_gate:
  max-nav-step-speed-mps = 0

Keep:
  VISO_POS_M_NSE = 4 first
  vision-extra-predict-sec = 0
  min-nav-inliers = 70
  nav-filter-alpha = 0.25
```

### Test SIFT-SRC2-NO-STEPGATE - Disable visual step gate

CSV:

```text
live_sift_src2_no_stepgate_alpha025_inl70.csv
```

Result:

```text
rows = 102
duration = 30 s
accepted = 102/102
rejected = 0
VPE status = sending after alignment
EKF flags stayed 831
rollback reason = pos_horiz_variance 0.3630 > 0.35
```

Interpretation:

```text
Disabling visual_step_speed_gate fixed estimator freeze/reject gaps, but did not
fix source-set-2 hover stability. The stream became continuous, yet the vehicle
still drifted and EKF horizontal position variance grew.

The failure now looks like visual-velocity prediction amplifying SIFT jitter:
raw SIFT median error stayed low, but TX pose error grew during the drift. Since
the publisher uses vision-speed-source=visual in rate mode, noisy finite-
difference velocity can push VPE ahead/sideways even during hover.
```

Next setting:

```text
Disable visual velocity prediction for hover/source-set-2 stability:
  vision-speed-source = zero

Keep:
  VISO_POS_M_NSE = 4
  max-nav-step-speed-mps = 0
  min-nav-inliers = 70
  nav-filter-alpha = 0.25
  vision-publish-mode = rate
  vision-rate-hz = 4
  vision-timestamp-source = send
```

### Test SIFT-SRC2-ZERO-SPEED - Disable VPE forward velocity prediction

CSV:

```text
live_sift_src2_zero_speed_alpha025_inl70.csv
```

Settings:

```text
vision-speed-source = zero
vision-extra-predict-sec = 0
max-nav-step-speed-mps = 0
min-nav-inliers = 70
nav-filter-alpha = 0.25
vision-publish-mode = rate
vision-rate-hz = 4
vision-timestamp-source = send
VISO_POS_M_NSE = 4
```

Result:

```text
rows = 138
duration ~= 42 s
match status = OK for all rows
nav rejects = low_inliers only
EKF flags stayed 831 until rollback
rollback reason = pos_horiz_variance 0.4669 > 0.35

raw SIFT/Gazebo error:
  median ~= 0.13 m
  p95 ~= 1.83 m
  max ~= 2.02 m

VPE age:
  median ~= 0.56 s
  p95 ~= 0.78 s
  max ~= 2.85 s
```

Interpretation:

```text
Disabling visual velocity prediction removed one amplifier, but did not solve
source-set-2 hover stability. The first part of the run is still good; then the
vehicle starts moving under source set 2, SIFT/VPE follows with delay, EKF
horizontal variance grows, and the rollback trips.

This points away from "VSE/visual velocity is the only problem". The remaining
issue is a closed-loop estimator/controller interaction: SIFT hover jitter is
small in observe mode, but once AP is allowed to control from that VPE, the
small XY changes are chased by the controller and become real vehicle motion.
```

Next setting:

```text
Make VPE much calmer before touching ArduPilot controller gains:
  nav-filter-alpha = 0.10
  vision-speed-source = zero
  max-nav-step-speed-mps = 0
  min-nav-inliers = 70
  VISO_POS_M_NSE = 4

If this also fails around the same time, the next investigation should move to
EKF/controller tuning rather than SIFT matching correctness.
```

### Test SIFT-SRC2-ZERO-SPEED-ALPHA010 - Very slow visual filter

CSV:

```text
live_sift_src2_zero_speed_alpha010_inl70.csv
```

Settings:

```text
vision-speed-source = zero
vision-extra-predict-sec = 0
max-nav-step-speed-mps = 0
min-nav-inliers = 70
nav-filter-alpha = 0.10
vision-publish-mode = rate
vision-rate-hz = 4
vision-timestamp-source = send
VISO_POS_M_NSE = 4
```

Result:

```text
rows = 145
duration ~= 49 s
match status = OK for all rows
nav rejects = none
EKF flags stayed 831 until rollback
rollback reason = pos_horiz_variance 0.3543 > 0.35

raw SIFT/Gazebo error:
  median ~= 0.42 m
  p95 ~= 1.55 m
  max ~= 2.51 m

filtered nav/Gazebo error:
  median ~= 0.42 m
  p95 ~= 2.33 m
  max ~= 2.65 m

VPE TX/Gazebo error:
  median ~= 0.38 m
  p95 ~= 2.80 m
  max ~= 3.29 m
```

Interpretation:

```text
Alpha 0.10 did not fix source-set-2 hover stability. It made the visual filter
too slow: once the vehicle started moving and/or a bad SIFT fix entered, the
filtered nav estimate lagged behind the raw fixes for several seconds. Around
14:21:06, raw SIFT had a bad east jump, then raw fixes recovered; however the
low-alpha filter kept publishing the stale offset, so VPE TX error stayed large
and EKF variance grew.

The next step should not be "more smoothing". The filter needs faster recovery
from residuals/outliers.
```

Next setting:

```text
Return to the better alpha and lower residual reset threshold:
  nav-filter-alpha = 0.25
  nav-filter-reset-residual-m = 1.0
  vision-speed-source = zero
  max-nav-step-speed-mps = 0
  min-nav-inliers = 70

This lets the visual estimate snap back to raw SIFT when the filter residual is
large, instead of dragging stale VPE for multiple seconds.
```

### Test SIFT-SRC2-ZERO-SPEED-ALPHA025-RESET1 - Residual reset at 1 m

CSV:

```text
live_sift_src2_zero_speed_alpha025_reset1_inl70.csv
```

Settings:

```text
vision-speed-source = zero
vision-extra-predict-sec = 0
max-nav-step-speed-mps = 0
min-nav-inliers = 70
nav-filter-alpha = 0.25
nav-filter-reset-residual-m = 1.0
vision-publish-mode = rate
vision-rate-hz = 4
vision-timestamp-source = send
VISO_POS_M_NSE = 4
```

Result:

```text
rows = 135
match status = OK for all rows
nav rejects = none
nav_filter_reset = 4 rows
EKF flags stayed 831 until rollback
rollback reason = pos_horiz_variance 0.3697 > 0.35

raw SIFT/Gazebo error:
  median ~= 0.40 m
  p95 ~= 2.02 m
  max ~= 2.68 m

filtered nav/Gazebo error:
  median ~= 0.32 m
  p95 ~= 2.54 m
  max ~= 3.02 m

VPE TX/Gazebo error:
  median ~= 0.16 m
  p95 ~= 3.16 m
  max ~= 3.66 m

VPE age:
  median ~= 0.59 s
  p95 ~= 0.84 s
  max ~= 1.00 s
```

Interpretation:

```text
Residual reset improved the nominal/median TX error, but it did not prevent the
closed-loop drift. The stream was continuous, there were no nav rejects, and
VPE age stayed under 1 s. The failure is therefore not caused by missing VPE or
large publish gaps.

The run used vision_align_seed_source=global_position_seed. Since VPE is fused
as ExternalNav in the vehicle's local NED frame, aligning the outgoing VPE to
GLOBAL_POSITION-derived seed can introduce a small XY discontinuity relative to
LOCAL_POSITION_NED at the EKF source switch. The next test should lock vision
alignment to local_ned_seed before switching to source set 2.
```

Next setting:

```text
Use local NED for VPE alignment:
  telemetry-seed-source = local

Keep:
  nav-filter-alpha = 0.25
  nav-filter-reset-residual-m = 1.0
  vision-speed-source = zero
  max-nav-step-speed-mps = 0
  min-nav-inliers = 70
```

### Test SIFT-SRC2-LOCALALIGN-ALPHA025-RESET1 - Local NED alignment

CSV:

```text
live_sift_src2_localalign_alpha025_reset1_inl70.csv
```

Settings:

```text
telemetry-seed-source = local
vision-align-source = telemetry
vision-speed-source = zero
vision-extra-predict-sec = 0
max-nav-step-speed-mps = 0
min-nav-inliers = 70
nav-filter-alpha = 0.25
nav-filter-reset-residual-m = 1.0
vision-publish-mode = rate
vision-rate-hz = 4
vision-timestamp-source = send
VISO_POS_M_NSE = 4
```

Result:

```text
rows = 246
duration ~= 75 s
vision_align_seed_source = local_ned_seed
match status = OK for all rows
nav rejects = low_inliers only once
nav_filter_reset = 85 rows
EKF flags stayed 831 until rollback
rollback reason = pos_horiz_variance 0.4128 > 0.35

raw SIFT/Gazebo error:
  median ~= 1.79 m
  p95 ~= 3.91 m
  max ~= 5.04 m

filtered nav/Gazebo error:
  median ~= 1.83 m
  p95 ~= 4.18 m
  max ~= 5.05 m

VPE TX/Gazebo error:
  median ~= 3.15 m
  p95 ~= 5.22 m
  max ~= 7.86 m

Vehicle/Gazebo truth movement:
  > 0.5 m at row 71
  > 1 m at row 75
  > 10 m at row 110
  > 50 m at row 169
  > 100 m at row 220
```

Interpretation:

```text
Local NED alignment removed the frame-mismatch suspicion but exposed the closed
loop problem more clearly. The EKF kept horizontal aiding flags, VPE continued
at under 1 s age, and ArduPilot followed the external-nav stream. The vehicle
physically ran away because early small VPE/SIFT errors were chased by the
GUIDED position controller; once motion started, SIFT latency caused larger
along-track errors and the loop amplified itself.

At this point further SIFT matching tweaks are unlikely to answer the core
question by themselves. The next controlled experiment should remove SIFT from
the loop and publish Gazebo truth with synthetic delay/noise that matches the
observed SIFT stream. If noisy/delayed truth also destabilizes, the next work is
EKF/controller/noise tuning. If noisy/delayed truth remains stable, the issue is
specific to the SIFT estimate behavior and filtering.
```

Tooling update:

```text
vision_debug/gazebo_truth_bridge.py now supports:
  --pose-delay-sec
  --pos-bias-north
  --pos-bias-east
  --pos-noise-std
  --pos-noise-tau-sec
  --noise-seed
```

### Test GT-DELAY06 - Gazebo truth VPE with 0.6 s pose delay

Log:

```text
gazebo_truth_vpe4hz_delay06.log
```

Settings:

```text
rate = 4 Hz
send_speed = false
speed_source = zero
attitude_source = zero
pose_delay_sec = 0.6
pos_noise_std = 0.0
```

Result:

```text
sent range = 1..523
EKF flags = 831 throughout
XY VPE/local error:
  median ~= 0.11 m
  p95 ~= 0.27 m
  max ~= 0.35 m
EKF pos_horiz_variance:
  median ~= 0.006
  p95 ~= 0.006
  max ~= 0.013
```

Interpretation:

```text
Pose delay alone did not destabilize source-set-2 hover in this stationary
test. ArduPilot can hold with 4 Hz VPE-only when the position is smooth, even
when the pose is delayed.
```

### Test GT-DELAY06-NOISE025 - Gazebo truth VPE with SIFT-like noise

Log:

```text
gazebo_truth_vpe4hz_delay06_noise025.log
```

Settings:

```text
rate = 4 Hz
send_speed = false
speed_source = zero
attitude_source = zero
pose_delay_sec = 0.6
pos_noise_std = 0.25 m
pos_noise_tau_sec = 0.6 s
noise_seed = 7
```

Result:

```text
sent range = 5..172
safe-switch rollback reason = pos_horiz_variance 0.3649 > 0.35
EKF flags = 831 until rollback
XY VPE/local error:
  median ~= 0.63 m
  p95 ~= 1.82 m
  max ~= 2.61 m
EKF pos_horiz_variance:
  median ~= 0.006
  p95 ~= 0.006
  max ~= 0.015 in bridge printout
Monitor saw pos_horiz_variance reach 0.3649 and rolled back.
```

Interpretation:

```text
This confirms the main closed-loop problem: ArduPilot can handle smooth delayed
VPE, but it does not tolerate a jittery XY VPE stream with current EKF/controller
settings. The vehicle chases the noisy position estimate, starts moving, and
variance eventually grows. This reproduces the SIFT failure without SIFT in the
loop, so the next priority is EKF/controller measurement-noise tuning rather
than more SIFT matching optimization.
```

Next setting:

```text
Repeat GT-DELAY06-NOISE025 while increasing VISO_POS_M_NSE:
  first 8 m
  then 10 m if needed

Goal: reduce EKF/controller reaction to VPE jitter while keeping ExternalNav
usable for XY position.
```

### Test GT-DELAY06-NOISE025-VISO8 - Noisy GT with higher VISO noise

Log:

```text
gazebo_truth_vpe4hz_delay06_noise025_viso8.log
```

Settings:

```text
rate = 4 Hz
send_speed = false
speed_source = zero
attitude_source = zero
pose_delay_sec = 0.6
pos_noise_std = 0.25 m
pos_noise_tau_sec = 0.6 s
VISO_POS_M_NSE = 8
```

Result:

```text
sent range = 1..164
safe-switch rollback reason = pos_horiz_variance 0.3569 > 0.35
EKF flags = 831 until rollback
Monitor local displacement during source set 2 ~= 0.95 m
Monitor max distance from switch point ~= 0.98 m

XY VPE/local error from bridge:
  median ~= 0.77 m
  p95 ~= 1.69 m
  max ~= 2.60 m

EKF pos_horiz_variance from bridge printout:
  median ~= 0.006
  p95 ~= 0.008
  max ~= 0.357
```

Interpretation:

```text
Increasing VISO_POS_M_NSE from 4 to 8 did not clear the conservative
safe-switch variance threshold. It slightly reduced the observed vehicle
displacement compared to the VISO=4 noisy GT run, but pos_horiz_variance still
briefly crossed 0.35.

Important nuance: this was a safe-switch rollback, not necessarily an ArduPilot
mode/failsafe LAND. The vehicle stayed within about 1 m of the switch point in
the MAVLink monitor. The next check should raise only the safe-switch monitor
threshold to see whether the variance spike is transient and recoverable.
```

Next setting:

```text
Keep VISO_POS_M_NSE = 8.
Repeat GT-DELAY06-NOISE025 but monitor with:
  safe-switch-source-set --max-pos-horiz-variance 0.8

If it survives 120 s without AP mode failsafe and without large XY drift, then
the previous rollback threshold was too strict for noisy ExternalNav tests.
If it still drifts or AP itself failsafes, then continue with VISO_POS_M_NSE=10
or controller tuning.
```

### Test GT-DELAY06-NOISE025-VISO8-THRESH08 - Higher monitor threshold

Log:

```text
gazebo_truth_vpe4hz_delay06_noise025_viso8_thresh08.log
```

Settings:

```text
rate = 4 Hz
send_speed = false
speed_source = zero
attitude_source = zero
pose_delay_sec = 0.6
pos_noise_std = 0.25 m
pos_noise_tau_sec = 0.6 s
VISO_POS_M_NSE = 8
safe-switch max_pos_horiz_variance = 0.8
```

Result:

```text
sent range = 1..352
estimated source-set-2 duration ~= 88 s
safe-switch rollback reason = pos_horiz_variance 0.8082 > 0.8
EKF flags = 831 until rollback
No AP LAND/failsafe STATUSTEXT seen in the pasted monitor output

Monitor local displacement from switch point:
  final ~= 3.15 m
  max ~= 3.15 m

Monitor XY speed:
  median ~= 0.95 m/s
  p95 ~= 2.53 m/s
  max ~= 3.19 m/s

Monitor pos_horiz_variance:
  median ~= 0.256
  p95 ~= 0.552
  max before rollback print ~= 0.777
  rollback checked 0.8082

Bridge VPE/local error:
  median ~= 1.80 m
  p95 ~= 4.70 m
  max ~= 7.09 m
```

Interpretation:

```text
Raising the safe-switch threshold showed that the 0.35 limit was an early
warning, not the root cause. With the higher threshold the vehicle stayed in
GUIDED longer, but the closed loop still became unstable: XY speed increased,
local position drifted about 3 m, and pos_horiz_variance eventually crossed 0.8.

The noisy/delayed GT experiment reproduces the SIFT failure without SIFT. The
remaining work is EKF/controller tuning and/or filtering the VPE stream to be
much less jittery before it reaches ArduPilot.
```

Next setting:

```text
Continue the controlled GT test before returning to SIFT:
  VISO_POS_M_NSE = 10
  safe-switch max_pos_horiz_variance = 0.8

If VISO=10 still drifts, reduce controller aggressiveness for XY position hold
instead of only increasing VISO noise.
```

### Test GT-DELAY06-NOISE025-VISO10-THRESH08 - Higher VISO noise

Logs:

```text
gazebo_truth_vpe4hz_delay06_noise025_viso10_thresh08.log
<codex-attachment>/pasted-text.txt
```

Settings:

```text
rate = 4 Hz
send_speed = false
speed_source = zero
attitude_source = zero
pose_delay_sec = 0.6
pos_noise_std = 0.25 m
pos_noise_tau_sec = 0.6 s
VISO_POS_M_NSE = 10
safe-switch max_pos_horiz_variance = 0.8
```

Doctor monitor result:

```text
safe-switch result = SAFE_SWITCH_OK
monitor rows = 115
EKF flags = 831 for all MON rows

Monitor local displacement from first MON row:
  final ~= 2.03 m
  max ~= 2.96 m

Monitor XY speed:
  median ~= 0.83 m/s
  p95 ~= 2.15 m/s
  max ~= 3.03 m/s

Monitor pos_horiz_variance:
  median ~= 0.194
  p95 ~= 0.488
  max ~= 0.679
```

Bridge log after continuing past the doctor window:

```text
sent range = 1..584
approx bridge duration ~= 146 s
first logged pos_horiz_variance > 0.8:
  sent=562 pos_var=0.917
last logged row:
  sent=584 pos_var=1.530

Bridge VPE/local XY error:
  median ~= 1.63 m
  p95 ~= 7.66 m
  max ~= 14.41 m
```

Interpretation:

```text
VISO_POS_M_NSE=10 is clearly better than VISO_POS_M_NSE=8 for the 120 s doctor
window: source set 2 stayed healthy and the safe-switch monitor did not roll back.

It is not a complete fix. The bridge stayed open after the doctor monitor ended,
and the same noisy/delayed closed-loop test later crossed pos_horiz_variance=0.8
and reached pos_var=1.53. So VISO=10 buys margin, but noisy position updates can
still accumulate into oscillation/drift.

Use VISO_POS_M_NSE=10 as the next SIFT baseline. If SIFT still diverges, the next
lever should be reducing the effective noise before AP sees it and/or softening
XY position-control response, not simply raising the rollback threshold again.
```

Next SIFT baseline:

```text
VISO_POS_M_NSE = 10
safe-switch max_pos_horiz_variance = 0.8
VPE only, no VSE
vision timestamp = send
vision z = 0
vision attitude = 0
vision speed source = zero
nav_filter_alpha = 0.25
nav_filter_reset_residual_m = 1.0
min_nav_inliers = 70
```

### Test SIFT-SRC2-VISO10-ALPHA025-RESET1-THRESH08 - Failed closed loop

Logs:

```text
live_sift_src2_viso10_alpha025_reset1_thresh08.csv
<codex-attachment>/pasted-text.txt
```

Settings:

```text
VISO_POS_M_NSE = 10
safe-switch max_pos_horiz_variance = 0.8
VPE only, no VSE
vision_publish_mode = rate
vision_rate_hz = 4
vision_timestamp_source = send
vision_speed_source = zero
vision_z_source = zero
vision_attitude_source = zero
vision_align_source = telemetry
telemetry_seed_source = local
nav_filter_alpha = 0.25
nav_filter_reset_residual_m = 1.0
min_nav_inliers = 70
```

Doctor result:

```text
safe-switch result = rollback
rollback reason = pos_horiz_variance 0.9247 > 0.8000
EKF flags = 831 until rollback

MON rows before rollback = 26
doctor local displacement:
  final ~= 27.59 m
  max ~= 36.63 m
doctor XY speed:
  median ~= 1.31 m/s
  p95 ~= 11.57 m/s
  max ~= 16.67 m/s
doctor MON pos_horiz_variance:
  median ~= 0.047
  p95 ~= 0.412
  max MON before fail ~= 0.612
FAIL row pos_horiz_variance = 0.925
```

CSV result:

```text
rows = 293
time span = 82 s
accepted matches = 293
rejected nav rows = 1 low_inliers

Overall SIFT/Gazebo error:
  median ~= 0.37 m
  p90 ~= 8.21 m
  p95 ~= 9.89 m
  max ~= 12.42 m

Matcher duration:
  median ~= 0.17 s
  p95 ~= 0.35 s
  max ~= 0.48 s

Inliers:
  median ~= 92
  p95 ~= 129
  max ~= 149
```

Time-window split:

```text
0-45 s, hover/source-set-1-ish:
  truth displacement ~= 0.90 m
  error median ~= 0.12 m
  error p95 ~= 0.54 m
  residual p95 ~= 0.27 m
  tile = patch_r05_c05

45-60 s, source-set-2 onset:
  truth displacement ~= 24.81 m
  error median ~= 1.58 m
  error p95 ~= 5.15 m
  residual p95 ~= 2.49 m

60 s+, runaway:
  truth displacement ~= 362 m
  error median ~= 6.81 m
  error p95 ~= 11.00 m
  residual p95 ~= 10.14 m
  tiles crossed from patch_r05_c05 into r04/r03 patches
```

Interpretation:

```text
The matcher itself is not the immediate performance bottleneck in this run:
median runtime is about 0.17 s and hover accuracy before source-set-2 is strong.

The failure is closed-loop. After switching to source set 2, the vehicle starts
moving even without an intended Go To. Once the vehicle accelerates, the VPE
stream lags the true Gazebo position by several meters. AP then chases a noisy
and delayed horizontal position estimate, speed grows, and the variance monitor
eventually rolls back.

This is stronger than the controlled noisy-GT result. VISO=10 is not enough for
SIFT in closed loop. The next useful direction is not larger search area or lower
inliers; it is to make the VPE fed to AP more control-friendly:

1. Test a source-set-2 hover with position controller softened, or
2. Publish a more conservative ExternalNav estimate: stronger smoothing, lower
   innovation rate, and/or hold/freeze VPE when residual/jump grows.
```

### Test SIFT-SRC2-VISO10-PSC03-WPNAV1MS - Soft XY controller runaway

Logs:

```text
live_sift_src2_viso10_psc03_wpnav1ms.csv
<codex-attachment>/pasted-text.txt
```

Settings:

```text
VISO_POS_M_NSE = 10
PSC_POSXY_P = 0.3
WPNAV_SPEED = 100 cm/s
WPNAV_ACCEL = 50 cm/s/s
safe-switch max_pos_horiz_variance = 0.8
SIFT/VPE settings same as previous VISO10 baseline
```

Doctor result:

```text
safe-switch was manually interrupted
important: source set was unchanged after Ctrl-C
EKF flags = 831 in all printed MON rows

doctor MON rows = 71
doctor local displacement:
  final ~= 603 m
  max ~= 603 m
doctor XY speed:
  median ~= 1.21 m/s
  p95 ~= 7.47 m/s
  max ~= 8.35 m/s
doctor pos_horiz_variance:
  median ~= 0.000
  p95 ~= 0.325
  max ~= 0.410
```

CSV result:

```text
rows = 481
time span = 133 s
accepted matches = 481
nav rejected = 17
reject reasons:
  prediction_jump_gate = 10
  low_inliers = 7

Overall SIFT/Gazebo error:
  median ~= 1.02 m
  p90 ~= 9.12 m
  p95 ~= 9.99 m
  max ~= 21.09 m

Matcher duration:
  median ~= 0.177 s
  p95 ~= 0.235 s
  max ~= 1.543 s

Truth path:
  displacement ~= 726 m
  path length ~= 744 m
```

Time-window split:

```text
0-30 s:
  truth displacement ~= 0.1 m
  error median ~= 0.23 m
  error p95 ~= 0.49 m

30-60 s:
  truth displacement ~= 0.6 m
  error median ~= 0.13 m
  error p95 ~= 0.30 m

60-90 s:
  truth displacement ~= 96 m
  error median ~= 5.40 m
  error p95 ~= 8.40 m

90-120 s:
  truth displacement ~= 396 m
  error median ~= 8.38 m
  error p95 ~= 10.07 m

120 s+:
  truth displacement ~= 235 m inside the final window
  error median ~= 9.70 m
  error p95 ~= 12.07 m
```

Interpretation:

```text
Softening PSC_POSXY_P did not solve the closed-loop issue. It made the vehicle
run away much farther while EKF flags stayed healthy and pos_horiz_variance often
printed as 0.000. This means the monitor variance is not sufficient as the only
guard for this failure mode.

The first 60 s are strong evidence that the SIFT matcher is locally accurate when
the vehicle is not moving fast. The runaway starts after source set 2 / control
engagement. The next isolation should test whether GUIDED has a stale position
target by switching in LOITER/BRAKE instead of GUIDED.
```

Next isolation:

```text
Restore controller params or at least stop using PSC_POSXY_P=0.3 for this branch.
Run the same SIFT VPE publisher, but before source-set switch set the vehicle to
LOITER and wait 10 s. Then run safe-switch-source-set while the mode is LOITER.

If LOITER holds, the previous failures were likely GUIDED target/setpoint related.
If LOITER also drifts, the SIFT VPE stream itself is creating a biased/lagged
closed-loop position hold.
```

Safety script update:

```text
mavlink_doctor.py safe-switch-source-set now supports:
  --max-local-drift-m
  --max-xy-speed-mps

These are disabled by default. Use them in SIFT source-set-2 tests because the
PSC03 test showed that EKF pos_horiz_variance can stay low or print 0.000 while
the vehicle physically runs away hundreds of meters.
```

### Implementation Note - Fixed-rate Kalman VPE stream

Date: 2026-06-02

Purpose:

```text
Stop feeding AP directly from irregular SIFT pose fixes. SIFT now updates a 2D
constant-velocity Kalman filter at capture-time, while the MAVLink publisher can
send a fixed-rate current-time prediction.
```

New selectable VPE stream modes:

```text
--vision-stream-mode event_based_raw
  Old-style event/fix behavior for A/B tests.

--vision-stream-mode fixed_rate_hold_last
  Fixed-rate sender that holds the latest accepted visual pose.

--vision-stream-mode fixed_rate_kalman_predict
  Fixed-rate sender using VisualPositionKalman2D current-time prediction.
  This is the new default.
```

Kalman defaults:

```text
state = [N, E, vN, vE]
process_noise_pos = 0.5
process_noise_vel = 1.0
meas_noise_pos = 4.0
min_inliers = 40
max_jump_m = 3.0
max_innovation_m = 3.0
max_fix_age_sec = 0.5
max_predict_age_sec = 0.4
```

Logging:

```text
Match CSV now includes kf_* columns:
  kf_capture_time, kf_receive_time, kf_raw_north_m, kf_raw_east_m,
  kf_aligned_north_m, kf_aligned_east_m, kf_accepted,
  kf_reject_reason, kf_jump_m, kf_innovation_m, kf_fix_age_s,
  kf_north_m, kf_east_m, kf_vn_mps, kf_ve_mps, kf_update_count

Publish CSV defaults to:
  <log_csv_stem>_vision_tx.csv

Publish CSV columns include:
  send_time, vpe_usec, vpe_N, vpe_E, kf_N, kf_E, kf_vN, kf_vE,
  last_fix_age_s, publish_dt_s, publish_mode, stream_mode
```

Validation run:

```text
python3 -m compileall sau_sift_nav live_sift_nav.py vision_debug/mavlink_doctor.py
python3 live_sift_nav.py --help | rg 'vision-stream-mode|kf-|vision-publish-log'
small offline SharedState/Kalman smoke test passed
```

### Test SIFT-KF-SRC2-KFAGE04 - Kalman stream first flight

Logs:

```text
live_sift_kf_src2.csv
live_sift_kf_src2_vision_tx.csv
<codex-attachment>/pasted-text.txt
```

Settings:

```text
vision_stream_mode = fixed_rate_kalman_predict
vision_rate_hz = 10
kf_max_predict_age_sec = 0.4
kf_process_noise_pos = 0.5
kf_process_noise_vel = 1.0
kf_meas_noise_pos = 4.0
kf_min_inliers = 40
kf_max_jump_m = 3.0
kf_max_innovation_m = 3.0
kf_max_fix_age_sec = 0.5
safe-switch max_pos_horiz_variance = 0.8
safe-switch max_local_drift_m = 8
safe-switch max_xy_speed_mps = 3
```

Doctor result:

```text
rollback reason = pos_horiz_variance 0.9656 > 0.8000
EKF flags = 831 until rollback
MON rows = 21

doctor local displacement:
  final ~= 0.78 m
  max ~= 1.70 m

doctor XY speed:
  median ~= 0.41 m/s
  p95 ~= 1.68 m/s
  max ~= 1.76 m/s

doctor pos_horiz_variance:
  median ~= 0.111
  p95 ~= 0.355
  max MON before FAIL ~= 0.540
  FAIL row pos_horiz_variance = 0.966
```

Match CSV result:

```text
rows = 189
time span = 53 s
all matches = OK
nav rejected = 26 low_inliers

Kalman measurements:
  accepted = 183
  rejected = 6
  reject reasons:
    innovation_gate = 4
    stale_fix = 1
    jump_gate = 1

SIFT/Gazebo error:
  median ~= 0.37 m
  p90 ~= 2.62 m
  p95 ~= 3.82 m
  max ~= 4.70 m

Kalman innovation:
  median ~= 0.24 m
  p90 ~= 1.75 m
  p95 ~= 2.38 m
  max ~= 7.12 m

Matcher duration:
  median ~= 0.20 s
  p95 ~= 0.27 s
  max ~= 1.41 s
```

Publish CSV result:

```text
publisher loop rows = 537
publisher loop span ~= 53.9 s
publish_dt median ~= 0.100 s
publish_dt p95 ~= 0.103 s
publish_dt max ~= 0.112 s

sent VPE rows = 230
skipped rows = 307
skip reasons:
  stale_estimate = 287
  waiting_estimate = 20

effective sent VPE rate ~= 4.5 Hz
last_fix_age on sent rows:
  median ~= 0.33 s
  p95 ~= 0.40 s
  max ~= 0.40 s
```

Interpretation:

```text
The Kalman architecture improved the failure mode: the vehicle did not run away
hundreds of meters. The source-set-2 test rolled back on EKF variance while local
drift was still under 2 m.

However, with kf_max_predict_age_sec=0.4 the actual VPE stream is not 10 Hz.
The loop runs at 10 Hz, but more than half the publish attempts are skipped as
stale_estimate. The next test should keep the same Kalman gates and increase only
kf_max_predict_age_sec to 1.0 so AP receives a continuous 10 Hz predicted pose
stream.
```

### Implementation Note - Offline Kalman tuning

Date: 2026-06-02

Added:

```text
tools/tune_visual_kalman.py
```

Capabilities:

```text
Reads one or more live_sift*.csv files.
Uses kf_capture_time as SIFT measurement timestamp.
Uses a separate 10 Hz publish timeline for replay output.
Interpolates Gazebo truth N/E linearly to each replay publish timestamp.
Computes raw SIFT baseline metrics in two forms:
  raw_fix_times = raw SIFT fixes at their own capture timestamps
  raw_hold_last_publish_times = raw SIFT held on the same 10 Hz publish timeline
Runs grid search over:
  meas_noise_pos_m
  process_accel_noise_mps2
  max_innovation_m
  max_predict_age_s
  min_inliers
Writes:
  kalman_tuning_results.csv
  raw_baseline_metrics.csv
  best_visual_kalman_config.json
  top3_visual_kalman_configs.json
  best_kalman_replay.csv
  tuning_plots/*.png when matplotlib is available
```

Kalman process model:

```text
state = [N, E, vN, vE]
q = sigma_a^2
Q_axis = [[dt^4/4*q, dt^3/2*q],
          [dt^3/2*q, dt^2*q]]
```

Live config loading:

```text
live_sift_nav.py --vision-kf-config best_visual_kalman_config.json
```

First tuning run:

```text
input = live_sift_kf_src2.csv
mode = hover_mode
configs = 960
```

Raw baseline:

```text
baseline = raw_fix_times
median_error_m ~= 0.407
p95_error_m ~= 3.386
max_error_m ~= 4.257
rms_error_m ~= 1.323
path_length_m ~= 55.300
jitter_p95_m ~= 0.869
```

Raw hold-last 10 Hz baseline:

```text
baseline = raw_hold_last_publish_times
median_error_m ~= 0.431
p95_error_m ~= 3.828
max_error_m ~= 4.879
rms_error_m ~= 1.429
path_length_m ~= 54.796
jitter_p95_m ~= 0.469
```

Best offline config:

```text
KF_MEAS_NOISE_POS_M = 3.0
KF_PROCESS_ACCEL_NOISE_MPS2 = 2.0
KF_INITIAL_POS_STD_M = 2.0
KF_INITIAL_VEL_STD_MPS = 1.0
MAX_INNOVATION_M = 2.0
MAX_JUMP_M = 3.0
MAX_PREDICT_AGE_S = 0.3
MAX_FIX_AGE_S = 0.5
MIN_INLIERS = 30
```

Best replay metrics:

```text
median_error_m ~= 0.343
p95_error_m ~= 2.155
max_error_m ~= 5.882
rms_error_m ~= 1.078
path_length_m ~= 17.830
jitter_p95_m ~= 0.173
accepted_fix_count = 166
rejected_fix_count = 22
publish_count = 438
score ~= 6.305
```

Interpretation:

```text
Offline tuning strongly prefers a much smoother VPE trajectory for hover:
path length drops from about 55 m to about 18 m and jitter p95 drops from about
0.87 m to about 0.17 m. p95 error also improves, but max error increases because
the hover-mode score prioritizes jitter/path length.

This is not a closed-loop guarantee; it only chooses better live candidates.
Next live test should use --vision-kf-config best_visual_kalman_config.json and
the guarded safe-switch command.
```

### Implementation Note - Validation log only mode

Date: 2026-06-02

Added a validation-only logging mode before any source set 2 flight:

```text
live_sift_nav.py --run-mode validation_log_only
```

Behavior:

```text
The vehicle remains on GPS/source set 1.
The live app does not send MAV_CMD_SET_EKF_SOURCE_SET.
--configure-ekf is ignored in validation_log_only mode.
SIFT matching, visual alignment, Kalman filtering, optional 10 Hz VPE TX, and
Gazebo truth logging can all run.
```

New validation CSV:

```text
--validation-log-csv sift_validation_*.csv
--validation-log-rate-hz 10
```

Columns include:

```text
time, true_north_m, true_east_m
raw_sift_north_m, raw_sift_east_m
aligned_sift_north_m, aligned_sift_east_m
sift_inliers, sift_accepted, sift_reject_reason, sift_jump_m, sift_innovation_m
sift_capture_time, sift_receive_time
kf_north_m, kf_east_m, kf_vnorth_mps, kf_veast_mps
kf_last_fix_age_s, kf_predict_age_s, kf_initialized
vpe_publish_time, vpe_north_m, vpe_east_m, vpe_usec
vpe_publish_dt_s, vpe_rate_hz, vpe_publish_jitter_s, vpe_status, vpe_sent_count
```

Offline replay updates:

```text
tools/tune_visual_kalman.py now accepts validation CSV columns.
Validation rows are deduplicated by sift_capture_time before Kalman replay, so a
10 Hz logger does not replay the same SIFT fix multiple times.
Replay applies a SIFT measurement only after sift_receive_time, but updates the
Kalman state at sift_capture_time. This prevents optimistic replay results that
assume a frame result is available before SIFT processing finishes.
Metrics now include bias_north_m, bias_east_m, bias_norm_m, reject_ratio,
last_fix_age_median_s, last_fix_age_p95_s, replay_duration_s,
effective_publish_rate_hz, publish_dt_* and publish_jitter_p95_s.
--min-effective-publish-rate-hz can exclude configs that are too sparse for
ExternalNav; validation used 4.0 Hz.
Plots now include:
  trajectory.png
  truth_trajectory.png
  raw_sift_trajectory.png
  kalman_trajectory.png
  error_vs_time.png
  innovation_vs_time.png
  last_fix_age_vs_time.png
  publish_dt_vs_time.png
```

### Test SIFT-VALIDATION-100M-HOVER

Date: 2026-06-02

Setup:

```text
run_mode = validation_log_only
vehicle source set = 1 / GPS
duration ~= 120 s
height ~= 100 m
input = sift_validation_100m_hover.csv
min_effective_publish_rate_hz = 4.0
```

Validation log:

```text
unique SIFT fixes = 430
fix rate ~= 3.58 Hz
SIFT fix age median ~= 0.254 s
SIFT fix age p95 ~= 0.349 s
inliers min/median/p95/max = 71 / 85 / 97 / 105
live validation VPE sent = 214 messages over ~120 s with old config
```

Raw SIFT baseline:

```text
median_error_m ~= 0.276
p95_error_m ~= 0.487
max_error_m ~= 0.806
rms_error_m ~= 0.295
bias_norm_m ~= 0.214
path_length_m ~= 72.521
jitter_p95_m ~= 0.416
```

Best 4 Hz-eligible Kalman config:

```text
KF_MEAS_NOISE_POS_M = 6.0
KF_PROCESS_ACCEL_NOISE_MPS2 = 0.2
MAX_INNOVATION_M = 2.0
MAX_PREDICT_AGE_S = 0.4
MIN_INLIERS = 80
```

Best replay metrics:

```text
median_error_m ~= 0.213
p95_error_m ~= 0.333
max_error_m ~= 0.380
rms_error_m ~= 0.232
bias_norm_m ~= 0.206
path_length_m ~= 5.318
jitter_p95_m ~= 0.031
accepted_fix_count = 369
rejected_fix_count = 60
reject_ratio ~= 0.140
last_fix_age_p95_s ~= 0.394
effective_publish_rate_hz ~= 4.45
```

Interpretation:

```text
The previous pure-score best with MAX_PREDICT_AGE_S=0.3 was smoother but only
about 1.55 Hz after receive-time availability was modeled, so it is not suitable
for ExternalNav. With a 4 Hz minimum rate, MAX_PREDICT_AGE_S=0.4 becomes the best
candidate. It sharply reduces hover path length and jitter while improving p95
error versus raw SIFT.
```

### Test SIFT-VALIDATION-100M-HOVER-V2

Date: 2026-06-02

Setup:

```text
run_mode = validation_log_only
vehicle source set = 1 / GPS
config = best_visual_kalman_validation_100m_hover.json
duration ~= 120 s
input = sift_validation_100m_hover_v2.csv
min_effective_publish_rate_hz = 4.0
```

Validation log:

```text
unique SIFT fixes = 426
fix rate ~= 3.54 Hz
SIFT fix age median ~= 0.239 s
SIFT fix age p95 ~= 0.310 s
inliers min/median/p95/max = 74 / 89 / 101 / 107
live VPE TX sent = 629 messages over ~120 s
live VPE TX effective rate ~= 5.23 Hz
live VPE sent age p95 ~= 0.391 s
live VPE publish dt p95 ~= 0.103 s
```

Raw SIFT baseline:

```text
median_error_m ~= 0.200
p95_error_m ~= 0.297
max_error_m ~= 0.652
rms_error_m ~= 0.208
bias_norm_m ~= 0.113
path_length_m ~= 66.715
jitter_p95_m ~= 0.371
```

Best 4 Hz-eligible Kalman config:

```text
KF_MEAS_NOISE_POS_M = 6.0
KF_PROCESS_ACCEL_NOISE_MPS2 = 0.2
MAX_INNOVATION_M = 2.0
MAX_PREDICT_AGE_S = 0.4
MIN_INLIERS = 80
```

Best replay metrics:

```text
median_error_m ~= 0.159
p95_error_m ~= 0.198
max_error_m ~= 0.216
rms_error_m ~= 0.156
bias_norm_m ~= 0.116
path_length_m ~= 4.238
jitter_p95_m ~= 0.020
accepted_fix_count = 403
rejected_fix_count = 22
reject_ratio ~= 0.052
last_fix_age_p95_s ~= 0.392
effective_publish_rate_hz ~= 5.33
```

Interpretation:

```text
V2 validates the selected hover config more strongly than V1. Live VPE TX is now
above 5 Hz, last-fix age stays below about 0.4 s p95, and Kalman improves both
error and smoothness against raw SIFT. This is good enough for a guarded source
set 2 hover-only test, but not yet for waypoint/goto motion.
```

### Test SIFT-SRC2-HOVER-GUARD-02

Date: 2026-06-02

Setup:

```text
source set switch = guarded 1 -> 2 -> 1 rollback
Gazebo truth guard = enabled
max_gz_drift_m = 3.0
KF config = best_visual_kalman_src2_hover_fail_replay.json
```

Safe-switch result:

```text
EKF flags stayed healthy at 831 during the early drift.
LOCAL_POSITION_NED drift was still modest: about N=1.30, E=-0.08 m.
Gazebo guard rolled back first: gz_drift=3.234 > 3.000 m.
```

Validation CSV summary:

```text
raw/aligned SIFT still tracked reasonably at first, but p95 error grew to about 2.4 m.
KF/VPE p95 error became about 10 m after the filter stopped accepting large corrections.
SIFT accepted rows = 423, rejected rows = 46.
Main reject reason after escape = jump_gate, 34 rows.
VPE status rows: sending=346, stale_estimate=113.
KF last-fix age reached about 4 s after jump_gate rejections.
```

Interpretation:

```text
The failure is no longer simply "SIFT cannot match". During source set 2 drift,
SIFT can still produce high-inlier measurements, but the Kalman jump gate rejects
large corrective fixes. The VPE output then freezes/stales while the real Gazebo
vehicle keeps moving. LOCAL_POSITION_NED and EKF flags are not independent enough
to catch this, so Gazebo truth guard must remain enabled during all source set 2
tests.

Next change: add an optional high-inlier Kalman reset path. If a visual fix has
enough inliers and the residual is large, the KF can reset/re-anchor instead of
rejecting and freezing. This path is disabled by default and should only be used
with Gazebo truth guard during tests.
```

### Test SIFT-SRC2-HOVER-GUARD-03-RESET

Date: 2026-06-02

Setup:

```text
KF config = best_visual_kalman_src2_hover_fail_replay.json
high-inlier reset = enabled
kf_reset_min_inliers = 90
kf_reset_residual_m = 8
Gazebo truth guard max drift = 3 m
```

Safe-switch result:

```text
Rollback reason = gz_drift=3.168 > 3.000
EKF flags still reported healthy at rollback.
```

CSV summary:

```text
validation duration ~= 78.35 s
aligned raw SIFT median/p95/max error ~= 0.172 / 2.030 / 6.740 m
KF median/p95/max error ~= 0.102 / 4.682 / 10.719 m
VPE median/p95/max error ~= 0.102 / 4.721 / 10.381 m
SIFT accepted/rejected = 743 / 37
main reject reason = jump_gate, 29 rows
reset events = 3, reset_reason=jump_gate
VPE status rows: sending=628, stale_estimate=143
```

Interpretation:

```text
The reset path works, but the thresholds were too conservative for the escape
case. During the important recovery window, good corrective SIFT fixes appeared
with about 80-87 inliers, so kf_reset_min_inliers=90 delayed reset until after
the guard had already rolled back.

Also, the hover-tuned KF remained too smooth for the source-set-2 escape motion.
The vehicle moved several meters while the KF/VPE lagged behind. This is not a
pure hover-noise problem anymore; source-set-2 testing needs a motion-capable KF
candidate plus earlier reset.
```

Motion replay on this failed source-set-2 log:

```text
input = sift_src2_hover_test_03_reset_validation.csv
mode = motion_mode
min_effective_publish_rate_hz = 4.0
best config = best_visual_kalman_src2_hover_test_03_motion.json

raw baseline median/p95/max error ~= 0.170 / 1.303 / 5.104 m
raw hold-last p95 error ~= 1.429 m

best config:
KF_MEAS_NOISE_POS_M = 5.0
KF_PROCESS_ACCEL_NOISE_MPS2 = 0.5
MAX_INNOVATION_M = 2.0
MAX_PREDICT_AGE_S = 0.5
MIN_INLIERS = 30

best replay median/p95 error ~= 0.100 / 1.277 m
path_length_m ~= 8.950
jitter_p95_m ~= 0.055
effective_publish_rate_hz ~= 8.10
```

### Test SIFT-SRC2-HOVER-GUARD-04-MOTION-RESET

Date: 2026-06-02

Setup:

```text
KF config = best_visual_kalman_src2_hover_test_03_motion.json
kf_reset_min_inliers = 80
kf_reset_residual_m = 6
Gazebo truth guard max drift = 5 m
```

Safe-switch result:

```text
Rollback reason = gz_drift=5.230 > 5.000
EKF flags still reported healthy at rollback.
```

CSV summary:

```text
validation duration ~= 50.82 s
aligned raw SIFT median/p95/max error ~= 0.316 / 2.472 / 3.973 m
KF median/p95/max error ~= 0.268 / 5.230 / 7.074 m
VPE median/p95/max error ~= 0.266 / 5.245 / 7.210 m
SIFT accepted/rejected = 449 / 56
reject reasons = jump_gate 24, innovation_gate 9, stale_fix 2, waiting_fix 21
reset events = 0
VPE status rows: sending=279, stale_estimate=205
KF last-fix age p95 ~= 1.28 s
```

Interpretation:

```text
The reset threshold was still too high; no reset event occurred. More importantly,
the closed-loop source-set-2 escape shows the Kalman output lagging raw SIFT. In
this run raw aligned SIFT p95 error was much better than KF/VPE p95 error.

Next test should isolate whether the KF smoothing itself is hurting the EKF. Run
a fixed-rate hold-last raw visual pose test, keeping Gazebo guard enabled. If raw
hold-last behaves better, the live source-set-2 mode should not use the hover KF
as the main VPE source until a latency-aware/motion-aware filter is built.
```
