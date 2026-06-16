# Vision Debug Utilities

This directory contains diagnostic tools and lab notes used during the
MAVLink, EKF3 ExternalNav, Gazebo truth, and VPE experiments.

These files are intentionally separated from the main SIFT localization
pipeline. The thesis-version result is the SIFT image-to-map localization
accuracy evaluated against Gazebo ground truth. The EKF/source-set work should
be treated as experimental integration and limitation evidence.

## Tools

- `mavlink_doctor.py`
  - Reads MAVLink status, EKF flags, local/global position, and key parameters.
  - Sends controlled `VISION_POSITION_ESTIMATE` test streams.
  - Supports EKF source-set switching helpers and guarded rollback checks.
- `gazebo_truth_bridge.py`
  - Reads Gazebo model pose.
  - Can publish Gazebo ground-truth pose as VPE for isolating the ArduPilot/EKF
    side from SIFT errors.
- `extnav_xy.parm`
  - Reference parameter set for the ExternalNav horizontal-position tests.
- `TEST_LOG.md`
  - Chronological lab notebook of the integration experiments.
- `SIFT_NEXT_STEPS.md`
  - Historical notes from the period where SIFT filtering and VPE publishing
    were being investigated.

## How To Interpret This Folder

Use this folder to understand how the integration experiments were debugged.
Do not present these notes as a validated autonomous flight-control solution.

The important conclusion for the thesis version is:

```text
SIFT visual localization was implemented and evaluated against Gazebo truth.
MAVLink/EKF3 ExternalNav integration was investigated, but it remained
experimental and should be discussed as a limitation/future-work topic.
```

Selected raw logs from this integration work are included under:

```text
examples/experimental_logs/
```

The curated main accuracy logs are under:

```text
examples/logs/
```

## Typical Diagnostic Uses

Check vehicle/MAVLink state:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  status \
  --duration 10
```

Send a short dummy VPE stream for VisualOdom health checks:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  stream-vision \
  --pose-source manual \
  --x 0 \
  --y 0 \
  --z 0 \
  --rate 10 \
  --duration 30
```

Switch EKF source set with a rollback guard:

```bash
python3 vision_debug/mavlink_doctor.py \
  --mavlink udp:127.0.0.1:14550 \
  safe-switch-source-set \
  --to-set 2 \
  --rollback-set 1 \
  --monitor-sec 90
```

Run Gazebo truth as a VPE source for isolating EKF behavior from SIFT:

```bash
python3 vision_debug/gazebo_truth_bridge.py \
  --mavlink udp:127.0.0.1:14550 \
  --gz-topic /world/iris_runway/pose/info \
  --model iris_with_gimbal \
  --axis enu \
  --rate 4 \
  --no-send-speed \
  --attitude-source zero \
  --duration 0
```

These commands depend on a running SITL/Gazebo environment and are provided as
reproducibility aids, not as required steps for the report figures.
