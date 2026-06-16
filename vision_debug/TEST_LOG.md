# Experimental Integration Log Summary

This document summarizes the integration experiments at a high level. The
selected raw logs are kept under `examples/experimental_logs/`.

## Experiment Groups

### Gazebo Truth VPE Bridge

Purpose:

- Feed Gazebo model pose as `VISION_POSITION_ESTIMATE`.
- Isolate ArduPilot/EKF ExternalNav behavior from SIFT matching error.
- Test VPE-only horizontal position fusion with no `VISION_SPEED_ESTIMATE`.

Representative logs:

```text
examples/experimental_logs/gazebo_truth_vpe4hz*.log
```

### SIFT Observe / Performance Logs

Purpose:

- Compare SIFT master-map matching under different camera/FOV/resolution
  settings.
- Inspect inlier counts, good match counts, processing time, and accepted fix
  rates.

Representative logs:

```text
examples/experimental_logs/live_sift_master_*.csv
examples/logs/sift_observe_100m_*.csv
```

### VPE / Source-Set-2 Logs

Purpose:

- Test SIFT-derived VPE output during hover and low-speed guided movement.
- Observe EKF variance, vehicle drift, source-set switching behavior, and
  guarded rollback.

Representative logs:

```text
examples/experimental_logs/live_sift_vpe_*.csv
examples/experimental_logs/live_sift_src2_*.csv
examples/experimental_logs/sift_src2_hover_test_*.csv
```

### Kalman Validation / Replay Logs

Purpose:

- Replay SIFT fixes offline.
- Compare raw SIFT, hold-last output, and Kalman-smoothed output against Gazebo
  truth.
- Check whether smoothing reduces jitter without adding too much lag.

Representative logs:

```text
examples/experimental_logs/sift_validation_*.csv
examples/experimental_logs/kalman_tuning_*.csv
examples/experimental_logs/best_kalman_replay_*.csv
examples/experimental_logs/best_visual_kalman_*.json
```

## Thesis Interpretation

The stable thesis result is the visual localization accuracy analysis. The
MAVLink/EKF3 work is useful engineering evidence, but it is not presented as a
validated autonomous flight-control solution.

Recommended wording:

```text
ExternalNav integration was investigated in SITL. While controlled Gazebo-truth
VPE tests helped verify the message and source-selection path, live SIFT-driven
closed-loop operation remained sensitive to estimator jitter and latency. For
this reason, the thesis evaluation focuses on visual localization accuracy
against Gazebo ground truth.
```
