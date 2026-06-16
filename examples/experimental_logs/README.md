# Experimental Integration Logs

This folder keeps a selected subset of raw experiment logs from the MAVLink/EKF3
ExternalNav, VPE, SIFT performance, and Kalman-filter investigations.

They are included so reviewers can inspect the integration evidence without
requiring the full local experiment archive.

These files are not the main thesis success metrics. The main accuracy results
are in `examples/logs/` and the report summaries are in `chapter4_outputs/`.

Included groups:

- `gazebo_truth_vpe4hz*.log`
  - Gazebo ground-truth VPE bridge logs for isolating EKF behavior.
- `live_sift_master_*.csv`
  - SIFT master-map performance and camera-geometry observation logs.
- `live_sift_2ms*.csv`
  - Early low-speed guided movement and inlier-threshold logs.
- `live_sift_vpe_*.csv`
  - VPE publishing mode experiments.
- `live_sift_src2_*.csv`
  - Source-set-2 SIFT/VPE experiments.
- `sift_src2_hover_test_*.csv`
  - Guarded source-set hover/motion test logs.
- `sift_validation_*.csv`, `raw_baseline_*.csv`, `kalman_tuning_*.csv`,
  `best_kalman_replay_*.csv`
  - Validation and offline Kalman replay/tuning logs.
- `best_visual_kalman_*.json`, `top3_visual_kalman_*.json`
  - Selected Kalman tuning configuration outputs.

Interpretation boundary:

```text
Use these logs as experimental integration and limitation evidence.
Do not use them to claim a fully validated EKF-driven autonomous navigation
solution.
```
