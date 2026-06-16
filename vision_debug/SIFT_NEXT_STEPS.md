# SIFT / VPE Integration Notes

This file is a concise historical summary of the SIFT-to-VPE integration work.
It is retained for reproducibility and project history. It is not the main
thesis result.

## Context

The project first aimed to feed SIFT-based position estimates into ArduPilot
EKF3 through `VISION_POSITION_ESTIMATE`. Controlled Gazebo-truth tests showed
that the MAVLink message route and ExternalNav horizontal-position source could
be made to work in SITL. However, live SIFT estimates contained jitter and
occasional jumps that were not stable enough for closed-loop EKF source-set-2
operation.

## Main Finding

```text
SIFT localization works as an image-to-map estimator and can be evaluated
against Gazebo truth.

The EKF/VPE navigation path remained experimental because raw SIFT jumps and
filter latency could increase EKF variance or vehicle drift during source-set-2
tests.
```

## Implemented Tracks

- SIFT master-map patch generation.
- Runtime patch-window matching with RANSAC homography.
- Gazebo truth comparison and CSV logging.
- VPE-only MAVLink output experiments.
- Gazebo-truth VPE bridge for isolating EKF behavior from SIFT error.
- Offline Kalman replay/tuning experiments.
- Guarded source-set switching tests with rollback.

## Evidence Locations

Main thesis accuracy evidence:

```text
examples/logs/
chapter4_outputs/
```

Selected experimental integration evidence:

```text
examples/experimental_logs/
vision_debug/TEST_LOG.md
```

## Interpretation Boundary

Use this file to understand why EKF/VPE integration was treated as experimental.
Do not cite it as proof of a fully validated autonomous navigation stack.
