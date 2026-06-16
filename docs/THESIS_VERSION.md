# Thesis Version Notes

This repository snapshot represents the bachelor-thesis implementation of the
UAV visual map localization project.

## Versioning Plan

Recommended public repository flow:

```text
main          stable public branch
thesis-2026   frozen thesis submission branch
develop       future development branch
```

Initial release tag:

```text
v1.0.0-thesis
```

Documentation-only thesis maintenance releases may use:

```text
v1.0.x-thesis
```

## What This Version Claims

- SIFT-based image-to-map localization was implemented.
- Reference-map patching and descriptor extraction were implemented.
- Live camera-frame matching and Gazebo truth comparison were implemented.
- Report-oriented metrics and figures were generated from simulation logs.
- MAVLink/EKF3 external-navigation experiments were investigated.

## What This Version Does Not Claim

- It does not claim a fully validated autonomous navigation stack.
- It does not claim that EKF3 external-navigation switching is stable in all
  tested conditions.
- It does not claim field-flight validation.

## Future Work Direction

Future branches can add non-SIFT visual localization methods, improved motion
models, better sensor fusion, and cleaner dataset packaging.
