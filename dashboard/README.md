# SIFT Navigation Dashboard

This directory contains the React/Vite dashboard used by the configurable SIFT
navigation launcher. The dashboard provides a browser UI for selecting runtime
options, starting and stopping the localization process, sending selected
MAVLink helper commands, and monitoring camera, matching, telemetry, and
ground-truth status.

## Requirements

- Node.js 20.19.0 or newer
- npm

## Development

```bash
npm install
npm run dev
```

## Production Build

```bash
npm install
npm run build
```

The built files are served by the Python launcher from the repository root.
