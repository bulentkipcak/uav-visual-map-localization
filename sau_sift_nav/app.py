from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
from pathlib import Path
from typing import Any, Optional, Sequence

import cv2

from .config import parse_scales
from .gazebo_truth import GazeboTruthThread
from .geometry import PREVIEW_MAP, MapGeometry, MasterMapGeometry
from .localizer import SiftLocalizer
from .logging import write_log_metadata
from .mavlink_io import MavlinkBridge, VisionMavlinkSender, VisionPublisherThread
from .patches import MasterPatchDb
from .state import SharedState
from .threads import MatcherThread, ValidationLoggerThread
from .video import VideoThread
from .web import make_server


MASTER_SIFT_DIR = Path("QGIS/SAU CAMPUS/output/SIFT")
MASTER_PATCH_DIR = MASTER_SIFT_DIR / "patches"


def apply_kf_config(args: argparse.Namespace) -> None:
    if not args.vision_kf_config:
        return
    path = Path(args.vision_kf_config)
    if not path.exists():
        print(f"Kalman config missing, using CLI/defaults: {path}")
        return
    with path.open(encoding="utf-8") as f:
        config = json.load(f)
    key_map = {
        "KF_MEAS_NOISE_POS_M": "kf_meas_noise_pos",
        "meas_noise_pos_m": "kf_meas_noise_pos",
        "KF_PROCESS_ACCEL_NOISE_MPS2": "kf_process_accel_noise_mps2",
        "process_accel_noise_mps2": "kf_process_accel_noise_mps2",
        "KF_INITIAL_POS_STD_M": "kf_initial_pos_std_m",
        "initial_pos_std_m": "kf_initial_pos_std_m",
        "KF_INITIAL_VEL_STD_MPS": "kf_initial_vel_std_mps",
        "initial_vel_std_mps": "kf_initial_vel_std_mps",
        "MAX_INNOVATION_M": "kf_max_innovation_m",
        "max_innovation_m": "kf_max_innovation_m",
        "MAX_JUMP_M": "kf_max_jump_m",
        "max_jump_m": "kf_max_jump_m",
        "MAX_PREDICT_AGE_S": "kf_max_predict_age_sec",
        "max_predict_age_s": "kf_max_predict_age_sec",
        "MAX_FIX_AGE_S": "kf_max_fix_age_sec",
        "max_fix_age_s": "kf_max_fix_age_sec",
        "MIN_INLIERS": "kf_min_inliers",
        "min_inliers": "kf_min_inliers",
        "KF_RESET_ON_HIGH_INLIER_JUMP": "kf_reset_on_high_inlier_jump",
        "reset_on_high_inlier_jump": "kf_reset_on_high_inlier_jump",
        "KF_RESET_MIN_INLIERS": "kf_reset_min_inliers",
        "reset_min_inliers": "kf_reset_min_inliers",
        "KF_RESET_RESIDUAL_M": "kf_reset_residual_m",
        "reset_residual_m": "kf_reset_residual_m",
    }
    applied = []
    for key, attr in key_map.items():
        if key not in config:
            continue
        value = config[key]
        if attr in {"kf_min_inliers", "kf_reset_min_inliers"}:
            value = int(value)
        elif attr == "kf_reset_on_high_inlier_jump":
            value = bool(value)
        else:
            value = float(value)
        setattr(args, attr, value)
        applied.append(f"{attr}={value}")
    if applied:
        print(f"Loaded Kalman config {path}: {', '.join(applied)}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Live SAU SIFT visual navigation dashboard")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--modern-dashboard-port",
        type=int,
        default=0,
        help="Optional FastAPI/WebSocket dashboard API port; 0 disables the modern dashboard",
    )
    parser.add_argument(
        "--modern-dashboard-host",
        default=None,
        help="Host for --modern-dashboard-port; defaults to --host",
    )
    parser.add_argument(
        "--modern-dashboard-static",
        default="dashboard/dist",
        help="Built React dashboard directory served by the FastAPI dashboard when present",
    )
    parser.add_argument("--video-source", default="udp", help="'udp', camera index, file path, or gst:<pipeline>")
    parser.add_argument("--udp-port", type=int, default=5600)
    parser.add_argument("--mavlink", default="udpin:127.0.0.1:14550")
    parser.add_argument(
        "--run-mode",
        choices=["live", "validation_log_only"],
        default="live",
        help="validation_log_only logs SIFT/Kalman/VPE against truth without sending any EKF source-set switch command",
    )
    parser.add_argument("--vision-mavlink", default=None, help="Optional separate MAVLink endpoint for vision TX")
    parser.add_argument("--no-mavlink", action="store_true")
    parser.add_argument("--configure-ekf", action="store_true", help="Send EKF3 ExternalNav XY parameter set")
    parser.add_argument(
        "--map-source",
        choices=["legacy", "sift-master"],
        default="legacy",
        help="legacy uses the old map_*/tiles_sift DB; sift-master uses QGIS/SAU CAMPUS/output/SIFT/patches",
    )
    parser.add_argument("--master-patch-dir", default=str(MASTER_PATCH_DIR))
    parser.add_argument("--send-vision", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--send-vision-speed", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--vision-speed-source", choices=["zero", "visual", "local"], default="zero")
    parser.add_argument("--vision-z-source", choices=["zero", "telemetry"], default="zero")
    parser.add_argument("--vision-attitude-source", choices=["telemetry", "zero"], default="telemetry")
    parser.add_argument(
        "--vision-align-source",
        choices=["none", "telemetry"],
        default="none",
        help="Freeze a VPE XY offset before EKF source switching; telemetry aligns the first visual fix to the current telemetry seed",
    )
    parser.add_argument(
        "--vision-align-max-age-sec",
        type=float,
        default=1.0,
        help="Maximum telemetry age accepted when locking --vision-align-source telemetry",
    )
    parser.add_argument(
        "--vision-publish-mode",
        choices=["rate", "fix"],
        default="rate",
        help="rate republishes the latest valid pose at --vision-rate-hz; fix sends once per newly accepted visual fix",
    )
    parser.add_argument(
        "--vision-stream-mode",
        choices=["event_based_raw", "fixed_rate_hold_last", "fixed_rate_kalman_predict"],
        default="fixed_rate_kalman_predict",
        help="VPE source model: old event fix, fixed-rate hold-last, or fixed-rate Kalman current-time prediction",
    )
    parser.add_argument(
        "--vision-timestamp-source",
        choices=["frame", "send"],
        default="frame",
        help="Timestamp VPE with the original frame time or the send time; send pairs best with rate mode and predicted poses",
    )
    parser.add_argument("--vision-rate-hz", type=float, default=10.0)
    parser.add_argument("--vision-max-age-sec", type=float, default=3.0)
    parser.add_argument("--kf-max-predict-age-sec", type=float, default=0.4)
    parser.add_argument("--kf-process-noise-pos", type=float, default=0.5)
    parser.add_argument("--kf-process-noise-vel", type=float, default=1.0)
    parser.add_argument("--kf-process-accel-noise-mps2", type=float, default=None)
    parser.add_argument("--kf-meas-noise-pos", type=float, default=4.0)
    parser.add_argument("--kf-initial-pos-std-m", type=float, default=2.0)
    parser.add_argument("--kf-initial-vel-std-mps", type=float, default=1.0)
    parser.add_argument("--kf-min-inliers", type=int, default=40)
    parser.add_argument("--kf-max-jump-m", type=float, default=3.0)
    parser.add_argument("--kf-max-innovation-m", type=float, default=3.0)
    parser.add_argument("--kf-max-fix-age-sec", type=float, default=0.5)
    parser.add_argument("--kf-reset-on-high-inlier-jump", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--kf-reset-min-inliers", type=int, default=90)
    parser.add_argument("--kf-reset-residual-m", type=float, default=8.0)
    parser.add_argument(
        "--vision-kf-config",
        default=None,
        help="Optional JSON config produced by tools/tune_visual_kalman.py; CLI values still fill missing keys",
    )
    parser.add_argument(
        "--vision-extra-predict-sec",
        type=float,
        default=0.0,
        help="Extra seconds to project the visual pose forward in rate mode; useful for measured SIFT latency compensation",
    )
    parser.add_argument("--publish-gps-seed", action="store_true", help="Publish telemetry seed before first visual fix")
    parser.add_argument("--scales", default="1/4x", help="Comma list: 1x,1/2x,1/4x")
    parser.add_argument("--interval-sec", type=float, default=0.25)
    parser.add_argument("--search-radius-m", type=float, default=120.0)
    parser.add_argument("--max-search-radius-m", type=float, default=700.0)
    parser.add_argument("--search-growth", type=float, default=1.8)
    parser.add_argument(
        "--search-nav-max-age-sec",
        type=float,
        default=3.0,
        help="Fall back to telemetry search seed when visual nav estimate is older than this; 0 disables fallback",
    )
    parser.add_argument("--max-tiles-per-scale", type=int, default=10)
    parser.add_argument("--early-stop-inliers", type=int, default=180)
    parser.add_argument("--min-nav-inliers", type=int, default=80)
    parser.add_argument("--max-nav-error-m", type=float, default=15.0)
    parser.add_argument("--max-nav-jump-m", type=float, default=35.0)
    parser.add_argument(
        "--nav-filter-alpha",
        type=float,
        default=1.0,
        help="EMA alpha for accepted visual fixes before publishing/search prediction; 1 keeps raw SIFT fixes",
    )
    parser.add_argument(
        "--max-nav-step-speed-mps",
        type=float,
        default=0.0,
        help="Reject visual fixes that imply faster motion than this from the previous visual estimate; 0 disables",
    )
    parser.add_argument(
        "--nav-filter-reset-residual-m",
        type=float,
        default=5.0,
        help="Reset the visual filter to raw SIFT when residual exceeds this, or when search falls back to telemetry/global; 0 disables residual reset",
    )
    parser.add_argument(
        "--max-nav-step-slack-m",
        type=float,
        default=0.0,
        help="Extra meters allowed by --max-nav-step-speed-mps to absorb frame timing jitter",
    )
    parser.add_argument(
        "--telemetry-position-gate",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Reject visual fixes that are too far from MAVLink local/GPS telemetry; disable after EKF source set 2 because telemetry is no longer independent",
    )
    parser.add_argument(
        "--telemetry-seed-source",
        choices=["auto", "local", "global"],
        default="auto",
        help="Telemetry source for search seed/error gate: auto prefers LOCAL_POSITION_NED, global uses GLOBAL_POSITION_INT",
    )
    parser.add_argument(
        "--telemetry-max-age-sec",
        type=float,
        default=1.0,
        help="Ignore LOCAL_POSITION_NED/GLOBAL_POSITION_INT telemetry older than this for seed, gate, velocity, and truth display; 0 disables age gating",
    )
    parser.add_argument("--no-gps-search-seed", action="store_true")
    parser.add_argument("--resize-w", type=int, default=None)
    parser.add_argument("--nfeatures", type=int, default=700)
    parser.add_argument("--ratio", type=float, default=0.75)
    parser.add_argument("--min-inliers", type=int, default=30)
    parser.add_argument("--ransac-thresh", type=float, default=4.0)
    parser.add_argument("--cache-tiles", type=int, default=256)
    parser.add_argument("--preview-map", default=PREVIEW_MAP)
    parser.add_argument("--log-csv", default="live_results.csv")
    parser.add_argument(
        "--vision-publish-log-csv",
        default=None,
        help="CSV for every VPE publish/skip; default derives from --log-csv when vision TX is enabled",
    )
    parser.add_argument(
        "--duration-sec",
        "--duration",
        dest="duration_sec",
        type=float,
        default=0.0,
        help="Stop the dashboard automatically after this many seconds; 0 runs until stopped",
    )
    parser.add_argument(
        "--validation-log-csv",
        default=None,
        help="10 Hz validation CSV for truth/raw SIFT/Kalman/VPE; default derives from --log-csv in validation mode",
    )
    parser.add_argument("--validation-log-rate-hz", type=float, default=10.0)
    parser.add_argument("--gazebo-truth", action="store_true", help="Log independent Gazebo pose truth into state/CSV")
    parser.add_argument("--gz-topic", default="/world/iris_runway/pose/info")
    parser.add_argument("--gz-model", default="iris_with_gimbal")
    parser.add_argument(
        "--gazebo-truth-bootstrap",
        choices=["telemetry", "map-origin"],
        default="telemetry",
        help="telemetry aligns Gazebo deltas to current GPS/global seed; map-origin uses map world axes directly",
    )
    parser.add_argument("--gazebo-truth-rate-hz", type=float, default=10.0)
    parser.add_argument("--ned-mode", default="enu", choices=["enu", "xy", "neg_enu", "neg_xy"])
    parser.add_argument("--home-lat", type=float, default=40.74318017142464)
    parser.add_argument("--home-lon", type=float, default=30.331305257172342)
    parser.add_argument("--home-alt", type=float, default=212.5)
    parser.add_argument("--once", action="store_true", help="Run one image through SIFT and print JSON")
    parser.add_argument("--image", default=None, help="Image path for --once")
    parser.add_argument("--hint-north", type=float, default=None)
    parser.add_argument("--hint-east", type=float, default=None)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    apply_kf_config(args)
    run_metadata = {
        "argv": list(argv) if argv is not None else sys.argv[1:],
        "parameters": vars(args),
    }
    if args.run_mode == "validation_log_only" and args.configure_ekf:
        print("validation_log_only: ignoring --configure-ekf; EKF parameters/source sets are left untouched")
        args.configure_ekf = False
    if args.map_source == "sift-master":
        geometry = MasterMapGeometry(
            ned_mode=args.ned_mode,
            home_lat=args.home_lat,
            home_lon=args.home_lon,
            home_alt=args.home_alt,
        )
        patch_db = MasterPatchDb(Path(args.master_patch_dir), geometry, cache_tiles=args.cache_tiles)
        scales = [patch_db.scale]
        localizer = SiftLocalizer(
            scales=scales,
            geometry=geometry,
            ratio=args.ratio,
            min_inliers=args.min_inliers,
            ransac_reproj_thresh=args.ransac_thresh,
            resize_w=args.resize_w,
            nfeatures=args.nfeatures,
            cache_tiles=args.cache_tiles,
            max_tiles_per_scale=args.max_tiles_per_scale,
            early_stop_inliers=args.early_stop_inliers,
            dbs=[patch_db],
        )
        if args.preview_map == PREVIEW_MAP:
            args.preview_map = str(Path(args.master_patch_dir).parent / "SIFT_master.png")
    else:
        geometry = MapGeometry(
            ned_mode=args.ned_mode,
            home_lat=args.home_lat,
            home_lon=args.home_lon,
            home_alt=args.home_alt,
        )
        scales = parse_scales(args.scales)
        localizer = SiftLocalizer(
            scales=scales,
            geometry=geometry,
            ratio=args.ratio,
            min_inliers=args.min_inliers,
            ransac_reproj_thresh=args.ransac_thresh,
            resize_w=args.resize_w,
            nfeatures=args.nfeatures,
            cache_tiles=args.cache_tiles,
            max_tiles_per_scale=args.max_tiles_per_scale,
            early_stop_inliers=args.early_stop_inliers,
        )

    if args.once:
        return run_once(args, geometry, localizer)

    if not Path(args.preview_map).exists():
        raise SystemExit(f"Preview map missing: {args.preview_map}")

    state = SharedState(
        geometry=geometry,
        base_search_radius_m=args.search_radius_m,
        max_search_radius_m=args.max_search_radius_m,
        search_growth=args.search_growth,
        min_nav_inliers=args.min_nav_inliers,
        max_nav_error_m=args.max_nav_error_m,
        max_nav_jump_m=args.max_nav_jump_m,
        telemetry_position_gate=args.telemetry_position_gate,
        telemetry_seed_source=args.telemetry_seed_source,
        telemetry_max_age_sec=args.telemetry_max_age_sec,
        search_nav_max_age_sec=args.search_nav_max_age_sec,
        nav_filter_alpha=args.nav_filter_alpha,
        nav_filter_reset_residual_m=args.nav_filter_reset_residual_m,
        max_nav_step_speed_mps=args.max_nav_step_speed_mps,
        max_nav_step_slack_m=args.max_nav_step_slack_m,
        vision_align_source=args.vision_align_source,
        vision_align_max_age_sec=args.vision_align_max_age_sec,
        vision_stream_mode=args.vision_stream_mode,
        kf_process_noise_pos=args.kf_process_noise_pos,
        kf_process_noise_vel=args.kf_process_noise_vel,
        kf_process_accel_noise_mps2=args.kf_process_accel_noise_mps2,
        kf_meas_noise_pos=args.kf_meas_noise_pos,
        kf_initial_pos_std_m=args.kf_initial_pos_std_m,
        kf_initial_vel_std_mps=args.kf_initial_vel_std_mps,
        kf_min_inliers=args.kf_min_inliers,
        kf_max_jump_m=args.kf_max_jump_m,
        kf_max_innovation_m=args.kf_max_innovation_m,
        kf_max_fix_age_sec=args.kf_max_fix_age_sec,
        kf_reset_on_high_inlier_jump=args.kf_reset_on_high_inlier_jump,
        kf_reset_min_inliers=args.kf_reset_min_inliers,
        kf_reset_residual_m=args.kf_reset_residual_m,
    )
    video_thread = VideoThread(state, source=args.video_source, udp_port=args.udp_port)
    matcher_thread = MatcherThread(
        state,
        localizer=localizer,
        interval_sec=args.interval_sec,
        log_csv=args.log_csv or None,
        allow_gps_search_seed=not args.no_gps_search_seed,
    )
    threads: list[Any] = [video_thread, matcher_thread]
    validation_log_csv = args.validation_log_csv
    if validation_log_csv is None and args.run_mode == "validation_log_only" and args.log_csv:
        log_path = Path(args.log_csv)
        validation_log_csv = str(log_path.with_name(f"{log_path.stem}_validation{log_path.suffix or '.csv'}"))
    if args.log_csv:
        write_log_metadata(args.log_csv, {**run_metadata, "log_kind": "match"})
    if validation_log_csv:
        write_log_metadata(validation_log_csv, {**run_metadata, "log_kind": "validation"})
        threads.append(ValidationLoggerThread(state, validation_log_csv, rate_hz=args.validation_log_rate_hz))
    if args.gazebo_truth:
        threads.append(
            GazeboTruthThread(
                state,
                topic=args.gz_topic,
                model=args.gz_model,
                bootstrap_mode=args.gazebo_truth_bootstrap,
                rate_hz=args.gazebo_truth_rate_hz,
            )
        )

    mavlink_bridge: Optional[MavlinkBridge] = None
    vision_publish_log_csv = args.vision_publish_log_csv
    if vision_publish_log_csv is None and args.send_vision and args.log_csv:
        log_path = Path(args.log_csv)
        vision_publish_log_csv = str(log_path.with_name(f"{log_path.stem}_vision_tx{log_path.suffix or '.csv'}"))
    if vision_publish_log_csv:
        write_log_metadata(vision_publish_log_csv, {**run_metadata, "log_kind": "vision_publish"})
    if not args.no_mavlink:
        mavlink_bridge = MavlinkBridge(state, args.mavlink, configure_ekf=args.configure_ekf)
        threads.append(mavlink_bridge)
        if args.send_vision:
            vision_endpoint = args.vision_mavlink or args.mavlink
            vision_sender: Any = mavlink_bridge
            if args.vision_mavlink and args.vision_mavlink != args.mavlink:
                vision_sender = VisionMavlinkSender(state, vision_endpoint)
                threads.append(vision_sender)
            threads.append(
                VisionPublisherThread(
                    state,
                    mavlink=vision_sender,
                    rate_hz=args.vision_rate_hz,
                    max_age_sec=args.kf_max_predict_age_sec
                    if args.vision_stream_mode == "fixed_rate_kalman_predict"
                    else args.vision_max_age_sec,
                    allow_gps_seed=args.publish_gps_seed,
                    send_speed=args.send_vision_speed,
                    velocity_source=args.vision_speed_source,
                    z_source=args.vision_z_source,
                    attitude_source=args.vision_attitude_source,
                    publish_mode=args.vision_publish_mode,
                    timestamp_source=args.vision_timestamp_source,
                    extra_predict_sec=args.vision_extra_predict_sec,
                    stream_mode=args.vision_stream_mode,
                    publish_log_csv=vision_publish_log_csv,
                )
            )
    else:
        state.update_telemetry({"status": "disabled"})

    modern_dashboard: Any = None
    if args.modern_dashboard_port > 0:
        from .modern_api import ModernDashboardThread

        modern_dashboard = ModernDashboardThread(
            host=args.modern_dashboard_host or args.host,
            port=args.modern_dashboard_port,
            state=state,
            preview_map=args.preview_map,
            static_dir=args.modern_dashboard_static,
        )
        threads.append(modern_dashboard)

    for thread in threads:
        thread.start()

    server = make_server(args.host, args.port, state, args.preview_map)

    shutdown_requested = threading.Event()

    def request_shutdown(reason: str) -> None:
        if shutdown_requested.is_set():
            return
        shutdown_requested.set()
        print(f"Shutdown requested: {reason}")
        threading.Thread(target=server.shutdown, daemon=True).start()

    def handle_signal(signum: int, frame: Any) -> None:
        try:
            reason = signal.Signals(signum).name
        except ValueError:
            reason = f"signal {signum}"
        request_shutdown(reason)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    print(f"Dashboard: http://{args.host}:{args.port}")
    if modern_dashboard is not None:
        print(f"Modern dashboard/API: http://{modern_dashboard.host}:{modern_dashboard.port}")
    print(f"Run mode: {args.run_mode}")
    print(f"Video source: {args.video_source} (udp port {args.udp_port})")
    print(f"MAVLink: {'disabled' if args.no_mavlink else args.mavlink}")
    vision_target = args.vision_mavlink or args.mavlink
    print(f"Vision TX: {'off' if args.no_mavlink or not args.send_vision else str(args.vision_rate_hz) + ' Hz via ' + vision_target}")
    print(f"Vision speed TX: {'off' if args.no_mavlink or not args.send_vision or not args.send_vision_speed else 'on'}")
    print(f"Vision speed source: {args.vision_speed_source}")
    print(f"Vision z source: {args.vision_z_source}")
    print(f"Vision attitude source: {args.vision_attitude_source}")
    print(f"Vision align source: {args.vision_align_source} max_age={args.vision_align_max_age_sec}s")
    print(f"Vision publish mode: {args.vision_publish_mode}")
    print(f"Vision stream mode: {args.vision_stream_mode}")
    print(f"Vision timestamp source: {args.vision_timestamp_source}")
    print(f"Vision extra predict: {args.vision_extra_predict_sec}s")
    print(f"Vision publish log: {vision_publish_log_csv or 'off'}")
    print(f"Validation log: {validation_log_csv or 'off'} @ {args.validation_log_rate_hz} Hz")
    if args.run_mode == "validation_log_only":
        print("Validation mode: EKF source-set switching is disabled; keep vehicle on source set 1/GPS.")
    print(f"Kalman config: {args.vision_kf_config or 'defaults/CLI'}")
    print(f"Kalman: accel_noise={args.kf_process_accel_noise_mps2 if args.kf_process_accel_noise_mps2 is not None else args.kf_process_noise_vel}m/s^2 r_pos={args.kf_meas_noise_pos} init_pos={args.kf_initial_pos_std_m}m init_vel={args.kf_initial_vel_std_mps}m/s min_inliers={args.kf_min_inliers} jump={args.kf_max_jump_m}m innovation={args.kf_max_innovation_m}m fix_age={args.kf_max_fix_age_sec}s predict_age={args.kf_max_predict_age_sec}s")
    print(f"Map source: {args.map_source}")
    if args.map_source == "sift-master":
        print(f"Master patch DB: {args.master_patch_dir}")
    print(f"Scales: {', '.join(scale.name for scale in scales)}")
    print(f"ROI: radius={args.search_radius_m}m max={args.max_search_radius_m}m max_tiles={args.max_tiles_per_scale}")
    print(f"Nav gates: min_inliers={args.min_nav_inliers} telemetry_error={'off' if not args.telemetry_position_gate else str(args.max_nav_error_m) + 'm'} telemetry_source={args.telemetry_seed_source} telemetry_max_age={args.telemetry_max_age_sec}s max_jump={args.max_nav_jump_m}m max_age={args.vision_max_age_sec}s")
    print(f"Nav filter: alpha={args.nav_filter_alpha} reset_residual={args.nav_filter_reset_residual_m}m step_speed={args.max_nav_step_speed_mps}m/s slack={args.max_nav_step_slack_m}m")
    print(f"Gazebo truth: {'on' if args.gazebo_truth else 'off'} topic={args.gz_topic} model={args.gz_model} bootstrap={args.gazebo_truth_bootstrap}")
    print(f"Search nav max age: {args.search_nav_max_age_sec}s")
    print(f"Duration: {'until stopped' if args.duration_sec <= 0 else str(args.duration_sec) + 's'}")
    print(f"Map scale: {geometry.meters_per_px:.5f} m/px, NED mode: {geometry.ned_mode}")

    duration_timer: Optional[threading.Timer] = None
    if args.duration_sec > 0:
        duration_timer = threading.Timer(
            args.duration_sec,
            request_shutdown,
            args=(f"duration {args.duration_sec:g}s reached",),
        )
        duration_timer.daemon = True
        duration_timer.start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        request_shutdown("KeyboardInterrupt")
    finally:
        if duration_timer is not None:
            duration_timer.cancel()
        for thread in threads:
            thread.stop()
        for thread in threads:
            thread.join(timeout=2.0)
        server.server_close()

    return 0


def run_once(args: argparse.Namespace, geometry: MapGeometry, localizer: SiftLocalizer) -> int:
    if not args.image:
        raise SystemExit("--once needs --image")
    frame = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if frame is None:
        raise SystemExit(f"Image could not be read: {args.image}")

    hint = None
    if args.hint_north is not None and args.hint_east is not None:
        px = geometry.ned_to_pixel(args.hint_north, args.hint_east)
        hint = {
            "source": "cli_hint",
            "center_ned": [args.hint_north, args.hint_east],
            "center_px_1x": [float(px[0]), float(px[1])],
            "radius_m": args.search_radius_m,
        }

    result = localizer.match_frame(frame, search_hint=hint)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0
