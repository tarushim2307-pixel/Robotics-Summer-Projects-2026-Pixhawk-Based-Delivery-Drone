"""
main.py
────────
Orchestrator — ties together all three modules in sequence:

  Phase 1 ── Mission planning
             User enters waypoints → mission uploaded → drone flies autonomously

  Phase 2 ── Proximity check
             Wait until drone is within NEAR_MARKER_RADIUS_M of the final waypoint

  Phase 3 ── ArUco detection + Precision Landing
             Stream LANDING_TARGET to PX4 until touchdown

Usage:
    python main.py                        # connects via serial /dev/ttyUSB0
    python main.py udp:127.0.0.1:14550   # SITL / UDP
    python main.py /dev/ttyACM0 921600   # serial with explicit baud
"""

import sys
import time
import logging
import cv2
from pymavlink import mavutil
from serial.serialutil import SerialException

from aruco_detector    import ArucoDetector
from precision_landing import PrecisionLander
from mission_planner   import MissionPlanner

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(name)-12s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("Main")

# ── Configuration — edit these to match your setup ───────────────────────────
CONNECTION_STRING = "/dev/cu.usbserial-D30K0P9O"   # or "udp:127.0.0.1:14550" for SITL
BAUD_RATE         = 57600

CAMERA_MATRIX_PATH = "camera_matrix.npy"
DIST_COEFFS_PATH   = "dist_coeffs.npy"
MARKER_SIZE_M      = 0.2             # physical ArUco marker side length (metres)
TARGET_MARKER_ID   = 0               # set to None to accept any marker ID
CAMERA_INDEX       = 0               # OpenCV camera index for FPV cam

TAKEOFF_ALT_M      = 10.0            # takeoff altitude AGL
CRUISE_ALT_M       = 15.0            # cruise altitude AGL
FINAL_ALT_M        = 5.0             # altitude of final waypoint above marker
NEAR_MARKER_RADIUS = 8.0             # metres — triggers Phase 3

LANDING_TARGET_RATE_HZ = 20.0        # LANDING_TARGET send rate


# ─────────────────────────────────────────────────────────────────────────────

def get_waypoints_from_user() -> list[tuple[float, float]]:
    """
    Interactive prompt — returns list of (lat, lon) tuples.
    The LAST waypoint is treated as the ArUco marker location.
    """
    print("\n" + "═" * 60)
    print("  Drone Mission Planner — PX4 + ArUco Precision Landing")
    print("═" * 60)
    print("Enter waypoints one by one.")
    print("The LAST waypoint must be directly above the ArUco marker.\n")

    waypoints = []
    while True:
        idx = len(waypoints) + 1
        raw = input(f"  Waypoint {idx} (lat,lon) — or ENTER to finish: ").strip()
        if raw == "":
            if len(waypoints) < 1:
                print("  ⚠ Enter at least one waypoint (the marker location).")
                continue
            break
        try:
            lat_s, lon_s = raw.split(",")
            lat, lon = float(lat_s.strip()), float(lon_s.strip())
            waypoints.append((lat, lon))
            print(f"  ✓ Added: {lat:.6f}, {lon:.6f}")
        except ValueError:
            print("  ✗ Invalid format — use:  lat,lon  e.g.  28.704060, 77.102493")

    print(f"\n  Final waypoint (ArUco marker): {waypoints[-1][0]:.6f}, {waypoints[-1][1]:.6f}")
    print("═" * 60 + "\n")
    return waypoints


def connect(connection_string: str, baud: int) -> mavutil.mavfile:
    log.info(f"Connecting to PX4 at {connection_string} …")
    try:
        master = mavutil.mavlink_connection(connection_string, baud=baud)
        master.wait_heartbeat()
        log.info(f"✓ Heartbeat from system {master.target_system} "
                 f"component {master.target_component}")
        return master
    except SerialException as exc:
        log.error("Could not open serial port %s: %s", connection_string, exc)
        if connection_string.startswith("/dev/"):
            log.error("On macOS use /dev/cu.usbmodem* or /dev/tty.usbmodem* instead of Linux-style /dev/ttyACM0")
        log.error("Verify the flight controller is connected and the port is not in use.")
        sys.exit(1)
    except Exception as exc:
        log.error("Failed to connect to PX4: %s", exc)
        sys.exit(1)


def phase1_mission(master, waypoints) -> bool:
    """Upload and start the mission. Returns True on success."""
    planner = MissionPlanner(master)

    log.info("Phase 1: Uploading mission …")
    ok = planner.plan_and_upload(
        waypoints,
        takeoff_alt=TAKEOFF_ALT_M,
        cruise_alt=CRUISE_ALT_M,
        final_alt=FINAL_ALT_M,
    )
    if not ok:
        log.error("Mission upload failed — aborting")
        return False

    confirm = input("Mission uploaded. Start flight? [y/N]: ").strip().lower()
    if confirm != 'y':
        log.info("Aborted by user")
        return False

    if not planner.start_mission():
        log.error("Failed to start mission")
        return False

    log.info("Phase 1: Mission running …")
    final_lat, final_lon = waypoints[-1]
    reached = planner.monitor_until_final_waypoint(
        final_lat, final_lon,
        near_radius=NEAR_MARKER_RADIUS,
    )
    return reached


def phase2_aruco_and_land(master):
    """Run ArUco detection and stream LANDING_TARGET until touchdown."""
    log.info("Phase 2 & 3: Starting ArUco detection + Precision Landing …")

    detector = ArucoDetector(
        camera_matrix_path=CAMERA_MATRIX_PATH,
        dist_coeffs_path=DIST_COEFFS_PATH,
        marker_size_m=MARKER_SIZE_M,
        camera_index=CAMERA_INDEX,
        target_marker_id=TARGET_MARKER_ID,
    )
    lander = PrecisionLander(master, send_rate_hz=LANDING_TARGET_RATE_HZ)
    lander.start()   # sends MAV_CMD_NAV_LAND and begins streaming

    log.info("Streaming LANDING_TARGET … Press Q in the video window to abort.")

    try:
        while True:
            ok, frame = detector.get_frame()
            if not ok:
                log.warning("Camera frame dropped")
                time.sleep(0.01)
                continue

            detections = detector.detect(frame)
            primary    = detections[0] if detections else None

            lander.update_detection(primary)

            if primary:
                d = primary
                log.info(
                    f"Marker {d['id']} | dist={d['distance']:.2f}m | "
                    f"pos=({d['pos']['x']:.3f}, {d['pos']['y']:.3f}, {d['pos']['z']:.3f}) | "
                    f"angle_x={d['angle_x']:.3f}rad  angle_y={d['angle_y']:.3f}rad"
                )

            detector.annotate_frame(frame, detections)
            cv2.imshow("Precision Landing — ArUco", frame)

            # ── Check for landed state via HEARTBEAT ─────────────────────────
            hb = master.recv_match(type="HEARTBEAT", blocking=False)
            if hb:
                # PX4 custom_mode 6 = Land; check armed flag dropped
                armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if not armed:
                    log.info("Drone disarmed — landing complete ✓")
                    break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                log.info("Aborted by user — sending disarm")
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0,
                )
                break

    finally:
        lander.stop()
        detector.release()

    log.info("Precision landing sequence complete")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    # Parse optional CLI args
    conn_str = sys.argv[1] if len(sys.argv) > 1 else CONNECTION_STRING
    baud     = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD_RATE

    # ── Connect (shared connection for all modules) ───────────────────────────
    master = connect(conn_str, baud)

    # ── Get waypoints from user ───────────────────────────────────────────────
    waypoints = get_waypoints_from_user()

    # ── Phase 1: Mission flight ───────────────────────────────────────────────
    near_marker = phase1_mission(master, waypoints)

    if not near_marker:
        log.error("Did not reach marker proximity — not starting precision landing")
        sys.exit(1)

    # ── Phase 2 + 3: ArUco detection → Precision landing ─────────────────────
    phase2_aruco_and_land(master)


if __name__ == "__main__":
    main()
