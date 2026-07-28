"""
precision_landing.py
─────────────────────
Module 2 — Precision landing using LANDING_TARGET MAVLink messages.
Consumes detection data from aruco_detector.py and streams it to PX4.
Only called by main.py once the drone is near the ArUco marker.
"""

import time
import math
import threading
import logging
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO, format="[PrecLand] %(message)s")
log = logging.getLogger(__name__)


class PrecisionLander:
    def __init__(
        self,
        master: mavutil.mavfile,
        send_rate_hz: float = 20.0,
        marker_lost_timeout_s: float = 3.0,
    ):
        """
        Args:
            master            : active pymavlink connection (shared with mission_planner)
            send_rate_hz      : how often to send LANDING_TARGET (10–50 Hz recommended)
            marker_lost_timeout_s : seconds before declaring marker lost
        """
        self.master           = master
        self.send_interval    = 1.0 / send_rate_hz
        self.lost_timeout     = marker_lost_timeout_s

        self._lock            = threading.Lock()
        self._latest          = None        # latest detection dict from aruco_detector
        self._last_seen       = None        # timestamp of last detection
        self._running         = False
        self._thread          = None

        log.info(f"PrecisionLander ready | rate={send_rate_hz}Hz | lost_timeout={marker_lost_timeout_s}s")

    # ── Public API ──────────────────────────────────────────────────────────

    def update_detection(self, detection: dict | None):
        """
        Feed the latest ArUco detection result.
        Call this from the main loop every frame.
        Pass None if no marker was detected this frame.
        """
        with self._lock:
            if detection is not None:
                self._latest    = detection
                self._last_seen = time.time()
            # If detection is None we keep _latest stale but track timeout via _last_seen

    def start(self):
        """Start the background sender thread and command PX4 to precision-land."""
        if self._running:
            return
        self._running = True
        self._thread  = threading.Thread(target=self._sender_loop, daemon=True)
        self._thread.start()
        log.info("Sender thread started")
        self._command_land()

    def stop(self):
        """Stop the sender thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        log.info("Sender thread stopped")

    def is_marker_visible(self) -> bool:
        """True if a marker was seen within the timeout window."""
        with self._lock:
            if self._last_seen is None:
                return False
            return (time.time() - self._last_seen) < self.lost_timeout

    # ── Internal ────────────────────────────────────────────────────────────

    def _sender_loop(self):
        """Background thread: send LANDING_TARGET at the configured rate."""
        while self._running:
            t0 = time.time()

            with self._lock:
                det     = self._latest
                seen_at = self._last_seen

            if det is not None and seen_at is not None:
                age = time.time() - seen_at
                if age < self.lost_timeout:
                    self._send_landing_target(
                        det["angle_x"],
                        det["angle_y"],
                        det["distance"],
                    )
                else:
                    log.warning(f"Marker lost for {age:.1f}s — not sending LANDING_TARGET")
            else:
                log.debug("Waiting for first detection …")

            elapsed = time.time() - t0
            time.sleep(max(0.0, self.send_interval - elapsed))

    def _send_landing_target(self, angle_x: float, angle_y: float, distance: float):
        """
        Send MAVLink LANDING_TARGET message.

        angle_x  : horizontal angular offset (radians) — positive = target is to the right
        angle_y  : vertical angular offset   (radians) — positive = target is below center
        distance : distance to target        (metres)
        """
        self.master.mav.landing_target_send(
            int(time.time() * 1e6),              # time_usec
            0,                                    # target_num
            mavutil.mavlink.MAV_FRAME_BODY_FRD,   # frame
            angle_x,                              # angle_x  (rad)
            angle_y,                              # angle_y  (rad)
            distance,                             # distance (m)
            0.0,                                  # size_x   (optional)
            0.0,                                  # size_y   (optional)
        )
        log.debug(f"LANDING_TARGET sent | ax={math.degrees(angle_x):.2f}° "
                  f"ay={math.degrees(angle_y):.2f}° dist={distance:.2f}m")

    def _command_land(self):
        """Send MAV_CMD_NAV_LAND so PX4 enters precision landing mode."""
        log.info("Sending MAV_CMD_NAV_LAND …")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,            # confirmation
            0, 0, 0,      # params 1-3 unused
            float('nan'), # param4: yaw (nan = keep current)
            0, 0, 0,      # lat, lon, alt (0 = current position)
        )
        ack = self._wait_ack(mavutil.mavlink.MAV_CMD_NAV_LAND, timeout=5)
        if ack:
            log.info("Land command acknowledged by PX4")
        else:
            log.warning("No ACK received for land command — check PX4 state")

    def _wait_ack(self, command_id: int, timeout: float = 5.0) -> bool:
        """Wait for COMMAND_ACK for a specific command."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if msg and msg.command == command_id:
                result = msg.result
                if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return True
                else:
                    log.error(f"Command {command_id} rejected: MAV_RESULT={result}")
                    return False
        return False


# ── Standalone test (requires active PX4 connection) ────────────────────────
if __name__ == "__main__":
    import sys
    from aruco_detector import ArucoDetector
    import cv2

    conn_str = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14550"
    log.info(f"Connecting to {conn_str} …")
    master = mavutil.mavlink_connection(conn_str, baud=921600)
    master.wait_heartbeat()
    log.info("Heartbeat received")

    det     = ArucoDetector(marker_size_m=0.2)
    lander  = PrecisionLander(master)
    lander.start()

    log.info("Streaming LANDING_TARGET. Press Q to stop.")
    try:
        while True:
            ok, frame = det.get_frame()
            if not ok:
                continue
            detections = det.detect(frame)
            lander.update_detection(detections[0] if detections else None)
            det.annotate_frame(frame, detections)
            cv2.imshow("Precision Landing", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        lander.stop()
        det.release()
