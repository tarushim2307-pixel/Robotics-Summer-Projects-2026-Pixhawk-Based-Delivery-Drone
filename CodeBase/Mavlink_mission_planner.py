"""
mission_planner.py
───────────────────
Module 3 — Mission planning via pymavlink.
Takes (latitude, longitude) from the user, builds a waypoint mission,
uploads it to PX4, starts it, and monitors progress.
Returns True when the drone has reached the final waypoint
(i.e. is near the ArUco marker location) so main.py can trigger landing.
"""

import time
import math
import logging
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

logging.basicConfig(level=logging.INFO, format="[Mission] %(message)s")
log = logging.getLogger(__name__)

# ── Defaults ─────────────────────────────────────────────────────────────────
DEFAULT_TAKEOFF_ALT_M  = 10.0    # metres AGL for takeoff waypoint
DEFAULT_CRUISE_ALT_M   = 15.0    # metres AGL for cruise waypoints
DEFAULT_FINAL_ALT_M    = 5.0     # metres AGL for final waypoint above marker
DEFAULT_ACCEPT_RADIUS  = 2.0     # metres — waypoint acceptance radius
NEAR_MARKER_RADIUS_M   = 8.0     # metres ground distance → "close enough to marker"


def haversine_distance(lat1, lon1, lat2, lon2) -> float:
    """Return distance in metres between two GPS coordinates."""
    R  = 6_371_000
    p  = math.pi / 180
    a  = (math.sin((lat2 - lat1) * p / 2) ** 2 +
          math.cos(lat1 * p) * math.cos(lat2 * p) *
          math.sin((lon2 - lon1) * p / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


class MissionPlanner:
    def __init__(self, master: mavutil.mavfile):
        self.master = master
        log.info("MissionPlanner ready")

    # ── Public API ────────────────────────────────────────────────────────────

    def plan_and_upload(
        self,
        waypoints_latlon: list[tuple[float, float]],
        takeoff_alt:  float = DEFAULT_TAKEOFF_ALT_M,
        cruise_alt:   float = DEFAULT_CRUISE_ALT_M,
        final_alt:    float = DEFAULT_FINAL_ALT_M,
        accept_radius: float = DEFAULT_ACCEPT_RADIUS,
    ) -> bool:
        """
        Build mission items from a list of (lat, lon) tuples and upload to PX4.

        Mission structure:
          Item 0 : MAV_CMD_NAV_TAKEOFF  (altitude = takeoff_alt)
          Item 1…N-1 : MAV_CMD_NAV_WAYPOINT at cruise_alt
          Item N : final waypoint above marker at final_alt

        Returns True if upload succeeded.
        """
        if not waypoints_latlon:
            log.error("No waypoints provided")
            return False

        items = self._build_mission(waypoints_latlon, takeoff_alt, cruise_alt,
                                    final_alt, accept_radius)
        log.info(f"Built {len(items)} mission items")
        self._print_mission(items)

        ok = self._upload_mission(items)
        if ok:
            log.info("Mission uploaded successfully")
        return ok

    def start_mission(self) -> bool:
        """Arm, switch to Mission mode, and start from item 0."""
        # Reset mission cursor to the beginning
        self.master.mav.mission_set_current_send(
            self.master.target_system,
            self.master.target_component,
            0,
        )
        time.sleep(0.3)

        if not self._arm():
            return False
        if not self._set_mode("MISSION"):
            return False
        log.info("Mission started ✓")
        return True
    def monitor_until_final_waypoint(
        self,
        final_lat: float,
        final_lon: float,
        poll_interval: float = 1.0,
        near_radius: float = NEAR_MARKER_RADIUS_M,
    ) -> bool:
        """
        Block until the drone reaches (within near_radius metres of) the
        final waypoint (ArUco marker position).

        Returns True when near the marker, False if mission aborted/failed.
        """
        log.info(f"Monitoring mission … waiting to arrive within {near_radius}m of marker")
        try:
            while True:
                msg = self.master.recv_match(
                    type=["GLOBAL_POSITION_INT", "MISSION_ITEM_REACHED",
                          "HEARTBEAT", "STATUSTEXT"],
                    blocking=True, timeout=poll_interval
                )

                if msg is None:
                    log.warning("No MAVLink message received — checking connection …")
                    continue

                if msg.get_type() == "STATUSTEXT":
                    log.info(f"PX4: {msg.text.strip()}")

                if msg.get_type() == "HEARTBEAT":
                    mode_flags = msg.base_mode
                    # Check if drone disarmed unexpectedly
                    armed = bool(mode_flags & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if not armed:
                        log.error("Drone disarmed unexpectedly — aborting monitor")
                        return False

                if msg.get_type() == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.relative_alt / 1000.0   # mm → m
                    dist = haversine_distance(lat, lon, final_lat, final_lon)
                    log.info(f"Position: lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m | "
                             f"dist_to_marker={dist:.1f}m")

                    if dist <= near_radius:
                        log.info(f"✓ Drone is within {dist:.1f}m of ArUco marker — "
                                 f"handing off to precision landing")
                        return True

                if msg.get_type() == "MISSION_ITEM_REACHED":
                    log.info(f"Reached mission item #{msg.seq}")

        except KeyboardInterrupt:
            log.info("Monitoring interrupted by user")
            return False

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _build_mission(self, waypoints_latlon, takeoff_alt, cruise_alt,
                       final_alt, accept_radius) -> list:
        items = []
        seq   = 0

        # Item 0: dummy home (required by PX4 as first item)
        items.append(self._make_item(
            seq=seq, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=0, autocontinue=1,
            param1=0, param2=0, param3=0, param4=0,
            x=0.0, y=0.0, z=0.0,
        ))
        seq += 1

        # Item 1: Takeoff
        items.append(self._make_item(
            seq=seq, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            current=1, autocontinue=1,
            param1=15,  # min pitch degrees
            param2=0, param3=0, param4=float('nan'),
            x=waypoints_latlon[0][0],
            y=waypoints_latlon[0][1],
            z=takeoff_alt,
        ))
        seq += 1

        # Intermediate cruise waypoints (all except final)
        for lat, lon in waypoints_latlon[:-1]:
            items.append(self._make_item(
                seq=seq, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                current=0, autocontinue=1,
                param1=0,               # hold time
                param2=accept_radius,   # acceptance radius (m)
                param3=0,               # pass-through
                param4=float('nan'),    # yaw
                x=lat, y=lon, z=cruise_alt,
            ))
            seq += 1

        # Final waypoint above the ArUco marker at lower altitude
        final_lat, final_lon = waypoints_latlon[-1]
        items.append(self._make_item(
            seq=seq, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current=0, autocontinue=0,   # autocontinue=0 → hold here
            param1=5,                    # hold 5 seconds
            param2=accept_radius,
            param3=0,
            param4=float('nan'),
            x=final_lat, y=final_lon, z=final_alt,
        ))

        return items

    @staticmethod
    def _make_item(seq, frame, command, current, autocontinue,
                   param1, param2, param3, param4, x, y, z):
        return {
            "seq": seq, "frame": frame, "command": command,
            "current": current, "autocontinue": autocontinue,
            "param1": param1, "param2": param2,
            "param3": param3, "param4": param4,
            "x": x, "y": y, "z": z,
        }

    def _upload_mission(self, items: list) -> bool:
        count = len(items)

        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            count,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )

        deadline = time.time() + 30
        while time.time() < deadline:                        # ← loop until ACK, not until uploaded==count
            msg = self.master.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                blocking=True, timeout=3,
            )
            if msg is None:
                log.warning("Timeout waiting for MISSION_REQUEST")
                continue

            mtype = msg.get_type()

            if mtype in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                idx = msg.seq
                if idx >= count:
                    log.error(f"PX4 requested out-of-range item {idx}")
                    return False
                item = items[idx]
                self.master.mav.mission_item_int_send(
                    self.master.target_system,
                    self.master.target_component,
                    item["seq"], item["frame"], item["command"],
                    item["current"], item["autocontinue"],
                    item["param1"], item["param2"], item["param3"], item["param4"],
                    int(item["x"] * 1e7),
                    int(item["y"] * 1e7),
                    item["z"],
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
                log.info(f"  Sent item {idx}/{count - 1}")

            elif mtype == "MISSION_ACK":                     # ← now always reachable
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    log.info("Mission ACK: ACCEPTED")
                    return True
                else:
                    log.error(f"Mission ACK error: {msg.type}")
                    return False

        log.error("Mission upload timed out")
        return False

    def _arm(self) -> bool:
        log.info("Arming …")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,              # 1 = arm
            0, 0, 0, 0, 0, 0,
        )
        msg = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            log.info("Armed ✓")
            return True
        log.error(f"Arm failed: {msg.result if msg else 'no ACK'}")
        return False

    def _set_mode(self, mode_name: str) -> bool:
        mapping = self.master.mode_mapping()
        if mode_name not in mapping:
            log.error(f"Unknown mode '{mode_name}'. Available: {list(mapping.keys())}")
            return False

        mode_val = mapping[mode_name]

        # PX4 mode_mapping returns (base_mode, custom_mode, custom_sub_mode) tuples
        if isinstance(mode_val, tuple):
            base_mode, custom_mode, custom_sub_mode = mode_val
        else:
            # ArduPilot returns plain integers
            base_mode      = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            custom_mode    = mode_val
            custom_sub_mode = 0

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            float(base_mode),
            float(custom_mode),
            float(custom_sub_mode),
            0.0, 0.0, 0.0, 0.0,
        )
        log.info(f"Set mode '{mode_name}' "
                f"(base={base_mode} custom={custom_mode} sub={custom_sub_mode})")

        # Confirm via HEARTBEAT
        deadline = time.time() + 5
        while time.time() < deadline:
            hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and hb.get_srcSystem() == self.master.target_system:
                if self.master.flightmode == mode_name:
                    log.info(f"Mode confirmed: {mode_name} ✓")
                    return True
        log.error(f"Mode '{mode_name}' not confirmed within 5s "
                f"(current: {self.master.flightmode})")
        return False

    @staticmethod
    def _print_mission(items):
        log.info("── Mission Plan ──────────────────────────────────")
        for it in items:
            log.info(f"  [{it['seq']:2d}] cmd={it['command']:4d}  "
                     f"lat={it['x']:.6f}  lon={it['y']:.6f}  alt={it['z']:.1f}m")
        log.info("──────────────────────────────────────────────────")


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys

    conn_str = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14550"
    log.info(f"Connecting to {conn_str} …")
    master = mavutil.mavlink_connection(conn_str, baud=921600)
    master.wait_heartbeat()
    log.info("Heartbeat received")

    planner = MissionPlanner(master)

    print("\n── Mission Planner Test ──")
    raw = input("Enter waypoints as  lat,lon  (semicolon-separated): ")
    waypoints = [tuple(map(float, p.strip().split(","))) for p in raw.split(";")]

    ok = planner.plan_and_upload(waypoints)
    if not ok:
        print("Upload failed")
        sys.exit(1)

    go = input("Start mission? [y/N] ")
    if go.lower() == 'y':
        planner.start_mission()
        final_lat, final_lon = waypoints[-1]
        reached = planner.monitor_until_final_waypoint(final_lat, final_lon)
        print(f"\nNear marker: {reached}")