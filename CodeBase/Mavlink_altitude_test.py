"""
altitude_test.py
─────────────────
Standalone altitude test mission for PX4 via pymavlink.

The drone:
  1. Arms in place (lat/lon unchanged throughout)
  2. Takes off vertically to a user-specified target altitude (AGL)
  3. Holds position and streams altitude error to the console
  4. Lands vertically back to the same spot

Abort / failsafe at ANY time:
  • Press  Ctrl+C  → immediate RTL (Return-to-Launch)
  • The monitor loop also sends RTL if the drone disarms unexpectedly
    or if a MAVLink heartbeat is lost for > HEARTBEAT_TIMEOUT_S seconds.

Usage:
    python altitude_test.py                          # serial /dev/ttyUSB0
    python altitude_test.py udp:127.0.0.1:14550      # SITL
    python altitude_test.py /dev/cu.usbserial-XXXX 57600
"""

import sys
import time
import math
import logging
import threading
from pymavlink import mavutil

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(name)-14s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("AltitudeTest")

# ── Configuration — edit to match your setup ──────────────────────────────────
DEFAULT_CONNECTION  = "/dev/cu.usbserial-D30K0P9O"
DEFAULT_BAUD        = 57600

HOLD_DURATION_S     = 10.0      # seconds to hover at target altitude before landing
ALT_REACHED_THRESH  = 0.5       # metres — "close enough to target" threshold
POLL_INTERVAL_S     = 0.5       # position polling rate
HEARTBEAT_TIMEOUT_S = 5.0       # seconds without heartbeat → RTL

# ── Helpers ───────────────────────────────────────────────────────────────────

def connect(connection_string: str, baud: int) -> mavutil.mavfile:
    log.info(f"Connecting to PX4 at {connection_string} …")
    master = mavutil.mavlink_connection(connection_string, baud=baud)
    master.wait_heartbeat(timeout=15)
    log.info(
        f"✓ Heartbeat from system {master.target_system} "
        f"component {master.target_component}"
    )
    return master


def send_rtl(master: mavutil.mavfile, reason: str = ""):
    """Switch to RTL mode — safest abort for a PX4 drone in the air."""
    tag = f" ({reason})" if reason else ""
    log.warning(f"⚠ ABORT{tag} — sending RTL")
    mapping = master.mode_mapping()
    rtl_val = mapping.get("RTL") or mapping.get("AUTO.RTL")
    if rtl_val is None:
        log.error("RTL mode not found in mode_mapping — sending disarm instead")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        return

    if isinstance(rtl_val, tuple):
        base_mode, custom_mode, custom_sub_mode = rtl_val
    else:
        base_mode       = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode     = rtl_val
        custom_sub_mode = 0

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        float(base_mode), float(custom_mode), float(custom_sub_mode),
        0.0, 0.0, 0.0, 0.0,
    )


def arm(master: mavutil.mavfile) -> bool:
    log.info("Arming …")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0,   # param1=1 → arm
    )
    msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        log.info("Armed ✓")
        return True
    log.error(f"Arm failed: {msg.result if msg else 'no ACK'}")
    return False


def set_mode(master: mavutil.mavfile, mode_name: str) -> bool:
    mapping = master.mode_mapping()
    if mode_name not in mapping:
        log.error(f"Mode '{mode_name}' not found. Available: {list(mapping.keys())}")
        return False

    val = mapping[mode_name]
    if isinstance(val, tuple):
        base_mode, custom_mode, custom_sub_mode = val
    else:
        base_mode       = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode     = val
        custom_sub_mode = 0

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        float(base_mode), float(custom_mode), float(custom_sub_mode),
        0.0, 0.0, 0.0, 0.0,
    )
    log.info(f"Set mode → '{mode_name}'")

    deadline = time.time() + 5
    while time.time() < deadline:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.get_srcSystem() == master.target_system:
            if master.flightmode == mode_name:
                log.info(f"Mode confirmed: {mode_name} ✓")
                return True
    log.error(f"Mode '{mode_name}' not confirmed (current: {master.flightmode})")
    return False


def send_takeoff(master: mavutil.mavfile, target_alt_m: float) -> bool:
    """Send MAV_CMD_NAV_TAKEOFF and wait for ACK."""
    log.info(f"Sending takeoff command to {target_alt_m:.1f}m AGL …")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0,            # param1-3 unused
        float("nan"),       # param4: yaw — NaN = keep current
        0, 0,               # param5-6: lat/lon = 0 → use current position
        float(target_alt_m),
    )
    msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        log.info("Takeoff command accepted ✓")
        return True
    log.error(f"Takeoff command rejected: {msg.result if msg else 'no ACK'}")
    return False


def send_land(master: mavutil.mavfile):
    """Send MAV_CMD_NAV_LAND at current lat/lon (param5=param6=0 → stay in place)."""
    log.info("Sending LAND command …")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0,        # abort alt, precision land mode, empty
        float("nan"),   # yaw
        0, 0,           # lat=0, lon=0 → land at current position
        0,              # alt (ignored)
    )


# ── Heartbeat watchdog ────────────────────────────────────────────────────────

class HeartbeatWatchdog(threading.Thread):
    """
    Background thread — triggers RTL if no heartbeat is received within
    HEARTBEAT_TIMEOUT_S seconds.  Call .cancel() when done.
    """
    def __init__(self, master: mavutil.mavfile, abort_event: threading.Event):
        super().__init__(daemon=True)
        self.master      = master
        self.abort_event = abort_event
        self._last_hb    = time.time()
        self._cancelled  = False

    def touch(self):
        """Call this each time a heartbeat is received."""
        self._last_hb = time.time()

    def cancel(self):
        self._cancelled = True

    def run(self):
        while not self._cancelled:
            time.sleep(1.0)
            if time.time() - self._last_hb > HEARTBEAT_TIMEOUT_S:
                log.error(
                    f"No heartbeat for >{HEARTBEAT_TIMEOUT_S}s — triggering RTL"
                )
                send_rtl(self.master, "heartbeat lost")
                self.abort_event.set()
                break


# ── Phase: climb to target altitude ──────────────────────────────────────────

def wait_for_altitude(
    master: mavutil.mavfile,
    target_alt_m: float,
    abort_event: threading.Event,
    watchdog: HeartbeatWatchdog,
) -> bool:
    """
    Poll GLOBAL_POSITION_INT until relative_alt >= target_alt - ALT_REACHED_THRESH.
    Returns True when reached, False if aborted.
    """
    log.info(f"Climbing to {target_alt_m:.1f}m … (Ctrl+C to abort)")
    while not abort_event.is_set():
        msg = master.recv_match(
            type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"],
            blocking=True, timeout=2,
        )
        if msg is None:
            continue

        if msg.get_type() == "HEARTBEAT":
            watchdog.touch()
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                log.error("Drone disarmed during climb — aborting")
                abort_event.set()
                return False

        if msg.get_type() == "STATUSTEXT":
            log.info(f"PX4: {msg.text.strip()}")

        if msg.get_type() == "GLOBAL_POSITION_INT":
            current_alt = msg.relative_alt / 1000.0   # mm → m
            err = target_alt_m - current_alt
            log.info(f"  Climbing … current={current_alt:.2f}m  target={target_alt_m:.1f}m  "
                     f"error={err:+.2f}m")
            if current_alt >= target_alt_m - ALT_REACHED_THRESH:
                log.info(f"✓ Target altitude reached: {current_alt:.2f}m")
                return True

    return False


# ── Phase: hold and display error ────────────────────────────────────────────

def hold_and_report(
    master: mavutil.mavfile,
    target_alt_m: float,
    hold_seconds: float,
    abort_event: threading.Event,
    watchdog: HeartbeatWatchdog,
):
    """
    Hover at target altitude for hold_seconds, printing altitude error
    every poll cycle.
    """
    log.info(f"Holding at {target_alt_m:.1f}m for {hold_seconds:.0f}s …")
    log.info("━" * 58)
    log.info(f"  {'TIME':>6}  {'CURRENT ALT':>12}  {'TARGET ALT':>11}  {'ERROR':>8}  {'STATUS'}")
    log.info("━" * 58)

    start = time.time()
    while not abort_event.is_set():
        elapsed = time.time() - start
        if elapsed >= hold_seconds:
            break

        msg = master.recv_match(
            type=["GLOBAL_POSITION_INT", "HEARTBEAT", "STATUSTEXT"],
            blocking=True, timeout=2,
        )
        if msg is None:
            continue

        if msg.get_type() == "HEARTBEAT":
            watchdog.touch()
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                log.error("Drone disarmed during hold — aborting")
                abort_event.set()
                return

        if msg.get_type() == "STATUSTEXT":
            log.info(f"PX4: {msg.text.strip()}")

        if msg.get_type() == "GLOBAL_POSITION_INT":
            current_alt = msg.relative_alt / 1000.0   # mm → m
            error_m     = current_alt - target_alt_m
            abs_err     = abs(error_m)
            remaining   = hold_seconds - elapsed

            # Colour-coded status text (works in most terminals)
            if abs_err <= 0.3:
                status = "✓ EXCELLENT"
            elif abs_err <= 0.7:
                status = "~ GOOD"
            elif abs_err <= 1.5:
                status = "△ MODERATE"
            else:
                status = "✗ POOR"

            log.info(
                f"  {elapsed:5.1f}s  {current_alt:10.3f}m  {target_alt_m:10.1f}m  "
                f"{error_m:+8.3f}m  {status}   ({remaining:.0f}s left)"
            )

    log.info("━" * 58)
    if not abort_event.is_set():
        log.info("Hold complete.")


# ── Phase: land in place ──────────────────────────────────────────────────────

def wait_for_landing(
    master: mavutil.mavfile,
    abort_event: threading.Event,
    watchdog: HeartbeatWatchdog,
):
    """Wait until the drone disarms (PX4 auto-disarms on touchdown)."""
    log.info("Landing … waiting for touchdown and auto-disarm")
    while not abort_event.is_set():
        msg = master.recv_match(
            type=["HEARTBEAT", "STATUSTEXT", "GLOBAL_POSITION_INT"],
            blocking=True, timeout=2,
        )
        if msg is None:
            continue

        if msg.get_type() == "HEARTBEAT":
            watchdog.touch()
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                log.info("✓ Drone disarmed — landing complete")
                return

        if msg.get_type() == "STATUSTEXT":
            log.info(f"PX4: {msg.text.strip()}")

        if msg.get_type() == "GLOBAL_POSITION_INT":
            alt = msg.relative_alt / 1000.0
            log.info(f"  Landing … alt={alt:.2f}m")


# ── Main ──────────────────────────────────────────────────────────────────────

def run_altitude_test(master: mavutil.mavfile, target_alt_m: float):
    abort_event = threading.Event()
    watchdog    = HeartbeatWatchdog(master, abort_event)
    watchdog.start()

    try:
        # ── 1. Arm ────────────────────────────────────────────────────────────
        if not arm(master):
            log.error("Arming failed — test aborted")
            return

        # ── 2. Switch to Takeoff / Guided mode ───────────────────────────────
        # PX4 accepts MAV_CMD_NAV_TAKEOFF from any mode that allows it;
        # switching to GUIDED (aka OFFBOARD in PX4) is the safest approach
        # for a simple up-and-down without a mission plan.
        # If your build doesn't have GUIDED, try "AUTO.TAKEOFF".
        mode_set = set_mode(master, "TAKEOFF")
        if not mode_set:
            log.warning(
                "GUIDED mode not available — trying STABILIZED; "
                "takeoff command may still work depending on PX4 version"
            )

        # ── 3. Takeoff ────────────────────────────────────────────────────────
        if not send_takeoff(master, target_alt_m):
            log.error("Takeoff command failed — aborting")
            send_rtl(master, "takeoff rejected")
            return

        # ── 4. Wait to reach target altitude ──────────────────────────────────
        reached = wait_for_altitude(master, target_alt_m, abort_event, watchdog)
        if not reached or abort_event.is_set():
            send_rtl(master, "altitude not reached")
            wait_for_landing(master, abort_event, watchdog)
            return

        # ── 5. Hold + report altitude error ───────────────────────────────────
        hold_and_report(master, target_alt_m, HOLD_DURATION_S, abort_event, watchdog)

        if abort_event.is_set():
            send_rtl(master, "abort during hold")
            wait_for_landing(master, abort_event, watchdog)
            return

        # ── 6. Land in place ──────────────────────────────────────────────────
        send_land(master)
        wait_for_landing(master, abort_event, watchdog)

    except KeyboardInterrupt:
        log.warning("Ctrl+C detected — initiating RTL")
        abort_event.set()
        send_rtl(master, "user abort")
        wait_for_landing(master, abort_event, watchdog)

    finally:
        watchdog.cancel()
        log.info("Altitude test finished.")


def main():
    conn_str = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CONNECTION
    baud     = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    master = connect(conn_str, baud)

    # ── Get target altitude from user ─────────────────────────────────────────
    print("\n" + "═" * 55)
    print("  PX4 Altitude Test — vertical climb, hold, and land")
    print("═" * 55)
    print(f"  Drone will climb straight up, hold for {HOLD_DURATION_S:.0f}s, then land.")
    print("  Latitude and longitude will NOT change.\n")
    print("  Press  Ctrl+C  at any time to trigger RTL (Return-to-Launch).\n")

    while True:
        raw = input("  Target altitude in metres (AGL, e.g. 10): ").strip()
        try:
            target_alt = float(raw)
            if target_alt <= 0:
                raise ValueError
            break
        except ValueError:
            print("  ✗ Enter a positive number.")

    confirm = input(f"\n  Fly to {target_alt:.1f}m and return? [y/N]: ").strip().lower()
    if confirm != "y":
        print("  Aborted.")
        return

    print()
    run_altitude_test(master, target_alt)


if __name__ == "__main__":
    main()
