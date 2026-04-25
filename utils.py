"""
utils.py — Utility helpers for the VANET Emergency Vehicle Alert System
==========================================================================
Contains:
  • Euclidean distance calculation
  • Direction / heading math
  • Time-to-collision (TTC) estimation
  • Console color codes
  • Logging helpers
"""

import math
import sys
import time
from typing import Tuple, Optional

# Force UTF-8 output on Windows so box-drawing chars and emoji print correctly
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


# ─────────────────────────────────────────────────
# ANSI Console Colors
# ─────────────────────────────────────────────────
class Colors:
    RESET   = "\033[0m"
    RED     = "\033[91m"
    YELLOW  = "\033[93m"
    GREEN   = "\033[92m"
    CYAN    = "\033[96m"
    BLUE    = "\033[94m"
    MAGENTA = "\033[95m"
    WHITE   = "\033[97m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"


# ─────────────────────────────────────────────────
# Distance & Geometry
# ─────────────────────────────────────────────────

def euclidean_distance(pos1: Tuple[float, float],
                       pos2: Tuple[float, float]) -> float:
    """
    Return Euclidean distance between two (x, y) coordinate tuples.
    SUMO coordinates are in metres, so result is in metres.
    """
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    return math.sqrt(dx * dx + dy * dy)


def compute_angle(pos_from: Tuple[float, float],
                  pos_to: Tuple[float, float]) -> float:
    """
    Return the angle (degrees, 0–360) from pos_from to pos_to.
    0° = East, 90° = North (standard math convention).
    """
    dx = pos_to[0] - pos_from[0]
    dy = pos_to[1] - pos_from[1]
    angle = math.degrees(math.atan2(dy, dx)) % 360
    return angle


def angle_difference(a1: float, a2: float) -> float:
    """
    Smallest signed angular difference between two headings (degrees).
    Returns value in [−180, +180].
    """
    diff = (a2 - a1 + 180) % 360 - 180
    return diff


def is_vehicle_ahead(ambulance_pos: Tuple[float, float],
                     ambulance_angle: float,
                     vehicle_pos: Tuple[float, float],
                     fov_degrees: float = 60.0) -> bool:
    """
    Determine if `vehicle_pos` lies within the ambulance's forward cone.

    Parameters
    ----------
    ambulance_pos   : (x, y) position of ambulance
    ambulance_angle : heading of ambulance in degrees (SUMO convention:
                      0=North, clockwise)
    vehicle_pos     : (x, y) position of the other vehicle
    fov_degrees     : half-angle of the forward field-of-view cone

    Returns True if the vehicle is ahead of the ambulance.
    """
    # Convert SUMO heading (0=North, CW) to math angle (0=East, CCW)
    math_heading = (90 - ambulance_angle) % 360

    # Angle from ambulance to the vehicle
    angle_to_vehicle = compute_angle(ambulance_pos, vehicle_pos)

    # Angular separation
    diff = abs(angle_difference(math_heading, angle_to_vehicle))
    return diff <= fov_degrees


# ─────────────────────────────────────────────────
# Time-to-Collision (TTC)
# ─────────────────────────────────────────────────

def time_to_collision(distance: float,
                      ambulance_speed: float,
                      vehicle_speed: float,
                      ambulance_angle: float,
                      vehicle_angle: float) -> Optional[float]:
    """
    Estimate time-to-collision (seconds) between ambulance and a vehicle.

    Uses a simplified 1-D closing-speed model along the ambulance's heading.
    Returns None if the vehicles are not closing (diverging).

    Parameters
    ----------
    distance        : metres between the two vehicles
    ambulance_speed : m/s
    vehicle_speed   : m/s
    ambulance_angle : heading of ambulance (SUMO degrees)
    vehicle_angle   : heading of other vehicle (SUMO degrees)
    """
    if distance <= 0:
        return 0.0

    # Project both velocity vectors onto ambulance heading
    amb_rad  = math.radians(ambulance_angle)
    veh_rad  = math.radians(vehicle_angle)

    # Component along ambulance's direction of travel
    amb_forward = ambulance_speed                          # always positive along own heading
    veh_forward = vehicle_speed * math.cos(veh_rad - amb_rad)

    closing_speed = amb_forward - veh_forward              # positive → approaching

    if closing_speed <= 0.5:          # not closing fast enough to matter
        return None

    return distance / closing_speed


# ─────────────────────────────────────────────────
# Pretty Console Logging
# ─────────────────────────────────────────────────

_start_time: float = time.time()


def log_header(title: str) -> None:
    """Print a bold section header."""
    bar = "═" * 60
    print(f"\n{Colors.CYAN}{Colors.BOLD}{bar}")
    print(f"  {title}")
    print(f"{bar}{Colors.RESET}\n")


def log_step(step: int, sim_time: float) -> None:
    """Print a per-step separator."""
    print(f"{Colors.DIM}{'─'*55}{Colors.RESET}")
    print(f"{Colors.BLUE}⏱  Step {step:04d}  |  Sim time: {sim_time:6.1f}s{Colors.RESET}")


def log_ambulance(veh_id: str,
                  pos: Tuple[float, float],
                  speed: float,
                  angle: float) -> None:
    """Log ambulance status."""
    print(
        f"  {Colors.RED}🚑 {veh_id:20s}{Colors.RESET}"
        f"  pos=({pos[0]:6.1f}, {pos[1]:6.1f})"
        f"  speed={speed:5.1f} m/s"
        f"  heading={angle:5.1f}°"
    )


def log_alert(vehicle_id: str,
              distance: float,
              ttc: Optional[float] = None,
              lane_change: str = "") -> None:
    """Print a VANET alert message."""
    ttc_str = f"  TTC={ttc:5.1f}s" if ttc is not None else ""
    lc_str  = f"  {Colors.MAGENTA}[{lane_change}]{Colors.RESET}" if lane_change else ""
    print(
        f"  {Colors.YELLOW}⚠️  ALERT{Colors.RESET}: Vehicle "
        f"{Colors.WHITE}{Colors.BOLD}{vehicle_id}{Colors.RESET}"
        f"  →  Ambulance approaching!  "
        f"dist={distance:6.1f}m{ttc_str}{lc_str}"
    )


def log_metrics(total_alerted: int,
                avg_distance: float,
                sim_time: float,
                alert_log: list) -> None:
    """Print final simulation metrics."""
    log_header("SIMULATION METRICS REPORT")

    print(f"  {'Total vehicles alerted:':<35} {Colors.YELLOW}{total_alerted}{Colors.RESET}")
    print(f"  {'Average alert distance:':<35} {Colors.YELLOW}{avg_distance:.2f} m{Colors.RESET}")
    print(f"  {'Simulation duration:':<35} {Colors.YELLOW}{sim_time:.1f} s{Colors.RESET}")
    print(f"  {'Wall-clock time:':<35} {Colors.YELLOW}{time.time()-_start_time:.1f} s{Colors.RESET}")

    if alert_log:
        print(f"\n  {Colors.CYAN}Alert Log (last 10 events):{Colors.RESET}")
        for entry in alert_log[-10:]:
            ttc_str = "--" if entry.ttc is None else f"{entry.ttc:.1f}s"
            print(f"    t={entry.time:6.1f}s  {entry.vehicle:20s}  "
                  f"dist={entry.distance:6.1f}m  "
                  f"TTC={ttc_str}")

    print()
