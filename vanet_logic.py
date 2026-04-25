"""
vanet_logic.py — Core VANET Broadcast & Alert Logic
=====================================================
Implements the Vehicle-to-Vehicle (V2V) communication model:

  1. Ambulance broadcasts a "beacon" every simulation step
  2. Vehicles within ALERT_RANGE receive the beacon
  3. Only vehicles AHEAD of the ambulance get an alert
  4. Alerted vehicles are colored YELLOW in the SUMO GUI
  5. Time-to-Collision (TTC) is computed as a bonus metric
  6. A basic lane-change suggestion is generated if TTC < TTC_DANGER

Usage (called from main.py):
    vanet = VANETSystem(traci)
    results = vanet.process_step(step, sim_time)
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import traci  # SUMO TraCI API

from utils import (
    euclidean_distance,
    is_vehicle_ahead,
    time_to_collision,
    log_alert,
    log_ambulance,
    Colors,
)


# ─────────────────────────────────────────────────
# Configuration Constants
# ─────────────────────────────────────────────────

AMBULANCE_ID_PREFIX  = "ambulance"   # any veh ID starting with this is ambulance
ALERT_RANGE          = 100.0         # metres — VANET broadcast range
FOV_DEGREES          = 70.0          # forward cone half-angle for "ahead" check
TTC_DANGER_THRESHOLD = 10.0          # seconds — below this, suggest lane change

# SUMO GUI colors  (R, G, B, Alpha) — values 0–255
COLOR_AMBULANCE   = (255,   0,   0, 255)   # Red
COLOR_ALERTED     = (255, 200,   0, 255)   # Yellow-orange
COLOR_NORMAL_CAR  = (  0, 100, 255, 255)   # Blue
COLOR_CLEAR       = (  0, 180,  80, 255)   # Green — no longer alerted


# ─────────────────────────────────────────────────
# Data Classes
# ─────────────────────────────────────────────────

@dataclass
class VehicleState:
    """Snapshot of one vehicle at a given simulation step."""
    veh_id  : str
    pos     : Tuple[float, float]
    speed   : float          # m/s
    angle   : float          # degrees, SUMO convention (0=North, CW)
    lane    : str            # current lane ID


@dataclass
class AlertEvent:
    """One VANET alert — ambulance to a specific vehicle."""
    time         : float
    vehicle      : str
    distance     : float
    ttc          : Optional[float]
    lane_advice  : str


@dataclass
class StepResult:
    """Aggregated output of one simulation step."""
    step              : int
    sim_time          : float
    ambulance         : Optional[VehicleState]
    alerted_vehicles  : List[str]
    alert_events      : List[AlertEvent]
    all_vehicle_ids   : List[str]


# ─────────────────────────────────────────────────
# VANET System
# ─────────────────────────────────────────────────

class VANETSystem:
    """
    Manages VANET broadcast logic across all simulation steps.

    Maintains per-vehicle state across steps so it can:
      - Distinguish newly alerted vs. already-alerted vehicles
      - Reset colors when vehicles move out of range
      - Accumulate metrics across the entire simulation
    """

    def __init__(self) -> None:
        # Vehicles currently in alert state (veh_id → step first alerted)
        self._alerted: Dict[str, int] = {}

        # All-time alert log (for final metrics)
        self.alert_log      : List[AlertEvent] = []
        self.alerted_set    : set              = set()   # unique IDs ever alerted
        self._distance_acc  : float            = 0.0
        self._distance_count: int              = 0

    # ──────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────

    def process_step(self, step: int, sim_time: float) -> StepResult:
        """
        Execute one VANET broadcast cycle.

        Steps:
          1. Collect vehicle snapshots from TraCI
          2. Find the ambulance
          3. For every other vehicle: compute distance & direction
          4. Trigger/clear alerts
          5. Apply GUI colors
          6. Return StepResult with diagnostics
        """
        all_ids = list(traci.vehicle.getIDList())

        # ── Collect snapshots ──
        vehicles: Dict[str, VehicleState] = {}
        for vid in all_ids:
            try:
                vehicles[vid] = VehicleState(
                    veh_id = vid,
                    pos    = traci.vehicle.getPosition(vid),
                    speed  = traci.vehicle.getSpeed(vid),
                    angle  = traci.vehicle.getAngle(vid),
                    lane   = traci.vehicle.getLaneID(vid),
                )
            except traci.exceptions.TraCIException:
                continue   # vehicle may have just left the network

        # ── Find ambulance ──
        ambulance: Optional[VehicleState] = None
        for vid, state in vehicles.items():
            if vid.startswith(AMBULANCE_ID_PREFIX):
                ambulance = state
                # Force ambulance color to RED every step (in case SUMO resets it)
                traci.vehicle.setColor(vid, COLOR_AMBULANCE)
                break

        result = StepResult(
            step             = step,
            sim_time         = sim_time,
            ambulance        = ambulance,
            alerted_vehicles = [],
            alert_events     = [],
            all_vehicle_ids  = all_ids,
        )

        if ambulance is None:
            # Ambulance not yet spawned or has finished its route
            self._clear_all_alerts(vehicles)
            return result

        # ── Log ambulance status ──
        log_ambulance(
            ambulance.veh_id,
            ambulance.pos,
            ambulance.speed,
            ambulance.angle,
        )

        # ── Broadcast beacon to all other vehicles ──
        currently_alerted: set = set()

        for vid, state in vehicles.items():
            if vid.startswith(AMBULANCE_ID_PREFIX):
                continue  # skip self

            distance = euclidean_distance(ambulance.pos, state.pos)

            # ── VANET Alert Condition ──
            #   1. Within broadcast range
            #   2. Vehicle is ahead of ambulance (direction check)
            if distance <= ALERT_RANGE and is_vehicle_ahead(
                ambulance.pos,
                ambulance.angle,
                state.pos,
                fov_degrees=FOV_DEGREES,
            ):
                currently_alerted.add(vid)

                # TTC computation (bonus metric)
                ttc = time_to_collision(
                    distance        = distance,
                    ambulance_speed = ambulance.speed,
                    vehicle_speed   = state.speed,
                    ambulance_angle = ambulance.angle,
                    vehicle_angle   = state.angle,
                )

                # Lane-change suggestion
                lane_advice = self._suggest_lane_change(
                    state, ambulance, distance, ttc
                )

                # Log alert event
                event = AlertEvent(
                    time        = sim_time,
                    vehicle     = vid,
                    distance    = distance,
                    ttc         = ttc,
                    lane_advice = lane_advice,
                )
                result.alert_events.append(event)
                result.alerted_vehicles.append(vid)

                # Print to console (only on first alert or every 20 steps)
                if vid not in self._alerted or step % 20 == 0:
                    log_alert(vid, distance, ttc, lane_advice)

                # Accumulate metrics
                if vid not in self.alerted_set:
                    self.alerted_set.add(vid)
                    self._distance_acc   += distance
                    self._distance_count += 1
                    self.alert_log.append(event)

                # Apply YELLOW color in GUI
                traci.vehicle.setColor(vid, COLOR_ALERTED)

                # Track alert state
                if vid not in self._alerted:
                    self._alerted[vid] = step

            else:
                # Vehicle is out of range or behind ambulance
                if vid in self._alerted:
                    # Just exited alert zone — reset color to green briefly
                    traci.vehicle.setColor(vid, COLOR_CLEAR)
                    del self._alerted[vid]
                else:
                    # Normal vehicle — blue
                    try:
                        traci.vehicle.setColor(vid, COLOR_NORMAL_CAR)
                    except traci.exceptions.TraCIException:
                        pass

        return result

    # ──────────────────────────────────────────────
    # Metrics
    # ──────────────────────────────────────────────

    def total_alerted(self) -> int:
        """Total unique vehicles that were ever alerted."""
        return len(self.alerted_set)

    def average_alert_distance(self) -> float:
        """Mean distance at which vehicles were first alerted (metres)."""
        if self._distance_count == 0:
            return 0.0
        return self._distance_acc / self._distance_count

    # ──────────────────────────────────────────────
    # Private Helpers
    # ──────────────────────────────────────────────

    @staticmethod
    def _suggest_lane_change(vehicle    : VehicleState,
                             ambulance  : VehicleState,
                             distance   : float,
                             ttc        : Optional[float]) -> str:
        """
        Basic lane-change suggestion logic.

        Heuristic:
          - If TTC < TTC_DANGER and distance < 60 m → suggest change NOW
          - If vehicle is in lane 0 (right lane) → move to lane 1
          - If already in lane 1 → slow down / stop on shoulder
        """
        if ttc is None:
            return ""

        if ttc > TTC_DANGER_THRESHOLD or distance > 60:
            return ""

        # Extract lane index from lane ID (e.g. "j00_j10_0" → 0)
        try:
            lane_idx = int(vehicle.lane.split("_")[-1])
        except (ValueError, IndexError):
            lane_idx = 0

        if lane_idx == 0:
            # Apply the suggestion via TraCI — request lane change to lane 1
            try:
                traci.vehicle.changeLane(vehicle.veh_id, 1, duration=5.0)
            except traci.exceptions.TraCIException:
                pass
            return "MOVE RIGHT → lane 1"
        else:
            # Already in right-most lane — slow down
            try:
                traci.vehicle.slowDown(vehicle.veh_id, speed=2.0, duration=5.0)
            except traci.exceptions.TraCIException:
                pass
            return "SLOW DOWN / YIELD"

    def _clear_all_alerts(self, vehicles: Dict[str, VehicleState]) -> None:
        """Reset all vehicles to normal color (ambulance gone)."""
        for vid in list(self._alerted.keys()):
            if vid in vehicles:
                try:
                    traci.vehicle.setColor(vid, COLOR_NORMAL_CAR)
                except traci.exceptions.TraCIException:
                    pass
        self._alerted.clear()
