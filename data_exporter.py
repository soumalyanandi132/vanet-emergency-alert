"""
data_exporter.py — TraCI → JSON bridge for the VANET live dashboard
====================================================================
Writes two files every simulation step so the dashboard can replay
real SUMO trajectories:

  vanet_live.json   — current snapshot (overwritten each step)
  vanet_replay.json — full history (appended each step, written at end)

Usage (called automatically by main.py):
    exporter = DataExporter(output_dir=".")
    exporter.record(step, sim_time, result, vanet)
    exporter.save_replay()   # call once at end of simulation
"""

import json
import time
from dataclasses import asdict
from pathlib import Path
from typing import Optional

import traci
import traci.exceptions


# ── SUMO 5×5 grid origin (matches network.nod.xml j00 position) ──────────────
# These values let us convert SUMO (x, y) metres → (lat, lng) degrees.
# Adjust ORIGIN_LAT / ORIGIN_LNG to your real OSM export origin if you
# later switch to a real-world network.
ORIGIN_LAT  = 28.6139    # latitude  of node j00
ORIGIN_LNG  = 77.2090    # longitude of node j00
METRES_PER_DEG_LAT = 111_320.0
METRES_PER_DEG_LNG = 111_320.0 * 0.857   # cos(28.6°) ≈ 0.857


def xy_to_latlng(x: float, y: float):
    """Convert SUMO network (x, y) metres to (lat, lng) degrees."""
    lat = ORIGIN_LAT + y / METRES_PER_DEG_LAT
    lng = ORIGIN_LNG + x / METRES_PER_DEG_LNG
    return round(lat, 7), round(lng, 7)


class DataExporter:
    """
    Records vehicle state each step and writes JSON for the dashboard.

    Parameters
    ----------
    output_dir : directory where vanet_live.json / vanet_replay.json are written
    alert_range: VANET broadcast range in metres (used by dashboard overlay)
    fov_degrees: forward cone half-angle (informational, written to metadata)
    """

    def __init__(self,
                 output_dir: str = ".",
                 alert_range: float = 100.0,
                 fov_degrees: float = 70.0):
        self._out = Path(output_dir)
        self._out.mkdir(parents=True, exist_ok=True)
        self._live_path   = self._out / "vanet_live.json"
        self._replay_path = self._out / "vanet_replay.json"
        self._replay_frames = []
        self._alert_range = alert_range
        self._fov_degrees = fov_degrees
        self._wall_start  = time.time()

    # ── Public API ────────────────────────────────────────────────────────────

    def record(self, step: int, sim_time: float, result, vanet) -> None:
        """
        Capture one simulation step and flush vanet_live.json.

        Parameters
        ----------
        step     : integer step counter
        sim_time : simulated seconds elapsed
        result   : StepResult from vanet_logic.VANETSystem.process_step()
        vanet    : VANETSystem instance (for aggregate metrics)
        """
        frame = self._build_frame(step, sim_time, result, vanet)
        self._replay_frames.append(frame)
        self._write_live(frame)

    def save_replay(self) -> None:
        """Write the full replay file (call once, at simulation end)."""
        payload = {
            "meta": {
                "origin_lat":   ORIGIN_LAT,
                "origin_lng":   ORIGIN_LNG,
                "alert_range_m": self._alert_range,
                "fov_degrees":  self._fov_degrees,
                "total_frames": len(self._replay_frames),
            },
            "frames": self._replay_frames,
        }
        self._replay_path.write_text(
            json.dumps(payload, separators=(",", ":")), encoding="utf-8"
        )
        print(f"  [Exporter] Replay saved → {self._replay_path}  "
              f"({len(self._replay_frames)} frames)")

    # ── Private helpers ───────────────────────────────────────────────────────

    def _build_frame(self, step, sim_time, result, vanet) -> dict:
        all_ids = result.all_vehicle_ids
        alerted = set(result.alerted_vehicles)

        vehicles_out = []
        for vid in all_ids:
            try:
                x, y  = traci.vehicle.getPosition(vid)
                speed  = traci.vehicle.getSpeed(vid)
                angle  = traci.vehicle.getAngle(vid)
                vtype  = traci.vehicle.getTypeID(vid)
                lane   = traci.vehicle.getLaneID(vid)
                lat, lng = xy_to_latlng(x, y)

                is_amb = vid.startswith("ambulance")
                state  = (
                    "ambulance" if is_amb
                    else "alerted" if vid in alerted
                    else "normal"
                )

                vehicles_out.append({
                    "id":    vid,
                    "lat":   lat,
                    "lng":   lng,
                    "speed": round(speed, 2),
                    "angle": round(angle, 1),
                    "type":  vtype,
                    "lane":  lane,
                    "state": state,
                })
            except traci.exceptions.TraCIException:
                continue

        # Alert events for this step
        alert_events_out = []
        for ev in result.alert_events:
            alert_events_out.append({
                "vehicle":  ev.vehicle,
                "distance": round(ev.distance, 1),
                "ttc":      round(ev.ttc, 2) if ev.ttc is not None else None,
                "advice":   ev.lane_advice,
            })

        # Ambulance position (convenience top-level field for the dashboard)
        amb_lat, amb_lng = None, None
        if result.ambulance:
            ax, ay = traci.vehicle.getPosition(result.ambulance.veh_id) \
                     if result.ambulance else (0, 0)
            try:
                ax, ay = traci.vehicle.getPosition(result.ambulance.veh_id)
                amb_lat, amb_lng = xy_to_latlng(ax, ay)
            except traci.exceptions.TraCIException:
                pass

        return {
            "step":        step,
            "sim_time":    sim_time,
            "wall_time":   round(time.time() - self._wall_start, 2),
            "vehicles":    vehicles_out,
            "alert_events": alert_events_out,
            "ambulance_lat": amb_lat,
            "ambulance_lng": amb_lng,
            "alert_range_m": self._alert_range,
            "metrics": {
                "total_alerted":   vanet.total_alerted(),
                "avg_alert_dist":  round(vanet.average_alert_distance(), 2),
                "alerted_this_step": len(alerted),
            },
        }

    def _write_live(self, frame: dict) -> None:
        """Atomically overwrite vanet_live.json with the latest frame."""
        tmp = self._live_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(frame, separators=(",", ":")), encoding="utf-8")
        tmp.replace(self._live_path)