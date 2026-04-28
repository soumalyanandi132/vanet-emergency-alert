import os
import sys
import time
import argparse
from pathlib import Path

# ── SUMO setup ────────────────────────────────────────────
sumo_home = os.environ.get("SUMO_HOME", "")
if sumo_home:
    sys.path.append(os.path.join(sumo_home, "tools"))

try:
    import traci
    import traci.exceptions
except ImportError:
    print("ERROR: traci not found. Run: pip install traci sumolib")
    sys.exit(1)

from vanet_logic import VANETSystem
from utils import log_header, log_step, log_metrics, Colors
from data_exporter import DataExporter          # ← NEW

BASE_DIR    = Path(__file__).parent
CONFIG_FILE = BASE_DIR / "simulation.sumocfg"
STEP_LENGTH = 0.5
PRINT_EVERY = 4


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--no-gui",   action="store_true", default=False)
    p.add_argument("--steps",    type=int, default=600)
    p.add_argument("--delay",    type=int, default=None, metavar="MS",
                   help="Milliseconds to pause between each simulation step.")
    p.add_argument("--export",   action="store_true", default=True,
                   help="Write vanet_live.json + vanet_replay.json (default: on).")
    p.add_argument("--no-export", dest="export", action="store_false",
                   help="Disable JSON export.")
    p.add_argument("--out-dir",  default=".", metavar="DIR",
                   help="Directory for exported JSON files (default: current dir).")
    return p.parse_args()


def run_simulation(args):
    log_header("VANET Emergency Vehicle Alert System")
    print(f"  Config : {CONFIG_FILE}")
    print(f"  GUI    : {'DISABLED' if args.no_gui else 'ENABLED'}")
    print(f"  Export : {'ON → ' + args.out_dir if args.export else 'OFF'}\n")

    binary = "sumo" if args.no_gui else "sumo-gui"

    if args.delay is not None:
        step_delay_ms = args.delay
    else:
        step_delay_ms = 0 if args.no_gui else 100

    sumo_cmd = [
        binary,
        "-c", str(CONFIG_FILE),
        "--step-length", str(STEP_LENGTH),
        "--collision.action", "warn",
        "--ignore-route-errors",
        "--no-step-log",
        "--start",
        "--quit-on-end",
        "--random",
    ]
    if not args.no_gui and step_delay_ms > 0:
        sumo_cmd += ["--delay", str(step_delay_ms)]

    print("  Connecting to SUMO...")
    traci.start(sumo_cmd)
    print(f"{Colors.GREEN}  ✓ SUMO started and TraCI connected!{Colors.RESET}")

    if step_delay_ms > 0:
        print(f"  ⏳ Step delay : {step_delay_ms} ms  "
              f"(~{args.steps * step_delay_ms / 1000:.0f}s wall-clock)\n")
    else:
        print()

    vanet    = VANETSystem()
    exporter = DataExporter(output_dir=args.out_dir) if args.export else None

    if exporter:
        print(f"{Colors.CYAN}  [Exporter] Writing live data to:"
              f" {Path(args.out_dir) / 'vanet_live.json'}{Colors.RESET}")
        print(f"{Colors.CYAN}  [Exporter] Open dashboard.html in a browser to watch live.{Colors.RESET}\n")

    total_steps    = 0
    ambulances_seen: set = set()
    AMB_ICONS = {
        "ambulance_0": "🚑", "ambulance_1": "🚨",
        "ambulance_2": "🏥", "ambulance_3": "⚡",
    }

    try:
        log_header("Simulation Running")

        for step in range(args.steps):
            traci.simulationStep()
            sim_time = round(step * STEP_LENGTH, 1)
            total_steps += 1

            all_ids = list(traci.vehicle.getIDList())

            for vid in all_ids:
                if vid.startswith("ambulance") and vid not in ambulances_seen:
                    ambulances_seen.add(vid)
                    icon = AMB_ICONS.get(vid, "🚑")
                    print(f"\n{Colors.RED}{Colors.BOLD}"
                          f"  {icon}  {vid.upper()} entered the network at t={sim_time}s!"
                          f"{Colors.RESET}\n")

            _preempt_traffic_lights_for_ambulance(all_ids)

            if step_delay_ms > 0:
                time.sleep(step_delay_ms / 1000.0)

            result = vanet.process_step(step, sim_time)

            # ── Export snapshot ──────────────────────────────────────────
            if exporter:
                exporter.record(step, sim_time, result, vanet)
            # ────────────────────────────────────────────────────────────

            if step % PRINT_EVERY == 0:
                log_step(step, sim_time)
                n_amb = len([v for v in all_ids if v.startswith("ambulance")])
                if not all_ids:
                    print(f"  {Colors.DIM}  Waiting for vehicles to spawn...{Colors.RESET}")
                elif n_amb == 0:
                    print(f"  {Colors.DIM}  {len(all_ids)} vehicles. "
                          f"First ambulance spawns at t=5s...{Colors.RESET}")
                elif not result.alerted_vehicles:
                    print(f"  {Colors.GREEN}  {n_amb} ambulance(s) moving — "
                          f"no vehicles in alert zones yet{Colors.RESET}")

        print(f"\n{Colors.CYAN}  Simulation finished all {args.steps} steps.{Colors.RESET}")

    except traci.exceptions.FatalTraCIError as e:
        print(f"\n{Colors.YELLOW}  SUMO closed: {e}{Colors.RESET}")

    finally:
        final_time = round(total_steps * STEP_LENGTH, 1)
        log_metrics(
            total_alerted=vanet.total_alerted(),
            avg_distance=vanet.average_alert_distance(),
            sim_time=final_time,
            alert_log=vanet.alert_log,
        )
        if exporter:
            exporter.save_replay()
        try:
            traci.close()
        except Exception:
            pass
        print(f"{Colors.CYAN}  Done! 🏁{Colors.RESET}\n")


def _preempt_traffic_lights_for_ambulance(all_ids: list) -> None:
    ambulance_ids = [v for v in all_ids if v.startswith("ambulance")]
    if not ambulance_ids:
        return
    for amb_id in ambulance_ids:
        try:
            tls_data = traci.vehicle.getNextTLS(amb_id)
            for (tls_id, _link_index, dist, _state) in tls_data:
                if dist < 80.0:
                    num_phases = len(traci.trafficlight.getControlledLinks(tls_id))
                    green_state = "G" * max(num_phases, 1)
                    traci.trafficlight.setRedYellowGreenState(tls_id, green_state)
        except traci.exceptions.TraCIException:
            pass


if __name__ == "__main__":
    args = parse_args()
    run_simulation(args)