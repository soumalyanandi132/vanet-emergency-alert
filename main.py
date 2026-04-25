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

BASE_DIR    = Path(__file__).parent
CONFIG_FILE = BASE_DIR / "simulation.sumocfg"
STEP_LENGTH = 0.5
PRINT_EVERY = 4   # print every 4 steps to reduce console spam


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--no-gui",  action="store_true", default=False)
    p.add_argument("--steps",   type=int, default=600)
    p.add_argument(
        "--delay",
        type=int,
        default=None,
        metavar="MS",
        help="Milliseconds to pause between each simulation step. "
             "Default: 100 ms in GUI mode, 0 in headless mode.",
    )
    return p.parse_args()


def run_simulation(args):
    log_header("VANET Emergency Vehicle Alert System")
    print(f"  Config : {CONFIG_FILE}")
    print(f"  GUI    : {'DISABLED' if args.no_gui else 'ENABLED'}\n")

    binary = "sumo" if args.no_gui else "sumo-gui"

    # Determine step delay: explicit flag > mode default
    if args.delay is not None:
        step_delay_ms = args.delay
    else:
        step_delay_ms = 0 if args.no_gui else 100   # 100 ms default in GUI

    sumo_cmd = [
        binary,
        "-c", str(CONFIG_FILE),
        "--step-length", str(STEP_LENGTH),
        "--collision.action", "warn",
        "--ignore-route-errors",
        "--no-step-log",
        "--start",                  # auto-start in GUI
        "--quit-on-end",
        "--random",                 # randomise flow departure positions
    ]

    # Ask the SUMO-GUI itself to slow down its rendering loop
    if not args.no_gui and step_delay_ms > 0:
        sumo_cmd += ["--delay", str(step_delay_ms)]

    print("  Connecting to SUMO...")
    traci.start(sumo_cmd)
    print(f"{Colors.GREEN}  ✓ SUMO started and TraCI connected!{Colors.RESET}")
    if step_delay_ms > 0:
        print(f"  ⏳ Step delay : {step_delay_ms} ms  "
              f"(estimated wall-clock: ~{args.steps * step_delay_ms / 1000:.0f}s)\n")
    else:
        print()

    vanet = VANETSystem()
    total_steps = 0
    ambulance_seen = False

    try:
        log_header("Simulation Running — watch the SUMO window!")

        for step in range(args.steps):
            traci.simulationStep()
            sim_time = round(step * STEP_LENGTH, 1)
            total_steps += 1

            # Get all vehicles currently in the simulation
            all_ids = list(traci.vehicle.getIDList())

            # Check if ambulance has spawned
            ambulance_in_sim = any(v.startswith("ambulance") for v in all_ids)

            if ambulance_in_sim and not ambulance_seen:
                ambulance_seen = True
                print(f"\n{Colors.RED}{Colors.BOLD}"
                      f"  🚑 AMBULANCE entered the network at t={sim_time}s!"
                      f"{Colors.RESET}\n")

            # Grant ambulance green lights at every upcoming junction
            _preempt_traffic_lights_for_ambulance(all_ids)

            # Slow down the Python control loop to match desired pace
            if step_delay_ms > 0:
                time.sleep(step_delay_ms / 1000.0)

            # Run VANET logic
            result = vanet.process_step(step, sim_time)

            # Print status every PRINT_EVERY steps
            if step % PRINT_EVERY == 0:
                log_step(step, sim_time)
                if not all_ids:
                    print(f"  {Colors.DIM}  Waiting for vehicles to spawn...{Colors.RESET}")
                elif not ambulance_in_sim:
                    print(f"  {Colors.DIM}  {len(all_ids)} vehicles in network. "
                          f"Ambulance spawns at t=5s...{Colors.RESET}")
                elif not result.alerted_vehicles:
                    print(f"  {Colors.GREEN}  Ambulance moving — "
                          f"no vehicles in alert zone yet{Colors.RESET}")

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
        try:
            traci.close()
        except Exception:
            pass
        print(f"{Colors.CYAN}  Done! 🏁{Colors.RESET}\n")


def _preempt_traffic_lights_for_ambulance(all_ids: list) -> None:
    """
    When an ambulance is within 80 m of a traffic-light junction,
    force that junction to show ALL-GREEN on the ambulance's road
    for one step — effectively preempting the signal.

    vClass=emergency already handles most cases in SUMO, but this
    gives an additional explicit override via TraCI.
    """
    ambulance_ids = [v for v in all_ids if v.startswith("ambulance")]
    if not ambulance_ids:
        return

    for amb_id in ambulance_ids:
        try:
            # Get upcoming traffic lights within 150 m
            tls_data = traci.vehicle.getNextTLS(amb_id)
            for (tls_id, _link_index, dist, _state) in tls_data:
                if dist < 80.0:  # within 80 m — force green
                    controlled_links = traci.trafficlight.getControlledLinks(tls_id)
                    num_phases = len(traci.trafficlight.getControlledLinks(tls_id))
                    # Build an all-green state string
                    green_state = "G" * max(num_phases, 1)
                    traci.trafficlight.setRedYellowGreenState(tls_id, green_state)
        except traci.exceptions.TraCIException:
            pass


if __name__ == "__main__":
    args = parse_args()
    run_simulation(args)