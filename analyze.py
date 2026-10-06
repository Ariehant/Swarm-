"""Analyze one run's coordinator + three agent logs, never invent flight evidence."""
import argparse
import json
import statistics
from pathlib import Path


def read(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("coordinator")
    p.add_argument("agents", nargs=3, help="drone1 drone2 drone3 JSONL logs from the same run")
    a = p.parse_args()
    rows = read(a.coordinator)
    samples = [r for r in rows if r["event"] == "snapshot" and r["phase"] in
               {"TAKEOFF", "AUTO", "MANUAL", "HOLD", "APPROACH", "DESCEND"}]
    errors = [r["metrics"]["edge_error_m"] for r in samples if r["metrics"]]
    agents = [read(path) for path in a.agents]
    def times(event, predicate=lambda r: True):
        return [next((r["mono"] for r in records if r["event"] == event and predicate(r)), None)
                for records in agents]
    def skew(v):
        return max(v)-min(v) if all(x is not None for x in v) else None
    # Lift-off is first ground -> airborne transition. Ignore preflight UNKNOWN transitions.
    liftoff = []
    touchdown = []
    for records in agents:
        up, down, was_ground = None, None, False
        for r in records:
            if r["event"] != "ground_state":
                continue
            if was_ground and not r["ground"] and up is None:
                up = r["mono"]
            if up is not None and r["ground"] and down is None:
                down = r["mono"]
            was_ground = r["ground"]
        liftoff.append(up)
        touchdown.append(down)
    phases = [r for r in rows if r["event"] == "phase"]
    report = {
        "final_phase": phases[-1]["phase"] if phases else "NO_PHASES",
        "flight_samples": len(errors),
        "max_edge_error_m": max(errors) if errors else None,
        "rms_edge_error_m": (statistics.mean(x*x for x in errors)**0.5) if errors else None,
        "min_separation_m": min((r["metrics"]["min_separation_m"] for r in samples if r["metrics"]), default=None),
        "liftoff_skew_s": skew(liftoff), "touchdown_skew_s": skew(touchdown),
        "land_command_skew_s": skew(times("land_release")),
        "aborts": [r.get("reason") for r in phases if r["phase"] == "ABORT"],
        "note": "Measured MAVLink estimates; not independent Gazebo ground truth. Null means insufficient evidence."
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
