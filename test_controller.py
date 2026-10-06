import copy
import math
import unittest
from pathlib import Path
from swarm.common import (IDS, FLIGHT, correction, distance, healthy, load_config,
                          metrics, nominal, offsets, step, to_gps, to_ne, validate_waypoint)
from swarm.coordinator import Mission

CONFIG = Path(__file__).resolve().parents[1]/"config/mission.json"


class MemoryLog:
    def __init__(self):
        self.events = []

    def write(self, event, **kw):
        self.events.append({"event": event, **kw})


def peers(c, now, center=(0, 0, 0.2), armed=False, offboard=False):
    return {i: {"p": nominal(c, center, i), "sent": now, "pos_age": 0,
                "gps_age": 0, "hb_age": 0, "land_age": 0, "home": True,
                "fix": 3, "eph": 0.5, "fault": "", "armed": armed,
                "offboard": offboard, "ground": not armed, "speed": 0} for i in IDS}


class Tests(unittest.TestCase):
    def setUp(self):
        self.c = load_config(CONFIG)
        self.log = MemoryLog()
        self.m = Mission(self.c, self.log)

    def test_triangle_and_gps_axes(self):
        r = offsets(self.c)
        for i, j in ((1, 2), (1, 3), (2, 3)):
            self.assertAlmostEqual(distance(r[i], r[j]), 12)
        for n, e in ((10, 30), (-20, 7), (0, 0)):
            lat, lon = to_gps(self.c, n, e)
            actual = to_ne(self.c, lat, lon)
            self.assertAlmostEqual(actual[0], n, places=6)
            self.assertAlmostEqual(actual[1], e, places=6)

    def test_invalid_override_rejected(self):
        for lat, lon, agl in ((float("nan"), 8, 8), (48, 8, 8), (47.397742, 8.545594, 100)):
            with self.assertRaises(ValueError):
                validate_waypoint(self.c, dict(lat=lat, lon=lon, agl=agl))

    def test_correction_sign_and_limit(self):
        t = peers(self.c, 100)
        self.assertEqual(correction(self.c, 1, t), [0, 0, 0])
        t[1]["p"][0] += 10
        d = correction(self.c, 1, t)
        self.assertLess(d[0], 0)
        self.assertLessEqual(math.sqrt(sum(x*x for x in d)), self.c["max_correction_m"]+1e-9)

    def test_no_start_with_missing_drone(self):
        t = peers(self.c, 100)
        del t[3]
        with self.assertRaises(ValueError):
            self.m.command({"action": "start"}, t, 100)
        self.assertEqual(self.m.phase, "WAIT")

    def test_no_start_on_wrong_marker(self):
        t = peers(self.c, 100)
        t[3]["p"][1] += 8
        with self.assertRaises(ValueError):
            self.m.command({"action": "start"}, t, 100)

    def test_fresh_packet_does_not_hide_stale_position(self):
        t = peers(self.c, 100)[1]
        t["pos_age"] = 0.6
        self.assertFalse(healthy(self.c, t, 100.2))

    def test_all_vehicle_mode_barrier(self):
        t = peers(self.c, 100)
        self.m.command({"action": "start"}, t, 100)
        self.m.tick(peers(self.c, 103), 103, 0.05)
        self.assertEqual(self.m.phase, "OFFBOARD")
        t = peers(self.c, 104, offboard=True)
        t[3]["offboard"] = False
        self.m.tick(t, 104, 0.05)
        self.assertEqual(self.m.phase, "OFFBOARD")
        t[3]["offboard"] = True
        self.m.tick(t, 104.1, 0.05)
        self.assertEqual(self.m.phase, "ARM")

    def test_partial_arm_timeout_aborts(self):
        self.m.transition("ARM", 100)
        t = peers(self.c, 113, offboard=True)
        t[1]["armed"] = True
        self.m.tick(t, 113, 0.05)
        self.assertEqual(self.m.phase, "ABORT")

    def test_telemetry_loss_aborts(self):
        self.m.transition("AUTO", 100)
        self.m.tick(peers(self.c, 100, armed=True, offboard=True), 101, 0.05)
        self.assertEqual(self.m.phase, "ABORT")

    def test_soft_hold_and_hard_abort(self):
        self.m.transition("AUTO", 100)
        self.m.center = [0, 0, 8]
        self.m.goal = [20, 0, 8]
        t = peers(self.c, 100, self.m.center, True, True)
        t[1]["p"][0] += 2.5
        self.m.tick(t, 100, 0.05)
        self.assertEqual(self.m.center, [0, 0, 8])
        self.assertTrue(self.m.paused_by_error)
        t[1]["p"][0] += 10
        self.m.tick(t, 100.05, 0.05)
        self.assertEqual(self.m.phase, "ABORT")

    def test_override_slew_and_resume(self):
        self.m.transition("AUTO", 100)
        self.m.center = [0, 0, 8]
        t = peers(self.c, 100, self.m.center, True, True)
        wp = {"action": "waypoint", **self.c["waypoints"][1]}
        self.m.command(wp, t, 100)
        self.m.tick(t, 100.05, 0.05)
        self.assertLessEqual(distance(self.m.center, [0, 0, 8]), 0.06+1e-9)
        self.m.command({"action": "hold"}, t, 100.1)
        self.assertEqual(self.m.phase, "HOLD")
        self.m.command({"action": "resume"}, t, 100.2)
        self.assertEqual(self.m.phase, "AUTO")
        self.assertEqual(self.m.index, 0)

    def test_complete_mission_with_ideal_followers(self):
        now = 100.0
        self.m.command({"action": "start"}, peers(self.c, now), now)
        for _ in range(6000):
            now += 0.05
            phase = self.m.phase
            airborne = phase in FLIGHT or phase == "LAND"
            armed = phase == "ARM" or airborne
            if phase == "LAND" and now > self.m.execute_at+3:
                armed = False
            t = peers(self.c, now, self.m.center, armed, phase not in {"WAIT", "PRESTREAM"})
            t0 = self.m.center[:]
            self.m.tick(t, now, 0.05)
            self.assertLessEqual(distance(t0, self.m.center), math.hypot(1.2, 0.6)*0.05+1e-9)
            if self.m.phase in {"DONE", "FAILED", "ABORT"}:
                break
        self.assertEqual(self.m.phase, "DONE")
        phases = [r["phase"] for r in self.log.events if r["event"] == "phase"]
        for p in ("TAKEOFF", "AUTO", "APPROACH", "DESCEND", "LAND"):
            self.assertIn(p, phases)

    def test_landing_needs_disarm_and_ground(self):
        self.m.transition("LAND", 100)
        t = peers(self.c, 102, armed=True)
        self.m.tick(t, 102, 0.05)
        self.assertEqual(self.m.phase, "LAND")
        t = peers(self.c, 103)
        t[3]["ground"] = False
        self.m.tick(t, 103, 0.05)
        self.assertEqual(self.m.phase, "LAND")
        t[3]["ground"] = True
        self.m.tick(t, 103, 0.05)
        self.assertEqual(self.m.phase, "DONE")


if __name__ == "__main__":
    unittest.main()
