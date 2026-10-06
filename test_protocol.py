"""Real pymavlink encode/decode over loopback; does not run PX4 or Gazebo."""
import os
import tempfile
import time
import unittest
from pathlib import Path
os.environ["MAVLINK20"] = "1"
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None
from swarm.common import load_config, nominal


@unittest.skipIf(mavutil is None, "pymavlink not installed")
class ProtocolTests(unittest.TestCase):
    def setUp(self):
        from swarm.agent import Agent
        from test_controller import peers
        self.peers = peers
        self.temp = tempfile.TemporaryDirectory()
        self.old_log_dir = os.environ.get("LOG_DIR")
        os.environ["LOG_DIR"] = self.temp.name
        self.c = load_config(Path(__file__).resolve().parents[1]/"config/mission.json")
        self.a = Agent(1, self.c)
        self.fc = mavutil.mavlink_connection("udpout:127.0.0.1:14540", source_system=1, source_component=1)
        self.fc.mav.heartbeat_send(2, 12, 1, 6 << 16, 3)
        time.sleep(0.02)
        self.a.receive_fc(time.monotonic())

    def tearDown(self):
        self.fc.close()
        self.a.fc.close()
        self.a.sock.close()
        self.a.log.f.close()
        self.temp.cleanup()
        if self.old_log_dir is None:
            os.environ.pop("LOG_DIR", None)
        else:
            os.environ["LOG_DIR"] = self.old_log_dir

    def recv(self, kind):
        return self.fc.recv_match(type=kind, blocking=True, timeout=1)

    def test_global_target_units_and_address(self):
        self.a.send_target([0, 0, 8])
        m = self.recv("SET_POSITION_TARGET_GLOBAL_INT")
        self.assertIsNotNone(m)
        self.assertEqual((m.target_system, m.target_component), (1, 1))
        self.assertEqual(m.coordinate_frame, 0)
        self.assertEqual(m.type_mask, 2552)
        self.assertEqual(m.lat_int, round(self.c["origin"]["lat"]*1e7))
        self.assertAlmostEqual(m.alt, 496, places=3)

    def test_stale_plan_latches_land_and_no_rearm(self):
        now = time.monotonic()
        self.a.engaged = True
        self.a.data["armed"] = True
        self.a.plan = {"phase": "AUTO", "sent": now-2, "execute_at": now-3}
        self.a.control(now)
        self.assertTrue(self.a.landing)
        m = self.recv("COMMAND_LONG")
        self.assertEqual(m.command, mavutil.mavlink.MAV_CMD_NAV_LAND)
        self.a.plan["sent"] = now+1
        self.a.plan["phase"] = "ARM"
        self.a.control(now+1.1)
        m = self.recv("COMMAND_LONG")
        self.assertEqual(m.command, mavutil.mavlink.MAV_CMD_NAV_LAND)

    def test_future_land_epoch_holds_before_release(self):
        now = time.monotonic()
        self.a.plan = {"phase": "LAND", "sent": now, "execute_at": now+0.5,
                       "center": [0, 0, 2], "peers": {}}
        self.a.data["armed"] = True
        self.a.control(now)
        self.assertFalse(self.a.landing)
        self.assertIsNotNone(self.recv("SET_POSITION_TARGET_GLOBAL_INT"))
        self.a.control(now+0.6)
        self.assertTrue(self.a.landing)
        self.assertEqual(self.recv("COMMAND_LONG").command, mavutil.mavlink.MAV_CMD_NAV_LAND)


if __name__ == "__main__":
    unittest.main()
