"""One logical Pi 5 companion per PX4 vehicle; no model emulation is implied."""
import argparse
import json
import math
import os
import socket
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil
from .common import (IDS, FLIGHT, STREAM, Log, correction, encode, healthy,
                     load_config, nominal, to_gps, to_ne)

M = mavutil.mavlink
# Use latitude, longitude, AMSL altitude and yaw; ignore v, a and yaw rate.
POSITION_YAW_MASK = 8 | 16 | 32 | 64 | 128 | 256 | 2048


class Agent:
    def __init__(self, i, c):
        self.i, self.c, self.log = i, c, Log(f"drone{i}")
        self.fc = mavutil.mavlink_connection(f"udpin:127.0.0.1:{14539+i}",
                                            source_system=240+i, source_component=191)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 16000+i))
        self.sock.setblocking(False)
        self.plan, self.seq = None, 0
        self.last_plan_seq = -1
        self.received = {k: -1e9 for k in ("pos", "gps", "hb", "land")}
        self.data = {"id": i, "p": [0, 0, 0], "speed": 0, "fix": 0, "eph": 999,
                     "home": False, "armed": False, "offboard": False, "ground": False}
        self.fault, self.engaged, self.landing = "", False, False
        self.last_cmd, self.last_heartbeat, self.last_request = {}, -1e9, -1e9
        self.last_log, self.last_target = 0, None
        self.first_land_command = None
        self.log.write("config", id=i, config=c)

    def command(self, command, params, now, interval=1.0):
        if now-self.last_cmd.get(command, -1e9) < interval:
            return
        self.last_cmd[command] = now
        self.fc.mav.command_long_send(self.i, 1, command, 0, *(params+[0]*7)[:7])
        self.log.write("command", command=command,
                       params=[x if math.isfinite(x) else None for x in params])

    def receive_fc(self, now):
        for _ in range(300):
            m = self.fc.recv_match(blocking=False)
            if m is None:
                break
            if m.get_srcSystem() != self.i or m.get_srcComponent() != 1:
                continue
            typ = m.get_type()
            if typ == "HEARTBEAT" and m.autopilot == M.MAV_AUTOPILOT_PX4:
                self.received["hb"] = now
                self.data.update(armed=bool(m.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED),
                                 offboard=((m.custom_mode >> 16) & 255) == 6,
                                 custom_mode=m.custom_mode)
            elif typ == "GLOBAL_POSITION_INT":
                self.received["pos"] = now
                self.data["p"] = to_ne(self.c, m.lat/1e7, m.lon/1e7)+[m.alt/1000-self.c["origin"]["amsl"]]
                self.data["speed"] = math.sqrt(m.vx*m.vx+m.vy*m.vy+m.vz*m.vz)/100
                self.data["gps"] = [m.lat/1e7, m.lon/1e7, m.alt/1000]
                self.data["boot_ms"] = m.time_boot_ms
            elif typ == "GPS_RAW_INT":
                self.received["gps"] = now
                self.data.update(fix=m.fix_type, eph=m.eph/100 if m.eph != 65535 else 999)
            elif typ == "HOME_POSITION":
                self.data["home"] = True
                self.data["home_gps"] = [m.latitude/1e7, m.longitude/1e7, m.altitude/1000]
            elif typ == "EXTENDED_SYS_STATE":
                self.received["land"] = now
                ground = m.landed_state == M.MAV_LANDED_STATE_ON_GROUND
                if self.data["ground"] != ground:
                    self.log.write("ground_state", ground=ground, landed_state=m.landed_state)
                self.data["ground"] = ground
            elif typ == "COMMAND_ACK":
                self.log.write("ack", command=m.command, result=m.result)
                # Heartbeat/state confirms completion. ACKs alone never release a barrier.
            elif typ == "STATUSTEXT":
                self.log.write("px4_status", severity=m.severity, text=m.text)

    def telemetry(self, now):
        return {**self.data, "fault": self.fault, "sent": now, "seq": self.seq,
                **{f"{k}_age": max(0, now-v) for k, v in self.received.items()}}

    def receive_plan(self, now):
        for _ in range(100):
            try:
                b, addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            if addr != ("127.0.0.1", 16000):
                continue
            try:
                p = json.loads(b)
                if p["seq"] <= self.last_plan_seq or not 0 <= now-p["sent"] < self.c["stale_s"]:
                    continue
                p["peers"] = {int(k): v for k, v in p["peers"].items()}
                self.plan, self.last_plan_seq = p, p["seq"]
            except (ValueError, KeyError, TypeError):
                continue

    def send_target(self, target):
        lat, lon = to_gps(self.c, *target[:2])
        self.fc.mav.set_position_target_global_int_send(
            self.data.get("boot_ms", 0), self.i, 1, M.MAV_FRAME_GLOBAL,
            POSITION_YAW_MASK, round(lat*1e7), round(lon*1e7),
            self.c["origin"]["amsl"]+target[2], 0, 0, 0, 0, 0, 0, 0, 0)
        self.last_target = target

    def control(self, now):
        p = self.plan
        if not p:
            return
        phase = p["phase"]
        if phase in STREAM:
            self.engaged = True
        if self.engaged and not self.landing:
            if now-p["sent"] >= self.c["stale_s"]:
                self.fault = "Coordinator lease expired"
            elif phase in STREAM and not all(healthy(self.c, p["peers"].get(i, {}), now) for i in IDS):
                self.fault = "Peer telemetry stale or unhealthy"
            elif phase in STREAM and not healthy(self.c, self.telemetry(now), now):
                self.fault = "Local telemetry unhealthy"
        if self.fault:
            self.landing = True
        if phase in {"LAND", "ABORT"} and now >= p["execute_at"]:
            self.landing = True
        if self.landing:
            # Never reenter OFFBOARD after a latched local fault or land request.
            if self.data["armed"]:
                if self.first_land_command is None:
                    self.first_land_command = now
                    self.log.write("land_release", phase=phase, scheduled=p["execute_at"])
                self.command(M.MAV_CMD_NAV_LAND, [0, 0, 0, float("nan"), float("nan"), float("nan"), float("nan")], now)
            return
        if phase in STREAM or (phase == "LAND" and now < p["execute_at"]):
            target = nominal(self.c, p["center"], self.i)
            if phase in FLIGHT and phase != "DESCEND":
                corr = correction(self.c, self.i, p["peers"])
                target = [x+y for x, y in zip(target, corr)]
            # Prearm target is the measured stationary home, avoiding ground creep.
            if phase in {"PRESTREAM", "OFFBOARD", "ARM"}:
                target = self.data["p"][:]
            self.send_target(target)
        if phase == "OFFBOARD" and not self.data["offboard"]:
            self.command(M.MAV_CMD_DO_SET_MODE, [M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6], now)
        elif phase == "ARM" and self.data["offboard"] and not self.data["armed"]:
            self.command(M.MAV_CMD_COMPONENT_ARM_DISARM, [1], now)

    def run(self):
        while True:
            now = time.monotonic()
            self.receive_fc(now)
            if now-self.last_heartbeat >= 1:
                self.fc.mav.heartbeat_send(M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID, 0, 0, M.MAV_STATE_ACTIVE)
                self.last_heartbeat = now
            if now-self.last_request >= 5 and now-self.received["hb"] < 3:
                for msg_id, hz in ((33, 20), (24, 5), (245, 5), (242, 1)):
                    self.fc.mav.command_long_send(self.i, 1, M.MAV_CMD_SET_MESSAGE_INTERVAL,
                                                  0, msg_id, 1e6/hz, 0, 0, 0, 0, 0)
                self.last_request = now
            self.receive_plan(now)
            self.control(now)
            t = self.telemetry(now)
            self.sock.sendto(encode(t), ("127.0.0.1", 16000))
            if now-self.last_log >= 0.2:
                self.log.write("telemetry", **t, target=self.last_target)
                self.last_log = now
            self.seq += 1
            time.sleep(max(0, 1/self.c["rate_hz"]-(time.monotonic()-now)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=int, choices=IDS, required=True)
    args = parser.parse_args()
    Agent(args.id, load_config()).run()


if __name__ == "__main__":
    main()
