"""Central mission authority. UDP snapshots fan out peer telemetry at 20 Hz."""
import json
import queue
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .common import (IDS, FLIGHT, Log, distance, encode, healthy, load_config,
                     metrics, nominal, step, validate_waypoint, waypoint)


class Mission:
    def __init__(self, c, log):
        self.c, self.log = c, log
        self.phase, self.reason = "WAIT", "Awaiting start command and all vehicles"
        self.entered = time.monotonic()
        self.execute_at = 0.0
        self.center = [0.0, 0.0, 0.2]
        self.goal = self.center[:]
        self.index = 0
        self.settled_since = None
        self.started = False
        self.stats = {}
        self.paused_by_error = False

    def transition(self, phase, now, reason="", delay=0):
        self.phase, self.reason, self.entered = phase, reason, now
        self.execute_at = now+delay
        self.settled_since = None
        self.log.write("phase", phase=phase, reason=reason, execute_at=self.execute_at)

    def abort(self, now, reason):
        if self.phase not in {"ABORT", "FAILED", "DONE"}:
            self.transition("ABORT", now, reason)

    def command(self, cmd, peers, now):
        action = cmd.get("action")
        if action == "abort":
            self.abort(now, "Operator abort: land at current positions")
        elif action == "start":
            if self.phase != "WAIT" or self.started:
                raise ValueError("Start is only accepted once in WAIT")
            if not all(healthy(self.c, peers.get(i, {}), now) for i in IDS):
                raise ValueError("All three vehicles must have healthy telemetry")
            for i in IDS:
                t = peers[i]
                if t["armed"] or not t["ground"]:
                    raise ValueError("All vehicles must be disarmed on the ground")
                if distance(t["p"], nominal(self.c, [0, 0, 0.2], i)) > 3:
                    raise ValueError(f"Drone {i}: GPS does not match its home marker")
            self.started = True
            self.center[2] = sum(peers[i]["p"][2] for i in IDS)/3
            self.transition("PRESTREAM", now, "Prestream ground targets")
        elif action in {"hold", "resume", "waypoint", "land"}:
            allowed = {"AUTO", "MANUAL", "HOLD"}
            if action == "land":
                allowed |= {"TAKEOFF", "APPROACH"}
            if self.phase not in allowed:
                raise ValueError(f"{action} is not permitted during {self.phase}; abort is always available")
            if action == "hold":
                self.transition("HOLD", now, "Operator hold at current reference")
            elif action == "resume":
                self.goal = waypoint(self.c, self.c["waypoints"][self.index])
                self.transition("AUTO", now, "Resume interrupted waypoint")
            elif action == "waypoint":
                self.goal = validate_waypoint(self.c, cmd)
                self.transition("MANUAL", now, "Manual centroid waypoint")
            else:
                self.goal = waypoint(self.c, self.c["landing"])
                self.transition("APPROACH", now, "Fly to designated landing triangle")
        else:
            raise ValueError("Unknown action")
        self.log.write("operator", command=cmd, phase=self.phase)

    def arrived(self, peers, now):
        good = (distance(self.center, self.goal) < 0.01 and
                all(distance(peers[i]["p"], nominal(self.c, self.goal, i)) < self.c["arrival_m"]
                    and peers[i].get("speed", 999) < 0.4 for i in IDS))
        self.settled_since = (self.settled_since or now) if good else None
        return self.settled_since is not None and now-self.settled_since >= self.c["settle_s"]

    def tick(self, peers, now, dt):
        c = self.c
        complete = all(healthy(c, peers.get(i, {}), now) for i in IDS)
        if all(i in peers and "p" in peers[i] for i in IDS):
            self.stats = metrics(c, self.center, peers)
        if self.phase in {"WAIT", "DONE", "FAILED"}:
            return
        if self.phase in {"LAND", "ABORT"}:
            # Never infer touchdown from missing telemetry or a low altitude alone.
            if all(now-peers.get(i, {}).get("sent", -1e9) < c["stale_s"]
                   and peers[i].get("hb_age", 999) < c["heartbeat_stale_s"]
                   and peers[i].get("land_age", 999) < 2
                   and not peers[i].get("armed", True) and peers[i].get("ground", False)
                   for i in IDS):
                self.transition("DONE" if self.phase == "LAND" else "FAILED", now, self.reason)
            # Keep landing requests active; do not falsely complete after timeout.
            return
        if not complete:
            self.abort(now, "Lost or unhealthy vehicle/peer telemetry")
            return
        if now-self.entered > (c["hold_timeout_s"] if self.phase == "HOLD" else
                              c["motion_timeout_s"] if self.phase in FLIGHT else 12):
            self.abort(now, "Phase deadline exceeded")
            return
        if self.phase in FLIGHT:
            if not all(peers[i]["armed"] and peers[i]["offboard"] for i in IDS):
                self.abort(now, "Vehicle left OFFBOARD or disarmed unexpectedly")
                return
            if self.stats["edge_error_m"] > c["edge_abort_m"] or self.stats["min_separation_m"] < c["min_separation_m"]:
                self.abort(now, "Triangle hard limit exceeded")
                return
            if any((p["p"][0]**2+p["p"][1]**2)**0.5 > c["radius_m"] or
                   p["p"][2] > c["max_agl_m"]+1 for p in peers.values()):
                self.abort(now, "Measured position outside flight volume")
                return
        if self.phase == "PRESTREAM" and now-self.entered >= 2.5:
            self.transition("OFFBOARD", now)
        elif self.phase == "OFFBOARD" and all(peers[i]["offboard"] for i in IDS):
            self.transition("ARM", now)
        elif self.phase == "ARM" and all(peers[i]["armed"] and peers[i]["offboard"] for i in IDS):
            self.goal = [0, 0, c["takeoff_agl_m"]]
            self.transition("TAKEOFF", now, "Common ascent release", delay=0.75)
        elif self.phase in FLIGHT:
            self.paused_by_error = (self.stats["edge_error_m"] > c["edge_hold_m"] or
                                    self.stats["tracking_error_m"] > c["tracking_hold_m"])
            if self.phase == "HOLD" or now < self.execute_at:
                return
            if not self.paused_by_error:
                self.center = step(c, self.center, self.goal, min(dt, 0.1))
            if not self.arrived(peers, now):
                return
            if self.phase == "TAKEOFF":
                self.goal = waypoint(c, c["waypoints"][self.index])
                self.transition("AUTO", now)
            elif self.phase == "AUTO":
                if self.index+1 < len(c["waypoints"]):
                    self.index += 1
                    self.goal = waypoint(c, c["waypoints"][self.index])
                    self.transition("AUTO", now, f"Waypoint {self.index+1}")
                else:
                    self.goal = waypoint(c, c["landing"])
                    self.transition("APPROACH", now)
            elif self.phase == "MANUAL":
                self.transition("HOLD", now, "Manual waypoint reached; resume or land")
            elif self.phase == "APPROACH":
                self.goal = self.goal[:2]+[c["landing_agl_m"]]
                self.transition("DESCEND", now, "Coordinated descent over landing markers")
            elif self.phase == "DESCEND":
                self.transition("LAND", now, "Common AUTO.LAND release", delay=0.75)

    def snapshot(self, peers, now, seq):
        return {"seq": seq, "sent": now, "phase": self.phase, "reason": self.reason,
                "execute_at": self.execute_at, "center": self.center[:], "goal": self.goal[:],
                "waypoint_index": self.index, "paused_by_error": self.paused_by_error,
                "metrics": self.stats, "peers": peers}


class API(BaseHTTPRequestHandler):
    server_version = "SwarmSITL/1.0"

    def log_message(self, *_):
        pass

    def reply(self, code, obj):
        b = encode(obj)
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        self.reply(200, self.server.snapshot) if self.path == "/status" else self.reply(404, {"error": "Not found"})

    def do_POST(self):
        if self.path != "/command":
            self.reply(404, {"error": "Not found"})
            return
        try:
            n = int(self.headers.get("Content-Length", 0))
            if not 0 < n <= 2048:
                raise ValueError("Invalid body length")
            self.connection.settimeout(2)
            cmd = json.loads(self.rfile.read(n))
            if not isinstance(cmd, dict):
                raise ValueError("JSON object required")
            response = queue.Queue(maxsize=1)
            self.server.commands.put_nowait((cmd, response, time.monotonic()))
            code, obj = response.get(timeout=3)
            self.reply(code, obj)
        except (ValueError, KeyError, queue.Empty, queue.Full, TimeoutError) as e:
            self.reply(400, {"error": str(e)})


def main():
    c, log = load_config(), Log("coordinator")
    m = Mission(c, log)
    peers = {}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 16000))
    sock.setblocking(False)
    api = ThreadingHTTPServer(("0.0.0.0", 8080), API)
    api.commands, api.snapshot = queue.Queue(maxsize=20), {}
    threading.Thread(target=api.serve_forever, daemon=True).start()
    seq, last, last_log = 0, time.monotonic(), 0.0
    log.write("config", config=c)
    while True:
        now = time.monotonic()
        for _ in range(100):
            try:
                b, addr = sock.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                t = json.loads(b)
                i = t["id"]
                if i not in IDS or addr != ("127.0.0.1", 16000+i):
                    continue
                if not 0 <= now-t["sent"] < c["stale_s"]:
                    continue
                if t["seq"] <= peers.get(i, {}).get("seq", -1):
                    continue
                peers[i] = t
            except (ValueError, KeyError, TypeError):
                continue
        for _ in range(20):
            try:
                cmd, response, received = api.commands.get_nowait()
            except queue.Empty:
                break
            try:
                if now-received > 2:
                    raise ValueError("Command expired")
                m.command(cmd, peers, now)
                response.put((200, {"accepted": True, "phase": m.phase}))
            except (ValueError, KeyError, TypeError) as e:
                response.put((409, {"accepted": False, "error": str(e)}))
        m.tick(peers, now, now-last)
        snapshot = m.snapshot(peers, now, seq)
        api.snapshot = snapshot
        packet = encode(snapshot)
        for i in IDS:
            sock.sendto(packet, ("127.0.0.1", 16000+i))
        if now-last_log >= 0.2:
            log.write("snapshot", **snapshot)
            last_log = now
        seq += 1
        last = now
        time.sleep(max(0, 1/c["rate_hz"]-(time.monotonic()-now)))


if __name__ == "__main__":
    main()
