import json
import math
import os
import time
from pathlib import Path

IDS = (1, 2, 3)
R = 6378137.0
FLIGHT = {"TAKEOFF", "AUTO", "MANUAL", "HOLD", "APPROACH", "DESCEND"}
STREAM = FLIGHT | {"PRESTREAM", "OFFBOARD", "ARM"}


def load_config(path=None):
    with open(path or os.getenv("MISSION", "/app/config/mission.json")) as f:
        c = json.load(f)
    if not (8 <= c["side_m"] <= 40 and 5 <= c["takeoff_agl_m"] <= c["max_agl_m"]):
        raise ValueError("Invalid triangle side or takeoff height")
    if not 5 <= c["rate_hz"] <= 50:
        raise ValueError("Invalid update rate")
    for key in ("speed_mps", "vertical_mps", "stale_s", "arrival_m", "settle_s"):
        if not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(key)
    if not c["waypoints"]:
        raise ValueError("Mission needs waypoints")
    for wp in c["waypoints"] + [c["landing"]]:
        validate_waypoint(c, wp)
    return c


def to_ne(c, lat, lon):
    o = c["origin"]
    return [math.radians(lat-o["lat"])*R,
            math.radians(lon-o["lon"])*R*math.cos(math.radians(o["lat"]))]


def to_gps(c, north, east):
    o = c["origin"]
    return [o["lat"]+math.degrees(north/R),
            o["lon"]+math.degrees(east/(R*math.cos(math.radians(o["lat"]))))]


def offsets(c):
    s = c["side_m"]
    return {1: [s/math.sqrt(3), 0, 0],
            2: [-s/(2*math.sqrt(3)), -s/2, 0],
            3: [-s/(2*math.sqrt(3)), s/2, 0]}


def nominal(c, center, i):
    return [a+b for a, b in zip(center, offsets(c)[i])]


def waypoint(c, wp):
    return to_ne(c, wp["lat"], wp["lon"]) + [wp["agl"]]


def validate_waypoint(c, wp):
    values = [wp[k] for k in ("lat", "lon", "agl")]
    if not all(type(x) in (float, int) and math.isfinite(x) for x in values):
        raise ValueError("Coordinates must be finite numbers")
    if not (-90 <= values[0] <= 90 and -180 <= values[1] <= 180):
        raise ValueError("Invalid latitude/longitude")
    if not 3 <= values[2] <= c["max_agl_m"]:
        raise ValueError("Waypoint altitude outside configured AGL range")
    p = waypoint(c, wp)
    if math.hypot(*p[:2])+c["side_m"]/math.sqrt(3)+c["max_correction_m"] > c["radius_m"]:
        raise ValueError("Triangle would exceed the operating radius")
    return p


def distance(a, b):
    return math.dist(a, b)


def step(c, p, goal, dt):
    """Bound horizontal and vertical reference speed, including after override."""
    d = [b-a for a, b in zip(p, goal)]
    h = math.hypot(*d[:2])
    q = min(1.0, c["speed_mps"]*dt/h) if h else 0.0
    vz = max(-c["vertical_mps"]*dt, min(c["vertical_mps"]*dt, d[2]))
    return [p[0]+d[0]*q, p[1]+d[1]*q, p[2]+vz]


def correction(c, i, peers):
    """All-to-all relative-position error; bounded, no integral windup."""
    r = offsets(c)
    pi = peers[i]["p"]
    e = [sum((peers[j]["p"][k]-pi[k])-(r[j][k]-r[i][k])
             for j in IDS if j != i)/2*c["peer_gain"] for k in range(3)]
    n = math.sqrt(sum(x*x for x in e))
    return [x*min(1, c["max_correction_m"]/n) for x in e] if n else e


def metrics(c, center, peers):
    edges = [distance(peers[i]["p"], peers[j]["p"])
             for i, j in ((1, 2), (2, 3), (1, 3))]
    return {"edge_error_m": max(abs(x-c["side_m"]) for x in edges),
            "min_separation_m": min(edges), "edges_m": edges,
            "tracking_error_m": max(distance(peers[i]["p"], nominal(c, center, i)) for i in IDS)}


def healthy(c, t, now):
    transit = max(0, now-t.get("sent", -1e9))
    return (transit < c["stale_s"] and
            t.get("pos_age", 1e9)+transit < c["stale_s"] and
            t.get("gps_age", 1e9)+transit < 2 and
            t.get("hb_age", 1e9)+transit < c["heartbeat_stale_s"] and
            t.get("land_age", 1e9)+transit < 2 and
            t.get("fix", 0) >= 3 and t.get("eph", 999) <= c["gps_eph_max_m"] and
            t.get("home", False) and not t.get("fault") and
            len(t.get("p", [])) == 3 and all(math.isfinite(x) for x in t["p"]))


class Log:
    def __init__(self, name):
        p = Path(os.getenv("LOG_DIR", "/logs"))
        p.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        self.path = p / f"{name}-{stamp}-{os.getpid()}.jsonl"
        self.f = self.path.open("a", buffering=1)

    def write(self, event, **fields):
        self.f.write(json.dumps({"wall": time.time(), "mono": time.monotonic(),
                                 "event": event, **fields}, allow_nan=False)+"\n")


def encode(data):
    return json.dumps(data, allow_nan=False, separators=(",", ":")).encode()
