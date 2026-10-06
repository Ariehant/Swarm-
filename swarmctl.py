#!/usr/bin/env python3
import argparse
import json
import sys
import urllib.error
import urllib.request


def main():
    p = argparse.ArgumentParser(description="Control the whole triangle, never one isolated vertex")
    p.add_argument("action", choices=["status", "start", "hold", "resume", "waypoint", "land", "abort"])
    p.add_argument("--lat", type=float)
    p.add_argument("--lon", type=float)
    p.add_argument("--agl", type=float)
    p.add_argument("--url", default="http://127.0.0.1:8080")
    a = p.parse_args()
    payload = {"action": a.action}
    if a.action == "waypoint":
        if None in (a.lat, a.lon, a.agl):
            p.error("waypoint requires --lat, --lon, --agl")
        payload.update(lat=a.lat, lon=a.lon, agl=a.agl)
    request = urllib.request.Request(a.url+("/status" if a.action == "status" else "/command"),
                                    data=None if a.action == "status" else json.dumps(payload).encode(),
                                    headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=5) as r:
            print(json.dumps(json.load(r), indent=2))
    except urllib.error.HTTPError as e:
        print(e.read().decode(), file=sys.stderr)
        sys.exit(1)
    except urllib.error.URLError as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
