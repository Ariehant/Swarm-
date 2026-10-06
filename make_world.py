"""Derive the world from PX4's own default, retaining its sensor plugins."""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from swarm.common import IDS, load_config, nominal, to_gps, waypoint


def build(source, destination, config, manifest):
    c = load_config(config)
    tree = ET.parse(source)
    w = tree.getroot().find("world")
    w.set("name", "swarm")
    spherical = w.find("spherical_coordinates")
    if spherical is not None:
        w.remove(spherical)
    spherical = ET.SubElement(w, "spherical_coordinates")
    for key, value in {"surface_model": "EARTH_WGS84", "world_frame_orientation": "ENU",
                       "latitude_deg": c["origin"]["lat"], "longitude_deg": c["origin"]["lon"],
                       "elevation": c["origin"]["amsl"], "heading_deg": 0}.items():
        ET.SubElement(spherical, key).text = str(value)
    physics = w.find("physics")
    if physics is not None:
        factor = physics.find("real_time_factor")
        if factor is None:
            factor = ET.SubElement(physics, "real_time_factor")
        factor.text = "1.0"
    records = []
    for group, center, color in (("home", [0, 0, 0], "0.1 0.6 1 1"),
                                  ("landing", waypoint(c, c["landing"]), "0.1 1 0.3 1")):
        for i in IDS:
            n, e, _ = nominal(c, center, i)
            model = ET.SubElement(w, "model", name=f"{group}_{i}")
            ET.SubElement(model, "static").text = "true"
            ET.SubElement(model, "pose").text = f"{e} {n} 0.012 0 0 0"
            link = ET.SubElement(model, "link", name="pad")
            visual = ET.SubElement(link, "visual", name="disc")
            cylinder = ET.SubElement(ET.SubElement(visual, "geometry"), "cylinder")
            ET.SubElement(cylinder, "radius").text = "1.2"
            ET.SubElement(cylinder, "length").text = "0.01"
            material = ET.SubElement(visual, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color
            lat, lon = to_gps(c, n, e)
            records.append({"marker": f"{group}_{i}", "id": i, "north": n, "east": e,
                            "lat": lat, "lon": lon, "ground_amsl": c["origin"]["amsl"]})
    ET.indent(tree)
    tree.write(destination, encoding="unicode", xml_declaration=True)
    Path(manifest).write_text(json.dumps(records, indent=2)+"\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("source")
    p.add_argument("destination")
    p.add_argument("--config", default="/app/config/mission.json")
    p.add_argument("--manifest", default="/logs/markers.json")
    a = p.parse_args()
    build(a.source, a.destination, a.config, a.manifest)
