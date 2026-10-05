#!/usr/bin/env python
"""Generate the crawlspace Gazebo world and its terrain mesh.

Writes worlds/crawlspace.world and models/crawlspace_terrain/meshes/terrain.stl,
both of which are committed. To change the environment, edit the layout below
and re-run (hand edits to the .world file get overwritten). Runs on Python 2 or 3
with no extra packages:

    python scripts/generate_crawlspace.py
"""
from __future__ import division, print_function

import copy
import math
import os
import random
import struct
import xml.etree.ElementTree as ET

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Metres. World origin is the centre of the crawlspace at nominal dirt level, x runs
# along its length, z up. Sized like a typical US crawlspace section: 8 x 6 m between
# the foundation walls and 0.75 m from the dirt to the underside of the floor joists.
SEED = 7
HALF_X, HALF_Y = 4.0, 3.0  # inside faces of the foundation walls
WALL_T = 0.2
WALL_BOTTOM = -0.3  # walls go below the dirt so there are no gaps under them
WALL_TOP = 0.75  # underside of the floor framing
JOIST_DEPTH, JOIST_W, JOIST_SPACING = 0.235, 0.04, 0.4  # 2x10s at 16" centres
SUBFLOOR_T = 0.018
GIRDER_W = 0.12
FOOTING_TOP = 0.05
PIERS = [(-2.0, 0.0), (0.0, 0.0), (2.0, 0.0)]  # concrete piers under the centre girder
JACK_POSTS = [(1.0, 2.2), (-1.4, -1.8)]  # steel posts propping up two joists (x on a joist)
ACCESS_HALF_W, ACCESS_BOTTOM, ACCESS_TOP = 0.35, 0.1, 0.55  # opening in the west wall
SPAWN = (-3.3, 0.0)  # robot start, just inside the access opening (keep in sync with launch)
TERRAIN_RES = 0.1
TERRAIN_URI = "model://crawlspace_terrain/meshes/terrain.stl"

COLORS = {
    "dirt": (0.42, 0.33, 0.24),
    "block": (0.62, 0.62, 0.6),
    "concrete": (0.52, 0.52, 0.5),
    "lumber": (0.76, 0.62, 0.42),
    "plywood": (0.7, 0.58, 0.4),
    "steel": (0.35, 0.36, 0.38),
    "pvc": (0.92, 0.92, 0.9),
    "copper": (0.72, 0.45, 0.2),
    "duct": (0.75, 0.76, 0.78),
    "rock": (0.45, 0.43, 0.4),
    "scrap": (0.62, 0.5, 0.33),
    "insulation": (0.95, 0.62, 0.68),
}


# ---------------------------------------------------------------- terrain

def _smoothstep(edge0, edge1, x):
    t = min(max((x - edge0) / (edge1 - edge0), 0.0), 1.0)
    return t * t * (3 - 2 * t)


def _segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    t = min(max(((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy), 0.0), 1.0)
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _make_height_fn(rng):
    # Spoil mounds, shallow hollows, an old drainage rut and low ripples, flattened
    # around the spawn point and anything standing on the ground.
    bumps = [(rng.uniform(-3.4, 3.4), rng.uniform(-2.5, 2.5), rng.uniform(0.3, 0.7), rng.uniform(0.03, 0.09))
             for _ in range(7)]
    bumps += [(rng.uniform(-3.4, 3.4), rng.uniform(-2.5, 2.5), rng.uniform(0.4, 0.7), -rng.uniform(0.02, 0.05))
              for _ in range(3)]
    ripples = [(rng.uniform(0, 2 * math.pi), rng.uniform(0.7, 1.8), rng.uniform(0, 2 * math.pi),
                rng.uniform(0.004, 0.01)) for _ in range(4)]
    rut = (-1.5, -HALF_Y, 2.5, HALF_Y, 0.15, 0.035)  # from (x, y) to (x, y), width sigma, depth
    flat = [(SPAWN, 0.45, 0.9)] + [(p, 0.35, 0.6) for p in PIERS] + [(p, 0.2, 0.4) for p in JACK_POSTS]

    def height(x, y):
        h = sum(a * math.exp(-((x - bx) ** 2 + (y - by) ** 2) / (2 * s * s)) for bx, by, s, a in bumps)
        h += sum(a * math.sin(2 * math.pi / wl * (x * math.cos(d) + y * math.sin(d)) + ph)
                 for d, wl, ph, a in ripples)
        ax, ay, bx, by, width, depth = rut
        h -= depth * math.exp(-_segment_distance(x, y, ax, ay, bx, by) ** 2 / (2 * width * width))
        for (fx, fy), inner, outer in flat:
            h *= _smoothstep(inner, outer, math.hypot(x - fx, y - fy))
        return h

    return height


height = _make_height_fn(random.Random(SEED))


def write_terrain_stl(path):
    """Binary STL heightfield covering the crawlspace out to the outside of the walls."""
    half_x, half_y = HALF_X + WALL_T, HALF_Y + WALL_T
    nx, ny = int(round(2 * half_x / TERRAIN_RES)) + 1, int(round(2 * half_y / TERRAIN_RES)) + 1
    xs = [-half_x + i * TERRAIN_RES for i in range(nx)]
    ys = [-half_y + j * TERRAIN_RES for j in range(ny)]
    pts = [[(x, y, height(x, y)) for x in xs] for y in ys]
    tris = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            a, b, c, d = pts[j][i], pts[j][i + 1], pts[j + 1][i + 1], pts[j + 1][i]
            tris += [(a, b, c), (a, c, d)]  # counter-clockwise from above, normals up
    with open(path, "wb") as f:
        f.write(b"spiderbot crawlspace terrain".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            u = [b[k] - a[k] for k in range(3)]
            v = [c[k] - a[k] for k in range(3)]
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            norm = math.sqrt(sum(k * k for k in n))
            f.write(struct.pack("<12fH", *([k / norm for k in n] + list(a) + list(b) + list(c) + [0])))
    return len(tris)


# ---------------------------------------------------------------- SDF helpers

def _fmt(*values):
    out = []
    for v in values:
        s = ("%.4f" % v).rstrip("0").rstrip(".")
        out.append("0" if s in ("-0", "") else s)
    return " ".join(out)


def _sub(parent, tag, text=None, **attrib):
    element = ET.SubElement(parent, tag, attrib)
    if text is not None:
        element.text = text
    return element


def box(sx, sy, sz):
    geometry = ET.Element("geometry")
    _sub(_sub(geometry, "box"), "size", _fmt(sx, sy, sz))
    return geometry


def cylinder(radius, length):
    geometry = ET.Element("geometry")
    shape = _sub(geometry, "cylinder")
    _sub(shape, "radius", _fmt(radius))
    _sub(shape, "length", _fmt(length))
    return geometry


def mesh(uri):
    geometry = ET.Element("geometry")
    _sub(_sub(geometry, "mesh"), "uri", uri)
    return geometry


def static_model(world, name):
    """Add a static model with a single link and return the link."""
    model = _sub(world, "model", name=name)
    _sub(model, "static", "true")
    return _sub(model, "link", name="link")


def add_shape(link, name, geometry, color, xyz, rpy=(0, 0, 0)):
    """Add a matching visual and collision element to a link."""
    rgba = _fmt(*(tuple(color) + (1,)))
    for kind in ("visual", "collision"):
        element = _sub(link, kind, name=name)
        _sub(element, "pose", _fmt(*(tuple(xyz) + tuple(rpy))))
        element.append(copy.deepcopy(geometry))
        if kind == "visual":
            material = _sub(element, "material")
            _sub(material, "ambient", rgba)
            _sub(material, "diffuse", rgba)
            _sub(material, "specular", "0.05 0.05 0.05 1")


def _point_light(world, name, xyz, color, attenuation):
    light = _sub(world, "light", name=name, type="point")
    _sub(light, "pose", _fmt(*(tuple(xyz) + (0, 0, 0))))
    _sub(light, "diffuse", _fmt(*(tuple(color) + (1,))))
    _sub(light, "specular", "0.1 0.1 0.1 1")
    att = _sub(light, "attenuation")
    for tag, value in zip(("range", "constant", "linear", "quadratic"), attenuation):
        _sub(att, tag, _fmt(value))
    _sub(light, "cast_shadows", "false")


def _indent(element, level=0):
    pad = "\n" + level * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = pad + "  "
        for child in element:
            _indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    if level and (not element.tail or not element.tail.strip()):
        element.tail = pad


# ---------------------------------------------------------------- world

def _clear_of_fixtures(x, y, margin):
    keep_out = [(SPAWN, 0.6)] + [(p, 0.45) for p in PIERS] + [(p, 0.3) for p in JACK_POSTS]
    return all(math.hypot(x - px, y - py) > r + margin for (px, py), r in keep_out)


def build_world():
    rng = random.Random(SEED + 1)
    sdf = ET.Element("sdf", version="1.6")
    sdf.append(ET.Comment(" Generated by scripts/generate_crawlspace.py - edit that and re-run "))
    world = _sub(sdf, "world", name="crawlspace")

    physics = _sub(world, "physics", type="ode")
    _sub(physics, "max_step_size", "0.001")
    _sub(physics, "real_time_factor", "1")
    _sub(physics, "real_time_update_rate", "1000")
    _sub(world, "gravity", "0 0 -9.8")

    # Dim and enclosed: low ambient light, a work light and daylight through the access opening
    scene = _sub(world, "scene")
    _sub(scene, "ambient", "0.3 0.3 0.3 1")
    _sub(scene, "background", "0.05 0.05 0.05 1")
    _sub(scene, "shadows", "false")
    _point_light(world, "work_light", (0.5, -0.8, 0.7), (1.0, 0.85, 0.65), (8, 0.4, 0.1, 0.02))
    _point_light(world, "access_daylight", (-3.85, 0, 0.35), (0.75, 0.8, 0.9), (4, 0.5, 0.3, 0.1))

    add_shape(static_model(world, "terrain"), "dirt", mesh(TERRAIN_URI), COLORS["dirt"], (0, 0, 0))

    # Foundation walls. North/south run the full outside length, east/west fit between
    # them, and the west wall is split around the access opening.
    link = static_model(world, "foundation_walls")
    wall_h, wall_z = WALL_TOP - WALL_BOTTOM, (WALL_TOP + WALL_BOTTOM) / 2
    for name, sign in (("north", 1), ("south", -1)):
        add_shape(link, name, box(2 * (HALF_X + WALL_T), WALL_T, wall_h), COLORS["block"],
                  (0, sign * (HALF_Y + WALL_T / 2), wall_z))
    add_shape(link, "east", box(WALL_T, 2 * HALF_Y, wall_h), COLORS["block"], (HALF_X + WALL_T / 2, 0, wall_z))
    west_x, side = -(HALF_X + WALL_T / 2), HALF_Y - ACCESS_HALF_W
    for name, sign in (("west_north", 1), ("west_south", -1)):
        add_shape(link, name, box(WALL_T, side, wall_h), COLORS["block"],
                  (west_x, sign * (ACCESS_HALF_W + side / 2), wall_z))
    add_shape(link, "west_below_access", box(WALL_T, 2 * ACCESS_HALF_W, ACCESS_BOTTOM - WALL_BOTTOM),
              COLORS["block"], (west_x, 0, (ACCESS_BOTTOM + WALL_BOTTOM) / 2))
    add_shape(link, "west_above_access", box(WALL_T, 2 * ACCESS_HALF_W, WALL_TOP - ACCESS_TOP),
              COLORS["block"], (west_x, 0, (WALL_TOP + ACCESS_TOP) / 2))

    # Floor framing: centre girder (top flush with the walls), joists across it, rim joists
    link = static_model(world, "floor_framing")
    joist_z = WALL_TOP + JOIST_DEPTH / 2
    add_shape(link, "girder", box(2 * HALF_X, GIRDER_W, JOIST_DEPTH), COLORS["lumber"],
              (0, 0, WALL_TOP - JOIST_DEPTH / 2))
    rim_x, rim_y = HALF_X + WALL_T - JOIST_W / 2, HALF_Y + WALL_T - JOIST_W / 2
    for k in range(int(round(2 * HALF_X / JOIST_SPACING))):
        add_shape(link, "joist_%02d" % k, box(JOIST_W, 2 * rim_y - JOIST_W, JOIST_DEPTH), COLORS["lumber"],
                  (-HALF_X + JOIST_SPACING / 2 + k * JOIST_SPACING, 0, joist_z))
    for name, sign in (("rim_north", 1), ("rim_south", -1)):
        add_shape(link, name, box(2 * (HALF_X + WALL_T), JOIST_W, JOIST_DEPTH), COLORS["lumber"],
                  (0, sign * rim_y, joist_z))
    for name, sign in (("rim_east", 1), ("rim_west", -1)):
        add_shape(link, name, box(JOIST_W, 2 * rim_y - JOIST_W, JOIST_DEPTH), COLORS["lumber"],
                  (sign * rim_x, 0, joist_z))

    add_shape(static_model(world, "subfloor"), "plywood",
              box(2 * (HALF_X + WALL_T), 2 * (HALF_Y + WALL_T), SUBFLOOR_T), COLORS["plywood"],
              (0, 0, WALL_TOP + JOIST_DEPTH + SUBFLOOR_T / 2))

    link = static_model(world, "piers")
    pier_h = WALL_TOP - JOIST_DEPTH - FOOTING_TOP
    for i, (x, y) in enumerate(PIERS):
        add_shape(link, "footing_%d" % i, box(0.5, 0.5, 0.2), COLORS["concrete"], (x, y, FOOTING_TOP - 0.1))
        add_shape(link, "pier_%d" % i, box(0.4, 0.4, pier_h), COLORS["block"], (x, y, FOOTING_TOP + pier_h / 2))

    link = static_model(world, "jack_posts")
    post_h = WALL_TOP - FOOTING_TOP
    for i, (x, y) in enumerate(JACK_POSTS):
        add_shape(link, "pad_%d" % i, box(0.3, 0.3, 0.1), COLORS["concrete"], (x, y, FOOTING_TOP - 0.05))
        add_shape(link, "post_%d" % i, cylinder(0.025, post_h), COLORS["steel"], (x, y, FOOTING_TOP + post_h / 2))
        add_shape(link, "plate_%d" % i, box(0.15, 0.15, 0.008), COLORS["steel"], (x, y, WALL_TOP - 0.004))

    # PVC drain hung under the joists, rising into the floor at x = 2.5, plus two copper supply lines
    link = static_model(world, "plumbing")
    drain_r, drain_y, drain_z, drain_x0, drain_x1 = 0.05, 1.6, WALL_TOP - 0.09, -HALF_X + 0.1, 2.5
    add_shape(link, "drain", cylinder(drain_r, drain_x1 - drain_x0), COLORS["pvc"],
              ((drain_x0 + drain_x1) / 2, drain_y, drain_z), (0, math.pi / 2, 0))
    riser_h = WALL_TOP + JOIST_DEPTH - drain_z
    add_shape(link, "drain_riser", cylinder(drain_r, riser_h), COLORS["pvc"],
              (drain_x1, drain_y, drain_z + riser_h / 2))
    for i, y in enumerate((-1.0, -1.08)):
        add_shape(link, "supply_%d" % i, cylinder(0.011, 7.0), COLORS["copper"],
                  (-0.4, y, WALL_TOP - 0.03), (0, math.pi / 2, 0))

    # Flex duct from the south wall to a floor register boot, between two joists
    link = static_model(world, "hvac")
    duct_r, duct_x, duct_y0, duct_y1 = 0.15, 2.4, -HALF_Y, -0.45
    add_shape(link, "flex_duct", cylinder(duct_r, duct_y1 - duct_y0), COLORS["duct"],
              (duct_x, (duct_y0 + duct_y1) / 2, WALL_TOP - duct_r - 0.02), (math.pi / 2, 0, 0))
    add_shape(link, "register_boot", box(0.3, 0.3, 0.3), COLORS["duct"], (duct_x, duct_y1 + 0.15, WALL_TOP))

    # Ground clutter, partly sunk into the dirt
    link = static_model(world, "rocks")
    count = 0
    while count < 30:
        x, y = rng.uniform(-HALF_X + 0.3, HALF_X - 0.3), rng.uniform(-HALF_Y + 0.3, HALF_Y - 0.3)
        size = rng.uniform(0.03, 0.1)
        sx, sy, sz = size * rng.uniform(0.7, 1.3), size * rng.uniform(0.7, 1.3), size * rng.uniform(0.4, 0.8)
        shade = rng.uniform(0.8, 1.2)
        if not _clear_of_fixtures(x, y, size):
            continue
        add_shape(link, "rock_%02d" % count, box(sx, sy, sz), [min(c * shade, 1) for c in COLORS["rock"]],
                  (x, y, height(x, y) + 0.2 * sz),
                  (rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), rng.uniform(0, math.pi)))
        count += 1

    link = static_model(world, "wood_scraps")  # 2x4 offcuts
    count = 0
    while count < 5:
        x, y = rng.uniform(-HALF_X + 0.5, HALF_X - 0.5), rng.uniform(-HALF_Y + 0.5, HALF_Y - 0.5)
        length, yaw = rng.uniform(0.25, 0.7), rng.uniform(0, math.pi)
        if not _clear_of_fixtures(x, y, length / 2):
            continue
        ends = [height(x + s * length / 2 * math.cos(yaw), y + s * length / 2 * math.sin(yaw)) for s in (-1, 1)]
        add_shape(link, "scrap_%d" % count, box(length, 0.089, 0.038), COLORS["scrap"],
                  (x, y, (sum(ends) + height(x, y)) / 3 + 0.015), (0, 0, yaw))
        count += 1

    # A batt of insulation that has fallen out of the joists
    x, y = 1.3, -2.3
    add_shape(static_model(world, "fallen_insulation"), "batt", box(1.2, 0.38, 0.09), COLORS["insulation"],
              (x, y, height(x, y) + 0.03), (0, 0, 0.4))
    return sdf


def main():
    stl_path = os.path.join(PKG_DIR, "models", "crawlspace_terrain", "meshes", "terrain.stl")
    world_path = os.path.join(PKG_DIR, "worlds", "crawlspace.world")
    for path in (stl_path, world_path):
        if not os.path.isdir(os.path.dirname(path)):
            os.makedirs(os.path.dirname(path))
    triangles = write_terrain_stl(stl_path)
    sdf = build_world()
    _indent(sdf)
    ET.ElementTree(sdf).write(world_path, encoding="utf-8", xml_declaration=True)
    print("Wrote %s (%d triangles) and %s" % (stl_path, triangles, world_path))


if __name__ == "__main__":
    main()
