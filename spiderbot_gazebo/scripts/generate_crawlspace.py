#!/usr/bin/env python
"""Generate the crawlspace Gazebo world, its meshes, moisture map and answer key.

Writes (all committed):
  worlds/crawlspace.world                      Gazebo world
  worlds/crawlspace_moisture.{pgm,yaml}        ground-truth soil moisture, map_server format
  worlds/crawlspace_ground_truth.yaml          where every pest sign and decoy is
  models/crawlspace_terrain/meshes/*.stl       dirt surface and damp-soil overlay

To change the environment, edit the layout below and re-run (hand edits to the
outputs get overwritten). Runs on Python 2 or 3 with no extra packages:

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

# Soil moisture, as relative moisture 1-98 (dry dirt ~18, saturated ~95). It's 1-98
# rather than 0-100 so Foxglove's Costmap colouring shows a plain blue-to-red scale.
MOISTURE_RES = 0.05
DRY_MOISTURE = 18
LEAKS = [  # (x, y, spread sigma, peak added moisture): why the crawlspace is damp
    (2.5, 1.6, 0.55, 65),  # leaking toilet flange at the drain riser
    (-1.95, -1.04, 0.6, 55),  # dripping fitting on the copper supply lines
]
NORTH_SEEPAGE = 30  # outside grade slopes toward the north wall, so it seeps along there
DAMP_MOISTURE = 45  # above this the dirt is visibly darker
DAMP_URI = "model://crawlspace_terrain/meshes/damp_soil.stl"

# Pest signs, placed where they'd plausibly be: termite tubes and damaged wood where it's
# damp, rodent signs along walls and by the fallen insulation. Decoys look similar but
# aren't pest signs, and sit in dry areas.
MUD_TUBES = [  # (x, y on a vertical face, outward normal yaw, z bottom, z top, where)
    (-2.08, -0.2, -math.pi / 2, FOOTING_TOP, WALL_TOP - JOIST_DEPTH, "pier_0 south face, up to the girder"),
    (-1.92, -0.2, -math.pi / 2, FOOTING_TOP, 0.3, "pier_0 south face, partly built"),
    (1.6, HALF_Y, -math.pi / 2, None, 0.45, "north wall, partly built"),
    (2.05, HALF_Y, -math.pi / 2, None, WALL_TOP, "north wall, up to the rim joist"),
    (2.85, HALF_Y, -math.pi / 2, None, WALL_TOP, "north wall, up to the rim joist"),
]
WOOD_DAMAGE = [  # (joist x, y from, y to, where); None x = north rim joist from x to x
    (2.6, 1.25, 1.95, "joist beside the leaking drain riser"),
    (2.2, 1.35, 1.85, "joist beside the leaking drain riser"),
    (None, 1.7, 2.95, "north rim joist above the mud tubes"),
]
DROPPINGS = [(-2.4, 2.88, "along the north wall"), (2.05, -1.95, "by the fallen insulation"),
             (0.36, 0.22, "beside pier_1")]
BURROWS = [(3.86, 1.2, "at the base of the east wall"), (0.62, -2.62, "by the fallen insulation")]
MUD_SMEAR = (-2.4, -HALF_Y, math.pi / 2)  # decoy: splashed mud on the south wall, not a tube
WATER_STAIN = (-3.0, 0.4, 0.9)  # decoy: old, now-dry stain on a joist (x, y from, y to)
PEBBLES = (-1.2, 1.3)  # decoy: loose pebbles, not droppings

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
    "damp_dirt": (0.26, 0.19, 0.13),
    "mud": (0.47, 0.35, 0.22),
    "damaged_wood": (0.33, 0.23, 0.13),
    "water_stain": (0.62, 0.56, 0.47),
    "droppings": (0.08, 0.06, 0.05),
    "burrow": (0.05, 0.04, 0.03),
    "fresh_dirt": (0.55, 0.45, 0.33),
    "pebble": (0.5, 0.5, 0.48),
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


def _make_moisture_fn(rng):
    ripples = [(rng.uniform(0, 2 * math.pi), rng.uniform(0.6, 1.5), rng.uniform(0, 2 * math.pi)) for _ in range(3)]

    def moisture(x, y):
        noise = sum(math.sin(2 * math.pi / wl * (x * math.cos(d) + y * math.sin(d)) + ph) for d, wl, ph in ripples)
        m = DRY_MOISTURE + 1.5 * noise
        m += sum(a * math.exp(-((x - lx) ** 2 + (y - ly) ** 2) / (2 * s * s)) for lx, ly, s, a in LEAKS)
        m += 400 * max(0.0, -height(x, y))  # water collects in hollows and the rut
        m += NORTH_SEEPAGE * _smoothstep(HALF_Y - 0.8, HALF_Y, y) * (0.75 + 0.25 * math.sin(3 * x))
        return min(max(m, 1), 98)

    return moisture


moisture = _make_moisture_fn(random.Random(SEED + 2))


def _grid_triangles(res, z_fn, keep=None):
    """Triangles of a heightfield over the crawlspace out to the outside of the walls,
    optionally only the cells whose centre passes keep(x, y)."""
    half_x, half_y = HALF_X + WALL_T, HALF_Y + WALL_T
    nx, ny = int(round(2 * half_x / res)) + 1, int(round(2 * half_y / res)) + 1
    xs = [-half_x + i * res for i in range(nx)]
    ys = [-half_y + j * res for j in range(ny)]
    pts = [[(x, y, z_fn(x, y)) for x in xs] for y in ys]
    tris = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            if keep and not keep(xs[i] + res / 2, ys[j] + res / 2):
                continue
            a, b, c, d = pts[j][i], pts[j][i + 1], pts[j + 1][i + 1], pts[j + 1][i]
            tris += [(a, b, c), (a, c, d)]  # counter-clockwise from above, normals up
    return tris


def write_stl(path, tris):
    """Binary STL file, named in its header after the file."""
    name = os.path.splitext(os.path.basename(path))[0].replace("_", " ")
    with open(path, "wb") as f:
        f.write(("spiderbot crawlspace %s" % name).encode("ascii").ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            u = [b[k] - a[k] for k in range(3)]
            v = [c[k] - a[k] for k in range(3)]
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            norm = math.sqrt(sum(k * k for k in n))
            f.write(struct.pack("<12fH", *([k / norm for k in n] + list(a) + list(b) + list(c) + [0])))
    return len(tris)


def write_moisture_map(pgm_path, yaml_path):
    """Moisture grid as a map_server map: raw pixel values are the moisture (1-98)."""
    half_x, half_y = HALF_X + WALL_T, HALF_Y + WALL_T
    nx, ny = int(round(2 * half_x / MOISTURE_RES)), int(round(2 * half_y / MOISTURE_RES))
    pixels = bytearray()
    for j in reversed(range(ny)):  # image rows run from +y down to -y
        for i in range(nx):
            pixels.append(int(round(moisture(-half_x + (i + 0.5) * MOISTURE_RES,
                                              -half_y + (j + 0.5) * MOISTURE_RES))))
    with open(pgm_path, "wb") as f:
        f.write(("P5\n%d %d\n255\n" % (nx, ny)).encode("ascii"))
        f.write(pixels)
    with open(yaml_path, "w") as f:
        f.write("# Generated by scripts/generate_crawlspace.py. Ground-truth relative soil moisture,\n"
                "# 1 (dry) to 98 (saturated); with mode raw, map_server publishes pixel values as-is.\n")
        f.write("image: %s\nresolution: %s\norigin: [%s, %s, 0.0]\n" % (
            os.path.basename(pgm_path), _fmt(MOISTURE_RES), _fmt(-half_x), _fmt(-half_y)))
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: raw\n")


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


def sphere(radius):
    geometry = ET.Element("geometry")
    _sub(_sub(geometry, "sphere"), "radius", _fmt(radius))
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


def add_shape(link, name, geometry, color, xyz, rpy=(0, 0, 0), collide=True):
    """Add a visual and, unless collide is False, a matching collision element to a link."""
    rgba = _fmt(*(tuple(color) + (1,)))
    for kind in ("visual", "collision") if collide else ("visual",):
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
    keep_out += [((px, py), 0.25) for px, py, _ in DROPPINGS + BURROWS] + [(PEBBLES, 0.25)]
    return all(math.hypot(x - px, y - py) > r + margin for (px, py), r in keep_out)


def _face_point(x, y, yaw, out, along):
    """Point `out` metres off a vertical face whose outward normal is at `yaw`, `along` metres along it."""
    return (x + out * math.cos(yaw) - along * math.sin(yaw), y + out * math.sin(yaw) + along * math.cos(yaw))


def _mud_tube(link, name, x, y, yaw, z0, z1, rng):
    """Termite mud tube climbing a vertical face, as a column of wobbling segments."""
    z, k, along = z0, 0, 0.0
    while z < z1 - 0.005:
        h = min(rng.uniform(0.03, 0.06), z1 - z)
        along += rng.uniform(-0.006, 0.006)
        add_shape(link, "%s_%d" % (name, k), box(0.008, rng.uniform(0.012, 0.018), h), COLORS["mud"],
                  _face_point(x, y, yaw, 0.004, along) + (z + h / 2,), (0, 0, yaw), collide=False)
        z, k = z + h, k + 1


def _joist_band(link, name, joist_x, y0, y1, color):
    """Discoloured band wrapping the bottom of a joist between y0 and y1."""
    add_shape(link, name, box(JOIST_W + 0.006, y1 - y0, 0.08), color, (joist_x, (y0 + y1) / 2, WALL_TOP + 0.037),
              collide=False)


def _scatter(link, name, cx, cy, radius, count, make, color, rng):
    """Small items lying on the dirt within `radius` of (cx, cy); make(rng) -> (geometry, lift, rpy)."""
    for k in range(count):
        a, r = rng.uniform(0, 2 * math.pi), radius * math.sqrt(rng.random())
        x, y = cx + r * math.cos(a), cy + r * math.sin(a)
        geometry, lift, rpy = make(rng)
        add_shape(link, "%s_%d" % (name, k), geometry, color, (x, y, height(x, y) + lift), rpy, collide=False)


def _dropping(rng):
    return cylinder(0.0025, 0.01), 0.0025, (math.pi / 2, 0, rng.uniform(0, math.pi))


def _pebble(rng):
    r = rng.uniform(0.004, 0.007)
    return sphere(r), 0.6 * r, (0, 0, 0)


def add_pest_signs(world, rng):
    """Add the pest signs and decoys (visual only); return answer-key entries."""
    truth = []

    def record(kind, x, y, z, where, decoy=False):
        truth.append({"type": kind, "decoy": decoy, "x": x, "y": y, "z": z, "where": where})

    link = static_model(world, "termite_mud_tubes")
    for i, (x, y, yaw, z0, z1, where) in enumerate(MUD_TUBES):
        if z0 is None:  # start just below the dirt at the foot of the face
            z0 = height(*_face_point(x, y, yaw, 0.02, 0)) - 0.01
        _mud_tube(link, "tube_%d" % i, x, y, yaw, z0, z1, rng)
        record("termite_mud_tube", x, y, (z0 + z1) / 2, where)

    link = static_model(world, "damaged_wood")
    rim_face_y = HALF_Y + WALL_T - JOIST_W - 0.003
    for i, (joist_x, a, b, where) in enumerate(WOOD_DAMAGE):
        if joist_x is None:
            add_shape(link, "damage_%d" % i, box(b - a, 0.006, 0.12), COLORS["damaged_wood"],
                      ((a + b) / 2, rim_face_y, WALL_TOP + 0.06), collide=False)
            record("damaged_wood", (a + b) / 2, rim_face_y, WALL_TOP + 0.06, where)
        else:
            _joist_band(link, "damage_%d" % i, joist_x, a, b, COLORS["damaged_wood"])
            record("damaged_wood", joist_x, (a + b) / 2, WALL_TOP + 0.037, where)

    link = static_model(world, "rodent_droppings")
    for i, (x, y, where) in enumerate(DROPPINGS):
        _scatter(link, "cluster_%d" % i, x, y, 0.08, 12, _dropping, COLORS["droppings"], rng)
        record("rodent_droppings", x, y, height(x, y), where)

    # Burrow entrance with a dome of dug-out dirt beside it, on the side toward the room
    link = static_model(world, "rodent_burrows")
    for i, (x, y, where) in enumerate(BURROWS):
        add_shape(link, "hole_%d" % i, cylinder(0.045, 0.004), COLORS["burrow"], (x, y, height(x, y) + 0.002),
                  collide=False)
        d = math.hypot(x, y)
        sx, sy = x - 0.11 * x / d, y - 0.11 * y / d
        add_shape(link, "spoil_%d" % i, sphere(0.07), COLORS["fresh_dirt"], (sx, sy, height(sx, sy) - 0.045),
                  collide=False)
        record("rodent_burrow", x, y, height(x, y), where)

    link = static_model(world, "decoys")
    x, y, yaw = MUD_SMEAR
    for k in range(3):
        add_shape(link, "mud_smear_%d" % k, box(0.004, rng.uniform(0.08, 0.18), rng.uniform(0.05, 0.11)),
                  COLORS["mud"], _face_point(x, y, yaw, 0.002, rng.uniform(-0.08, 0.08)) + (rng.uniform(0.15, 0.25),),
                  (rng.uniform(-0.6, 0.6), 0, yaw), collide=False)
    record("mud_smear", x, y, 0.2, "south wall, splashed mud rather than a tube", decoy=True)
    joist_x, a, b = WATER_STAIN
    _joist_band(link, "water_stain", joist_x, a, b, COLORS["water_stain"])
    record("water_stain", joist_x, (a + b) / 2, WALL_TOP + 0.037, "joist, old dry stain rather than damage", decoy=True)
    x, y = PEBBLES
    _scatter(link, "pebble", x, y, 0.09, 12, _pebble, COLORS["pebble"], rng)
    record("pebbles", x, y, height(x, y), "loose pebbles rather than droppings", decoy=True)
    return truth


def write_ground_truth(path, truth):
    with open(path, "w") as f:
        f.write("# Generated by scripts/generate_crawlspace.py. Answer key for scoring pest-detection runs:\n"
                "# every pest sign and decoy, world frame, metres. soil_moisture is the ground-truth\n"
                "# moisture (1-98) of the dirt below it; the full map is crawlspace_moisture.yaml.\n"
                "signs:\n")
        for i, s in enumerate(truth):
            f.write("  - {id: %d, type: %s, decoy: %s, x: %s, y: %s, z: %s, soil_moisture: %d, where: \"%s\"}\n" % (
                i, s["type"], "true" if s["decoy"] else "false", _fmt(s["x"]), _fmt(s["y"]), _fmt(s["z"]),
                int(round(moisture(s["x"], s["y"]))), s["where"]))


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
    add_shape(static_model(world, "damp_soil"), "damp", mesh(DAMP_URI), COLORS["damp_dirt"], (0, 0, 0),
              collide=False)

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

    truth = add_pest_signs(world, random.Random(SEED + 3))
    return sdf, truth


def main():
    meshes = os.path.join(PKG_DIR, "models", "crawlspace_terrain", "meshes")
    worlds = os.path.join(PKG_DIR, "worlds")
    for directory in (meshes, worlds):
        if not os.path.isdir(directory):
            os.makedirs(directory)

    n = write_stl(os.path.join(meshes, "terrain.stl"), _grid_triangles(TERRAIN_RES, height))
    # Damp patches: a finer copy of the dirt surface, lifted 6 mm, wherever it's wet enough
    m = write_stl(os.path.join(meshes, "damp_soil.stl"),
                  _grid_triangles(MOISTURE_RES, lambda x, y: height(x, y) + 0.006,
                                  lambda x, y: moisture(x, y) >= DAMP_MOISTURE))
    write_moisture_map(os.path.join(worlds, "crawlspace_moisture.pgm"), os.path.join(worlds, "crawlspace_moisture.yaml"))
    sdf, truth = build_world()
    _indent(sdf)
    ET.ElementTree(sdf).write(os.path.join(worlds, "crawlspace.world"), encoding="utf-8", xml_declaration=True)
    write_ground_truth(os.path.join(worlds, "crawlspace_ground_truth.yaml"), truth)
    print("Wrote terrain (%d triangles), damp soil (%d triangles), moisture map, world and answer key "
          "(%d signs) under %s" % (n, m, len(truth), PKG_DIR))


if __name__ == "__main__":
    main()
