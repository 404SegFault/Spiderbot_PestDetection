# spiderbot_gazebo

Gazebo 9 world for the pest-detection PoC: the crawlspace under a house, with
uneven dirt, piers, plumbing and clutter for the hexapod to navigate, pest signs to
find, and a ground-truth soil-moisture map. Gazebo runs headless on the VM, and the
same world is mirrored to Foxglove as markers.

## Run and view in Foxglove

```bash
# SSH session 1
roslaunch spiderbot_gazebo crawlspace.launch

# SSH session 2
roslaunch rosbridge_server rosbridge_websocket.launch
```

In Foxglove's 3D panel:

1. Under **Frame**, set **Display frame** to `world`.
2. Under **Topics**, turn on `/world_markers` (eye icon). It takes a few seconds to appear,
   because the meshes are about 3.4 MB over rosbridge.
3. Each Gazebo model is a marker namespace under `/world_markers` (`terrain`, `piers`,
   `termite_mud_tubes`, `decoys`, ...), so you can hide parts. The `subfloor` is left out
   on purpose, so you can see in from above.
4. To see the ground-truth moisture, turn on `/moisture_truth` and set its **Color mode** to
   **Costmap** (blue = dry, red = wet). It's drawn 12 cm above the dirt so the bumps don't
   hide it; turn it off again to see the ground.

| Launch arg | Default | |
|---|---|---|
| `gazebo` | `true` | Start gzserver with the world. `false` shows only the Foxglove markers. |
| `preview_robot` | `true` | Show the robot standing at the spawn point, for scale. This is visual only; the robot isn't simulated in Gazebo yet. |
| `spawn_x/y/z/yaw` | `-3.3 0 0.125 0` | Spawn point, just inside the access opening, facing +x. |

The ALSA sound errors in the Gazebo output are harmless on a VM without audio.

## The world

Origin is the centre of the crawlspace at nominal dirt level, with x along its length.

| Feature | Details |
|---|---|
| Space | 8 x 6 m between concrete-block foundation walls, 0.75 m from dirt to joists |
| Terrain | Dirt mesh with mounds up to ~9 cm, hollows, a drainage rut and ripples; flat at the spawn point and around footings |
| Access opening | 0.7 x 0.45 m in the west wall, next to the spawn point, lit from outside |
| Structure | Centre girder on 3 concrete piers (x = -2, 0, 2), 2x10 joists at 16" centres, 2 steel jack posts |
| Services | PVC drain rising into the floor, two copper supply lines, an HVAC flex duct and register boot |
| Clutter | 30 rocks, 5 wood offcuts, a fallen insulation batt |
| Lighting | Dim ambient light, a warm work light near the centre, daylight through the access opening |
| Moisture | Dry dirt (~18 on a 1-98 scale), wet around a leaking drain riser and a dripping supply-line fitting, seeping along the north wall, wetter in hollows and the rut. Dirt above 45 is visibly darker. |
| Pest signs | Termite mud tubes on pier 0 and the north wall, damaged wood on joists and the rim by the leak, three rodent-dropping clusters, two burrows |
| Decoys | A mud smear (not a tube), an old dry water stain on a joist, loose pebbles (not droppings) |

Everything is static. The piers, jack posts and clutter are the obstacles at robot
height; the pipes and duct hang 0.43 m or more above the dirt. Pest signs and decoys
are visual only, so they don't get in the robot's way.

### Answer key

`worlds/crawlspace_ground_truth.yaml` lists every pest sign and decoy: its type, position,
whether it's a decoy, and the soil moisture there. Detection runs get scored against it.
`worlds/crawlspace_moisture.yaml` (map_server format) is the full ground-truth moisture
map; the simulated moisture probe will read from it.

## Changing the world

These files are generated, so don't edit them by hand:

- `worlds/crawlspace.world`
- `worlds/crawlspace_moisture.pgm` and `.yaml`
- `worlds/crawlspace_ground_truth.yaml`
- `models/crawlspace_terrain/meshes/*.stl`

Instead, edit the layout in `scripts/generate_crawlspace.py`, then run it and commit the
regenerated files:

```bash
python scripts/generate_crawlspace.py   # Python 2 or 3, no extra packages
```

`scripts/world_markers.py` reads whatever is in the `.world` file, so Foxglove picks up
changes without further edits. It handles box, cylinder, sphere and binary-STL mesh visuals.
