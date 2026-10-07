# spiderbot_gazebo

Gazebo 9 world for the pest-detection PoC: the crawlspace under a house, with
uneven dirt, piers, plumbing and clutter for the hexapod to navigate, pest signs to
find, and a ground-truth soil-moisture map. The Spiderbot stands at the spawn point
with a working lidar and depth camera. Gazebo runs headless on the VM, and the same
world is mirrored to Foxglove as markers.

## Run and view in Foxglove

The robot's camera needs an X display to render, and the VM has none, so Gazebo runs
inside Xvfb (a virtual display). Install it once with `sudo apt install xvfb`.

```bash
# SSH session 1
xvfb-run -a -s "-screen 0 1280x1024x24" roslaunch spiderbot_gazebo crawlspace.launch

# SSH session 2
roslaunch rosbridge_server rosbridge_websocket.launch
```

Without `xvfb-run` everything still runs, including the lidar, but the camera publishes
nothing. If Gazebo dies at startup with exit code 255, an old gzserver is probably still
running: `killall -9 gzserver` and launch again.

In Foxglove's 3D panel:

1. Under **Frame**, set **Display frame** to `world`.
2. Under **Topics**, turn on `/world_markers` (eye icon). It takes a few seconds to appear,
   because the meshes are about 3.4 MB over rosbridge.
3. Each Gazebo model is a marker namespace under `/world_markers` (`terrain`, `piers`,
   `termite_mud_tubes`, `decoys`, ...), so you can hide parts. The `subfloor` is left out
   on purpose, so you can see in from above.
4. Turn on `/robot_description` for the robot and `/scan` for the lidar hits.
5. To see the ground-truth moisture, turn on `/moisture_truth` and set its **Color mode** to
   **Costmap** (blue = dry, red = wet). It's drawn 12 cm above the dirt so the bumps don't
   hide it; turn it off again to see the ground.

For the camera, add **Image** panels:

| Topic | What it is |
|---|---|
| `/camera/rgb/image_raw/compressed` | Colour camera, JPEG, 10 Hz (about 27 KB a frame) |
| `/camera/depth/image_raw_throttle` | Depth in metres, 1 Hz. Pixels closer than 0.6 m are blank, as on the real Astra. |

Don't open the raw `/camera/rgb/image_raw`, `/camera/depth/image_raw` or
`/camera/depth/points` in Foxglove: at about 1 MB or more a frame they swamp rosbridge.
Nodes on the VM use them directly.

| Launch arg | Default | |
|---|---|---|
| `gazebo` | `true` | Start gzserver with the world. `false` shows only the Foxglove markers. |
| `robot` | `true` | Spawn the Spiderbot with its sensors. It stands still with its legs in the standing pose until the walking node exists. |
| `spawn_x/y/z/yaw` | `-3.3 0 0 0` | Where `base_footprint` spawns: on the dirt just inside the access opening, facing +x. |

The ALSA sound errors in the Gazebo output are harmless on a VM without audio.

### Robot sensors

| Topic | Sensor | Details |
|---|---|---|
| `/scan` | YDLIDAR G4 (`laser_link`) | 360 deg, 720 beams, 0.12-16 m, 7 Hz, 1 cm noise. Scan plane 19 cm up: sees walls, piers and posts, passes over rocks and under pipes. |
| `/camera/rgb/image_raw` | Astra-class colour (`camera_optical_frame`) | 640x480, 58 deg wide, 10 Hz |
| `/camera/depth/image_raw`, `/camera/depth/points` | Astra-class depth | 640x480 metres (32FC1) and point cloud, 0.6-8 m |

Gazebo pauses a camera that has no subscribers, so the first frame after subscribing
is the last one from before the pause; later frames are live. Nodes that read single
frames with `rospy.wait_for_message` should skip the first.

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
