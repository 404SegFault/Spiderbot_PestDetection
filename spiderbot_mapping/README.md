# spiderbot_mapping

Builds two maps while you drive the robot around:

- **Floor plan** (`/map`): gmapping lidar SLAM from `/scan` and odometry. It also corrects
  the odometry's drift, publishing TF `map -> odom`.
- **Moisture map** (`/moisture_map`): `moisture_mapper.py` places each `/moisture` probe
  reading where SLAM thinks the probe is, averaging readings into 10 cm cells.

It only needs `/scan`, `/moisture` (`sensor_msgs/RelativeHumidity`, header frame = probe
tip) and TF `odom -> base_footprint -> sensor frames`, so it runs the same on the simulator or
the real robot. `crawlspace.launch` in `spiderbot_gazebo` starts it by default (`mapping:=false`
turns it off); on its own it's:

```bash
roslaunch spiderbot_mapping mapping.launch
```

## Viewing in Foxglove

In the 3D panel, turn on:

- `/map`, with **Color mode** set to **Map** (white free, black walls, grey unknown).
- `/moisture_map`, with **Color mode** set to **Costmap** (blue dry to red wet; unmeasured
  cells are transparent). It's drawn 12 cm above the floor plan so terrain bumps don't hide it.

## Saving

```bash
rosservice call /save_maps
```

Or use **Save maps** on the phone page. Each save makes a new folder
`~/spiderbot_maps/<date>_<time>/` (on the machine running mapping) with:

| File | Contents |
|---|---|
| `map.pgm`, `map.yaml` | Floor plan, map_server format (loadable with `map_server`) |
| `moisture.pgm`, `moisture.yaml` | Moisture map, map_server `raw` mode: pixel = moisture 1 (dry) to 98 (saturated), 0 = not measured |

Both are in the `map` frame, whose origin is where the robot started.

## Settings

gmapping's settings are in [config/gmapping.yaml](config/gmapping.yaml), tuned for a small
space and a slow robot: 5 cm cells, map updates every 2 s, and a new scan used every 10 cm
or 6 degrees of movement. The moisture mapper's are private params: `~resolution` (0.1 m),
`~radius` each reading covers (0.12 m), `~size` of the map (20 m square), `~save_dir`.

## How well it works (simulation test)

Driving a 20 m loop around the crawlspace:

- **Odometry alone** ended 0.36 m from where the robot really was (worst 0.54 m).
- **SLAM** ended 0.06 m off (worst 0.17 m).
- **Floor plan:** 98% of the wall cells were within 5 cm of a real wall, pier or post.
- **Moisture map:** cells were within 1.8 points of the ground truth on average (1-98 scale).

Mapping is noticeably better when you turn slowly; fast spins give gmapping less overlap
between scans.
