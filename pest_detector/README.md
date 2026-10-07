# pest_detector

Finds pest signs in the robot's camera, places them on the SLAM map, rates their risk
using the measured soil moisture, and writes inspection reports. `crawlspace.launch` in
`spiderbot_gazebo` starts it by default (`detection:=false` turns it off). On its own:

```bash
roslaunch pest_detector detection.launch
```

It needs the colour and depth camera, TF to the `map` frame (so mapping must be running),
and `/moisture_map` for risk ratings.

## What it detects

| Sign | How it's recognised |
|---|---|
| Termite mud tube | Thin, vertical, mud-brown line darker than the wall or pier around it, between the ground and 0.65 m up |
| Rodent droppings | Cluster of 4+ small near-black specks, each clearly darker than the dirt around it |
| Rodent burrow | Near-black ellipse on the ground, 2-7 times wider than tall |

The camera is tilted down for the ground, so **damaged wood on the joists overhead isn't in
view** and isn't detected; the report lists it as "not detectable".

The rules are deliberately simple and tuned on the simulated crawlspace
(`src/pest_detector/rules.py`). They tell the decoys apart by shape and colour:
- a mud smear is a blob, not a thin line;
- pebbles are pale, not dark;
- an old water stain is on the joists, out of view.

On real crawlspace images, a trained model would replace them. Anything with the same
`detect(bgr_image) -> [Detection(label, score, box, point)]` interface plugs into
`pest_detector_node.py` unchanged.

## Pipeline

1. **`pest_detector_node.py`** runs the detector about 3 times a second and locates each
   sign in 3D from the depth image. The depth camera can't measure closer than 0.6 m, so
   for nearer signs on the ground it uses where the pixel's ray meets the ground. Signs at
   the wrong height are dropped.
2. **`sign_mapper.py`** places detections on the map and merges repeat sightings of the same
   sign. It **confirms** a sign once it's been seen 3 times, which filters out one-off false
   detections, then rates it:

| Sign | Risk |
|---|---|
| Termite mud tube | **HIGH** if the soil within 0.6 m is damp (moisture 45+), otherwise MEDIUM. Termites need moisture. |
| Rodent burrow | **HIGH**: an active entry point |
| Rodent droppings | MEDIUM |

Soil moisture comes from the probe, so it's only known where the robot has walked. Walk the
probe up to a sign to rate it properly; until then the report says "not measured".

## Topics and services

| Name | Type | |
|---|---|---|
| `/pest_detection/image/compressed` | `sensor_msgs/CompressedImage` | Camera view with detections drawn on it (Foxglove Image panel or the phone page) |
| `/pest_detection/detections` | `vision_msgs/Detection2DArray` | Per-frame detections. `results[0].id` = 1 termite_mud_tube, 2 rodent_droppings, 3 rodent_burrow; `.pose` = 3D position in the camera frame |
| `/pest_detection/signs` | `visualization_msgs/MarkerArray` | Confirmed signs on the map: a disc (red HIGH, orange MEDIUM) and a label each |
| `/pest_detection/save_report` | `std_srvs/Trigger` | Save the maps and the report (also the phone page's **Save report** button) |

## The report

`rosservice call /pest_detection/save_report` (or **Save report** on the phone) saves a folder
`~/spiderbot_maps/<date>_<time>/` with the floor plan, moisture map, `signs.json` and
`report.md`:

- every confirmed sign, with its risk, position, nearby soil moisture and number of sightings;
- a soil moisture summary;
- in simulation, a **score against the world's answer key**: signs found and missed, false
  alarms, decoys ignored, and how close the robot got to each sign.

In the simulation test (a 12-stop inspection drive), it found all 10 detectable signs, with
no false alarms and all 3 decoys ignored.
