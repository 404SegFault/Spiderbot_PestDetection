# JetHexa Spiderbot — Pest Detection PoC (Simulation) — Implementation Plan

## 1. Context (read this before doing anything)

- **Hardware**: Hiwonder JetHexa hexapod (Jetson Nano onboard compute). Stock OS/stack: **Ubuntu 18.04 LTS + ROS1 Melodic**. This is NOT ROS2 — do not use `colcon`/`ament`/`rclpy`.
- **Goal of this PoC**: prove out a pest-detection pipeline (camera → OpenCV/vision model → detection event → some robot reaction, e.g. stop/flag/point) **entirely in simulation**, before anything touches the physical hexapod.
- **Dev machine**: macOS (Apple Silicon or Intel — confirm which). ROS1 Melodic + Gazebo do not run natively/reliably on macOS, so all ROS/Gazebo work happens **inside a Linux VM** (Ubuntu 18.04, e.g. via UTM/Parallels). Visualization/monitoring happens **from the Mac host** via **Foxglove Studio**, connecting to the VM over the network — not via RViz.
- **Existing material**: We have a Hiwonder-provided tutorial (Python/OpenCV/ROS) with what's likely a `jethexa_*` set of ROS packages, including Gazebo world/launch files and a Gazebo model/URDF of the hexapod. **Do not rebuild the hexapod model or base gait control from scratch.** First priority is locating and reusing whatever `jethexa_description`, `jethexa_gazebo`, `jethexa_bringup`-style packages already exist in the provided material, and building the pest-detection layer on top of them.
- **Operator background**: comfortable with ROS1 concepts (catkin, nodes/topics/services), has prior Gazebo + RViz + TurtleBot SLAM/camera-calibration experience from university, so no need to over-explain ROS basics — focus instructions on JetHexa/Gazebo/Foxglove specifics and any Melodic-era quirks.

## 2. Success criteria for this PoC

1. Gazebo (9.x, matching Melodic) launches with the JetHexa model in a world containing at least one static "pest" object (e.g. a colored cube/sprite standing in for a bug).
2. A ROS1 node subscribes to the robot's simulated camera topic and runs an OpenCV-based detector (start simple — colour/contour threshold is fine for PoC; leave a clean seam to swap in a trained model later) that publishes a detection message (bounding box + class + confidence) on a new topic, e.g. `/pest_detection/detections`.
3. On a positive detection, some simple, observable robot reaction fires — for the PoC this can be as small as publishing to `/leg/cmd` to pause gait, or lighting up a marker in the sim — the point is proving the vision→action wiring, not sophisticated behavior.
4. All of the above is visible live from the Mac via **Foxglove Studio**, connected to a bridge (`rosbridge_server` or `foxglove_bridge`) running inside the VM: camera feed, detection overlay/markers, and relevant topic list.
5. The whole thing is reproducible from a clean VM with a documented `catkin_make`/`catkin build` + `roslaunch` sequence.

## 3. Environment setup (do this first, verify each step)

1. Confirm VM: Ubuntu 18.04 LTS, ROS1 Melodic desktop-full (`ros-melodic-desktop-full`), Gazebo 9 (installed via the ROS repo, **not** the OSRF repo, to keep versions aligned with Melodic).
2. Set up a catkin workspace (e.g. `~/jethexa_ws/src`), and unpack/clone the Hiwonder tutorial's ROS packages into `src/`. Run `rosdep install` and `catkin_make` to confirm the stock packages build and their Gazebo demo launches before adding anything new.
3. Install `rosbridge_server` (`ros-melodic-rosbridge-server`) or `foxglove_bridge` (may need to build from source on Melodic — check compatibility first, fall back to `rosbridge_server` if it's not available for Melodic) inside the VM.
4. On the Mac: install Foxglove Studio (desktop app). Confirm VM networking mode (bridged/host-only) so the Mac can reach the VM's IP on the bridge port (default `9090` for rosbridge).
5. Smoke test: launch the stock JetHexa Gazebo demo, start the bridge, connect Foxglove from the Mac, confirm you can see `/clock`, TF, and the simulated camera topic live. **Do not proceed to pest-detection work until this smoke test passes.**

## 4. Architecture (PoC scope)

```
[Gazebo sim: JetHexa model + world w/ pest object]
        |
        | /jethexa/camera/image_raw (sensor_msgs/Image)
        v
[pest_detector_node]  (new, Python, OpenCV)
        |
        | /pest_detection/detections (custom msg or vision_msgs/Detection2DArray)
        | /pest_detection/annotated_image (sensor_msgs/Image, for Foxglove viewing)
        v
[reaction_node]  (new, minimal)
        |
        | /leg/cmd  or  /jethexa/... (whatever the stock control topic is)
        v
[Gazebo: hexapod pauses / reacts]

        (bridge, sitting alongside, not in the data path)
[rosbridge_server / foxglove_bridge] <---> Foxglove Studio (on Mac, over network)
```

New packages/nodes to create (keep separate from the vendor packages so upstream tutorial updates don't conflict):

- `pest_detection_poc/` (new catkin package)
  - `scripts/pest_detector_node.py`
  - `scripts/reaction_node.py`
  - `msg/Detection.msg` (skip if `vision_msgs` covers it — prefer reusing a standard message type over inventing one)
  - `launch/pest_detection_sim.launch` — starts Gazebo (via the vendor's world), the bridge, and the two new nodes together

## 5. Implementation phases

**Phase 0 — Environment smoke test** (Section 3 above). Gate before anything else.

**Phase 1 — Detection node, no reaction yet**
- Subscribe to the sim camera topic, confirm frames arrive (log shape/rate).
- Implement a deliberately simple detector first (e.g. HSV colour threshold + contour bounding box for a bright-coloured "pest" prop) — the goal is proving the pipeline, not detection accuracy.
- Publish detections + an annotated debug image.
- Verify in Foxglove: camera feed + annotated image + detection topic all visible and sane.

**Phase 2 — Reaction node**
- Subscribe to detections, on a positive hit publish a simple command (pause/stop gait, or a Gazebo marker/light toggle — whichever is less friction given the stock control interface).
- Verify the reaction is visible in Gazebo and reflected in Foxglove.

**Phase 3 — Launch file + repeatability**
- Single `roslaunch` that brings up world + bridge + both nodes.
- Document exact commands in a `README.md` in the new package, including the Foxglove connection URL/port.

**Phase 4 (stretch, only if time permits)** — swap the placeholder detector for a lightweight trained model (note: keep this behind a clean interface from Phase 1 so it's a drop-in swap, not a rewrite).

## 6. Explicit constraints / things to avoid

- No ROS2 packages, `colcon`, or `rclpy` — this is a ROS1 Melodic project end to end for now.
- Don't modify the vendor-provided JetHexa packages in place; build alongside them in a separate package so tutorial updates/re-syncs don't get clobbered.
- Don't invest in RViz configs — Foxglove is the visualization target for this PoC.
- Keep the detector swappable (Phase 4) rather than hard-coding the colour-threshold approach throughout the pipeline.

## 7. Open questions for whoever/whatever implements this to flag back

- Exact topic names/message types the vendor packages use for the camera and for gait/leg commands (inspect the tutorial package's launch/URDF files rather than guessing).
- Whether `foxglove_bridge` has a working Melodic build, or whether `rosbridge_server` is the only realistic option — decide during Phase 0 and note it in the README.
- What object to use as the simulated "pest" (simplest: a coloured primitive spawned in the Gazebo world) — confirm before Phase 1.
