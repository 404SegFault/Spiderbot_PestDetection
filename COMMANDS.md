# Running the Spiderbot simulation

Copy-paste commands for bringing the simulation up on the Ubuntu 18.04 / ROS Melodic VM
and viewing it in Foxglove on a Mac. Every "session" below is a separate SSH terminal
from the Mac into the VM.

Throughout: `<user>` is your VM login, `<vm-ip>` is the VM's address (`hostname -I` on
the VM prints it), and the workspace is `~/spiderbot_ws` with this repo checked out at
`~/spiderbot_ws/src/Spiderbot_PestDetection`.

## 1. One-time setup (VM)

```bash
# Tools the simulation needs that desktop-full doesn't include
sudo apt install xvfb ros-melodic-rosbridge-server

# Melodic is end-of-life, so a plain `rosdep update` skips it and every ROS
# dependency fails to resolve. Always use --include-eol-distros.
rosdep update --include-eol-distros

# Source the workspace in every new terminal automatically
echo "source ~/spiderbot_ws/devel/setup.bash" >> ~/.bashrc
```

## 2. Get the latest code (every time something new is pushed)

On the Mac, commit and push as usual. Then on the VM:

```bash
cd ~/spiderbot_ws/src/Spiderbot_PestDetection && git pull
cd ~/spiderbot_ws
rosdep install --from-paths src --ignore-src -y   # installs any new dependencies
catkin_make
source devel/setup.bash
```

## 3. Start the simulation

Open two SSH sessions from the Mac:

```bash
ssh <user>@<vm-ip>
```

**Session 1: Gazebo crawlspace with the robot and its sensors**

```bash
killall -9 gzserver gzclient 2>/dev/null   # clear any Gazebo left over from an earlier run
xvfb-run -a -s "-screen 0 1280x1024x24" roslaunch spiderbot_gazebo crawlspace.launch
```

`xvfb-run` gives Gazebo a virtual display, which the robot's camera needs to render.
Without it everything else still runs (including the lidar), but there are no camera
images.

The first time the camera is viewed after launching, it can take 5-10 seconds to start
while Gazebo sets up rendering; after that it's quick.

**Session 2: rosbridge (for Foxglove) and the phone teleop page**

```bash
roslaunch spiderbot_teleop web_teleop.launch
```

It's ready when it prints `Rosbridge WebSocket server started at ws://0.0.0.0:9090`. This
starts rosbridge, so don't also run `rosbridge_server` on its own (they'd clash over port 9090).

A separate `roscore` isn't needed: `roslaunch` starts one if none is running. If you do
run your own in a third session, start it first and leave it running.

### Launch options

Add these to the `roslaunch spiderbot_gazebo crawlspace.launch` line:

| Option | Effect |
|---|---|
| `gazebo:=false` | Skip Gazebo; Foxglove still shows the world markers, moisture map and robot model. No sensors. Doesn't need `xvfb-run`. |
| `robot:=false` | World only, no robot. |
| `mapping:=false` | Don't build maps (no gmapping or moisture mapper). Mapping needs Gazebo's lidar, so it does nothing with `gazebo:=false`. |
| `detection:=false` | Don't run pest detection. Detection needs mapping (it places signs on the map). |
| `spawn_x:=... spawn_y:=... spawn_yaw:=...` | Start the robot somewhere else (default `-3.3 0 0`, by the access opening). |

### Robot model on its own (no Gazebo)

To check the robot model by itself:

```bash
roslaunch spiderbot_description display.launch
```

Over a plain `ssh` session the legs sweep automatically. For slider controls instead,
install XQuartz on the Mac (`brew install --cask xquartz`, then log out and back in),
open it, and connect with `ssh -Y <user>@<vm-ip>`. The slider window then opens on the Mac.

## 4. Drive the robot

Velocity commands go on `/cmd_vel` (forward, sideways, turn). The robot walks over the
bumps, stops by itself 0.4 s after commands stop, and won't walk into anything its lidar
sees within 0.3 m in its direction of travel; back up or turn away to get going again.

**From a phone:** open `http://<vm-ip>:8080/` in the phone's browser. The phone must be on
the same network as the VM and able to reach its address. Hold the arrows to walk and turn,
**side** to sidestep, **Speed** for slow/fast, **STOP** to stop. Landscape works best.

If the phone can't reach the VM directly (for example the VM is on the Mac's private
"shared" network), forward both ports through the Mac instead. Open one SSH session like
this and keep it open:

```bash
ssh -L 0.0.0.0:8080:localhost:8080 -L 0.0.0.0:9090:localhost:9090 <user>@<vm-ip>
```

Then open `http://<mac-ip>:8080/` on the phone; `ipconfig getifaddr en0` on the Mac
prints `<mac-ip>`. Allow incoming connections if macOS asks.

**From a computer:** the same page works in any browser, with the keyboard too (arrows or
WASD, Q/E to sidestep, Space to stop). Or use Foxglove's **Teleop** panel on topic `/cmd_vel`
with its publish rate set to 10 Hz; at lower rates the robot stops between messages.

## 5. Inspect: maps, moisture and pest signs

Mapping and pest detection run automatically with the simulation. Drive around and:

- **gmapping** builds the floor plan from the lidar;
- the **moisture probe** under the robot's nose fills in a moisture map where it walks;
- the **detector** looks for termite mud tubes, rodent droppings and burrows in the camera.
  A sign is confirmed (and appears on the map) once it's been seen 3 times.

Tips for a good inspection run:

- **Drive slowly and turn gently.** Fast spins smear the floor plan.
- **Stop and look.** Face walls and piers from about 1-1.5 m for termite tubes, and the dirt
  1 m or less ahead for droppings and burrows.
- **Walk the probe up to what you find.** A termite tube's risk depends on the soil moisture
  right next to it; until the probe has been within 0.6 m, the report says "not measured".

The phone page's camera shows the detections boxed (tap the picture for the plain camera),
and the header counts confirmed signs.

**Save the report:** tap **Save report** on the phone page, or run:

```bash
rosservice call /pest_detection/save_report
```

Each save makes a new folder `~/spiderbot_maps/<date>_<time>/` on the VM containing:
- `report.md`: findings with risk ratings, a moisture summary, and in simulation a score
  against the world's answer key;
- `signs.json`;
- the floor plan and moisture map, as `map.pgm/.yaml` and `moisture.pgm/.yaml`.

`rosservice call /save_maps` saves just the maps. To copy everything to the Mac, run this on
the Mac: `scp -r <user>@<vm-ip>:spiderbot_maps ~/Desktop/`

## 6. View in Foxglove (Mac)

1. Open Foxglove → **Open connection** → **Rosbridge** → `ws://<vm-ip>:9090`.
2. Add a **3D** panel. In its settings:
   - **Frame → Display frame:** `world` (or `base_link` for `display.launch`)
   - **Topics:** turn on (eye icon):
     - `/world_markers`: the crawlspace. Takes a few seconds to appear (about 3.4 MB).
     - `/robot_description`: the robot model.
     - `/scan`: lidar hits. In its settings set **Color mode** to **Flat** (pick a bright
       colour) and **Point size** to about 4.
     - `/map`: the floor plan being built. Set **Color mode** to **Map**.
     - `/moisture_map`: the measured moisture. Set **Color mode** to **Costmap** (blue = dry,
       red = wet; unmeasured areas are transparent).
     - `/moisture_truth` (optional): the ground-truth moisture (answer key), same colours.
       Turn it off to see the measured map.
     - `/pest_detection/signs`: confirmed pest signs: a disc (red = high risk, orange =
       medium) with a label naming the sign, its risk and the soil moisture.
   - **Transforms:** switch **Labels** off to hide frame names; switch **Enable
     preloading** off to avoid the "At most ... transforms" error on long sessions.
3. Add **Image** panels:
   - `/pest_detection/image/compressed`: the camera with detections boxed.
   - `/camera/rgb/image_raw/compressed`: the plain colour camera.
   - `/camera/depth/image_raw_throttle`: depth, once a second.

Don't open `/camera/rgb/image_raw`, `/camera/depth/image_raw` or `/camera/depth/points`
in Foxglove. At 1 MB or more per message they swamp rosbridge.

If `ws://<vm-ip>:9090` won't connect, tunnel the port through SSH instead. Open the rosbridge
session with `ssh -L 9090:localhost:9090 <user>@<vm-ip>`, then connect Foxglove to
`ws://localhost:9090`.

## 7. Check what's running (any extra session)

```bash
rosnode list
rostopic list
rostopic hz /scan                          # about 7 Hz
rostopic hz /camera/rgb/image_raw          # about 10 Hz (needs xvfb-run)
rostopic echo -n1 /joint_states/name       # 18 joint names
rostopic echo -n1 /ground_truth/odom/pose/pose/position   # where the robot really is
rostopic echo -n1 /odom/pose/pose/position                # where its odometry thinks it is
gz stats                                   # Gazebo speed: Factor near 1.00 means real time
```

| Topic | What it is |
|---|---|
| `/world_markers` | Crawlspace as Foxglove markers, one namespace per Gazebo model |
| `/moisture_truth` | Ground-truth soil moisture map (answer key) |
| `/robot_description` | Robot model (URDF) as a topic, for Foxglove |
| `/cmd_vel` | Velocity commands in (forward, sideways, turn) |
| `/odom` | Odometry: where the robot thinks it is, with realistic drift. Also TF `odom -> base_footprint`. |
| `/ground_truth/odom` | Where the robot really is (simulation only, for checking and scoring) |
| `/map` | Floor plan from gmapping SLAM |
| `/moisture` | Moisture probe readings (`relative_humidity` 0.01 dry to 0.98 saturated), 5 Hz |
| `/moisture_map` | Measured moisture map: 1 (dry) to 98 (saturated), 0 = not measured |
| `/save_maps` (service) | Save the floor plan and moisture map to `~/spiderbot_maps/` |
| `/pest_detection/image/compressed` | Camera view with detections boxed, about 3 Hz |
| `/pest_detection/detections` | Per-frame detections (`vision_msgs/Detection2DArray`) |
| `/pest_detection/signs` | Confirmed pest signs on the map (markers) |
| `/pest_detection/save_report` (service) | Save the maps plus `report.md` and `signs.json` |
| `/joint_states` | Leg joint angles (walking animation) |
| `/tf`, `/tf_static` | Coordinate frames: `world -> map -> odom -> base_footprint -> base_link -> ...` (`map -> odom` comes from SLAM; `world` is the simulator's frame, fixed to where the robot started) |
| `/scan` | Lidar, 360 deg, 7 Hz |
| `/camera/rgb/image_raw` (`/compressed`) | Colour camera, 640x480, 10 Hz |
| `/camera/depth/image_raw`, `/camera/depth/points` | Depth image (metres) and point cloud, 0.6-8 m |
| `/camera/depth/image_raw_throttle` | 1 Hz depth copy for Foxglove |

## 8. Shut down

Press **Ctrl-C** in each session (Gazebo can take ~15 seconds to stop). Then make
sure nothing was left behind, which would stop the next launch:

```bash
killall -9 gzserver gzclient 2>/dev/null
rosclean purge -y     # deletes old ROS logs; run when it warns the log folder is over 1 GB
```

## 9. Troubleshooting

| Symptom | Fix |
|---|---|
| `[gazebo-1] process has died ... exit code 255` | An old gzserver is still running. `killall -9 gzserver gzclient`, then launch again. |
| `rosdep`: `Cannot locate rosdep definition for [...]` | `rosdep update --include-eol-distros`, then rerun `rosdep install`. |
| Foxglove: "Check that the rosbridge WebSocket server at ws://localhost:9090 is reachable" | Use `ws://<vm-ip>:9090`, or the SSH tunnel in section 6. Check session 2 is still running. |
| Foxglove: only coordinate axes, no robot | Turn on `/robot_description` under **Topics**. Don't add it as a custom URDF layer: over rosbridge that gives "Invalid topic". |
| Foxglove: "Failed to process all transforms on topic /tf" | 3D panel → **Transforms** → switch off **Enable preloading**, then reconnect. |
| Foxglove, on `/scan`: "N Infinity invalid values detected" | Expected. Beams that hit nothing (for example out through the access opening) are reported as infinity, the ROS standard for "no return", and mapping uses them as free space. Set `/scan`'s **Color mode** to **Flat** to clear the warning. |
| No camera images | Gazebo wasn't started under `xvfb-run`. Restart session 1 with the command in section 3. |
| `ALSA lib ... error` lines from Gazebo | Harmless: the VM has no sound card. |
| The robot won't walk forward | Its lidar sees something within 0.3 m in that direction (safety stop). Back up or turn away. |
| Phone page says "no connection to ws://...:9090" | Session 2 must be `web_teleop.launch` and still running. If the phone can't reach the VM, use the port forwarding in section 4. |
| Camera view takes a while to appear | Normal the first time after launching (5-10 s while Gazebo sets up rendering). |
| No `/map`, or the moisture map stays empty | Mapping needs the lidar, so Gazebo must be running (not `gazebo:=false`). The floor plan appears after the robot has moved about 10 cm. |
| No pest signs appear | Detection needs mapping running (it places signs on the map). Get within about 1.5 m and face the sign; it takes 3 sightings (about a second) to confirm. |
| A termite tube is only MEDIUM risk / moisture "not measured" | Walk the robot up to it (within 0.6 m) so the probe can measure the soil there; the rating updates. |
| Floor plan has doubled or smeared walls | You turned too fast for the scan matching. Drive more slowly; the map usually cleans up as you revisit an area. |
| Camera or robot motion very slow or frozen | Check Gazebo's speed with `gz stats` (Factor should be near 1.00). If it's far lower, something is starving the VM's CPU; restart the simulation. |
| The first camera frame you grab looks out of date | Normal: Gazebo pauses cameras with no subscribers, so the first frame after subscribing is old. Later frames are live. |

## 10. Changing the crawlspace

The world, its meshes, the moisture map and the answer key are generated. Edit the layout
in `spiderbot_gazebo/scripts/generate_crawlspace.py`, then regenerate (works on the VM or the Mac,
Python 2 or 3) and commit the results:

```bash
cd ~/spiderbot_ws/src/Spiderbot_PestDetection/spiderbot_gazebo
python scripts/generate_crawlspace.py
```
