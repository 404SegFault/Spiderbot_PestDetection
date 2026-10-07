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

**Session 2: rosbridge, the WebSocket server Foxglove connects to**

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

It's ready when it prints `Rosbridge WebSocket server started at ws://0.0.0.0:9090`.

A separate `roscore` isn't needed: `roslaunch` starts one if none is running. If you do
run your own in a third session, start it first and leave it running.

### Launch options

Add these to the `roslaunch spiderbot_gazebo crawlspace.launch` line:

| Option | Effect |
|---|---|
| `gazebo:=false` | Skip Gazebo; Foxglove still shows the world markers, moisture map and robot model. No sensors. Doesn't need `xvfb-run`. |
| `robot:=false` | World only, no robot. |
| `spawn_x:=... spawn_y:=... spawn_yaw:=...` | Start the robot somewhere else (default `-3.3 0 0`, by the access opening). |

### Robot model on its own (no Gazebo)

To check the robot model by itself:

```bash
roslaunch spiderbot_description display.launch
```

Over a plain `ssh` session the legs sweep automatically. For slider controls instead,
install XQuartz on the Mac (`brew install --cask xquartz`, then log out and back in),
open it, and connect with `ssh -Y <user>@<vm-ip>`. The slider window then opens on the Mac.

## 4. View in Foxglove (Mac)

1. Open Foxglove → **Open connection** → **Rosbridge** → `ws://<vm-ip>:9090`.
2. Add a **3D** panel. In its settings:
   - **Frame → Display frame:** `world` (or `base_link` for `display.launch`)
   - **Topics:** turn on (eye icon):
     - `/world_markers`: the crawlspace. Takes a few seconds to appear (about 3.4 MB).
     - `/robot_description`: the robot model.
     - `/scan`: lidar hits. In its settings set **Color mode** to **Flat** (pick a bright
       colour) and **Point size** to about 4.
     - `/moisture_truth` (optional): ground-truth moisture. Set **Color mode** to
       **Costmap** (blue = dry, red = wet).
   - **Transforms:** switch **Labels** off to hide frame names; switch **Enable
     preloading** off to avoid the "At most ... transforms" error on long sessions.
3. Add **Image** panels:
   - `/camera/rgb/image_raw/compressed`: the robot's colour camera.
   - `/camera/depth/image_raw_throttle`: depth, once a second.

Don't open `/camera/rgb/image_raw`, `/camera/depth/image_raw` or `/camera/depth/points`
in Foxglove. At 1 MB or more per message they swamp rosbridge.

If `ws://<vm-ip>:9090` won't connect, tunnel the port through SSH instead. Open the rosbridge
session with `ssh -L 9090:localhost:9090 <user>@<vm-ip>`, then connect Foxglove to
`ws://localhost:9090`.

## 5. Check what's running (any extra session)

```bash
rosnode list
rostopic list
rostopic hz /scan                          # about 7 Hz
rostopic hz /camera/rgb/image_raw          # about 10 Hz (needs xvfb-run)
rostopic echo -n1 /joint_states/name       # 18 joint names
rosrun tf tf_echo world base_footprint     # where the robot is
gz stats                                   # Gazebo speed: Factor near 1.00 means real time
```

| Topic | What it is |
|---|---|
| `/world_markers` | Crawlspace as Foxglove markers, one namespace per Gazebo model |
| `/moisture_truth` | Ground-truth soil moisture map (answer key) |
| `/robot_description` | Robot model (URDF) as a topic, for Foxglove |
| `/joint_states` | Leg joint angles |
| `/tf`, `/tf_static` | Coordinate frames |
| `/scan` | Lidar, 360 deg, 7 Hz |
| `/camera/rgb/image_raw` (`/compressed`) | Colour camera, 640x480, 10 Hz |
| `/camera/depth/image_raw`, `/camera/depth/points` | Depth image (metres) and point cloud, 0.6-8 m |
| `/camera/depth/image_raw_throttle` | 1 Hz depth copy for Foxglove |

## 6. Shut down

Press **Ctrl-C** in each session (Gazebo can take ~15 seconds to stop). Then make
sure nothing was left behind, which would stop the next launch:

```bash
killall -9 gzserver gzclient 2>/dev/null
rosclean purge -y     # deletes old ROS logs; run when it warns the log folder is over 1 GB
```

## 7. Troubleshooting

| Symptom | Fix |
|---|---|
| `[gazebo-1] process has died ... exit code 255` | An old gzserver is still running. `killall -9 gzserver gzclient`, then launch again. |
| `rosdep`: `Cannot locate rosdep definition for [...]` | `rosdep update --include-eol-distros`, then rerun `rosdep install`. |
| Foxglove: "Check that the rosbridge WebSocket server at ws://localhost:9090 is reachable" | Use `ws://<vm-ip>:9090`, or the SSH tunnel in section 4. Check session 2 is still running. |
| Foxglove: only coordinate axes, no robot | Turn on `/robot_description` under **Topics**. Don't add it as a custom URDF layer: over rosbridge that gives "Invalid topic". |
| Foxglove: "Failed to process all transforms on topic /tf" | 3D panel → **Transforms** → switch off **Enable preloading**, then reconnect. |
| Foxglove, on `/scan`: "N Infinity invalid values detected" | Expected. Beams that hit nothing (for example out through the access opening) are reported as infinity, the ROS standard for "no return", and mapping uses them as free space. Set `/scan`'s **Color mode** to **Flat** to clear the warning. |
| No camera images | Gazebo wasn't started under `xvfb-run`. Restart session 1 with the command in section 3. |
| `ALSA lib ... error` lines from Gazebo | Harmless: the VM has no sound card. |
| The first camera frame you grab looks out of date | Normal: Gazebo pauses cameras with no subscribers, so the first frame after subscribing is old. Later frames are live. |

## 8. Changing the crawlspace

The world, its meshes, the moisture map and the answer key are generated. Edit the layout
in `spiderbot_gazebo/scripts/generate_crawlspace.py`, then regenerate (works on the VM or the Mac,
Python 2 or 3) and commit the results:

```bash
cd ~/spiderbot_ws/src/Spiderbot_PestDetection/spiderbot_gazebo
python scripts/generate_crawlspace.py
```
