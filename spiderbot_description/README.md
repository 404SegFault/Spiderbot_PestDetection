# spiderbot_description

URDF model of the Spiderbot hexapod: an 18-DOF, primitive-geometry stand-in for the
Hiwonder JetHexa, with the Ultimate kit's lidar and depth camera. The model (dimensions,
frame names, joint sign conventions) is documented at the top of
[urdf/spiderbot.urdf.xacro](urdf/spiderbot.urdf.xacro). All dimensions and masses are
estimates until we have the real robot.

This package is the model only. To run it in Gazebo with working sensors, use
`spiderbot_gazebo` (see its README).

## Build

```bash
cd ~/spiderbot_ws
rosdep update --include-eol-distros               # Melodic is EOL; plain `rosdep update` skips it
rosdep install --from-paths src --ignore-src -y   # pulls joint_state_publisher_gui if missing
catkin_make
source devel/setup.bash
```

## Run and view in Foxglove

```bash
# SSH session 1
roslaunch spiderbot_description display.launch

# SSH session 2
roslaunch rosbridge_server rosbridge_websocket.launch
```

In Foxglove (connected to `ws://<vm-ip>:9090`), open a **3D** panel and:

1. Under **Topics**, turn on `/robot_description` (eye icon).
2. Set **Display frame** to `base_link`.

Foxglove only reads the `robot_description` *parameter* over a native ROS connection, not
over rosbridge. That's why the launch file also republishes it on the latched
`/robot_description` topic, which the 3D panel picks up by itself.

## Moving the joints

`display.launch` picks the joint source from `$DISPLAY`. You can override it with
`gui:=true` or `gui:=false`.

| How you SSH in | What runs | What you see |
|---|---|---|
| `ssh user@vm` (no display) | `joint_sweep.py` | every joint sweeps continuously, in a ripple around the body |
| `ssh -Y user@vm` + XQuartz | `joint_state_publisher_gui` | a slider window on your Mac, one slider per joint |

To get the sliders, do this once on the Mac:

1. `brew install --cask xquartz`, then log out and back in.
2. Open XQuartz, then `ssh -Y user@<vm-ip>`. `echo $DISPLAY` on the VM should print something like `localhost:10.0`.
3. If `$DISPLAY` is empty, enable `X11Forwarding yes` in the VM's `/etc/ssh/sshd_config` and `sudo apt install xauth`.
4. If the slider window opens blank, `export QT_X11_NO_MITSHM=1` before `roslaunch`.

The VM's own (broken) console display isn't involved either way.

## Sanity checks

```bash
rostopic echo -n1 /joint_states/name             # 18 joints
rostopic echo -n1 /robot_description | head -c 300
rosrun tf tf_echo base_link front_left_foot_link
```

At the zero pose (sliders centred), every foot sits 0.125 m below `base_link`, on the
ground plane of `base_footprint`. For example, `front_left_foot_link` is at about
(0.185, 0.160, -0.125) from `base_link`.

## Sensors and Gazebo settings

| Frame | What it is |
|---|---|
| `base_footprint` | Root frame, on the ground under the body |
| `laser_link` | YDLIDAR G4 scan centre on top of the body, about 19 cm above the ground |
| `camera_link` | Depth camera on the front edge, x forward, tilted 10 deg down (`camera_tilt` in the xacro) |
| `camera_optical_frame` | Same point in the optical convention (z forward) that the images use |

[urdf/spiderbot.gazebo.xacro](urdf/spiderbot.gazebo.xacro) holds everything that only
Gazebo uses: the sensor plugins, Gazebo colours, and kinematic links. Kinematic means
gravity and contacts never move the robot; the walking node places it each tick instead.

For the same reason the model has no collision shapes by default: nothing needs them, and
in Gazebo 9.0 the moving robot's own collision shapes can show up in its lidar scan. Pass
`collisions:=true` to xacro to add them (matching the visuals) if the robot is ever
simulated with real physics.
