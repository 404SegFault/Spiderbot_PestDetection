# spiderbot_description

URDF model of the Spiderbot hexapod: an 18-DOF, primitive-geometry stand-in for the
Hiwonder JetHexa. The model (dimensions, frame names, joint sign conventions) is
documented at the top of [urdf/spiderbot.urdf.xacro](urdf/spiderbot.urdf.xacro).
All dimensions are estimates until we have the real robot.

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

1. **Custom layers → + → URDF**, set **Source** to **Topic** and the topic to `/robot_description`.
2. Set **Display frame** to `base_link`.

Foxglove can't auto-load the URDF here. It only reads the `robot_description` *parameter*
over a native ROS connection, not over rosbridge. That's why the launch file also
republishes it on the latched `/robot_description` topic.

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

At the zero pose (sliders centred), every foot sits 0.125 m below `base_link`. For example,
`front_left_foot_link` is at about (0.185, 0.160, -0.125).
