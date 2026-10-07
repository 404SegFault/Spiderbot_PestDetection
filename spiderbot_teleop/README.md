# spiderbot_teleop

Drive the Spiderbot from a phone (or any browser): a web page with drive controls and
the robot's live camera. It publishes `geometry_msgs/Twist` on `/cmd_vel`, so it drives the
simulated robot now and the real one later.

```bash
roslaunch spiderbot_teleop web_teleop.launch     # rosbridge (port 9090) + the page (port 8080)
```

Then open `http://<vm-ip>:8080/` on the phone. This launch also starts rosbridge, so
Foxglove connects to the same `ws://<vm-ip>:9090` as before; don't run rosbridge
separately as well.

## Controls

| Control | Does |
|---|---|
| ▲ / ▼ | Walk forward / back (hold) |
| ⟲ / ⟳ | Turn left / right on the spot (hold) |
| ◀ side / side ▶ | Sidestep left / right (hold) |
| Speed | Slow (0.06 m/s, 0.3 rad/s) or fast (0.15 m/s, 0.6 rad/s) |
| STOP | Stop immediately |

Buttons can be combined (forward + turn walks an arc). On a computer the keyboard works
too: arrows or WASD to drive, Q/E to sidestep, F for speed, Space to stop.

The page sends commands 10 times a second while a button is held, and stop commands when
it's released. The robot also stops by itself if commands stop arriving for 0.4 s, for
example if the phone locks or the Wi-Fi drops.

## How it works

`www/index.html` is a single self-contained page with no libraries, so it works on a
network without internet. It talks to rosbridge directly using rosbridge's JSON protocol
over a WebSocket. `scripts/web_server.py` only serves the page. The camera view is
`/camera/rgb/image_raw/compressed`, cut to about 6 frames per second to save phone data.
Rosbridge is assumed to be on the same host as the page, port 9090; add
`?bridge=ws://host:port` to the page URL to point it elsewhere.
