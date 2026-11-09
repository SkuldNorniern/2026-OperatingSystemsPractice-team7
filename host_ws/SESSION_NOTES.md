# Handoff — 2026-11-07

Branch: `rust_base`. Changes are built locally and remain uncommitted. Existing
edits in `Reference/tb4_timesync/` were preserved.

## Restart

From repository root on this host:

```bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER
bash host_ws/scripts/run_demo.sh
```

This undocks/resets the robot and starts autonomous navigation. For stationary
recorded diagnostics instead:

```bash
bash host_ws/scripts/run_demo.sh --no-prepare explorer:=none mission:=false
```

Wait for `Managed nodes are active` and the explorer's goal log. Use
`explorer:=none` when sending manual RViz goals. Do not run two launches;
the wrapper now holds a lock. In zsh source ROS `setup.zsh`, not `setup.bash`.

## Changes

- Configured Ethernet `team7-tb4`: host `192.168.185.10`, Pi `192.168.185.3`.
  The wrapper selects wired Fast DDS plus shared memory. Wi-Fi internet remains.
- Host NTP serves the wired subnet; Pi follows `192.168.185.10` and serves Create3.
- ArUco dictionary corrected to `6X6_250`; IDs 10 and 11 verified earlier.
  Detector uses compressed camera input. RViz/ArUco exclude inherited Nix graphics
  libraries that prevented RViz opening.
- Reachable, segmented frontier goals use path distance, heading penalty and
  stand-off from unknown space. Cancellation finishes before another goal starts.
- Regulated Pure Pursuit replaces MPPI; forward limit 0.18 m/s, turn limit
  0.5 rad/s. Final `velocity_guard` blocks reverse and stale commands.
- Recovery clears/waits without backing up or automatic recovery spins.
- Launch starts only required Nav2 servers; asynchronous SLAM avoids a scan backlog.
- Automatic MCAP recording starts before undock/reset. Each run gets sensor data,
  TF, goals/actions, velocity commands, logs, settings, source diff and automatic
  `summary.json` under `~/team7_runs/<timestamp>/`.

## Validation and remaining limits

- Both ROS packages built. 25 algorithm tests and 3 guard tests passed.
- Isolated ROS integration verified reverse rejection, 0.18 m/s limiting and
  stopping after command timeout; no test velocity was sent to the robot.
- Single stationary Ethernet run activated all Nav2 servers in about 21 seconds.
  ComputePathToPose returned a valid 50-pose path without sending a motion goal.
- Live median ages: camera/detections 28 ms, odometry 39 ms, map 129 ms.
  The final recorder reported 3 transport losses; bag metadata and summary finalized.
- Preserved final validation: `~/team7_runs/validation_20261107_065010_wired/`.
  Earlier user runs are also under `~/team7_runs/`.
- Full autonomous driving quality still needs a recorded trial. Real obstacles
  still stop navigation; collision checking remains enabled.
- Third-party `aruco_tracker_autostart` sometimes exits with SIGSEGV during SIGINT
  shutdown; it ran and streamed during validation. Investigate its cleanup separately.
- Host project launches and recorders were stopped at handoff. Factory Pi drivers
  and chrony remain running.
