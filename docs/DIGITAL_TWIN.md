# Vision 60 digital twin

`vision60_simulation` is a Gazebo Fortress digital twin for repeatable
software-in-the-loop testing. It uses the supported ROS 2 Humble `ros_gz`
integration and avoids Gazebo Classic.

The first model is a 51 kg collision-envelope sensor platform using published
Vision 60 overall dimensions. It intentionally does not claim manufacturer
joint or motor fidelity. The simulated disaster world contains walls, rubble,
and an inclined slab. Gazebo produces IMU, 3D LiDAR, camera, camera calibration,
and simulation-clock topics through the same ROS contracts used by autonomy.
The model also accepts the safety-filtered `/cmd_vel_safe` contract and
publishes `/vision60/odom` plus `map -> odom -> base_link` transforms.

Build the dedicated image once:

```bash
docker build -f docker/Dockerfile.simulation \
  -t vision60-simulation:humble-fortress .
```

Run the headless physics and sensor test:

```bash
./scripts/test_vision60_digital_twin.sh
```

Run the motion, stop, and visual-evidence test:

```bash
./scripts/test_vision60_digital_twin_motion.sh
```

This commands 0.25 m/s for four seconds, stops for two seconds, checks measured
travel, stop drift, height, LiDAR, RGB camera, calibration, and IMU, then writes
`artifacts/digital_twin_motion_test/digital_twin_test_result.png` and
`test_report.json`, plus the full drive recording as
`digital_twin_drive_test.mp4`.

Run the end-to-end communication-loss recovery test:

```bash
./scripts/test_vision60_digital_twin_recovery.sh
```

It connects the production route recorder, recovery state machine, alternate
channel manager, mission synchronizer, return follower, and reentry follower to
Gazebo. The resulting video and JSON report are written below
`artifacts/digital_twin_recovery_test`.

Run the mapped Nav2 and live-LiDAR obstacle-avoidance test:

```bash
./scripts/test_vision60_digital_twin_obstacle_avoidance.sh
```

NavFn plans around the accumulated obstacle map, DWB tracks the detour using
the live LiDAR voxel layer, and Collision Monitor remains the final emergency
gate. The script measures obstacle and wall clearance and saves an MP4 below
`artifacts/digital_twin_obstacle_avoidance`.

Run surprise moving-obstacle injection and replanning:

```bash
./scripts/test_vision60_dynamic_obstacle_avoidance.sh
```

Run autonomous Frontier exploration with communication recovery:

```bash
./scripts/test_vision60_frontier_exploration.sh
```

The test reveals an unknown occupancy map, selects Nav2 goals with the pinned
`m-explore-ros2` engine, pauses on link loss, returns on the recorded route,
switches channel, synchronizes, performs reentry, and resumes exploration.
MP4, PNG, JSON metrics and logs are saved below
`artifacts/digital_twin_frontier_exploration`.

Run simulated victim/hazard detection and camera-LiDAR 3D localization:

```bash
./scripts/test_vision60_perception.sh
```

The test publishes standard `vision_msgs` 2D/3D detections and confirmed
`MissionEvent` candidates. The color detector is explicitly simulation-only;
the LiDAR association, map transform, event contract, and visualization are the
production-shaped interfaces. The current deterministic world passes with
0.26 m victim and 0.31 m hazard position error. MP4, PNG, JSON and logs are
saved below `artifacts/digital_twin_perception`.

Next, replace the fixed deployed-leg collision envelope with the manufacturer
joint model and walking SDK adapter when those licensed assets are available.
