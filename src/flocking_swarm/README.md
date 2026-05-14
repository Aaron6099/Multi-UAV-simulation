# flocking_swarm

9-UAV distributed flocking controller for PX4 SITL (Gazebo Garden), Python rclpy implementation. Based on MATLAB `flock_controller_improve.m` from chapter 12, with the three known bugs fixed:

1. Use **current velocity** for `f_n`, not initial `Va0`.
2. `swarm_center` index typo fixed.
3. Replaced hardcoded `/9` with actual neighbour count.

Obstacle avoidance (`f_o`) is **not** implemented in v1 — to be added later.

## Architecture

- `virtual_leader_node` (×1): publishes `/leader/state` (Float32MultiArray of length 8: t, N, E, D, vN, vE, vD, yaw) at 20 Hz, walking around a 50m × 50m rectangle.
- `flock_controller_node` (×9): one per drone. Subscribes to its own state plus the other 8 drones' positions plus the leader. Computes `u = f_f + f_n + leader_vel`. Publishes velocity setpoints to `/pxN/fmu/in/trajectory_setpoint`.
- `arming_node` (×1): waits 10 s for setpoint streams to start, then sends OFFBOARD mode + ARM to all 9 drones.

All inter-drone sensing is **distributed**: each controller subscribes to all 9 `vehicle_local_position` topics directly. There is no central state aggregator.

## Coordinate frames

- Each PX4 instance's local NED frame has **its own origin** at takeoff position.
- The flocking algorithm needs all positions in a **single world NED frame**.
- We do this by configuring each drone's `birth_position` (the offset from world NED origin to that drone's local NED origin) and adding it to the local position.
- Velocities are translation-invariant (world NED velocity == local NED velocity for non-rotating frames), so no transformation is needed for velocity.

## Build

```bash
cd ~/ros2_multi_offboard_ws/src
# Copy this package here (or extract from tarball)
cd ~/ros2_multi_offboard_ws
colcon build --packages-select flocking_swarm
source install/setup.bash
```

## Run

You need 4 terminals.

### Terminal 1: Gazebo

```bash
gz sim -r -s ~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
```

Wait until the world loads (you should see the empty Gazebo scene).

### Terminal 2: 9 PX4 SITL instances

```bash
cd ~/ros2_multi_offboard_ws/src/flocking_swarm
chmod +x start_9_px4.sh
bash start_9_px4.sh
```

This opens 9 gnome-terminal tabs (or runs in background if gnome-terminal is unavailable). Wait for each PX4 instance to print `Ready for takeoff!` before moving on. With 8-second spacing between instances, this takes about 1–2 minutes total.

If gnome-terminal isn't available, instances run in background and log to `~/px4_logs/px4_*.log`. Tail one to verify:

```bash
tail -f ~/px4_logs/px4_0.log
```

### Terminal 3: DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

You should see 9 client connections appear as each PX4 instance connects.

### Terminal 4: ROS2 nodes

```bash
cd ~/ros2_multi_offboard_ws
source install/setup.bash
ros2 launch flocking_swarm swarm_launch.py
```

The flow is:
1. All 9 `flock_controller_node` start streaming `OffboardControlMode` + `TrajectorySetpoint` (initially as hover commands).
2. After 10 seconds, `arming_node` sends mode-switch and arm commands to each drone in turn.
3. Drones lift off to `target_alt = -5 m` (5 m above takeoff).
4. Once `/leader/state` is being published and drones have own state, the flocking law takes over and they follow the leader around the rectangle.

## Verifying it works

In a separate terminal, you can check topic streams:

```bash
# Leader state
ros2 topic echo /leader/state --once

# Drone 0 setpoint stream
ros2 topic hz /fmu/in/trajectory_setpoint

# Drone 5 position
ros2 topic echo /px4_5/fmu/out/vehicle_local_position --once
```

Expected:
- `/leader/state` publishes at 20 Hz.
- All 9 `/pxN/fmu/in/trajectory_setpoint` topics publish at 20 Hz.
- All 9 `/pxN/fmu/out/vehicle_local_position` topics receive data after PX4 EKF converges.

## Tuning

Open `launch/swarm_launch.py` and edit `FLOCKING_PARAMS`:

- `desired_dist`: target inter-drone distance. Increase to spread the flock out.
- `c_pos`: position consensus gain. Higher = stronger formation tightening.
- `c_lead_pos`: leader pull. Higher = closer to leader, but at risk of overshooting.
- `max_speed`: hard cap on horizontal velocity command. Keep ≤ 5 m/s for safe SITL.

If the swarm oscillates around the leader, **lower `c_lead_pos`** (try 0.1).
If drones don't keep up with the leader, **raise `c_lead_vel`**.
If drones collide (distance < 1m), **raise `c_pos`** and/or **raise `desired_dist`**.

## Known issues / TODO

1. **Yaw consensus is approximate**. The MATLAB code mixes phi/theta/psi consensus, but a quadrotor's roll/pitch are not commandable separately from velocity. We only enforce yaw consensus, applied as a small lateral velocity offset.
2. **No obstacle avoidance** in v1.
3. **Birth position must be hand-configured** to match PX4_GZ_MODEL_POSE. If you change spawn poses, you must also update `BIRTH_POSITIONS_FLAT` in `launch/swarm_launch.py`.
4. **Frame coupling issue**: each drone reports `vehicle_local_position` relative to its OWN takeoff point. If two drones have different home positions but the EKF reference frame has any drift, the world-NED transform may be slightly off. For tighter formations (< 2m spacing), use vision-based or external tracking.
