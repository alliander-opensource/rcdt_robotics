<!--
SPDX-FileCopyrightText: Alliander N. V.

SPDX-License-Identifier: Apache-2.0
-->

# Nav2

According to the official [website](https://docs.nav2.org/):

"*Nav2 is the professionally-supported successor of the ROS Navigation Stack deploying the same kinds of technology powering Autonomous Vehicles brought down, optimized, and reworked for mobile and surface robotics.*"

This documentation shows Nav2 components that are used in this repository, sometimes with some additional explanation about implementation choices.

## GPS localization
It is possible to do GPS localization using Nav2. For this, the platform needs to be equipped with a GPS sensor, using e.g. `alliander_ublox`, and a sensor that provides an absolute yaw measurement. This can be an IMU (e.g. `alliander_xsens`), a dual GPS system, or pointcloud/image matching.
Nav2 uses the TF tree `map -> odom -> base_link -> <sensor_frames>`. To relate GPS latitude/longitude data to the `map` and `odom` frames, UTM zones are used. These zones divide the Earth into 60 vertical strips. Any (latitude, longitude) pair can then be expressed as a number of meters from the origin of a UTM zone.

### Global EKF
The `odom -> base_link` transform is provided by EKF, based on wheel odometry and IMU data. In GPS localization, `odometry/gps` is added as an additional odometry source. 

### `navsat_transform`
The `odometry/gps` measurements are provided by `navsat_transform`. It uses GPS data to relate the `map` frame origin and orientation to latitude and longitude coordinates. To do this, it subscribes to EKF's `odometry/global` output, but only for initialization: after `delay` seconds, it creates a snapshot based on the first GPS fix it gets, the current IMU heading, and the current `odometry/global` message. 
It uses this to create a _static_ transform between UTM and `map`, e.g. the datum. After that, each new GPS fix is run through that same static transform to output `odometry/gps` in meters. This is then fused by the global EKF, completing the loop.

:::{mermaid} ../diagrams/global_localization.mmd
:::

### Datum gating
It is important the `datum` be set correctly. If not, a large offset from the true position can cause accuracy errors. For this reason, the `datum_gate_node` was added to `alliander_nav2`: it only sets the datum after a few succesive valid GPS fixes with a suitable covariance are received.

## Planner
We choose to implement, for our Husarion platforms, the `SmacPlanner2D` planner. This because it is a standard planner in Nav2, and does not need a minimum turning radius. The `SmacPlannerHybrid` needs a minimum turning radius as large as the costmap resolution, a constraint that the Husarion platforms do not have. Using the Hybrid-A* planner sometimes results in no path being found, even though the Husarion robot is able to physically move to the goal pose. Using the classic 2D A* planner resolves this.

## Controller
With a relatively simple planner, we choose MPPI as the Husarion robots' controller. This because it is good for dynamic obstacle avoidance and produces smooth commands. The `mppi_generic` implementation supports GPU acceleration.


## Behaviour Tree
We implement a behaviour tree that was originally inspired by Nav2's `navigate_to_pose_w_replanning_and_recovery.xml`. See below our tree written out:

```text
MainTree
└── RecoveryNode (NavigateRecovery) [retries: 6]
    ├── PipelineSequence (NavigateWithReplanning)
    │   ├── ControllerSelector
    │   ├── PlannerSelector
    │   ├── RateController (4 Hz)
    │   │   └── RecoveryNode (ComputePathToPose) [retries: 20]
    │   │       ├── ComputePathToPose
    │   │       └── Wait (2s)
    │   └── RecoveryNode (FollowPath) [retries: 1]
    │       ├── ReactiveSequence (FollowPathWithGpsCheck)
    |       |   ├── IsSystemHealthy (custom ConditionNode)
    │       |   └── FollowPath
    │       └── Sequence (Contextual Recovery, in case FollowPath fails)
    │           ├── WouldAControllerRecoveryHelp?
    │           └── ClearEntireCostmap (Local)
    │
    └── Sequence (Global Recovery Branch, in case PipelineSequence a.k.a. navigation fails)
        ├── Fallback (If either says yes, proceed to ReactiveFallback)
        │   ├── WouldAControllerRecoveryHelp?
        │   └── WouldAPlannerRecoveryHelp?
        │
        └── ReactiveFallback (RecoveryFallback)
            ├── GoalUpdated? (If yes, then exit recovery, else proceed to RoundRobin)
            └── RoundRobin (RecoveryActions)
                ├── Sequence (ClearingActions)
                │   ├── ClearEntireCostmap (Local)
                │   └── ClearEntireCostmap (Global)
                ├── Spin (1.57 rad ≈ 90°)
                ├── Wait (5s)
                └── BackUp (0.30m @ 0.15 m/s)
```

In short, this behaviour tree performs autonomous navigation with constant replanning and recovery back-ups. Initially it tries to plan and follow a path, pausing navigation when the GPS signal is not sufficient enough. If either planning or path following fails, there are some local recovery sequences installed (e.g., clearing the local costmap). If navigation still fails, it moves on to a global recovery sequence, that can clear costmaps, spin the vehicle, wait for a period, or move the vehicle back a bit before retrying navigation.
