# FAR Planner

[![ROS 1 Noetic](https://img.shields.io/badge/ROS_1-Noetic-blue)](http://wiki.ros.org/noetic) [![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](LICENSE) [![Maintainer](https://img.shields.io/badge/Maintainer-NTU_Team_Robo-orange)](mailto:teamrobontu@gmail.com)

> **ROS 1 Noetic Package:** Builds with `catkin_make` as a `move_base` global planner plugin (integrates with `core-mode_switching_mpc_2026`).

A fast global planner plugin (`far_planner/FARPlannerROS`) for **ROS 1 (Noetic) `move_base`**. It finds short routes around obstacles, even when the map is still unknown. Built with `catkin` and designed to integrate with `move_base` and MPC local planners.

<!-- <p align="center">
  <img src="img/far-mpc.gif" alt="FAR Planner with an MPC local planner in the BARN simulator" width="800">
</p> -->


## Architecture

```mermaid
flowchart LR
    Costmap["Global Costmap (move_base)"] --> Contour["Contour Detector & Visibility Graph"]
    Contour --> Planner["FARPlannerROS (makePlan)"]
    Planner -->|"Global Path"| Local["Local Planner (MPC / DWA)"]
```

## How it works

1. Draws outlines (polygons) around the obstacles in the global costmap.
2. Connects the polygon corners that can see each other. This is the *visibility graph*.
3. Picks the shortest route through those corners and hands it to your local planner.

## Use it with your robot

Add these two lines to your `move_base` node:

```xml
<param name="base_global_planner" value="far_planner/FARPlannerROS" />
<rosparam file="$(find far_planner)/config/far_planner_params.yaml" command="load" ns="FARPlannerROS" />
```

## Main settings

All settings live in [`config/far_planner_params.yaml`](config/far_planner_params.yaml).

| Setting | What it does |
|---|---|
| `obstacle_cost_threshold` | How close FAR may go to obstacles. Lower keeps further away and closes more narrow gaps. |
| `path_clearance` | How far (m) the route is pushed away from obstacles. |
| `bench_log_enabled` | Turns the timing log on or off. |

## Example: BARN simulator

From the `the-barn-challenge` folder:

```bash
python3 run.py --world_idx 0 -l move_base_far_mpc.launch                         # run
python3 run.py --world_idx 0 -l move_base_far_mpc.launch -r -rc far_planner.rviz # run + rviz
```

`--world_idx` picks the world (0 to 299).

## Timing log

Once per second, `move_base` prints:

```text
[BENCH][FAR] total=27.0 ms | terrain=8.7 ms | dist_map=14.2 ms | ... | period: calls=11 avg=27.1 ms max=29.6 ms
```

`total` is the last plan, `avg`/`max` cover the last second, and the other numbers show each step.

## ROS Interfaces

| Interface | Name | Type | Description |
|---|---|---|---|
| Plugin | `far_planner/FARPlannerROS` | `nav_core::BaseGlobalPlanner` | `move_base` global planner (`makePlan`) |
| Costmap | `global_costmap` | `costmap_2d::Costmap2DROS` | Windowed obstacle costmap input |
| Topic (pub) | `/far_planner/viz_graph` (and debug markers) | `visualization_msgs/MarkerArray` | Visibility graph and planned route visualization |

## Reference

- **Original Repository:** [MichaelFYang/far_planner](https://github.com/MichaelFYang/far_planner)

F. Yang, C. Cao, H. Zhu, J. Oh, and J. Zhang. *FAR Planner: Fast, Attemptable Route Planner using
Dynamic Visibility Update.* IROS 2022, Kyoto, Japan. **Best Student Paper Award.**

## Contributors

- **Dun Yan Ong** (`@dunyanong`) — ROS `move_base` global planner plugin (`FARPlannerROS`), costmap windowing, and MPC integration
- **Fan Yang** — Original upstream FAR Planner author

Maintained by **Nanyang Technological University Team Robo** (`teamrobontu@gmail.com`).
## License

This project is licensed under the Apache-2.0 License — see the [LICENSE](LICENSE) file for details. Portions derived from upstream FAR Planner retain their original BSD attribution.
