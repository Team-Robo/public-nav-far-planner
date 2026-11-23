# far_planner — FAR visibility-graph planner as a move_base global planner

This package wraps the FAR planner (Fast, Attemptable Route planner, CMU; original sources under
`ref_code/far_planner`) as a `nav_core::BaseGlobalPlanner` plugin so that it replaces `navfn/NavfnROS`
inside `move_base`, while the existing `teamrobo2026/MpcLocalPlannerROS` keeps doing local control.

```
goal (odom, from run.py / rviz)
  -> move_base (goalToGlobalFrame -> global costmap frame)
     -> far_planner/FARPlannerROS::makePlan(start, goal)        [this package]
        -> dense plan in the global costmap frame (0.05 m spacing)
     -> teamrobo2026/MpcLocalPlannerROS::setPlan / computeVelocityCommands
        -> /cmd_vel
```

## Layout

| path | role |
|---|---|
| `include/far_planner/*.h`, `src/{utility,contour_*,dynamic_graph,graph_planner,map_handler,scan_handler,terrain_planner,planner_visualizer}.cpp` | FAR planning modules, copied verbatim from the original package (3 small patches, see below) |
| `src/far_globals.cpp` | FAR's static member definitions (moved out of the original node's `far_planner.cpp`) |
| `include/far_planner/far_planner_ros.h`, `src/far_planner_ros.cpp` | the move_base plugin glue (`far_planner::FARPlannerROS`) |
| `far_planner_plugin.xml`, `package.xml` `<export>` | pluginlib registration as `far_planner/FARPlannerROS` |
| `config/far_planner_params.yaml` | planner parameters (loaded under `move_base/FARPlannerROS`) |
| `config/far_planner.rviz` | rviz layout for FAR runs (`configs/` because `run.py -rc` only searches there) |
| `launch/move_base_far_mpc.launch` | copy of `teamrobo2026 move_base_mlda_2026.launch` with FAR as global planner |

Patches to the copied FAR sources: `utility.h` drops the `visibility_graph_msg` includes (only the removed
graph messenger used them); `contour_detector.{h,cpp}` gain the `approx_dist` parameter (polygon
approximation epsilon in metres) and `DrawCornerImage()` (the drawing of `ShowCornerImage()` without the
OpenCV window, with stable contour colours); `graph_planner.h` gains a public `ClearGoal()` wrapper;
`planner_visualizer.{h,cpp}` publish empty list markers as DELETE (rviz noetic marks empty point lists as
display errors).

## How the plugin drives FAR

`makePlan()` runs on move_base's planner thread (10 Hz) and performs one FAR cycle:

1. start/goal are checked against the global costmap frame (FAR's `world_frame` is forced to it; a
   TF fallback handles other frames);
2. the global costmap is converted into FAR's terrain cloud: cells with cost >= `obstacle_cost_threshold`
   become obstacle points, other known cells free points, unknown cells are skipped (attemptable);
3. FAR's local contour extraction and global v-graph update (`FARMaster::Loop` body);
4. goal node insertion and Dijkstra (`FARMaster::PlanningCallBack` body) -> `GraphPlanner::PathToGoal`;
5. the vertex path is densified at `plan_resolution`, every interior pose is pushed up the costmap
   distance field to `path_clearance` (or to the centre of a narrower gap), the last pose is the exact
   goal pose, all poses carry the global costmap frame and one stamp; published on `~/plan` too.

Within `GPlanner/converge_distance` of the goal a straight segment is returned so the local planner
finishes with its own tolerance. When FAR finds no path the previous plan is continued for up to
`max_plan_failures` cycles before `makePlan` returns false (move_base recovery/abort).

## Run

```
# example: BARN simulator runner (from the-barn-challenge)
python3 run.py --world_idx 0 -l move_base_far_mpc.launch
# original NavFn behaviour:
roslaunch far_planner move_base_far_mpc.launch global_planner:=navfn/NavfnROS
```

## Visualisation

```
python3 run.py --world_idx 0 -l move_base_far_mpc.launch -r -rc far_planner.rviz
```

`configs/far_planner.rviz` keeps the base navigation displays (costmaps, laser, robot, goal, plans) in the `odom`
frame and adds FAR's displays with the display settings of the original `rviz/default.rviz`
(marker colours come from FAR's visualizer code, so they match the original node):

| display | topic | default | shows |
|---|---|---|---|
| FAR/GlobalPolygon | `/viz_poly_topic` | on | global obstacle polygons (orange), unmatched contours (yellow) |
| FAR/GlobalGraph | `/viz_graph_topic` | on | visibility graph: free-space v-graph edges (emerald), polygon edges (red), trajectory edges (green); toggle other namespaces such as `global_vertex`, `global_vgraph`, `visibility_edge`, `to_goal_edge` under *Namespaces* |
| FAR/LocalGraph | `/viz_contour_topic` | off | polygons extracted this cycle and their vertices |
| FAR/GlobalPath | `/viz_path_topic` | on | FAR's vertex path (green = free-space route, blue = attemptable route through unknown space) |
| FAR/Goalpoint, GoalNodes | `/viz_node_topic` | on / off | free goal (green); GoalNodes shows every node marker |
| FAR/ObsPoints, FreePoints | `/FAR_obs_debug`, `/FAR_free_debug` | off | costmap cells FAR treats as obstacle / free (`obstacle_cost_threshold`) |
| FAR/ContourImage | `/move_base/FARPlannerROS/contour_image` | on | FAR's obstacle image (the original `is_opencv_visual` window) with the extracted contours, rotated to the top-down view (+x right, +y up, robot = red dot) |
| GlobalPlan / LocalPlan | `/move_base/FARPlannerROS/plan`, `/mpc_plan` | on | plan delivered to the MPC (green) / MPC prediction (orange) |

Layout: Displays on the left, top-down view of the course (odom, +y up, goal at the top) in the middle,
the contour image on the right, Time at the bottom. Views, Selection and Tool Properties are not part of
the layout; open them from the *Panels* menu. Marker sizes scale with `visualize_ratio` (0.3 here; the
original FAR uses 0.75 for large maps, 0.4 for matterport).

The debug clouds and the contour image are only built while a subscriber exists, at most at
`debug_viz_rate` (2 Hz). The original CMU displays for the terrain maps, local planner paths,
`/way_point`, voxel grid and viewpoint extension are not included because nothing publishes them here.

## Tuning notes (example: Jackal 0.43 m wide, 0.58 m gaps)

* `obstacle_cost_threshold` decides which gaps FAR considers passable: with the costmap inflation
  (radius 0.31, scaling 10) a threshold of 200 ~ 0.19 m half-corridor, so gaps under ~0.48 m are closed
  in FAR's map and 0.58 m gaps stay open. Match it to the local planner's feasible corridor.
* `path_clearance` (0.28 m) is what the MPC's obstacle constraints need (robot_radius 0.22 + margin).
* `CDetector/approx_dist` keeps rounded (inflated) obstacles as coarse polygons; keep it below FAR's dilation.
* `voxel_dim` = costmap resolution; `sensor_range`/`terrain_range` bound the contour image size.
