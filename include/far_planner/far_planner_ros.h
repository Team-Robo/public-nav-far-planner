/*
 * FAR Planner - move_base global planner plugin (nav_core::BaseGlobalPlanner)
 *
 * Thin glue around the unmodified FAR planning modules (ContourDetector,
 * ContourGraph, DynamicGraph, GraphPlanner, MapHandler). Compared with the
 * original standalone FARMaster node:
 *   - obstacles come from move_base's global costmap instead of a terrain cloud
 *   - the robot pose comes from makePlan()'s start pose instead of odometry
 *   - the goal comes from makePlan() instead of the /goal_point topic
 *   - the full visibility-graph path is returned as a dense global plan
 *     instead of publishing a single /way_point
 *   - plan poses are pushed away from obstacles along the costmap distance
 *     field (FAR itself only dilates obstacles by a few centimetres and relied
 *     on its own local planner for clearance)
 * The planning frame (FAR's "world_frame") is the global costmap frame.
 */

#ifndef FAR_PLANNER_ROS_H
#define FAR_PLANNER_ROS_H

#include <memory>
#include <string>
#include <vector>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "far_planner/utility.h"
#include "far_planner/dynamic_graph.h"
#include "far_planner/contour_detector.h"
#include "far_planner/contour_graph.h"
#include "far_planner/graph_planner.h"
#include "far_planner/map_handler.h"
#include "far_planner/planner_visualizer.h"
#include "far_planner/scan_handler.h"

namespace far_planner {

/** Equivalent of FARMasterParams plus the plugin-only parameters. */
struct FARPlannerParams {
    FARPlannerParams() = default;
    float robot_dim;
    float vehicle_height;
    float voxel_dim;
    float sensor_range;
    float terrain_range;
    float local_planner_range;
    float main_run_freq;
    float viz_ratio;
    bool  is_multi_layer;
    bool  is_viewpoint_extend;
    bool  is_visual_opencv;
    bool  is_static_env;
    bool  is_debug_output;
    bool  is_attempt_autoswitch;
    std::string world_frame;
    /* plugin only */
    float plan_resolution;          // [m] spacing of the returned plan poses
    float goal_update_tolerance;    // [m] re-insert the goal node only if the goal moved more than this
    int   free_cell_stride;         // subsample free costmap cells (1 = every cell)
    int   obstacle_cost_threshold;  // costmap cost >= this is an obstacle for FAR's polygons (254 = lethal only)
    float path_clearance;           // [m] plan poses are pushed away from obstacles up to this clearance (0 = off)
    float path_clearance_max_shift; // [m] maximum shift of a plan pose during clearance refinement
    float clearance_map_range;      // [m] half-size of the costmap window the clearance distance transform runs over (0 = whole costmap)
    int   clearance_cost_threshold; // costmap cost >= this is an obstacle for the clearance field (254 = lethal)
    int   max_plan_failures;        // reuse the previous plan for up to this many consecutive FAR failures
    float debug_viz_rate;           // [Hz] rviz debug output (contour image, debug clouds), only while subscribed (0 = off)
    bool  bench_log_enabled;        // [BENCH][FAR] computation-time log, same format as the MPC's [BENCH] lines
    float bench_log_period_s;       // [s] period of the [BENCH][FAR] log (0 = every makePlan call)
};

/** Wall-clock time [ms] of each makePlan stage, filled during one call for the [BENCH][FAR] log. */
struct FARBench {
    double terrain    = 0.0;  // global costmap -> FAR terrain cloud, map grid, kd-trees
    double dist_map   = 0.0;  // costmap distance transform used by the clearance refinement
    double contours   = 0.0;  // obstacle image, contour extraction, contour graph
    double vgraph     = 0.0;  // v-graph vertex matching / update, global polygons
    double viz        = 0.0;  // FAR marker publishing (/viz_*_topic)
    double debug_viz  = 0.0;  // contour image + debug clouds (only while subscribed)
    double search     = 0.0;  // goal node update, traversability, Dijkstra, path extraction
    double build_plan = 0.0;  // densify, clearance refinement, poses, publish
    bool   updated    = false;  // environment + v-graph were updated this call (rate limited by main_run_freq)
};

class FARPlannerROS : public nav_core::BaseGlobalPlanner {
public:
    FARPlannerROS();
    ~FARPlannerROS() override = default;

    /** nav_core::BaseGlobalPlanner */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

    /** nav_core::BaseGlobalPlanner
     *  start/goal are expected in the global costmap frame (move_base guarantees this);
     *  other frames are transformed with TF as a fallback. The plan is returned in the
     *  global costmap frame, ordered start -> goal, densified at plan_resolution.
     */
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    /* port of FARMaster::LoadROSParams with relative names under ~/<name>/ */
    void LoadParams(ros::NodeHandle& nh);

    /* frames: identity when already in the planning frame, TF fallback otherwise */
    bool ToPlanningFrame(const geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out);

    /* FARMaster::OdomCallBack equivalent, fed from the start pose */
    void UpdateRobotState(const geometry_msgs::PoseStamped& start);

    /* FARMaster::TerrainCallBack equivalent, fed from the global costmap */
    void UpdateTerrainFromCostmap();

    /* distance field of the global costmap used for clearance refinement */
    void  UpdateDistanceMap();
    float ClearanceAt(const float x, const float y) const;

    /* FARMaster::Loop body: local contour extraction + global v-graph update */
    bool UpdateVGraph();

    /* FARMaster::PlanningCallBack body: goal node + Dijkstra + path reconstruction */
    bool PlanToGoal(const Point3D& goal_p, NodePtrStack& global_path, bool& is_reach_goal, bool& is_free_nav);

    /* previous plan continued from the current robot position (transient FAR failures) */
    std::vector<Point3D> ResumeWaypoints(const std::vector<Point3D>& prev, const Point3D& robot) const;

    /* polyline -> dense, clearance-refined, ordered, consistently stamped PoseStamped plan */
    void BuildPlan(const std::vector<Point3D>& waypoints,
                   const geometry_msgs::PoseStamped& goal,
                   std::vector<geometry_msgs::PoseStamped>& plan) const;
    void DensifyPolyline(const std::vector<cv::Point2f>& in, const float res, std::vector<cv::Point2f>& out) const;
    void RefinePoints(std::vector<cv::Point2f>& pts) const;

    void PublishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

    /* rviz debug output (config/far_planner.rviz): FAR's contour image and obstacle/free clouds,
     * built only while someone subscribes */
    void PublishDebugViz();

    /* makePlan body; outcome names the exit taken (for the [BENCH][FAR] log) */
    bool MakePlanImpl(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan,
                      const char*& outcome);

    /* [BENCH][FAR] log: last call's stage breakdown + avg/max of all calls in the period */
    void LogBench(const double total_ms, const char* outcome, const std::size_t n_poses);

    bool initialized_;
    std::string name_;
    std::string global_frame_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    ros::Publisher plan_pub_;
    ros::Publisher image_pub_;       // ~/contour_image
    ros::Publisher obs_debug_pub_;   // /FAR_obs_debug  (same topic as the original FAR node)
    ros::Publisher free_debug_pub_;  // /FAR_free_debug (same topic as the original FAR node)
    ros::Time last_debug_time_;
    boost::mutex mutex_;

    /* [BENCH][FAR] computation-time log */
    FARBench  bench_;
    ros::Time bench_last_log_;
    int       bench_calls_;
    double    bench_sum_ms_, bench_max_ms_;

    /* TF fallback only (move_base already delivers poses in the global costmap frame) */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    /* FAR module objects. Modules that own a ros::NodeHandle are created in initialize()
     * so the plugin constructor is ROS-free (pluginlib instantiates before initialize()). */
    ContourDetector contour_detector_;
    std::unique_ptr<DynamicGraph> graph_manager_;
    std::unique_ptr<DPVisualizer> planner_viz_;
    std::unique_ptr<GraphPlanner> graph_planner_;
    ContourGraph    contour_graph_;
    MapHandler      map_handler_;
    ScanHandler     scan_handler_;

    /* parameters */
    FARPlannerParams    master_params_;
    ContourDetectParams cdetect_params_;
    DynamicGraphParams  graph_params_;
    GraphPlannerParams  gp_params_;
    ContourGraphParams  cg_params_;
    MapHandlerParams    map_params_;
    ScanHandlerParams   scan_params_;

    /* state (FARMaster equivalents) */
    Point3D robot_pos_, robot_heading_;
    bool is_cloud_init_, is_odom_init_, is_graph_init_;
    ros::Time last_update_time_;
    Point3D last_goal_;

    /* clearance field (metres to the nearest obstacle cell, CV_32F, costmap-aligned) */
    cv::Mat dist_map_;
    double  dist_origin_x_, dist_origin_y_, dist_res_;

    /* failure resilience */
    std::vector<Point3D> last_waypoints_;
    int fail_count_;

    PointCloudPtr temp_obs_ptr_;
    PointCloudPtr temp_free_ptr_;
    PointCloudPtr temp_cloud_ptr_;
    PointCloudPtr terrain_height_ptr_;

    NavNodePtr odom_node_ptr_;
    NavNodePtr nav_node_ptr_;
    NodePtrStack new_nodes_;
    NodePtrStack nav_graph_;
    NodePtrStack near_nav_graph_;
    NodePtrStack clear_nodes_;
    CTNodeStack new_ctnodes_;
    std::vector<PointStack> realworld_contour_;
};

}  // namespace far_planner

#endif  // FAR_PLANNER_ROS_H
