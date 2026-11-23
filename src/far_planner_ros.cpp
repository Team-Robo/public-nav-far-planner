/*
 * FAR Planner - move_base global planner plugin (nav_core::BaseGlobalPlanner)
 * See far_planner_ros.h for the overall design.
 */

#include "far_planner/far_planner_ros.h"

#include <cmath>
#include <algorithm>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(far_planner::FARPlannerROS, nav_core::BaseGlobalPlanner)

namespace far_planner {

namespace {
/* wall-clock milliseconds since t (computation time, independent of sim time) */
inline double MsSince(const ros::WallTime& t) { return (ros::WallTime::now() - t).toSec() * 1e3; }
}  // namespace

/***************************************************************************************/

FARPlannerROS::FARPlannerROS()
    : initialized_(false), costmap_ros_(NULL),
      bench_calls_(0), bench_sum_ms_(0.0), bench_max_ms_(0.0),
      is_cloud_init_(false), is_odom_init_(false), is_graph_init_(false),
      dist_origin_x_(0.0), dist_origin_y_(0.0), dist_res_(0.05),
      fail_count_(0), odom_node_ptr_(NULL), nav_node_ptr_(NULL) {}

void FARPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (initialized_) {
        ROS_WARN("FARPlannerROS: already initialized, ignoring second initialize()");
        return;
    }
    if (costmap_ros == NULL) {
        ROS_ERROR("FARPlannerROS: received a NULL costmap, cannot initialize");
        return;
    }
    name_         = name;
    costmap_ros_  = costmap_ros;
    global_frame_ = costmap_ros_->getGlobalFrameID();
    ros::NodeHandle pnh("~/" + name);
    this->LoadParams(pnh);
    plan_pub_       = pnh.advertise<nav_msgs::Path>("plan", 1);
    image_pub_      = pnh.advertise<sensor_msgs::Image>("contour_image", 1);
    obs_debug_pub_  = pnh.advertise<sensor_msgs::PointCloud2>("/FAR_obs_debug", 1);
    free_debug_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("/FAR_free_debug", 1);

    /* create the ROS-holding modules now that a NodeHandle is usable, then init all modules
     * (same order as FARMaster::Init) */
    graph_manager_.reset(new DynamicGraph());
    graph_planner_.reset(new GraphPlanner());
    planner_viz_.reset(new DPVisualizer());
    contour_detector_.Init(cdetect_params_);
    graph_manager_->Init(pnh, graph_params_);
    graph_planner_->Init(pnh, gp_params_);
    contour_graph_.Init(cg_params_);
    planner_viz_->Init(pnh);
    map_handler_.Init(map_params_);
    scan_handler_.Init(scan_params_);

    /* init internal state */
    odom_node_ptr_    = NULL;
    nav_node_ptr_     = NULL;
    is_cloud_init_    = false;
    is_odom_init_     = false;
    is_graph_init_    = false;
    last_update_time_ = ros::Time(0);
    last_debug_time_  = ros::Time(0);
    bench_last_log_   = ros::Time(0);
    bench_calls_      = 0;
    bench_sum_ms_     = bench_max_ms_ = 0.0;
    last_goal_        = Point3D(0,0,0);
    last_waypoints_.clear();
    fail_count_       = 0;
    dist_map_.release();

    temp_obs_ptr_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    temp_free_ptr_      = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    temp_cloud_ptr_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    terrain_height_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());

    FARUtil::kdtree_new_cloud_->setSortedResults(false);
    FARUtil::kdtree_filter_cloud_->setSortedResults(false);
    FARUtil::stack_new_cloud_->clear();
    FARUtil::stack_dyobs_cloud_->clear();
    FARUtil::robot_pos   = Point3D(0,0,0);
    FARUtil::free_odom_p = Point3D(0,0,0);
    robot_pos_     = Point3D(0,0,0);
    robot_heading_ = Point3D(1,0,0);

    initialized_ = true;
    ROS_INFO("FARPlannerROS [%s]: initialized; planning frame '%s' (global costmap frame), "
             "plan resolution %.3f m, voxel %.3f m, robot_dim %.2f m, nav clear dist %.3f m, "
             "obstacle cost >= %d, path clearance %.2f m (cost >= %d)",
             name_.c_str(), global_frame_.c_str(), master_params_.plan_resolution,
             master_params_.voxel_dim, master_params_.robot_dim, FARUtil::kNavClearDist,
             master_params_.obstacle_cost_threshold, master_params_.path_clearance,
             master_params_.clearance_cost_threshold);
}

/***************************************************************************************/

void FARPlannerROS::LoadParams(ros::NodeHandle& nh) {
    const std::string map_prefix     = "MapHandler/";
    const std::string utility_prefix = "Util/";
    const std::string planner_prefix = "GPlanner/";
    const std::string graph_prefix   = "Graph/";
    const std::string cdetect_prefix = "CDetector/";

    // master params (FARMaster::LoadROSParams equivalents)
    nh.param<float>("main_run_freq",         master_params_.main_run_freq, 10.0);
    nh.param<float>("voxel_dim",             master_params_.voxel_dim, 0.05);
    nh.param<float>("robot_dim",             master_params_.robot_dim, 0.4);
    nh.param<float>("vehicle_height",        master_params_.vehicle_height, 0.0);
    nh.param<float>("sensor_range",          master_params_.sensor_range, 8.0);
    nh.param<float>("terrain_range",         master_params_.terrain_range, 8.0);
    nh.param<float>("local_planner_range",   master_params_.local_planner_range, 1.0);
    nh.param<float>("visualize_ratio",       master_params_.viz_ratio, 1.0);
    nh.param<bool>("is_viewpoint_extend",    master_params_.is_viewpoint_extend, false);
    nh.param<bool>("is_multi_layer",         master_params_.is_multi_layer, false);
    nh.param<bool>("is_opencv_visual",       master_params_.is_visual_opencv, false);
    nh.param<bool>("is_static_env",          master_params_.is_static_env, true);
    nh.param<bool>("is_debug_output",        master_params_.is_debug_output, false);
    nh.param<bool>("is_attempt_autoswitch",  master_params_.is_attempt_autoswitch, true);
    // The planning frame is dictated by move_base's global costmap; a yaml world_frame is only a hint.
    std::string yaml_frame;
    nh.param<std::string>("world_frame", yaml_frame, global_frame_);
    if (!FARUtil::IsSameFrameID(yaml_frame, global_frame_)) {
        ROS_WARN("FARPlannerROS: param world_frame '%s' ignored, planning in the global costmap frame '%s'",
                 yaml_frame.c_str(), global_frame_.c_str());
    }
    master_params_.world_frame   = global_frame_;
    master_params_.terrain_range = std::min(master_params_.terrain_range, master_params_.sensor_range);
    if (master_params_.main_run_freq <= 0.0f) master_params_.main_run_freq = 10.0f;

    // plugin params
    const float costmap_res = (float)costmap_ros_->getCostmap()->getResolution();
    nh.param<float>("plan_resolution",          master_params_.plan_resolution, costmap_res);
    nh.param<float>("goal_update_tolerance",    master_params_.goal_update_tolerance, 0.1);
    nh.param<int>("free_cell_stride",           master_params_.free_cell_stride, 2);
    nh.param<int>("obstacle_cost_threshold",    master_params_.obstacle_cost_threshold, (int)costmap_2d::LETHAL_OBSTACLE);
    nh.param<float>("path_clearance",           master_params_.path_clearance, 0.28);
    nh.param<float>("path_clearance_max_shift", master_params_.path_clearance_max_shift, 0.5);
    nh.param<float>("clearance_map_range",      master_params_.clearance_map_range, 4.0);
    nh.param<int>("clearance_cost_threshold",   master_params_.clearance_cost_threshold, (int)costmap_2d::LETHAL_OBSTACLE);
    nh.param<int>("max_plan_failures",          master_params_.max_plan_failures, 30);
    nh.param<float>("debug_viz_rate",           master_params_.debug_viz_rate, 2.0);
    nh.param<bool>("bench_log_enabled",         master_params_.bench_log_enabled, true);
    nh.param<float>("bench_log_period_s",       master_params_.bench_log_period_s, 1.0);
    if (master_params_.bench_log_period_s < 0.0f) master_params_.bench_log_period_s = 0.0f;
    if (master_params_.plan_resolution <= 0.0f) master_params_.plan_resolution = costmap_res;
    if (master_params_.free_cell_stride < 1)    master_params_.free_cell_stride = 1;
    master_params_.obstacle_cost_threshold  = std::max(1, std::min((int)costmap_2d::LETHAL_OBSTACLE, master_params_.obstacle_cost_threshold));
    master_params_.clearance_cost_threshold = std::max(1, std::min((int)costmap_2d::LETHAL_OBSTACLE, master_params_.clearance_cost_threshold));
    if (master_params_.path_clearance_max_shift < 0.0f) master_params_.path_clearance_max_shift = 0.0f;
    if (master_params_.clearance_map_range < 0.0f) master_params_.clearance_map_range = 0.0f;  // 0 = whole costmap
    if (master_params_.max_plan_failures < 0)  master_params_.max_plan_failures = 0;

    // map handler params
    nh.param<float>(map_prefix + "floor_height",        map_params_.floor_height, 1.0);
    nh.param<float>(map_prefix + "cell_length",         map_params_.cell_length, 2.0);
    nh.param<float>(map_prefix + "map_grid_max_length", map_params_.grid_max_length, 60.0);
    nh.param<float>(map_prefix + "map_grad_max_height", map_params_.grid_max_height, 4.0);
    map_params_.height_voxel_dim = master_params_.voxel_dim * 2.0f;
    map_params_.cell_height      = map_params_.floor_height / 2.5f;
    map_params_.sensor_range     = master_params_.sensor_range;

    // utility params
    float nav_clear_dist_override = 0.0f;
    nh.param<float>(utility_prefix + "angle_noise",            FARUtil::kAngleNoise, 15.0);
    nh.param<float>(utility_prefix + "accept_max_align_angle", FARUtil::kAcceptAlign, 4.0);
    nh.param<float>(utility_prefix + "new_intensity_thred",    FARUtil::kNewPIThred, 2.0);
    nh.param<float>(utility_prefix + "nav_clear_dist",         nav_clear_dist_override, 0.0);
    nh.param<float>(utility_prefix + "terrain_free_Z",         FARUtil::kFreeZ, 0.1);
    nh.param<int>(utility_prefix   + "dyosb_update_thred",     FARUtil::kDyObsThred, 4);
    nh.param<int>(utility_prefix   + "new_point_counter",      FARUtil::KNewPointC, 5);
    nh.param<float>(utility_prefix + "dynamic_obs_dacay_time", FARUtil::kObsDecayTime, 10.0);
    nh.param<float>(utility_prefix + "new_points_decay_time",  FARUtil::kNewDecayTime, 1.0);
    nh.param<int>(utility_prefix   + "obs_inflate_size",       FARUtil::kObsInflate, 1);
    FARUtil::kLeafSize       = master_params_.voxel_dim;
    FARUtil::kNearDist       = master_params_.robot_dim;
    FARUtil::kHeightVoxel    = map_params_.height_voxel_dim;
    FARUtil::kMatchDist      = master_params_.robot_dim * 2.0f + FARUtil::kLeafSize;
    FARUtil::kNavClearDist   = master_params_.robot_dim / 2.0f + FARUtil::kLeafSize;
    if (nav_clear_dist_override > 0.0f) {  // explicit clearance override (original FAR always used the formula)
        FARUtil::kNavClearDist = nav_clear_dist_override;
    }
    FARUtil::kProjectDist    = master_params_.voxel_dim;
    FARUtil::worldFrameId    = master_params_.world_frame;
    FARUtil::kVizRatio       = master_params_.viz_ratio;
    FARUtil::kTolerZ         = map_params_.floor_height - FARUtil::kHeightVoxel;
    FARUtil::kCellLength     = map_params_.cell_length;
    FARUtil::kCellHeight     = map_params_.cell_height;
    FARUtil::kAcceptAlign    = FARUtil::kAcceptAlign / 180.0f * M_PI;
    FARUtil::kAngleNoise     = FARUtil::kAngleNoise  / 180.0f * M_PI;
    FARUtil::robot_dim       = master_params_.robot_dim;
    FARUtil::IsStaticEnv     = master_params_.is_static_env;
    FARUtil::IsDebug         = master_params_.is_debug_output;
    FARUtil::IsMultiLayer    = master_params_.is_multi_layer;
    FARUtil::vehicle_height  = master_params_.vehicle_height;
    FARUtil::kSensorRange    = master_params_.sensor_range;
    FARUtil::kMarginDist     = master_params_.sensor_range - FARUtil::kMatchDist;
    FARUtil::kMarginHeight   = FARUtil::kTolerZ - FARUtil::kCellHeight / 2.0f;
    FARUtil::kTerrainRange   = master_params_.terrain_range;
    FARUtil::kLocalPlanRange = master_params_.local_planner_range;

    // graph planner params
    nh.param<float>(planner_prefix + "converge_distance",    gp_params_.converge_dist, 0.3);
    nh.param<float>(planner_prefix + "goal_adjust_radius",   gp_params_.adjust_radius, 1.0);
    nh.param<int>(planner_prefix   + "free_counter_thred",   gp_params_.free_thred, 7);
    nh.param<int>(planner_prefix   + "reach_goal_vote_size", gp_params_.votes_size, 3);
    nh.param<int>(planner_prefix   + "path_momentum_thred",  gp_params_.momentum_thred, 3);
    gp_params_.momentum_dist = master_params_.robot_dim / 2.0f;
    gp_params_.is_autoswitch = master_params_.is_attempt_autoswitch;

    // contour graph params
    cg_params_.kPillarPerimeter = master_params_.robot_dim * 4.0f;

    // dynamic graph params
    nh.param<int>(graph_prefix    + "connect_votes_size",        graph_params_.votes_size, 10);
    nh.param<int>(graph_prefix    + "clear_dumper_thred",        graph_params_.dumper_thred, 4);
    nh.param<int>(graph_prefix    + "node_finalize_thred",       graph_params_.finalize_thred, 6);
    nh.param<int>(graph_prefix    + "filter_pool_size",          graph_params_.pool_size, 12);
    nh.param<float>(graph_prefix  + "connect_angle_thred",       graph_params_.kConnectAngleThred, 10.0);
    nh.param<float>(graph_prefix  + "dirs_filter_margin",        graph_params_.filter_dirs_margin, 10.0);
    graph_params_.filter_pos_margin        = FARUtil::kNavClearDist;
    graph_params_.filter_dirs_margin       = FARUtil::kAngleNoise;
    graph_params_.kConnectAngleThred       = FARUtil::kAcceptAlign;
    graph_params_.frontier_perimeter_thred = FARUtil::kMatchDist * 4.0f;

    // scan handler params (dynamic-env only, kept for module init)
    scan_params_.terrain_range = master_params_.terrain_range;
    scan_params_.voxel_size    = master_params_.voxel_dim;
    scan_params_.ceil_height   = map_params_.floor_height;

    // contour detector params
    nh.param<float>(cdetect_prefix       + "resize_ratio",       cdetect_params_.kRatio, 3.0);
    nh.param<int>(cdetect_prefix         + "filter_count_value", cdetect_params_.kThredValue, 3);
    nh.param<bool>(cdetect_prefix        + "is_save_img",        cdetect_params_.is_save_img, false);
    nh.param<std::string>(cdetect_prefix + "img_folder_path",    cdetect_params_.img_path, "");
    nh.param<float>(cdetect_prefix       + "approx_dist",        cdetect_params_.approx_dist, 0.0);
    int blur_size_override = 0;
    nh.param<int>(cdetect_prefix         + "blur_size",          blur_size_override, 0);
    cdetect_params_.kBlurSize    = (int)std::round(FARUtil::kNavClearDist / master_params_.voxel_dim);
    if (blur_size_override > 0) {  // resized-image pixels; 1 = no extra dilation (obstacle inflation comes from the costmap)
        cdetect_params_.kBlurSize = blur_size_override;
    }
    if (cdetect_params_.kBlurSize < 1) cdetect_params_.kBlurSize = 1;
    cdetect_params_.sensor_range = master_params_.sensor_range;
    cdetect_params_.voxel_dim    = master_params_.voxel_dim;
}

/***************************************************************************************/

bool FARPlannerROS::ToPlanningFrame(const geometry_msgs::PoseStamped& in, geometry_msgs::PoseStamped& out) {
    if (in.header.frame_id.empty()) {
        ROS_ERROR_THROTTLE(1.0, "FARPlannerROS: pose has an empty frame_id, cannot plan");
        return false;
    }
    if (FARUtil::IsSameFrameID(in.header.frame_id, global_frame_)) {
        out = in;
        out.header.frame_id = global_frame_;
        return true;
    }
    // Fallback only: move_base normally hands poses in the global costmap frame already.
    if (!tf_buffer_) {
        ROS_WARN("FARPlannerROS: pose in frame '%s' differs from planning frame '%s', starting a TF listener",
                 in.header.frame_id.c_str(), global_frame_.c_str());
        tf_buffer_.reset(new tf2_ros::Buffer());
        tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    }
    try {
        geometry_msgs::PoseStamped src = in;
        if (!tf_buffer_->canTransform(global_frame_, src.header.frame_id, src.header.stamp, ros::Duration(0.2))) {
            src.header.stamp = ros::Time(0);  // use the latest available transform
        }
        tf_buffer_->transform(src, out, global_frame_, ros::Duration(0.2));
        out.header.frame_id = global_frame_;
        return true;
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "FARPlannerROS: cannot transform pose from '%s' to '%s': %s",
                          in.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
        return false;
    }
}

void FARPlannerROS::UpdateRobotState(const geometry_msgs::PoseStamped& start) {
    robot_pos_ = Point3D(start.pose.position.x, start.pose.position.y, start.pose.position.z);
    FARUtil::robot_pos = robot_pos_;
    const geometry_msgs::Quaternion& q = start.pose.orientation;
    const double qn = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    const double yaw = (qn > 0.5) ? tf2::getYaw(q) : 0.0;
    robot_heading_ = Point3D(cos(yaw), sin(yaw), 0);
    if (!is_odom_init_) {
        FARUtil::systemStartTime = ros::Time::now().toSec();
        FARUtil::map_origin = robot_pos_;
        map_handler_.UpdateRobotPosition(robot_pos_);
        is_odom_init_ = true;
    }
}

/***************************************************************************************/

void FARPlannerROS::UpdateTerrainFromCostmap() {
    // update map grid robot center
    map_handler_.UpdateRobotPosition(FARUtil::robot_pos);
    /* Synthesize FAR's terrain cloud from the global costmap:
     *   cost >= obstacle_cost_threshold -> obstacle points (intensity above terrain_free_Z)
     *   other known cells               -> free points (intensity 0), subsampled by free_cell_stride
     *   NO_INFORMATION                  -> skipped (unknown space stays attemptable)
     * With the default threshold (lethal only) FAR sees the raw obstacle cells and its polygons keep
     * narrow gaps open; robot clearance is added to the returned plan by RefinePoints(). */
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    temp_cloud_ptr_->clear();
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
        const float range = master_params_.terrain_range;
        int mx0, my0, mx1, my1;
        costmap->worldToMapEnforceBounds(robot_pos_.x - range, robot_pos_.y - range, mx0, my0);
        costmap->worldToMapEnforceBounds(robot_pos_.x + range, robot_pos_.y + range, mx1, my1);
        const unsigned char* data = costmap->getCharMap();
        const float z     = robot_pos_.z - master_params_.vehicle_height;
        const float obs_i = FARUtil::kFreeZ + 1.0f;
        const int stride  = master_params_.free_cell_stride;
        const unsigned char obs_thr = (unsigned char)master_params_.obstacle_cost_threshold;
        temp_cloud_ptr_->reserve((std::size_t)(mx1 - mx0 + 1) * (std::size_t)(my1 - my0 + 1) / (std::size_t)(stride * stride) + 1024);
        for (int my = my0; my <= my1; ++my) {
            for (int mx = mx0; mx <= mx1; ++mx) {
                const unsigned char v = data[costmap->getIndex(mx, my)];
                PCLPoint p;
                if (v == costmap_2d::NO_INFORMATION) {
                    continue;
                } else if (v >= obs_thr) {
                    p.intensity = obs_i;
                } else if ((mx % stride) == 0 && (my % stride) == 0) {
                    p.intensity = 0.0f;
                } else {
                    continue;
                }
                double wx, wy;
                costmap->mapToWorld((unsigned int)mx, (unsigned int)my, wx, wy);
                p.x = (float)wx, p.y = (float)wy, p.z = z;
                temp_cloud_ptr_->push_back(p);
            }
        }
    }
    /* FARMaster::TerrainCallBack body (static environment). The costmap loop above already emits
     * only cells within [robot +/- terrain_range] (XY) at the single robot-z plane, so the original
     * CropBoxCloud(terrain_range, kTolerZ) here was redundant (a no-op beyond trimming a half-cell
     * edge ring) and has been dropped. */
    FARUtil::ExtractFreeAndObsCloud(temp_cloud_ptr_, temp_free_ptr_, temp_obs_ptr_);
    map_handler_.UpdateObsCloudGrid(temp_obs_ptr_);
    map_handler_.UpdateFreeCloudGrid(temp_free_ptr_);
    // extract new points
    FARUtil::ExtractNewObsPointCloud(temp_obs_ptr_,
                                     FARUtil::surround_obs_cloud_,
                                     FARUtil::cur_new_cloud_);
    // extract surround free cloud & update terrain height
    map_handler_.GetSurroundFreeCloud(FARUtil::surround_free_cloud_);
    map_handler_.UpdateTerrainHeightGrid(FARUtil::surround_free_cloud_, terrain_height_ptr_);
    // update surround obs cloud
    map_handler_.GetSurroundObsCloud(FARUtil::surround_obs_cloud_);
    FARUtil::cur_dyobs_cloud_->clear();
    // create and update kdtrees
    FARUtil::StackCloudByTime(FARUtil::cur_new_cloud_, FARUtil::stack_new_cloud_, FARUtil::kNewDecayTime);
    FARUtil::UpdateKdTrees(FARUtil::stack_new_cloud_);

    if (!FARUtil::surround_obs_cloud_->empty()) is_cloud_init_ = true;
}

void FARPlannerROS::UpdateDistanceMap() {
    if (master_params_.path_clearance <= 0.0f) return;
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    cv::Mat mask;
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
        const int W = (int)costmap->getSizeInCellsX();  // full-map row stride into getCharMap()
        const int H = (int)costmap->getSizeInCellsY();
        const unsigned char* data = costmap->getCharMap();
        const unsigned char thr = (unsigned char)master_params_.clearance_cost_threshold;
        /* Restrict the transform to a window around the robot (like UpdateTerrainFromCostmap),
         * instead of the whole ~40x40 m costmap. The window is padded beyond the useful query
         * radius by path_clearance_max_shift + path_clearance so obstacles just outside it still
         * register: any clearance value the refinement actually uses (below path_clearance) stays
         * exact. clearance_map_range <= 0 keeps the original whole-costmap behaviour. */
        int mx0 = 0, my0 = 0, mx1 = W - 1, my1 = H - 1;
        if (master_params_.clearance_map_range > 0.0f) {
            const float range = master_params_.clearance_map_range +
                                master_params_.path_clearance_max_shift + master_params_.path_clearance;
            costmap->worldToMapEnforceBounds(robot_pos_.x - range, robot_pos_.y - range, mx0, my0);
            costmap->worldToMapEnforceBounds(robot_pos_.x + range, robot_pos_.y + range, mx1, my1);
        }
        const int Ws = mx1 - mx0 + 1;   // window width  (mask cols)
        const int Hs = my1 - my0 + 1;   // window height (mask rows)
        mask.create(Hs, Ws, CV_8UC1);
        for (int my = my0; my <= my1; ++my) {
            unsigned char* row = mask.ptr<unsigned char>(my - my0);
            const unsigned char* src = data + (std::size_t)my * (std::size_t)W;
            for (int mx = mx0; mx <= mx1; ++mx) {
                row[mx - mx0] = (src[mx] != costmap_2d::NO_INFORMATION && src[mx] >= thr) ? 0 : 255;
            }
        }
        dist_res_      = costmap->getResolution();
        dist_origin_x_ = costmap->getOriginX() + (double)mx0 * dist_res_;  // window lower-left corner
        dist_origin_y_ = costmap->getOriginY() + (double)my0 * dist_res_;
    }
    cv::distanceTransform(mask, dist_map_, cv::DIST_L2, 5);  // pixels to the nearest obstacle cell
    dist_map_ *= (float)dist_res_;                            // -> metres
}

float FARPlannerROS::ClearanceAt(const float x, const float y) const {
    if (dist_map_.empty()) return 1e3f;
    const float fx = (x - (float)dist_origin_x_) / (float)dist_res_ - 0.5f;
    const float fy = (y - (float)dist_origin_y_) / (float)dist_res_ - 0.5f;
    if (fx < 0.0f || fy < 0.0f || fx >= (float)(dist_map_.cols - 1) || fy >= (float)(dist_map_.rows - 1)) return 1e3f;
    const int x0 = (int)fx, y0 = (int)fy;
    const float tx = fx - (float)x0, ty = fy - (float)y0;
    const float d00 = dist_map_.at<float>(y0, x0),     d10 = dist_map_.at<float>(y0, x0 + 1);
    const float d01 = dist_map_.at<float>(y0 + 1, x0), d11 = dist_map_.at<float>(y0 + 1, x0 + 1);
    return (1.0f - ty) * ((1.0f - tx) * d00 + tx * d10) + ty * ((1.0f - tx) * d01 + tx * d11);
}

/***************************************************************************************/

bool FARPlannerROS::UpdateVGraph() {
    /* FARMaster::Loop body (terminal output, timers and GraphMsger removed) */
    graph_manager_->UpdateRobotPosition(robot_pos_);
    odom_node_ptr_ = graph_manager_->GetOdomNode();
    if (odom_node_ptr_ == NULL) {
        ROS_WARN_THROTTLE(1.0, "FARPlannerROS: odom node not available yet");
        return false;
    }
    /* Extract Vertices and new nodes */
    ros::WallTime t_stage = ros::WallTime::now();
    contour_detector_.BuildTerrainImgAndExtractContour(odom_node_ptr_, FARUtil::surround_obs_cloud_, realworld_contour_);
    contour_graph_.UpdateContourGraph(odom_node_ptr_, realworld_contour_);
    /* Adjust heights with terrain */
    map_handler_.AdjustCTNodeHeight(ContourGraph::contour_graph_);
    map_handler_.AdjustNodesHeight(nav_graph_);
    bench_.contours = MsSince(t_stage);
    t_stage = ros::WallTime::now();
    // Truncate for local range nodes
    graph_manager_->UpdateGlobalNearNodes();
    near_nav_graph_ = graph_manager_->GetExtendLocalNode();
    // Match near nav nodes with contour
    contour_graph_.MatchContourWithNavGraph(nav_graph_, near_nav_graph_, new_ctnodes_);
    /* update planner graph */
    new_nodes_.clear();
    if (graph_manager_->ExtractGraphNodes(new_ctnodes_)) {
        new_nodes_ = graph_manager_->GetNewNodes();
    }
    /* Graph Updating */
    graph_manager_->UpdateNavGraph(new_nodes_, false, clear_nodes_);
    /* Update v-graph in other modules */
    nav_graph_ = graph_manager_->GetNavGraph();
    contour_graph_.ExtractGlobalContours();      // Global Polygon Update
    graph_planner_->UpdaetVGraph(nav_graph_);    // Graph Planner Update
    bench_.vgraph = MsSince(t_stage);
    t_stage = ros::WallTime::now();

    /* Viz Navigation Graph */
    const NavNodePtr last_internav_ptr = graph_manager_->GetLastInterNavNode();
    if (last_internav_ptr != NULL) {
        planner_viz_->VizPoint3D(last_internav_ptr->position, "last_nav_node", VizColor::MAGNA, 1.0);
    }
    planner_viz_->VizNodes(clear_nodes_, "clear_nodes", VizColor::ORANGE);
    planner_viz_->VizNodes(graph_manager_->GetOutContourNodes(), "out_contour", VizColor::YELLOW);
    planner_viz_->VizPoint3D(FARUtil::free_odom_p, "free_odom_position", VizColor::ORANGE, 1.0);
    planner_viz_->VizGraph(nav_graph_);
    planner_viz_->VizContourGraph(ContourGraph::contour_graph_);
    planner_viz_->VizGlobalPolygons(ContourGraph::global_contour_, ContourGraph::unmatched_contour_);
    bench_.viz = MsSince(t_stage);

    if (!is_graph_init_ && !nav_graph_.empty()) {
        is_graph_init_ = true;
        ROS_INFO("FARPlannerROS: v-graph initialized (%zu vertices)", nav_graph_.size());
    }
    return true;
}

/***************************************************************************************/

bool FARPlannerROS::PlanToGoal(const Point3D& goal_p, NodePtrStack& global_path, bool& is_reach_goal, bool& is_free_nav) {
    /* Goal node handling: FARMaster::WaypointCallBack equivalent.
     * UpdateGoal() resets FAR's momentum state, so it is only called for a new goal
     * (or after FAR dropped the goal node on success/failure), not on every makePlan(). */
    NavNodePtr goal_ptr = graph_planner_->GetGoalNodePtr();
    if (goal_ptr == NULL || (goal_p - last_goal_).norm_flat() > master_params_.goal_update_tolerance) {
        // FARMaster runs UpdateGraphTraverability(odom, NULL) every cycle before a goal exists; that
        // call caches the odom node inside GraphPlanner, which ReEvaluateGoalPosition dereferences.
        graph_planner_->UpdateGraphTraverability(odom_node_ptr_, NULL);
        graph_planner_->UpdateGoal(goal_p);
        last_goal_ = goal_p;
        goal_ptr = graph_planner_->GetGoalNodePtr();
        planner_viz_->VizPoint3D(goal_p, "original_goal", VizColor::RED, 1.5);
    }
    if (goal_ptr == NULL) {
        ROS_ERROR_THROTTLE(1.0, "FARPlannerROS: failed to insert the goal into the v-graph");
        return false;
    }
    /* FARMaster::PlanningCallBack body */
    // Update goal position with nearby terrain cloud
    const Point3D ori_p = graph_planner_->GetOriginNodePos(true);
    PointCloudPtr goal_obs(new pcl::PointCloud<PCLPoint>());
    PointCloudPtr goal_free(new pcl::PointCloud<PCLPoint>());
    map_handler_.GetCloudOfPoint(ori_p, goal_obs, CloudType::OBS_CLOUD, true);
    map_handler_.GetCloudOfPoint(ori_p, goal_free, CloudType::FREE_CLOUD, true);
    graph_planner_->UpdateFreeTerrainGrid(ori_p, goal_obs, goal_free);
    graph_planner_->ReEvaluateGoalPosition(goal_ptr, !master_params_.is_multi_layer);
    // Adding goal into v-graph
    graph_planner_->UpdateGoalNavNodeConnects(goal_ptr);
    graph_planner_->UpdaetVGraph(graph_manager_->GetNavGraph());
    // Update v-graph traversibility
    graph_planner_->UpdateGraphTraverability(odom_node_ptr_, goal_ptr);
    // Construct path to goal
    global_path.clear();
    Point3D current_free_goal;
    bool is_planning_fails = false;
    is_reach_goal = false;
    is_free_nav   = false;
    const bool ok = graph_planner_->PathToGoal(goal_ptr, global_path, nav_node_ptr_, current_free_goal,
                                               is_planning_fails, is_reach_goal, is_free_nav);
    if (!ok || global_path.size() < 2) {
        NodePtrStack empty_path;
        planner_viz_->VizPath(empty_path);
        return false;
    }
    planner_viz_->VizPoint3D(current_free_goal, "free_goal", VizColor::GREEN, 1.5);
    planner_viz_->VizPath(global_path, is_free_nav);
    return true;
}

std::vector<Point3D> FARPlannerROS::ResumeWaypoints(const std::vector<Point3D>& prev, const Point3D& robot) const {
    std::vector<Point3D> out;
    out.push_back(robot);
    if (prev.size() < 2) {
        if (!prev.empty()) out.push_back(prev.back());
        return out;
    }
    // continue from the segment of the previous polyline closest to the robot
    std::size_t best = 0;
    float best_d = FARUtil::kINF;
    for (std::size_t i = 0; i + 1 < prev.size(); ++i) {
        const Point3D& a = prev[i];
        const Point3D& b = prev[i + 1];
        const float abx = b.x - a.x, aby = b.y - a.y;
        const float L2 = abx * abx + aby * aby;
        float t = (L2 > 1e-6f) ? ((robot.x - a.x) * abx + (robot.y - a.y) * aby) / L2 : 0.0f;
        t = std::max(0.0f, std::min(1.0f, t));
        const float d = std::hypot(a.x + t * abx - robot.x, a.y + t * aby - robot.y);
        if (d < best_d) best_d = d, best = i;
    }
    for (std::size_t i = best + 1; i < prev.size(); ++i) out.push_back(prev[i]);
    return out;
}

/***************************************************************************************/

void FARPlannerROS::DensifyPolyline(const std::vector<cv::Point2f>& in, const float res, std::vector<cv::Point2f>& out) const {
    out.clear();
    if (in.empty()) return;
    for (std::size_t i = 0; i + 1 < in.size(); ++i) {
        const float dx = in[i + 1].x - in[i].x;
        const float dy = in[i + 1].y - in[i].y;
        const float L  = std::hypot(dx, dy);
        if (L < 1e-4f) continue;
        const int steps = std::max(1, (int)std::ceil(L / res));
        for (int s = 0; s < steps; ++s) {
            const float t = (float)s / (float)steps;
            out.emplace_back(in[i].x + t * dx, in[i].y + t * dy);
        }
    }
    out.push_back(in.back());
}

void FARPlannerROS::RefinePoints(std::vector<cv::Point2f>& pts) const {
    /* FAR routes through polygon vertices that sit only a few centimetres from the obstacles.
     * Push every interior sample up the costmap distance field until it has path_clearance
     * or reaches the ridge between two obstacles (the centre of a narrow gap). */
    const float target = master_params_.path_clearance;
    if (target <= 0.0f || dist_map_.empty() || pts.size() < 3) return;
    const float max_shift = master_params_.path_clearance_max_shift;
    const float step = 0.5f * (float)dist_res_;
    static const int kDirs = 16;
    float raw_min = 1e3f, refined_min = 1e3f;
    for (std::size_t i = 1; i + 1 < pts.size(); ++i) {  // start and goal stay fixed
        const cv::Point2f p0 = pts[i];
        const float c0 = this->ClearanceAt(p0.x, p0.y);
        raw_min = std::min(raw_min, c0);
        if (c0 >= target) { refined_min = std::min(refined_min, c0); continue; }
        /* Radial search: the nearest point with the target clearance, or, inside a gap narrower
         * than 2*target, the point with the largest clearance (the gap centre). Works for samples
         * lying inside an obstacle cell too, where a gradient would be zero (FAR's polygon
         * approximation can cut a few centimetres into the real obstacle cells). */
        cv::Point2f best = p0;
        float best_c = c0;
        // ClearanceAt returns a large sentinel (1e3) outside the windowed distance map; ignore it so
        // a sample near the window edge is not dragged to the boundary by a fake-huge clearance.
        static const float kMaxUsefulClearance = 1e2f;
        for (float r = step; r <= max_shift + 1e-6f; r += step) {
            for (int k = 0; k < kDirs; ++k) {
                const float a = (float)k * 2.0f * (float)M_PI / (float)kDirs;
                const cv::Point2f q(p0.x + r * std::cos(a), p0.y + r * std::sin(a));
                const float cq = this->ClearanceAt(q.x, q.y);
                if (cq > best_c && cq < kMaxUsefulClearance) best = q, best_c = cq;
            }
            if (best_c >= target) break;  // smallest shift that reaches the target
        }
        pts[i] = best;
        refined_min = std::min(refined_min, best_c);
    }
    ROS_INFO_THROTTLE(2.0, "FARPlannerROS: plan clearance to obstacles: raw min %.3f m -> refined min %.3f m (target %.2f m)",
                      raw_min, refined_min, target);
    // drop samples that collapsed onto each other, then restore uniform spacing
    std::vector<cv::Point2f> compact;
    compact.reserve(pts.size());
    for (const auto& p : pts) {
        if (!compact.empty() && std::hypot(p.x - compact.back().x, p.y - compact.back().y) < 0.25f * (float)dist_res_) continue;
        compact.push_back(p);
    }
    if (compact.size() < 2) return;
    compact.back() = pts.back();
    std::vector<cv::Point2f> dense;
    this->DensifyPolyline(compact, std::max(master_params_.plan_resolution, 0.01f), dense);
    pts.swap(dense);
}

void FARPlannerROS::BuildPlan(const std::vector<Point3D>& waypoints,
                              const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan) const {
    plan.clear();
    if (waypoints.empty()) return;
    const float res = std::max(master_params_.plan_resolution, 0.01f);
    std::vector<cv::Point2f> poly, pts;
    poly.reserve(waypoints.size());
    for (const auto& w : waypoints) poly.emplace_back(w.x, w.y);
    this->DensifyPolyline(poly, res, pts);
    this->RefinePoints(pts);
    if (pts.empty()) return;

    const ros::Time stamp = ros::Time::now();  // NavfnROS convention: one stamp for the whole plan
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp    = stamp;
    pose.pose.position.z = 0.0;
    const geometry_msgs::Quaternion& gq = goal.pose.orientation;
    const double gqn = gq.x*gq.x + gq.y*gq.y + gq.z*gq.z + gq.w*gq.w;
    double last_yaw = (gqn > 0.5) ? tf2::getYaw(gq) : 0.0;
    plan.reserve(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
        double yaw = last_yaw;
        if (i + 1 < pts.size()) {
            const float dx = pts[i + 1].x - pts[i].x;
            const float dy = pts[i + 1].y - pts[i].y;
            if (std::hypot(dx, dy) > 1e-5f) yaw = std::atan2(dy, dx);
        }
        pose.pose.position.x = pts[i].x;
        pose.pose.position.y = pts[i].y;
        if (i + 1 == pts.size() && gqn > 0.5) {
            pose.pose.orientation = gq;  // final pose: exact goal orientation
        } else {
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            pose.pose.orientation = tf2::toMsg(q);
        }
        plan.push_back(pose);
        last_yaw = yaw;
    }
}

void FARPlannerROS::PublishDebugViz() {
    const bool want_obs  = obs_debug_pub_.getNumSubscribers() > 0;
    const bool want_free = free_debug_pub_.getNumSubscribers() > 0;
    const bool want_img  = image_pub_.getNumSubscribers() > 0;
    if (!(want_obs || want_free || want_img) || master_params_.debug_viz_rate <= 0.0f) return;
    const ros::Time now = ros::Time::now();
    if (now < last_debug_time_) last_debug_time_ = ros::Time(0);  // sim time reset
    if ((now - last_debug_time_).toSec() < 1.0 / master_params_.debug_viz_rate) return;
    last_debug_time_ = now;
    /* FARMaster debug clouds: the obstacle / free cells FAR currently uses (from the global costmap) */
    if (want_obs)  planner_viz_->VizPointCloud(obs_debug_pub_,  FARUtil::surround_obs_cloud_);
    if (want_free) planner_viz_->VizPointCloud(free_debug_pub_, FARUtil::surround_free_cloud_);
    if (!want_img) return;
    /* FAR's "Obstacle Cloud Image" (is_opencv_visual in the original node), streamed instead of cv::imshow */
    cv::Mat far_img, img;
    contour_detector_.DrawCornerImage(contour_detector_.GetNewVertices(), far_img);
    if (far_img.empty()) return;
    // FAR's image has rows along +x and columns along +y; rotate it to the rviz top-down view (+x right, +y up)
    cv::transpose(far_img, img);
    cv::flip(img, img, 0);
    if (!img.isContinuous()) img = img.clone();
    sensor_msgs::Image msg;
    msg.header.stamp    = now;
    msg.header.frame_id = global_frame_;
    msg.height          = (uint32_t)img.rows;
    msg.width           = (uint32_t)img.cols;
    msg.encoding        = "bgr8";
    msg.is_bigendian    = 0;
    msg.step            = (uint32_t)(img.cols * img.elemSize());
    msg.data.assign(img.data, img.data + (std::size_t)msg.step * (std::size_t)msg.height);
    image_pub_.publish(msg);
}

void FARPlannerROS::PublishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    nav_msgs::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp    = plan.empty() ? ros::Time::now() : plan.front().header.stamp;
    path.poses = plan;
    plan_pub_.publish(path);
}

/***************************************************************************************/

bool FARPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    bench_ = FARBench();
    const ros::WallTime t_call = ros::WallTime::now();
    const char* outcome = "failed";
    const bool ok = this->MakePlanImpl(start, goal, plan, outcome);
    if (initialized_ && master_params_.bench_log_enabled) {
        this->LogBench(MsSince(t_call), outcome, plan.size());
    }
    return ok;
}

bool FARPlannerROS::MakePlanImpl(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan,
                                 const char*& outcome) {
    plan.clear();
    if (!initialized_) {
        ROS_ERROR_THROTTLE(1.0, "FARPlannerROS: not initialized, call initialize() first");
        outcome = "not_initialized";
        return false;
    }
    /* 1. frames */
    geometry_msgs::PoseStamped start_g, goal_g;
    if (!this->ToPlanningFrame(start, start_g) || !this->ToPlanningFrame(goal, goal_g)) {
        outcome = "frame_error";
        return false;
    }
    /* 2. robot state from the start pose */
    this->UpdateRobotState(start_g);
    const Point3D goal_p((float)goal_g.pose.position.x, (float)goal_g.pose.position.y, robot_pos_.z);
    const float goal_dist = (goal_p - robot_pos_).norm_flat();
    if ((goal_p - last_goal_).norm_flat() > master_params_.goal_update_tolerance) {  // new goal
        last_waypoints_.clear();
        fail_count_ = 0;
    }

    /* 3. environment + v-graph update (rate limited by main_run_freq) */
    const ros::Time now = ros::Time::now();
    const bool do_update = !is_graph_init_ ||
                           (now - last_update_time_).toSec() >= (0.99 / master_params_.main_run_freq);
    if (do_update) {
        bench_.updated = true;
        ros::WallTime t_stage = ros::WallTime::now();
        this->UpdateTerrainFromCostmap();
        bench_.terrain = MsSince(t_stage);
        t_stage = ros::WallTime::now();
        this->UpdateDistanceMap();
        bench_.dist_map = MsSince(t_stage);
        if (is_cloud_init_) {
            this->UpdateVGraph();  // fills bench_.contours / vgraph / viz
            t_stage = ros::WallTime::now();
            this->PublishDebugViz();
            bench_.debug_viz = MsSince(t_stage);
        }
        last_update_time_ = now;
    } else {
        graph_manager_->UpdateRobotPosition(robot_pos_);  // keep the odom node current for planning
        odom_node_ptr_ = graph_manager_->GetOdomNode();
    }

    /* 4. near the goal: hand the local planner a straight segment. FAR would otherwise
     *    reach converge_dist, drop the goal node and re-insert it every cycle. */
    if (goal_dist < gp_params_.converge_dist) {
        if (graph_planner_->GetGoalNodePtr() != NULL) graph_planner_->ClearGoal();
        std::vector<Point3D> wps{robot_pos_, goal_p};
        const ros::WallTime t_build = ros::WallTime::now();
        this->BuildPlan(wps, goal_g, plan);
        this->PublishPlan(plan);
        bench_.build_plan = MsSince(t_build);
        outcome = "near_goal";
        return true;
    }
    /* 5. no obstacle observed yet -> nothing can block a straight line (attemptable) */
    if (!is_cloud_init_ || !is_graph_init_ || odom_node_ptr_ == NULL) {
        ROS_WARN_THROTTLE(2.0, "FARPlannerROS: v-graph not initialized (no obstacles observed yet), returning straight-line plan");
        std::vector<Point3D> wps{robot_pos_, goal_p};
        last_waypoints_ = wps;
        const ros::WallTime t_build = ros::WallTime::now();
        this->BuildPlan(wps, goal_g, plan);
        this->PublishPlan(plan);
        bench_.build_plan = MsSince(t_build);
        outcome = "straight_line";
        return true;
    }
    /* 6. FAR planning on the v-graph */
    NodePtrStack global_path;
    bool is_reach_goal = false, is_free_nav = false;
    std::vector<Point3D> wps;
    const ros::WallTime t_search = ros::WallTime::now();
    const bool found = this->PlanToGoal(goal_p, global_path, is_reach_goal, is_free_nav);
    bench_.search = MsSince(t_search);
    if (found) {
        outcome = is_free_nav ? "path_free" : "path_attemptable";
        fail_count_ = 0;
        wps.reserve(global_path.size() + 1);
        for (const auto& node_ptr : global_path) {
            if (node_ptr == NULL) continue;
            if (!wps.empty() && (node_ptr->position - wps.back()).norm_flat() < 1e-3f) continue;
            wps.push_back(node_ptr->position);
        }
        if (wps.empty()) {
            outcome = "empty_path";
            return false;
        }
        wps.front() = robot_pos_;  // plan starts exactly at the start pose
    } else {
        /* FAR drops its goal node on failure and re-votes the goal edges after re-insertion, so a
         * failure can be transient (graph just changed, goal edge briefly blocked). Keep following
         * the previous plan for a bounded number of cycles before reporting failure to move_base. */
        ++fail_count_;
        if (last_waypoints_.empty() || fail_count_ > master_params_.max_plan_failures) {
            ROS_WARN_THROTTLE(1.0, "FARPlannerROS: no path to goal (%.2f, %.2f) found on the v-graph (%d consecutive failures)",
                              goal_p.x, goal_p.y, fail_count_);
            last_waypoints_.clear();
            outcome = "no_path";
            return false;
        }
        ROS_WARN_THROTTLE(1.0, "FARPlannerROS: no v-graph path this cycle (%d/%d), continuing the previous plan",
                          fail_count_, master_params_.max_plan_failures);
        wps = this->ResumeWaypoints(last_waypoints_, robot_pos_);
        outcome = "resumed_previous";
    }
    /* 7. v-graph path -> dense plan; FAR may shift the goal node into free space, the local planner
     *    still needs the real goal as the last pose */
    if ((wps.back() - goal_p).norm_flat() > master_params_.plan_resolution) {
        wps.push_back(goal_p);
    } else {
        wps.back() = goal_p;
    }
    last_waypoints_ = wps;
    const ros::WallTime t_build = ros::WallTime::now();
    this->BuildPlan(wps, goal_g, plan);
    this->PublishPlan(plan);
    bench_.build_plan = MsSince(t_build);
    ROS_DEBUG_THROTTLE(1.0, "FARPlannerROS: plan with %zu poses over %zu v-graph vertices (%s)",
                       plan.size(), global_path.size(), is_free_nav ? "free-space" : "attemptable");
    if (plan.empty()) outcome = "empty_path";
    return !plan.empty();
}

void FARPlannerROS::LogBench(const double total_ms, const char* outcome, const std::size_t n_poses) {
    /* Same style as the MPC's [BENCH] lines: wall-clock ms, one line per bench_log_period_s (sim time).
     * The stage breakdown is the last call's; calls/avg/max cover every call since the previous line. */
    ++bench_calls_;
    bench_sum_ms_ += total_ms;
    bench_max_ms_ = std::max(bench_max_ms_, total_ms);
    const ros::Time now = ros::Time::now();
    if (now < bench_last_log_) bench_last_log_ = ros::Time(0);  // sim time reset
    if ((now - bench_last_log_).toSec() < master_params_.bench_log_period_s) return;
    ROS_INFO_STREAM("[BENCH][FAR] total=" << total_ms << " ms"
                    << " | terrain=" << bench_.terrain << " ms"
                    << " | dist_map=" << bench_.dist_map << " ms"
                    << " | contours=" << bench_.contours << " ms"
                    << " | vgraph=" << bench_.vgraph << " ms"
                    << " | viz=" << bench_.viz << " ms"
                    << " | debug_viz=" << bench_.debug_viz << " ms"
                    << " | search=" << bench_.search << " ms"
                    << " | build_plan=" << bench_.build_plan << " ms"
                    << " | outcome=" << outcome << " updated=" << (bench_.updated ? 1 : 0)
                    << " | vertices=" << nav_graph_.size() << " contours=" << realworld_contour_.size()
                    << " poses=" << n_poses
                    << " | period: calls=" << bench_calls_ << " avg=" << bench_sum_ms_ / bench_calls_
                    << " ms max=" << bench_max_ms_ << " ms");
    bench_last_log_ = now;
    bench_calls_    = 0;
    bench_sum_ms_   = bench_max_ms_ = 0.0;
}

}  // namespace far_planner
