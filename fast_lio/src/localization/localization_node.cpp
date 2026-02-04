/**
 * @file localization_node.cpp
 * @brief Map-based localization using Open3D ICP registration
 *
 * This node subscribes to FAST-LIO odometry and point clouds,
 * performs ICP-based localization against a pre-built map,
 * and publishes the map->odom transform.
 *
 * Features:
 * - Multi-scale ICP registration
 * - FPFH + RANSAC global localization with multi-hypothesis
 * - Kalman filtering for pose smoothing (X, Y, Z axes)
 * - Distance-based submap update for efficiency
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <cmath>
#include <atomic>
#include <filesystem>
#include <optional>

#include "fast_lio/localization/open3d_registration.h"
#include "fast_lio/localization/open3d_conversions.h"
#include "fast_lio/localization/occupancy_grid_generator.hpp"

/**
 * @brief Simple 1D Kalman Filter for pose smoothing
 *
 * Applied independently to X, Y, Z axes to reduce high-frequency noise
 * from ICP registration results.
 */
class KalmanFilter
{
public:
    KalmanFilter() : process_var_(0.0), meas_var_(0.0),
                     post_estimate_(0.0), post_error_estimate_(1.0) {}

    void init(double process_var, double meas_var, double initial_value = 0.0, double initial_error = 1.0)
    {
        process_var_ = process_var;
        meas_var_ = meas_var;
        post_estimate_ = initial_value;
        post_error_estimate_ = initial_error;
    }

    void update(double measurement)
    {
        double prior_estimate = post_estimate_;
        double prior_error = post_error_estimate_ + process_var_;

        double denominator = prior_error + meas_var_;

        // Prevent division by zero
        if (std::abs(denominator) < 1e-10)
        {
            post_estimate_ = measurement;
            post_error_estimate_ = 1.0;
            return;
        }

        double kalman_gain = prior_error / denominator;
        post_estimate_ = prior_estimate + kalman_gain * (measurement - prior_estimate);
        post_error_estimate_ = (1 - kalman_gain) * prior_error;
    }

    double getEstimate() const { return post_estimate_; }

private:
    double process_var_;
    double meas_var_;
    double post_estimate_;
    double post_error_estimate_;
};

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode() : Node("localization_node"),
                         tf_buffer_(this->get_clock()),
                         tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
    {
        // Initialize matrices
        mat_body2odom_ = Eigen::Matrix4d::Identity();
        mat_odom2map_ = Eigen::Matrix4d::Identity();
        mat_initialpose_ = Eigen::Matrix4d::Identity();
        last_loc_position_ = Eigen::Vector3d(0, 0, -5000);

        // Initialize point clouds
        pcd_map_ori_ = std::make_shared<open3d::geometry::PointCloud>();
        pcd_map_fine_ = std::make_shared<open3d::geometry::PointCloud>();
        pcd_scan_cur_ = std::make_shared<open3d::geometry::PointCloud>();

        // Declare parameters
        this->declare_parameter<std::string>("map_path", "");
        this->declare_parameter<double>("voxel_size_fine", 0.05);
        this->declare_parameter<double>("loc_frequency", 2.0);
        this->declare_parameter<double>("fitness_threshold", 0.5);
        this->declare_parameter<double>("submap_radius", 50.0);
        this->declare_parameter<int>("max_points_source", 100000);
        this->declare_parameter<int>("max_points_target", 500000);
        this->declare_parameter<std::vector<double>>("initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});

        // Global localization parameters
        this->declare_parameter<bool>("use_global_localization", true);
        this->declare_parameter<double>("voxel_size_global", 0.25);
        this->declare_parameter<double>("fitness_threshold_global", 0.15);

        // Kalman filter parameters [process_var, measurement_var]
        this->declare_parameter<std::vector<double>>("kalman_x", std::vector<double>{0.001, 0.002});
        this->declare_parameter<std::vector<double>>("kalman_y", std::vector<double>{0.001, 0.005});
        this->declare_parameter<std::vector<double>>("kalman_z", std::vector<double>{0.00001, 0.04});

        // Distance-based submap update
        this->declare_parameter<double>("dis_updatemap", 3.0);

        // Point cloud queue size
        this->declare_parameter<int>("pcd_queue_maxsize", 5);

        // Debug options
        this->declare_parameter<bool>("save_scan", false);
        this->declare_parameter<std::string>("save_scan_path", "/tmp/localization_scans/");

        // OccupancyGrid parameters
        this->declare_parameter<bool>("occupancy_grid.publish", true);
        this->declare_parameter<double>("occupancy_grid.resolution", 0.05);
        this->declare_parameter<double>("occupancy_grid.z_min", -0.5);
        this->declare_parameter<double>("occupancy_grid.z_max", 1.5);

        // TF frame names
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "camera_init");
        this->declare_parameter<std::string>("body_frame", "body");

        // Get parameters
        std::string map_path;
        this->get_parameter("map_path", map_path);
        this->get_parameter("voxel_size_fine", voxel_size_fine_);
        this->get_parameter("loc_frequency", loc_frequency_);
        this->get_parameter("fitness_threshold", fitness_threshold_);
        this->get_parameter("submap_radius", submap_radius_);
        this->get_parameter("max_points_source", max_points_source_);
        this->get_parameter("max_points_target", max_points_target_);
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("body_frame", body_frame_);
        this->get_parameter("use_global_localization", use_global_localization_);
        this->get_parameter("voxel_size_global", voxel_size_global_);
        this->get_parameter("fitness_threshold_global", fitness_threshold_global_);

        // Kalman filter parameters
        std::vector<double> kf_x, kf_y, kf_z;
        this->get_parameter("kalman_x", kf_x);
        this->get_parameter("kalman_y", kf_y);
        this->get_parameter("kalman_z", kf_z);
        if (kf_x.size() >= 2) kalman_params_x_ = kf_x;
        if (kf_y.size() >= 2) kalman_params_y_ = kf_y;
        if (kf_z.size() >= 2) kalman_params_z_ = kf_z;

        // Distance-based submap update
        this->get_parameter("dis_updatemap", dis_updatemap_);

        // Point cloud queue size
        this->get_parameter("pcd_queue_maxsize", queue_max_size_);

        // Debug options
        this->get_parameter("save_scan", save_scan_);
        this->get_parameter("save_scan_path", save_scan_path_);

        // OccupancyGrid parameters
        this->get_parameter("occupancy_grid.publish", og_publish_);
        this->get_parameter("occupancy_grid.resolution", og_resolution_);
        this->get_parameter("occupancy_grid.z_min", og_z_min_);
        this->get_parameter("occupancy_grid.z_max", og_z_max_);

        std::vector<double> initial_pose;
        this->get_parameter("initial_pose", initial_pose);
        if (initial_pose.size() >= 6)
        {
            setInitialPose(initial_pose[0], initial_pose[1], initial_pose[2],
                           initial_pose[3], initial_pose[4], initial_pose[5]);
        }

        // Load map
        if (map_path.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "map_path parameter is empty!");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_path.c_str());
        if (!open3d::io::ReadPointCloud(map_path, *pcd_map_ori_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map from: %s", map_path.c_str());
            rclcpp::shutdown();
            return;
        }

        if (pcd_map_ori_->IsEmpty())
        {
            RCLCPP_ERROR(this->get_logger(), "Map is empty!");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Map loaded with %zu points", pcd_map_ori_->points_.size());

        // Downsample and compute normals
        pcd_map_fine_ = pcd_map_ori_->VoxelDownSample(voxel_size_fine_);
        pcd_map_fine_->EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(voxel_size_fine_ * 2, 30));
        RCLCPP_INFO(this->get_logger(), "Map downsampled to %zu points", pcd_map_fine_->points_.size());

        // Create publishers
        pub_odom2map_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/odom2map", 10);
        pub_body2map_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/body2map", 10);
        pub_body2map_kalman_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/body2map_kalman", 10);
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 10);
        pub_confidence_ = this->create_publisher<std_msgs::msg::Float32>("/localization/confidence", 10);
        pub_delay_ms_ = this->create_publisher<std_msgs::msg::Float32>("/localization/delay_ms", 10);

        // Use transient_local QoS for map so late subscribers receive it
        rclcpp::QoS map_qos(1);
        map_qos.transient_local();
        pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/localization/map", map_qos);

        // OccupancyGrid publisher (transient_local for late subscribers)
        if (og_publish_)
        {
            pub_occupancy_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                "/localization/occupancy_grid", map_qos);
        }

        // Create subscribers
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fast_lio/odometry", 50,
            std::bind(&LocalizationNode::odomCallback, this, std::placeholders::_1));

        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/fast_lio/cloud_registered_body", 50,
            std::bind(&LocalizationNode::cloudCallback, this, std::placeholders::_1));

        sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&LocalizationNode::initialPoseCallback, this, std::placeholders::_1));

        // Create TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publish map (once)
        publishMap();

        // Publish OccupancyGrid (once)
        if (og_publish_)
        {
            publishOccupancyGrid();
        }

        // Create TF timer for consistent TF publishing (50Hz)
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&LocalizationNode::tfTimerCallback, this));

        // Publish initial TF immediately
        publishTF(this->now());

        // Start localization thread
        loc_thread_ = std::thread(&LocalizationNode::localizationLoop, this);

        RCLCPP_INFO(this->get_logger(), "Localization node initialized");
    }

    ~LocalizationNode()
    {
        running_ = false;
        if (loc_thread_.joinable())
        {
            loc_thread_.join();
        }
    }

private:
    // ==================== Helper Functions ====================

    /**
     * @brief Convert 4x4 transformation matrix to geometry_msgs::msg::Pose
     */
    geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d& mat)
    {
        Eigen::Isometry3d iso;
        iso.matrix() = mat;
        return tf2::toMsg(iso);
    }

    /**
     * @brief Create an Odometry message from transform matrix
     */
    nav_msgs::msg::Odometry createOdometryMsg(
        const rclcpp::Time& stamp,
        const std::string& frame_id,
        const std::string& child_frame_id,
        const Eigen::Matrix4d& transform)
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        msg.child_frame_id = child_frame_id;
        msg.pose.pose = matrixToPose(transform);
        return msg;
    }

    /**
     * @brief Create an OrientedBoundingBox for point cloud cropping
     */
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> createCropBox(
        const Eigen::Vector3d& center,
        const Eigen::Matrix3d& rotation,
        const Eigen::Vector3d& extent)
    {
        auto obb = std::make_shared<open3d::geometry::OrientedBoundingBox>();
        obb->center_ = center;
        obb->R_ = rotation;
        obb->extent_ = extent;
        return obb;
    }

    /**
     * @brief Limit point cloud to maximum number of points via random downsampling
     */
    std::shared_ptr<open3d::geometry::PointCloud> limitPointCloud(
        std::shared_ptr<open3d::geometry::PointCloud> cloud,
        size_t max_points)
    {
        if (cloud->points_.size() > max_points)
        {
            double ratio = static_cast<double>(max_points) / cloud->points_.size();
            return cloud->RandomDownSample(ratio);
        }
        return cloud;
    }

    // ==================== Global Localization Helpers ====================

    /**
     * @brief Result of a single hypothesis evaluation
     */
    struct HypothesisResult
    {
        double fitness = 0.0;
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    };

    /**
     * @brief Prepare scan for global localization
     * @return pair of (scan point cloud in odom frame, body2odom transform), or nullopt if no scan
     */
    std::optional<std::pair<std::shared_ptr<open3d::geometry::PointCloud>, Eigen::Matrix4d>>
    prepareScanForGlobalLocalization()
    {
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        std::lock_guard<std::mutex> lock_cloud(cloud_mutex_);

        if (pcd_scan_cur_->IsEmpty())
        {
            RCLCPP_WARN(this->get_logger(), "No scan data for global localization");
            return std::nullopt;
        }

        auto pcd_scan = std::make_shared<open3d::geometry::PointCloud>(*pcd_scan_cur_);
        Eigen::Matrix4d mat_body2odom_cur = mat_body2odom_;

        // Transform scan to odom frame
        pcd_scan->Transform(mat_body2odom_cur);

        return std::make_pair(pcd_scan, mat_body2odom_cur);
    }

    /**
     * @brief Ensure map FPFH features are computed (lazy initialization)
     */
    void ensureMapFpfhComputed(double voxel_size)
    {
        if (!map_fpfh_computed_)
        {
            RCLCPP_INFO(this->get_logger(), "Computing FPFH features for map...");
            std::tie(pcd_map_global_, map_fpfh_) = pcd_tools::PreprocessPointCloudForFpfh(
                pcd_map_ori_, voxel_size);
            map_fpfh_computed_ = true;
            RCLCPP_INFO(this->get_logger(), "Map FPFH computed (%zu points)", pcd_map_global_->points_.size());
        }
    }

    /**
     * @brief Generate translation candidates for global localization
     * @param fpfh_result Result from FPFH registration
     * @param scan_down Downsampled scan point cloud
     * @return Vector of translation candidates and updated FPFH transform
     */
    std::pair<std::vector<Eigen::Vector3d>, Eigen::Matrix4d> generateTranslationCandidates(
        const open3d::pipelines::registration::RegistrationResult& fpfh_result,
        std::shared_ptr<open3d::geometry::PointCloud> scan_down)
    {
        std::vector<Eigen::Vector3d> translation_candidates;
        Eigen::Matrix4d fpfh_transform = fpfh_result.transformation_;

        // If FPFH completely failed (fitness < 0.02), try grid search around map centroid
        if (fpfh_result.fitness_ < 0.02)
        {
            RCLCPP_WARN(this->get_logger(), "FPFH failed, trying grid search around map centroid");
            Eigen::Vector3d map_centroid = pcd_map_global_->GetCenter();
            Eigen::Vector3d scan_centroid = scan_down->GetCenter();
            Eigen::Vector3d base_translation = map_centroid - scan_centroid;

            // Get map bounding box to determine grid step
            auto bbox = pcd_map_global_->GetAxisAlignedBoundingBox();
            double map_extent = std::max({
                bbox.max_bound_.x() - bbox.min_bound_.x(),
                bbox.max_bound_.y() - bbox.min_bound_.y()
            });
            double grid_step = map_extent / 4.0;  // Divide map into 4x4 regions

            // Create 3x3 grid around centroid
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    Eigen::Vector3d offset(dx * grid_step, dy * grid_step, 0.0);
                    translation_candidates.push_back(base_translation + offset);
                }
            }
            fpfh_transform = Eigen::Matrix4d::Identity();
            RCLCPP_INFO(this->get_logger(), "Grid search: %zu positions, step=%.2fm",
                        translation_candidates.size(), grid_step);
        }
        else
        {
            // Use FPFH translation
            translation_candidates.push_back(fpfh_transform.block<3, 1>(0, 3));
        }

        return {translation_candidates, fpfh_transform};
    }

    /**
     * @brief Evaluate a single hypothesis for global localization
     * @param scan_down Downsampled scan for ICP
     * @param pcd_scan_original Original scan for evaluation
     * @param translation Translation to apply
     * @param yaw_offset Yaw rotation offset in radians
     * @param base_rotation Base rotation from FPFH
     * @param voxel_size Voxel size for ICP
     * @return HypothesisResult with fitness and final transform
     */
    HypothesisResult evaluateSingleHypothesis(
        std::shared_ptr<open3d::geometry::PointCloud> scan_down,
        std::shared_ptr<open3d::geometry::PointCloud> pcd_scan_original,
        const Eigen::Vector3d& translation,
        double yaw_offset,
        const Eigen::Matrix3d& base_rotation,
        double voxel_size)
    {
        // Create rotation matrix for yaw offset around Z-axis
        Eigen::Matrix3d R_offset;
        R_offset = Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ());

        // Apply rotation offset to base rotation
        Eigen::Matrix4d hypothesis_transform = Eigen::Matrix4d::Identity();
        hypothesis_transform.block<3, 3>(0, 0) = R_offset * base_rotation;
        hypothesis_transform.block<3, 1>(0, 3) = translation;

        // Transform scan with this hypothesis
        auto scan_hypothesis = std::make_shared<open3d::geometry::PointCloud>(*scan_down);
        scan_hypothesis->Transform(hypothesis_transform);

        // Refine with ICP
        auto icp_result = pcd_tools::RegistrationMultiScaleIcp(
            scan_hypothesis, pcd_map_global_, voxel_size, 1, {1, 2, 3});

        // Compute final transformation for this hypothesis
        Eigen::Matrix4d final_transform = icp_result * hypothesis_transform;

        // Evaluate result
        auto eval = open3d::pipelines::registration::EvaluateRegistration(
            *pcd_scan_original, *pcd_map_ori_, voxel_size * 2, final_transform);

        return {eval.fitness_, final_transform};
    }

    // ==================== Core Functions ====================

    void setInitialPose(double x, double y, double z, double roll, double pitch, double yaw)
    {
        // Convert RPY (degrees) to rotation matrix
        Eigen::AngleAxisd rollAngle(roll * M_PI / 180.0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ());

        Eigen::Matrix3d rotation = (yawAngle * pitchAngle * rollAngle).matrix();

        mat_initialpose_.block<3, 3>(0, 0) = rotation;
        mat_initialpose_.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
        mat_odom2map_ = mat_initialpose_;

        RCLCPP_INFO(this->get_logger(), "Initial pose set: [%.2f, %.2f, %.2f] RPY: [%.1f, %.1f, %.1f]",
                    x, y, z, roll, pitch, yaw);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_time_ = msg->header.stamp;

        // Convert pose to matrix
        Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
        tf2::fromMsg(msg->pose.pose, isometry);
        mat_body2odom_ = isometry.matrix();

        // Compute body to map
        Eigen::Matrix4d mat_body2map = mat_odom2map_ * mat_body2odom_;

        // Publish body2map odometry
        pub_body2map_->publish(
            createOdometryMsg(msg->header.stamp, map_frame_, body_frame_, mat_body2map));

        // Publish odom2map odometry
        pub_odom2map_->publish(
            createOdometryMsg(msg->header.stamp, map_frame_, odom_frame_, mat_odom2map_));

        // TF is now published by tfTimerCallback at 50Hz

        // Publish pose with Kalman filtering
        if (loc_initialized_)
        {
            // Apply Kalman filtering to body2map position
            Eigen::Matrix4d mat_body2map_kalman = mat_body2map;

            kf_x_.update(mat_body2map(0, 3));
            kf_y_.update(mat_body2map(1, 3));
            kf_z_.update(mat_body2map(2, 3));

            double filtered_x = kf_x_.getEstimate();
            double filtered_y = kf_y_.getEstimate();
            double filtered_z = kf_z_.getEstimate();

            // NaN protection
            if (!std::isnan(filtered_x)) mat_body2map_kalman(0, 3) = filtered_x;
            if (!std::isnan(filtered_y)) mat_body2map_kalman(1, 3) = filtered_y;
            if (!std::isnan(filtered_z)) mat_body2map_kalman(2, 3) = filtered_z;

            // Publish Kalman-filtered body2map
            auto body2map_kalman_msg = createOdometryMsg(
                msg->header.stamp, map_frame_, body_frame_, mat_body2map_kalman);
            pub_body2map_kalman_->publish(body2map_kalman_msg);

            // Publish pose (Kalman-filtered)
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = msg->header.stamp;
            pose_msg.header.frame_id = map_frame_;
            pose_msg.pose = body2map_kalman_msg.pose.pose;
            pub_pose_->publish(pose_msg);

            // Publish confidence
            std_msgs::msg::Float32 conf_msg;
            conf_msg.data = current_fitness_;
            pub_confidence_->publish(conf_msg);

            // Publish processing delay
            std_msgs::msg::Float32 delay_msg;
            delay_msg.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
            pub_delay_ms_->publish(delay_msg);
        }
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        // Convert to Open3D
        open3d::geometry::PointCloud pcd_received;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr const_msg = msg;
        open3d_conversions::rosToOpen3d(const_msg, pcd_received, true);

        // Add to queue
        scan_queue_.push(pcd_received);
        if (scan_queue_.size() > static_cast<size_t>(queue_max_size_))
        {
            scan_queue_.pop();
        }

        // Merge queue into current scan
        pcd_scan_cur_->Clear();
        std::queue<open3d::geometry::PointCloud> temp_queue = scan_queue_;
        while (!temp_queue.empty())
        {
            *pcd_scan_cur_ += temp_queue.front();
            temp_queue.pop();
        }
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received initial pose from RVIZ");

        // Extract pose
        Eigen::Quaterniond q(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);

        std::lock_guard<std::mutex> lock(odom_mutex_);
        mat_initialpose_.block<3, 3>(0, 0) = q.matrix();
        mat_initialpose_.block<3, 1>(0, 3) = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);

        mat_odom2map_ = mat_initialpose_;
        loc_initialized_ = false;  // Re-initialize localization
        RCLCPP_INFO(this->get_logger(), "Initial pose updated, re-initializing localization");
    }

    void publishMap()
    {
        sensor_msgs::msg::PointCloud2 map_msg;
        auto map_coarse = pcd_map_ori_->VoxelDownSample(0.5);  // Coarse for visualization
        open3d_conversions::open3dToRos(*map_coarse, map_msg, map_frame_);
        map_msg.header.stamp = this->now();
        pub_map_->publish(map_msg);
    }

    void publishOccupancyGrid()
    {
        RCLCPP_INFO(this->get_logger(), "Generating OccupancyGrid from map (z: [%.2f, %.2f], res: %.3f)...",
                    og_z_min_, og_z_max_, og_resolution_);

        occupancy_grid::OccupancyGridParams og_params;
        og_params.resolution = og_resolution_;
        og_params.z_min = og_z_min_;
        og_params.z_max = og_z_max_;
        og_params.frame_id = map_frame_;

        occupancy_grid::OccupancyGridGenerator generator(og_params);
        auto grid = generator.generate(*pcd_map_ori_, this->now());

        if (grid)
        {
            RCLCPP_INFO(this->get_logger(), "OccupancyGrid generated: %u x %u cells (%.1f x %.1f m)",
                        grid->info.width, grid->info.height,
                        grid->info.width * og_resolution_,
                        grid->info.height * og_resolution_);
            pub_occupancy_grid_->publish(std::move(*grid));
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to generate OccupancyGrid (no points in Z range?)");
        }
    }

    void tfTimerCallback()
    {
        publishTF(this->now());
    }

    void publishTF(const rclcpp::Time& stamp)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = map_frame_;
        tf_msg.child_frame_id = odom_frame_;
        tf_msg.transform.translation.x = mat_odom2map_(0, 3);
        tf_msg.transform.translation.y = mat_odom2map_(1, 3);
        tf_msg.transform.translation.z = mat_odom2map_(2, 3);
        Eigen::Quaterniond q(mat_odom2map_.block<3, 3>(0, 0));
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf_msg);
    }

    void localizationLoop()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for odometry...");
        while (running_ && rclcpp::ok() && latest_odom_time_.seconds() == 0.0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(this->get_logger(), "Odometry received");

        // Wait for sufficient point cloud accumulation for global localization
        const size_t min_points_for_global = 5000;
        RCLCPP_INFO(this->get_logger(), "Waiting for point cloud (min %zu points for global loc)...",
                    min_points_for_global);
        while (running_ && rclcpp::ok())
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!pcd_scan_cur_->IsEmpty())
            {
                size_t num_points = pcd_scan_cur_->points_.size();
                if (num_points >= min_points_for_global || !use_global_localization_)
                {
                    RCLCPP_INFO(this->get_logger(), "Point cloud ready: %zu points", num_points);
                    break;
                }
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Accumulating points: %zu / %zu", num_points, min_points_for_global);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(this->get_logger(), "Point cloud received");

        // Initialize localization
        RCLCPP_INFO(this->get_logger(), "Starting localization initialization...");
        initializeLocalization();

        // Initialize Kalman filters with current position
        Eigen::Matrix4d init_body2map;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            init_body2map = mat_odom2map_ * mat_body2odom_;
        }

        RCLCPP_INFO(this->get_logger(), "Initializing Kalman filters with position: x=%.3f, y=%.3f, z=%.3f",
                    init_body2map(0, 3), init_body2map(1, 3), init_body2map(2, 3));

        kf_x_.init(kalman_params_x_[0], kalman_params_x_[1], init_body2map(0, 3), 1.0);
        kf_y_.init(kalman_params_y_[0], kalman_params_y_[1], init_body2map(1, 3), 1.0);
        kf_z_.init(kalman_params_z_[0], kalman_params_z_[1], init_body2map(2, 3), 1.0);

        RCLCPP_INFO(this->get_logger(), "Kalman filters: x[%.6f,%.6f], y[%.6f,%.6f], z[%.6f,%.6f]",
                    kalman_params_x_[0], kalman_params_x_[1],
                    kalman_params_y_[0], kalman_params_y_[1],
                    kalman_params_z_[0], kalman_params_z_[1]);

        loc_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Localization initialized with Kalman filtering");

        // Main localization loop
        auto last_loc_time = std::chrono::high_resolution_clock::now();
        double loc_interval_sec = 1.0 / loc_frequency_;

        while (running_ && rclcpp::ok())
        {
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_loc_time).count();

            if (elapsed < loc_interval_sec)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            performLocalization();
            last_loc_time = std::chrono::high_resolution_clock::now();
        }
    }

    void initializeLocalization()
    {
        // Try global localization first if enabled and no valid initial pose
        bool need_global = use_global_localization_ &&
                          (mat_initialpose_.block<3, 1>(0, 3).norm() < 0.1);

        if (need_global)
        {
            RCLCPP_INFO(this->get_logger(), "Attempting FPFH-based global localization...");

            const int max_global_attempts = 10;
            const double good_fitness_threshold = 0.8;  // Target fitness for global loc
            double best_global_fitness = 0.0;
            Eigen::Matrix4d best_global_transform = mat_odom2map_;

            for (int attempt = 1; attempt <= max_global_attempts && running_ && rclcpp::ok(); attempt++)
            {
                RCLCPP_INFO(this->get_logger(), "Global localization attempt %d/%d...", attempt, max_global_attempts);

                double fitness = performGlobalLocalizationWithFitness();

                if (fitness > best_global_fitness)
                {
                    best_global_fitness = fitness;
                    std::lock_guard<std::mutex> lock(odom_mutex_);
                    best_global_transform = mat_odom2map_;
                }

                RCLCPP_INFO(this->get_logger(), "Attempt %d fitness: %.3f, best so far: %.3f",
                            attempt, fitness, best_global_fitness);

                if (best_global_fitness >= good_fitness_threshold)
                {
                    RCLCPP_INFO(this->get_logger(), "Good fitness achieved (%.3f >= %.3f), stopping search",
                                best_global_fitness, good_fitness_threshold);
                    break;
                }

                // Wait a bit for new scan data
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            // Apply best result
            {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                mat_odom2map_ = best_global_transform;
            }

            if (best_global_fitness >= good_fitness_threshold)
            {
                RCLCPP_INFO(this->get_logger(), "Global localization: GOOD (fitness %.3f >= %.1f)", best_global_fitness, good_fitness_threshold);
            }
            else if (best_global_fitness >= fitness_threshold_global_)
            {
                RCLCPP_WARN(this->get_logger(), "Global localization: MARGINAL (fitness %.3f), may need manual adjustment", best_global_fitness);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Global localization: FAILED (fitness %.3f < %.3f), use RVIZ 2D Pose Estimate",
                             best_global_fitness, fitness_threshold_global_);
            }
        }

        // Fine-tune with ICP
        int success_count = 0;
        const int required_success = 2;
        int fail_count = 0;
        const int max_fails = 10;

        double final_init_fitness = 0.0;
        while (running_ && rclcpp::ok() && success_count < required_success && fail_count < max_fails)
        {
            double fitness = performLocalizationStep(true);
            final_init_fitness = fitness;

            if (fitness > fitness_threshold_)
            {
                success_count++;
                fail_count = 0;
                RCLCPP_INFO(this->get_logger(), "Init ICP pass %d/%d (fitness: %.3f)",
                            success_count, required_success, fitness);
            }
            else
            {
                success_count = 0;
                fail_count++;
                RCLCPP_WARN(this->get_logger(), "Init ICP failed %d/%d (fitness: %.3f < %.3f)",
                            fail_count, max_fails, fitness, fitness_threshold_);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (fail_count >= max_fails)
        {
            RCLCPP_ERROR(this->get_logger(), "INITIALIZATION FAILED after %d attempts. Use RVIZ 2D Pose Estimate.", max_fails);
        }
        else if (final_init_fitness >= 0.7)
        {
            RCLCPP_INFO(this->get_logger(), "INITIALIZATION: EXCELLENT (fitness %.3f)", final_init_fitness);
        }
        else if (final_init_fitness >= 0.5)
        {
            RCLCPP_INFO(this->get_logger(), "INITIALIZATION: GOOD (fitness %.3f)", final_init_fitness);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "INITIALIZATION: MARGINAL (fitness %.3f) - tracking may be unstable", final_init_fitness);
        }
    }

    // Returns the best fitness achieved (0.0 if failed)
    double performGlobalLocalizationWithFitness()
    {
        // Step 1: Prepare scan for global localization
        auto scan_result = prepareScanForGlobalLocalization();
        if (!scan_result.has_value())
        {
            return 0.0;
        }
        auto [pcd_scan, mat_body2odom_cur] = scan_result.value();

        double effective_voxel_size = voxel_size_global_;

        // Preprocess scan for FPFH
        auto [scan_down, scan_fpfh] = pcd_tools::PreprocessPointCloudForFpfh(pcd_scan, effective_voxel_size);

        if (scan_down->points_.size() < 500)
        {
            RCLCPP_WARN(this->get_logger(), "Not enough scan points: %zu < 500", scan_down->points_.size());
            return 0.0;
        }

        // Step 2: Ensure map FPFH is computed
        ensureMapFpfhComputed(effective_voxel_size);

        // Perform FPFH + RANSAC global registration (RANSAC has randomness, so results vary)
        auto fpfh_result = pcd_tools::RegistrationFpfh(
            scan_down, pcd_map_global_, scan_fpfh, map_fpfh_, effective_voxel_size, true);

        RCLCPP_INFO(this->get_logger(), "FPFH result: fitness=%.3f, rmse=%.3f",
                    fpfh_result.fitness_, fpfh_result.inlier_rmse_);

        // Step 3: Generate translation candidates
        auto [translation_candidates, fpfh_transform] = generateTranslationCandidates(
            fpfh_result, scan_down);

        // Multi-Hypothesis: Test rotation hypotheses (45 degree increments for indoor)
        std::vector<double> yaw_hypotheses = {
            0.0, M_PI_4, M_PI_2, 3*M_PI_4,
            M_PI, -3*M_PI_4, -M_PI_2, -M_PI_4
        };  // 0, 45, 90, 135, 180, -135, -90, -45 degrees

        double best_fitness = 0.0;
        Eigen::Matrix4d best_transform = Eigen::Matrix4d::Identity();
        int best_trans_idx = -1;
        int best_rot_idx = -1;

        size_t total_hypotheses = translation_candidates.size() * yaw_hypotheses.size();
        RCLCPP_INFO(this->get_logger(), "Testing %zu hypotheses (%zu translations x %zu rotations)...",
                    total_hypotheses, translation_candidates.size(), yaw_hypotheses.size());

        // Step 4: Evaluate all hypotheses
        Eigen::Matrix3d base_rotation = fpfh_transform.block<3, 3>(0, 0);

        for (size_t t = 0; t < translation_candidates.size(); t++)
        {
            const Eigen::Vector3d& translation = translation_candidates[t];

            for (size_t r = 0; r < yaw_hypotheses.size(); r++)
            {
                double yaw_offset = yaw_hypotheses[r];

                HypothesisResult result = evaluateSingleHypothesis(
                    scan_down, pcd_scan, translation, yaw_offset,
                    base_rotation, effective_voxel_size);

                // Only log if it's a good candidate or every 8th hypothesis
                if (result.fitness > 0.1 || (t * yaw_hypotheses.size() + r) % 8 == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "  Trans[%zu] Rot[%zu](%.0f°): fitness=%.3f",
                                t, r, yaw_offset * 180.0 / M_PI, result.fitness);
                }

                if (result.fitness > best_fitness)
                {
                    best_fitness = result.fitness;
                    best_transform = result.transform;
                    best_trans_idx = static_cast<int>(t);
                    best_rot_idx = static_cast<int>(r);
                }
            }
        }

        int best_hypothesis = best_rot_idx;

        if (best_hypothesis >= 0)
        {
            RCLCPP_INFO(this->get_logger(), "Best hypothesis: trans[%d] rot[%d](yaw=%.0f°) fitness=%.3f",
                        best_trans_idx, best_rot_idx,
                        yaw_hypotheses[best_hypothesis] * 180.0 / M_PI, best_fitness);

            // Always apply best transform (caller will decide if it's good enough)
            std::lock_guard<std::mutex> lock(odom_mutex_);
            mat_odom2map_ = best_transform;
        }

        return best_fitness;
    }

    void performLocalization()
    {
        double fitness = performLocalizationStep(false);
        current_fitness_ = fitness;

        if (fitness < 0.3)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                  "LOCALIZATION LOST (fitness: %.3f) - consider re-initializing", fitness);
        }
        else if (fitness < fitness_threshold_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Low confidence (fitness: %.3f < %.3f)", fitness, fitness_threshold_);
        }
    }

    double performLocalizationStep(bool is_init)
    {
        // Get current state
        Eigen::Matrix4d mat_body2odom_cur;
        Eigen::Matrix4d mat_body2map_cur;
        std::shared_ptr<open3d::geometry::PointCloud> pcd_scan;

        {
            std::lock_guard<std::mutex> lock_odom(odom_mutex_);
            std::lock_guard<std::mutex> lock_cloud(cloud_mutex_);

            if (pcd_scan_cur_->IsEmpty())
            {
                return 0.0;
            }

            mat_body2odom_cur = mat_body2odom_;
            mat_body2map_cur = mat_odom2map_ * mat_body2odom_;
            pcd_scan = std::make_shared<open3d::geometry::PointCloud>(*pcd_scan_cur_);
        }

        // Current position in map frame
        Eigen::Vector3d cur_position = mat_body2map_cur.block<3, 1>(0, 3);

        // Distance-based submap update: only recompute when moved enough
        double motion_distance = (cur_position - last_submap_position_).norm();
        if (motion_distance > dis_updatemap_ || !pcd_submap_cached_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Updating submap (moved %.2fm > %.2fm threshold)",
                         motion_distance, dis_updatemap_);

            auto OBB_map = createCropBox(
                cur_position,
                mat_body2map_cur.block<3, 3>(0, 0),
                Eigen::Vector3d(submap_radius_ * 2, submap_radius_ * 2, submap_radius_));

            pcd_submap_cached_ = pcd_map_fine_->Crop(*OBB_map);
            last_submap_position_ = cur_position;
        }

        if (!pcd_submap_cached_ || pcd_submap_cached_->IsEmpty())
        {
            RCLCPP_WARN(this->get_logger(), "No map points in current area");
            return 0.0;
        }

        auto map_crop = pcd_submap_cached_;

        // Crop scan
        auto OBB_scan = createCropBox(
            mat_body2odom_cur.block<3, 1>(0, 3),
            mat_body2odom_cur.block<3, 3>(0, 0),
            Eigen::Vector3d(submap_radius_ * 2, submap_radius_ * 2, submap_radius_));

        auto scan_crop = pcd_scan->Crop(*OBB_scan);
        if (scan_crop->IsEmpty())
        {
            RCLCPP_WARN(this->get_logger(), "No scan points");
            return 0.0;
        }

        // Prepare source (scan) and target (map)
        auto source = scan_crop->VoxelDownSample(voxel_size_fine_);
        auto target = map_crop;

        // Limit points
        source = limitPointCloud(source, static_cast<size_t>(max_points_source_));
        target = limitPointCloud(target, static_cast<size_t>(max_points_target_));

        // Save current odom2map for update calculation
        Eigen::Matrix4d mat_odom2map_cur;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            mat_odom2map_cur = mat_odom2map_;
        }

        // Transform source from body frame to map frame using current estimate
        // body → map = odom2map * body2odom
        Eigen::Matrix4d mat_body2map_init = mat_odom2map_cur * mat_body2odom_cur;
        source->Transform(mat_body2map_init);

        // Perform ICP
        std::vector<double> scales = is_init ? std::vector<double>{1, 2, 3} : std::vector<double>{1, 2};
        Eigen::Matrix4d icp_result = pcd_tools::RegistrationMultiScaleIcp(
            source, target, voxel_size_fine_, 1, scales);

        // Evaluate result
        source->Transform(icp_result);
        auto eval_result = open3d::pipelines::registration::EvaluateRegistration(
            *source, *target, voxel_size_fine_ * 3);

        double fitness = eval_result.fitness_;

        // Update odom2map if fitness is good
        // new_odom2map = icp_result * old_odom2map
        if (fitness > fitness_threshold_)
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            mat_odom2map_ = icp_result * mat_odom2map_cur;
        }

        // Save scan for debugging if enabled
        if (save_scan_)
        {
            try
            {
                std::filesystem::create_directories(save_scan_path_);
                std::string filename = save_scan_path_ + std::to_string(scan_save_count_) + "_scan.ply";
                open3d::io::WritePointCloud(filename, *source);
                scan_save_count_++;
                RCLCPP_DEBUG(this->get_logger(), "Saved scan to: %s", filename.c_str());
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Failed to save scan: %s", e.what());
            }
        }

        return fitness;
    }

    // TF
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_body2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_body2map_kalman_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_confidence_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_delay_ms_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;

    // Point clouds
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_ori_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_fine_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan_cur_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_submap_cached_;
    std::queue<open3d::geometry::PointCloud> scan_queue_;
    int queue_max_size_ = 5;

    // Kalman filters for pose smoothing
    KalmanFilter kf_x_;
    KalmanFilter kf_y_;
    KalmanFilter kf_z_;
    std::vector<double> kalman_params_x_ = {0.001, 0.002};
    std::vector<double> kalman_params_y_ = {0.001, 0.005};
    std::vector<double> kalman_params_z_ = {0.00001, 0.04};

    // Distance-based submap update
    double dis_updatemap_ = 3.0;
    Eigen::Vector3d last_submap_position_ = Eigen::Vector3d(0, 0, -5000);

    // Debug options
    bool save_scan_ = false;
    std::string save_scan_path_ = "/tmp/localization_scans/";
    int scan_save_count_ = 0;

    // OccupancyGrid parameters
    bool og_publish_ = true;
    double og_resolution_ = 0.05;
    double og_z_min_ = -0.5;
    double og_z_max_ = 1.5;

    // Global localization (FPFH)
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_global_;
    std::shared_ptr<open3d::pipelines::registration::Feature> map_fpfh_;
    bool map_fpfh_computed_ = false;

    // Matrices
    Eigen::Matrix4d mat_body2odom_;
    Eigen::Matrix4d mat_odom2map_;
    Eigen::Matrix4d mat_initialpose_;
    Eigen::Vector3d last_loc_position_;

    // Mutexes
    std::mutex odom_mutex_;
    std::mutex cloud_mutex_;

    // Parameters
    double voxel_size_fine_ = 0.05;
    double loc_frequency_ = 2.0;
    double fitness_threshold_ = 0.5;
    double submap_radius_ = 50.0;
    int max_points_source_ = 100000;
    int max_points_target_ = 500000;

    // Global localization parameters
    bool use_global_localization_ = true;
    double voxel_size_global_ = 0.25;
    double fitness_threshold_global_ = 0.15;

    // Frame names
    std::string map_frame_ = "map";
    std::string odom_frame_ = "camera_init";
    std::string body_frame_ = "body";

    // State
    rclcpp::Time latest_odom_time_;
    bool loc_initialized_ = false;
    double current_fitness_ = 0.0;
    std::atomic<bool> running_{true};
    std::thread loc_thread_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
