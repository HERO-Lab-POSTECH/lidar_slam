#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "fast_lio/localization/kalman_filter.h"

/**
 * @brief Map-based localization node using Open3D ICP registration.
 *
 * Class declaration is in this header so that method bodies can be split
 * across multiple .cpp files (localization_node.cpp + localization_engine.cpp).
 */
class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode();
    ~LocalizationNode() override;

private:
    // ===== Internal types (algorithm) =====
    struct HypothesisResult
    {
        double fitness = 0.0;
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    };

    // ===== Algorithm methods (defined in localization_engine.cpp) =====
    std::optional<std::pair<std::shared_ptr<open3d::geometry::PointCloud>, Eigen::Matrix4d>>
    prepareScanForGlobalLocalization();

    void ensureMapFpfhComputed(double voxel_size);

    std::pair<std::vector<Eigen::Vector3d>, Eigen::Matrix4d> generateTranslationCandidates(
        const open3d::pipelines::registration::RegistrationResult& fpfh_result,
        std::shared_ptr<open3d::geometry::PointCloud> scan_down);

    HypothesisResult evaluateSingleHypothesis(
        std::shared_ptr<open3d::geometry::PointCloud> scan_down,
        std::shared_ptr<open3d::geometry::PointCloud> pcd_scan_original,
        const Eigen::Vector3d& translation,
        double yaw_offset,
        const Eigen::Matrix3d& base_rotation,
        double voxel_size);

    void localizationLoop();
    void initializeLocalization();
    double performGlobalLocalizationWithFitness();
    void performLocalization();
    double performLocalizationStep(bool is_init);

    // ===== ROS plumbing methods (defined in localization_node.cpp) =====
    void setInitialPose(double x, double y, double z, double roll, double pitch, double yaw);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void publishMap();
    void publishOccupancyGrid();
    void tfTimerCallback();
    void publishTF(const rclcpp::Time& stamp);

    // ===== Members =====
    // TF
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_confidence_;
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
    fast_lio::localization::KalmanFilter kf_x_;
    fast_lio::localization::KalmanFilter kf_y_;
    fast_lio::localization::KalmanFilter kf_z_;
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
