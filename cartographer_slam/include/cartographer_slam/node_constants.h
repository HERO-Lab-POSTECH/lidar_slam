/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_SLAM_CARTOGRAPHER_SLAM_NODE_CONSTANTS_H
#define CARTOGRAPHER_SLAM_CARTOGRAPHER_SLAM_NODE_CONSTANTS_H

#include <string>
#include <vector>

namespace cartographer_ros {

// Default input topic names; expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kLivoxPointsTopic[] = "livox_points";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "odom";
constexpr char kNavSatFixTopic[] = "fix";
constexpr char kLandmarkTopic[] = "landmark";

// Output topic names with cartographer_2d prefix
constexpr char kOccupancyGridTopic[] = "cartographer_2d/map";
constexpr char kScanMatchedPointCloudTopic[] = "cartographer_2d/scan_matched_points2";
constexpr char kSubmapsTopic[] = "cartographer_2d/submaps";
constexpr char kTrackedPoseTopic[] = "cartographer_2d/tracked_pose";
constexpr char kOdometryOutputTopic[] = "cartographer_2d/odom";
constexpr char kTrajectoryNodesTopic[] = "cartographer_2d/trajectory_nodes";
constexpr char kLandmarkPosesTopic[] = "cartographer_2d/landmark_poses";
constexpr char kConstraintsTopic[] = "cartographer_2d/constraints";

// Service names
constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";
constexpr char kSubmapQueryServiceName[] = "submap_query";
constexpr char kTrajectoryQueryServiceName[] = "trajectory_query";
constexpr char kStartTrajectoryServiceName[] = "start_trajectory";
constexpr char kWriteStateServiceName[] = "write_state";
constexpr char kGetTrajectoryStatesServiceName[] = "get_trajectory_states";
constexpr char kReadMetricsServiceName[] = "read_metrics";
constexpr double kConstraintPublishPeriodSec = 0.5;
constexpr double kTopicMismatchCheckDelaySec = 3.0;

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

// For multiple topics adds numbers to the topic name and returns the list.
std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic,
                                                   int num_topics);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_SLAM_CARTOGRAPHER_SLAM_NODE_CONSTANTS_H
