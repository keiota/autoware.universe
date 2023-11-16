// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "behavior_path_planner/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner/utils/utils.hpp"
#include "object_recognition_utils/predicted_path_utils.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>

using namespace behavior_path_planner::utils::path_safety_checker;

// namespace
// {
//   using tier4_autoware_utils::createPoint;
//   using tier4_autoware_utils::createQuaternionFromRPY;

//   geometry_msgs::msg::Pose createPose(
//     double x, double y, double z, double roll, double pitch, double yaw)
//   {
//     geometry_msgs::msg::Pose p;
//     p.position = createPoint(x, y, z);
//     p.orientation = createQuaternionFromRPY(roll, pitch, yaw);
//     return p;
// }

//} // namespace

TEST(BehaviorPathPlanningObjectsFiltering, createPredictedPath)
{
    using autoware_auto_planning_msgs::msg::PathPoint;
    using autoware_auto_planning_msgs::msg::PathWithLaneId;
    using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
    using behavior_path_planner::utils::path_safety_checker::createPredictedPath;
    using behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams;
    using behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;

    //   std::vector<PoseWithVelocityStamped> createPredictedPath: 
    //   const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
    //   const std::vector<PathPointWithLaneId> & path_points,
    //   const geometry_msgs::msg::Pose & vehicle_pose,
    //   const double current_velocity,
    //   const size_t ego_seg_idx, 
    //   const bool is_object_front,
    //   const bool limit_to_max_velocity

    std::shared_ptr<EgoPredictedPathParams> ego_predicted_path_params;
    ego_predicted_path_params->max_velocity = 10.0;    

    std::vector<PathPointWithLaneId> path_points;

    // float initial_pose_value = 0.0;
    // float pose_increment = 1.0;
    // size_t point_sample = 10;
    // for (size_t idx = 0; idx < point_sample; ++idx) {
    //     PathPoint point;
    //     point.pose.position.x = std::exchange(initial_pose_value, initial_pose_value + pose_increment);
    //     point.pose.position.y = 0.0;
    //     point.pose.position.z = 0.0;
    //     point.longitudinal_velocity_mps = 0.1;  // [m/s]
    //     point.heading_rate_rps = 0.0;           // [rad/s]
    //     point.is_final = (idx == point_sample - 1);

    //     PathPointWithLaneId path_point_with_lane_id;
    //     path_point_with_lane_id.point = point;
    //     path_point_with_lane_id.lane_ids = std::vector<int64_t>();

    //     path_points.push_back(path_point_with_lane_id);
    // }

    //geometry_msgs::msg::Pose vehicle_pose = createPose(0,0,0,0,0,0);

    // double current_velocity = 0.0;
    // size_t ego_seg_idx = 0;
    // bool is_object_front = false;
    // bool limit_to_max_velocity = false;

    std::vector<PoseWithVelocityStamped> predicted_paths;
    std::vector<PoseWithVelocityStamped> ans_paths;

    std::cout << "path_points.size(): " << path_points.size() << std::endl;

    // predicted_paths = createPredictedPath(ego_predicted_path_params, 
    //                                       path_points, 
    //                                       vehicle_pose, 
    //                                       current_velocity, 
    //                                       ego_seg_idx, 
    //                                       is_object_front, 
    //                                       limit_to_max_velocity);

    // for (size_t idx = 0; idx < point_sample; ++idx) {
    //     PoseWithVelocityStamped pose_with_velocity_stamped;
    //     pose_with_velocity_stamped.pose = createPose(idx,0,0,0,0,0);
    //     pose_with_velocity_stamped.velocity = 0.1;
    //     ans_paths.push_back(pose_with_velocity_stamped);
    // }

    //get the length of predicted_paths and ans_paths
    // size_t predicted_paths_size = predicted_paths.size();
    // size_t ans_paths_size = ans_paths.size();

    // EXPECT_EQ(predicted_paths_size, ans_paths_size);
}