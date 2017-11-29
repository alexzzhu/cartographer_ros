-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  imu_sampling_ratio = 0.5,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
}

--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 200
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.scans_per_accumulation = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_2D.submaps.resolution = 0.2
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.min_range = 1.
TRAJECTORY_BUILDER_2D.max_range = 120.
TRAJECTORY_BUILDER_2D.min_z = -3.
TRAJECTORY_BUILDER_2D.max_z = 10
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.25
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 3e0
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 120.
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 120.
--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 7
SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 3e3
SPARSE_POSE_GRAPH.optimize_every_n_scans = 20
--SPARSE_POSE_GRAPH.matcher_rotation_weight = 5e3
--SPARSE_POSE_GRAPH.matcher_translation_weight = 5e2
--SPARSE_POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
--SPARSE_POSE_GRAPH.constraint_builder.max_constraint_distance = 60
SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.3
--SPARSE_POSE_GRAPH.constraint_builder.global_localization_min_score = 0.2
--SPARSE_POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e10
--SPARSE_POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e13
--SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.3
--SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 40.
--SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 40
--SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(20.)
--SPARSE_POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 800
--SPARSE_POSE_GRAPH.optimization_problem.acceleration_weight = 1e0
--SPARSE_POSE_GRAPH.optimization_problem.rotation_weight = 1e5
--SPARSE_POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = true
SPARSE_POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 800
SPARSE_POSE_GRAPH.max_num_final_iterations = 800
SPARSE_POSE_GRAPH.global_constraint_search_after_n_seconds = 10.

return options
