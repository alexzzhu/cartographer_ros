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
  tracking_frame = "imu0",
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
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
}

TRAJECTORY_BUILDER_3D.scans_per_accumulation = 1
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 100
TRAJECTORY_BUILDER_3D.min_range = 2
TRAJECTORY_BUILDER_3D.max_range = 120
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 40
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 80
--TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 5e3
SPARSE_POSE_GRAPH.optimize_every_n_scans = 1
SPARSE_POSE_GRAPH.constraint_builder.sampling_ratio = 0.2
SPARSE_POSE_GRAPH.constraint_builder.max_constraint_distance = 250
SPARSE_POSE_GRAPH.constraint_builder.min_score = 0.3
--SPARSE_POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e3
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.4
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 100.
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 200.
SPARSE_POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(60.)
SPARSE_POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 200
--SPARSE_POSE_GRAPH.optimization_problem.acceleration_weight = 1e5
--SPARSE_POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = true
SPARSE_POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100

return options
