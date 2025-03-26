include "cartographer_rplidar.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3, 
}

POSE_GRAPH.optimize_every_n_nodes = 5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(7.)
return options