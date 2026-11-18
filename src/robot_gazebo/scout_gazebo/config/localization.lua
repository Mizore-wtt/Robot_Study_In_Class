include "mapping.lua"

-- Current Cartographer replaces the obsolete pure_localization boolean.
TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.global_sampling_ratio = 0.2
-- Produce enough nodes for a global search even before the operator drives.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.

return options
