-- Localization baseline for industrial_warehouse_classic (loads pbstream).
include "tb3_warehouse_lds_2d_common.lua"

-- Keep only a small number of submaps during pure localization.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

return options

