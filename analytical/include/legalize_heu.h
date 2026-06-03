#pragma once

#include "floorplanner.h"

#include <iostream>

struct LocalMoveConfig {
    double max_search_dist = 30.0;     // 從 module 邊界往外找鄰居的距離
    double max_move_dist   = 30.0;     // 單軸最大位移限制（|d| <= max_move_dist）
    double disp_weight     = 1.0;      // 位移成本權重：disp_weight * d^2（各軸）
    double overlap_weight  = 1.0;      // 重疊面積成本權重
    double moved_weight_mul = 1.25;    // module 套用移動後，move_weight 乘上此倍率
    double max_module_weight = 1e6;    // move_weight 上限
};

struct LocalMoveResult {
    bool   rotate_90       = false;    // true: 先以中心點旋轉 90 度再做位移最佳化
    double dx              = 0.0;
    double dy              = 0.0;
    double overlap_before  = 0.0;
    double overlap_after_x = 0.0;
    double overlap_after_y = 0.0;
    double objective       = 0.0;
};

// heuristic legalization 入口（目前先實作每層中心 module 偵測）
void run_legalize_heu(PlacementEngine& engine, const PartitionConfig& pcfg);

// 找出 tier 內有重疊的 module，對其「附近」（距離 <= radius）的 movable module
// 以 rotate_prob 的機率執行 90° 旋轉（以中心點為軸，clamp 回 die 邊界）。
// 回傳實際旋轉的 module 數量。
// log != nullptr 時，每次旋轉都會寫一行 [shake_rot] 到該 stream（僅寫檔，不進 terminal）
int shake_nearby_rotations(std::vector<Module>&    modules,
                           const std::vector<Die>& dies,
                           int                     tier,
                           double                  radius      = 50.0,
                           double                  rotate_prob = 0.5,
                           unsigned                seed        = 42,
                           std::ostream*           log         = nullptr,
                           double                  log_phys_scale = 1.0);

// 在其他 module 固定下，對指定 module 做 local 1D+1D 最佳化：
// 先找 x 最佳位移，再以更新後位置找 y 最佳位移。
// log_phys_scale：die normalize 時為 1/geometry_scale，log 內位移換算為物理長度
LocalMoveResult optimize_module_local_move(std::vector<Module>& modules,
                                           const std::vector<Die>& dies,
                                           int                  target_module_id,
                                           const LocalMoveConfig& cfg,
                                           std::ostream*          move_log = &std::cout,
                                           double                 log_phys_scale = 1.0);
