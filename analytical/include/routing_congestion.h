// 3D IC Analytical Floorplanner - Routability-Driven Congestion Penalty
//
// density-style 局部場模型：
//   1. update_routing_congestion_map()（PlacementEngine 成員）
//      → 建立 H/V edge demand 快取（BinEdgeDemands）
//   2. calculate_routing_congestion_gradient()（本檔）
//      → 對每個可動 module 掃描影響半徑內的 hotspot edge，
//        以 bell 核計算斥力（horizontal edge → gy，vertical edge → gx）
//
// routing_congestion_alpha = 0 時立即 return，不影響既有行為。
#pragma once

#include "floorplanner.h"
#include <vector>

// 對 gx[i]/gy[i]（與 modules_[i] 對應，長度 = modules_.size()）累加壅塞梯度。
// gx_tsv/gy_tsv 非 nullptr 時同步計算 TSV 在上下層的合力（長度 = tsvs_.size()）。
// 呼叫前需已完成 engine.update_routing_congestion_map()。
void calculate_routing_congestion_gradient(
    const PlacementEngine& engine,
    std::vector<double>&   gx,
    std::vector<double>&   gy,
    std::vector<double>*   gx_tsv = nullptr,
    std::vector<double>*   gy_tsv = nullptr);
