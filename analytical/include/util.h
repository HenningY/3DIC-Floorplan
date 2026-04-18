// 3D IC Analytical Floorplanner - 位移分析工具
#pragma once

#include "floorplanner.h"

#include <string>
#include <vector>

// 設為 true 開啟位移分析；false 則所有函式為 no-op
static constexpr bool ENABLE_DISPLACEMENT_REPORT = true;

struct ModuleSnapshot {
    int         id;
    std::string name;
    double      x, y;
    double      area;
    int         num_nets;
};

// 擷取目前所有 movable modules 的中心座標
std::vector<ModuleSnapshot> record_positions(const PlacementEngine& engine);

// 計算 before/after 兩次快照的 Euclidean 位移，依大小排序後寫至 filepath
void write_displacement_report(
    const std::vector<ModuleSnapshot>& before,
    const std::vector<ModuleSnapshot>& after,
    const std::string&                 filepath);
