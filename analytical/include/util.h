// 3D IC Analytical Floorplanner - 位移分析工具
#pragma once

#include "floorplanner.h"

#include <iostream>
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

// ============================================================
// Intra-die HPWL：只計算所有 pin 都在同一層的 net
// ============================================================
struct IntraDieStats {
    double              total_hpwl;  // 全部層加總（已乘 die weight）
    std::vector<double> tier_hpwl;   // 每層加權 HPWL
    std::vector<int>    tier_count;  // 每層 intra-die net 數量
};

IntraDieStats compute_intra_die_hpwl(const PlacementEngine& engine);

// 在 std::ostream 上列印 IntraDieStats 的逐層明細
void print_intra_die_stats(const IntraDieStats& stats, std::ostream& os = std::cout);
