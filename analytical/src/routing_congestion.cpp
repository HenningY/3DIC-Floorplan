// 3D IC Analytical Floorplanner - Routability-Driven Congestion Penalty
//
// 兩階段 density-style bin 場模型：
//   1. update_routing_congestion_map()（floorplanner.cpp）
//      → 建立 H/V edge demand 快取（BinEdgeDemands）
//   2. calculate_routing_congestion_gradient()（本檔）
//      → 從 H/V 算出每個 bin 的 cell_avg（四邊平均 demand）
//      → 對每個可動 module，掃描影響半徑內的 overflow bin，
//        以 2D bell 核向 bin 中心方向施加斥力（gx + gy，與 density 機制相同）

#include "routing_congestion.h"
#include "util.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

// Bell 函數與梯度（與 PlacementEngine 內 static 版本相同語義）
// Φ(d, r) = 1 - (d/r)²   if |d| < r,  else 0
static inline double bell_func(double d, double r)
{
    if (std::fabs(d) >= r) return 0.0;
    const double u = d / r;
    return 1.0 - u * u;
}

// ∂Φ/∂d = -2d/r²   if |d| < r,  else 0
static inline double bell_grad(double d, double r)
{
    if (std::fabs(d) >= r) return 0.0;
    return -2.0 * d / (r * r);
}

// ============================================================
// 公開介面
// ============================================================
// bin 斥力輔助：掃描 (px,py) 在 die 的 cell_avg 中的 overflow bin 並累加斥力
// rx/ry：影響半徑；gx_out/gy_out：梯度累加目標
static void accumulate_cong_gradient(
    double px, double py, double rx, double ry,
    const Die& die, const std::vector<double>& cell_avg,
    double cap, double& gx_out, double& gy_out)
{
    const double bw = die.bin_w;
    const double bh = die.bin_h;
    const int R = die.bin_rows;
    const int C = die.bin_cols;

    const int r_lo = std::max(0,     static_cast<int>(std::floor((py - ry) / bh)));
    const int r_hi = std::min(R - 1, static_cast<int>(std::ceil ((py + ry) / bh)));
    const int c_lo = std::max(0,     static_cast<int>(std::floor((px - rx) / bw)));
    const int c_hi = std::min(C - 1, static_cast<int>(std::ceil ((px + rx) / bw)));

    for (int r = r_lo; r <= r_hi; ++r) {
        for (int c = c_lo; c <= c_hi; ++c) {
            const double overflow = cell_avg[static_cast<size_t>(r * C + c)] - cap;
            if (overflow <= 0.0) continue;
            const double xe = (c + 0.5) * bw;
            const double ye = (r + 0.5) * bh;
            const double dx = px - xe;
            const double dy = py - ye;
            gx_out += 2.0 * overflow * bell_grad(dx, rx) * bell_func(dy, ry);
            gy_out += 2.0 * overflow * bell_func(dx, rx) * bell_grad(dy, ry);
        }
    }
}

void calculate_routing_congestion_gradient(
    const PlacementEngine& engine,
    std::vector<double>&   gx,
    std::vector<double>&   gy,
    std::vector<double>*   gx_tsv,
    std::vector<double>*   gy_tsv)
{
    const PlacementConfig& cfg = engine.config();
    if (cfg.routing_congestion_alpha == 0.0
        && cfg.routing_congestion_max == 0.0) return;

    const double cap      = cfg.routing_capacity_C;
    const double sigma    = engine.smooth_sigma();

    const auto& modules   = engine.modules();
    const auto& dies      = engine.dies();
    const auto& tsvs      = engine.tsvs();
    const int   num_tiers = static_cast<int>(dies.size());
    const int   nmod      = static_cast<int>(modules.size());

    const BinEdgeDemands& dem = engine.routing_edge_demands();
    if (dem.H.empty()) return;

    std::vector<std::vector<double>> tier_cell_avg(static_cast<size_t>(num_tiers));
    for (int t = 0; t < num_tiers && t < static_cast<int>(dem.H.size()); ++t) {
        bin_edge_cells_from_hv(dies[static_cast<size_t>(t)],
                               dem.H[static_cast<size_t>(t)],
                               dem.V[static_cast<size_t>(t)],
                               tier_cell_avg[static_cast<size_t>(t)]);
    }

    // ---- Module congestion 斥力 ----
    for (int i = 0; i < nmod; ++i) {
        const Module& m = modules[i];
        if (m.is_terminal || m.is_fixed) continue;

        const int t = m.tier_id;
        if (t < 0 || t >= num_tiers) continue;
        if (t >= static_cast<int>(tier_cell_avg.size())) continue;

        const Die& die = dies[static_cast<size_t>(t)];
        const double rx = m.width  * 0.5 + std::max(sigma, die.bin_w);
        const double ry = m.height * 0.5 + std::max(sigma, die.bin_h);

        accumulate_cong_gradient(m.x, m.y, rx, ry,
                                 die, tier_cell_avg[static_cast<size_t>(t)],
                                 cap, gx[i], gy[i]);
    }

    // ---- TSV congestion 斥力（Phase 2）：tier_below + tier_above 合力 ----
    if (gx_tsv && gy_tsv && !tsvs.empty()) {
        const double tsv_rw = cfg.analytical_tsv_width  * 0.5;
        const double tsv_rh = cfg.analytical_tsv_height * 0.5;

        for (int i = 0; i < static_cast<int>(tsvs.size()); ++i) {
            const TSV& tsv = tsvs[static_cast<size_t>(i)];
            double& gxi = (*gx_tsv)[static_cast<size_t>(i)];
            double& gyi = (*gy_tsv)[static_cast<size_t>(i)];

            for (int tier : {tsv.tier_below(), tsv.tier_above()}) {
                if (tier < 0 || tier >= num_tiers) continue;
                if (tier >= static_cast<int>(tier_cell_avg.size())) continue;

                const Die& die = dies[static_cast<size_t>(tier)];
                const double rx = tsv_rw + std::max(sigma, die.bin_w);
                const double ry = tsv_rh + std::max(sigma, die.bin_h);

                accumulate_cong_gradient(tsv.x, tsv.y, rx, ry,
                                         die, tier_cell_avg[static_cast<size_t>(tier)],
                                         cap, gxi, gyi);
            }
        }
    }
}
