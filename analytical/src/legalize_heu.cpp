// Heuristic legalization — standalone experimental flow
#include "legalize_heu.h"
#include "util.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <set>
#include <unordered_set>
#include <vector>

// ============================================================
// 匿名 namespace：幾何輔助型別與 1D 求解器
// ============================================================
namespace {

constexpr double kEps = 1e-12;

// aspect ratio w/h 是否在 [ar_min, ar_max] 內
inline bool aspect_in_range(double w, double h, double ar_min, double ar_max)
{
    if (h < kEps) return false;
    const double ar = w / h;
    return ar >= ar_min - kEps && ar <= ar_max + kEps;
}

// normalize 時幾何長度乘 geometry_scale；log 輸出物理長度 = scaled * (1/scale)
inline double to_physical_len(double len_scaled, double inv_geometry_scale)
{
    return len_scaled * inv_geometry_scale;
}

// ── Soft shape curve ──────────────────────────────────────────
//
// 對固定面積 A = w0*h0，在 [ar_lo, ar_hi] 上均勻取 steps 個 aspect ratio，
// 每個 ar 對應 w = sqrt(A*ar), h = sqrt(A/ar)（等面積變形）。
// 原始 (w0,h0) 永遠保留；與已有形狀重複者跳過。
std::vector<std::pair<double,double>>
build_soft_shape_curve(double w0, double h0, const LocalMoveConfig& cfg)
{
    std::vector<std::pair<double,double>> shapes;
    shapes.push_back({w0, h0});

    const int steps = cfg.soft_aspect_curve_steps;
    if (steps <= 1 || w0 < kEps || h0 < kEps) return shapes;

    const double A     = w0 * h0;
    const double ar_lo = cfg.soft_aspect_min;
    const double ar_hi = cfg.soft_aspect_max;

    auto nearly_same = [](double wa, double ha, double wb, double hb) {
        return std::fabs(wa - wb) <= kEps && std::fabs(ha - hb) <= kEps;
    };

    for (int i = 0; i < steps; ++i) {
        const double ar = ar_lo + (ar_hi - ar_lo) * static_cast<double>(i) / (steps - 1);
        const double w  = std::sqrt(A * ar);
        const double h  = std::sqrt(A / ar);
        if (!aspect_in_range(w, h, ar_lo, ar_hi)) continue;
        bool dup = false;
        for (const auto& [ew, eh] : shapes)
            if (nearly_same(w, h, ew, eh)) { dup = true; break; }
        if (!dup) shapes.push_back({w, h});
    }
    return shapes;
}

// ── Side bias helpers ────────────────────────────────────────
//
// side_bias_linear: 傳入目前優化軸 (axis 1=x, 2=y) 與移動後中心座標
// 回傳 side bias 貢獻 = weight * sign * center_after
// axis 不符 / weight ≈ 0 時回傳 0
inline double side_bias_linear(const LocalMoveConfig& cfg, int axis, double center_after)
{
    if (cfg.side_bias_weight <= kEps || cfg.side_bias_axis != axis) return 0.0;
    return cfg.side_bias_weight * cfg.side_bias_sign * center_after;
}

// apply_side_bias_from_label: 依 sweep label 填寫 cfg 的 side bias 欄位
// bottom->top: axis=y(-1), top->bottom: axis=y(+1)
// left->right: axis=x(-1), right->left: axis=x(+1)
// 其他: 關閉
inline void apply_side_bias_from_label(LocalMoveConfig& cfg,
                                       const std::string& label,
                                       double weight)
{
    if (label == "bottom->top") {
        cfg.side_bias_axis   = 2;
        cfg.side_bias_sign   = -1.0;
        cfg.side_bias_weight = weight;
    } else if (label == "top->bottom") {
        cfg.side_bias_axis   = 2;
        cfg.side_bias_sign   = +1.0;
        cfg.side_bias_weight = weight;
    } else if (label == "left->right") {
        cfg.side_bias_axis   = 1;
        cfg.side_bias_sign   = -1.0;
        cfg.side_bias_weight = weight;
    } else if (label == "right->left") {
        cfg.side_bias_axis   = 1;
        cfg.side_bias_sign   = +1.0;
        cfg.side_bias_weight = weight;
    } else {
        cfg.side_bias_axis   = 0;
        cfg.side_bias_sign   = 0.0;
        cfg.side_bias_weight = 0.0;
    }
}

// ── 幾何輔助 ────────────────────────────────────────────────

struct BBox {
    double xmin = 0.0;
    double ymin = 0.0;
    double xmax = 0.0;
    double ymax = 0.0;
    bool   valid = false;
};

BBox compute_tier_module_bbox(const std::vector<Module>& modules, int tier)
{
    BBox box;
    for (const Module& m : modules) {
        if (m.is_terminal || m.tier_id != tier) continue;
        if (!box.valid) {
            box = { m.lx(), m.ly(), m.rx(), m.ry(), true };
        } else {
            box.xmin = std::min(box.xmin, m.lx());
            box.ymin = std::min(box.ymin, m.ly());
            box.xmax = std::max(box.xmax, m.rx());
            box.ymax = std::max(box.ymax, m.ry());
        }
    }
    return box;
}


double overlap_1d(double l1, double r1, double l2, double r2)
{
    return std::max(0.0, std::min(r1, r2) - std::max(l1, l2));
}

double overlap_area(const Module& a, const Module& b)
{
    return overlap_1d(a.lx(), a.rx(), b.lx(), b.rx())
         * overlap_1d(a.ly(), a.ry(), b.ly(), b.ry());
}

// ── 1D 鄰居結構 ─────────────────────────────────────────────
//
// 在單軸優化中，每個鄰居用三個量表示：
//   orth_overlap : 垂直軸上的重疊長（x-opt → y-overlap；y-opt → x-overlap）
//   l, r         : 可動軸上，固定 module 的區間邊界
//   pair_weight  : target.move_weight * neighbor.move_weight

struct Neighbor1D {
    double orth_overlap;
    double l;
    double r;
    double pair_weight;
};

double neighbor_1d_overlap(const Neighbor1D& nb,
                           double move_l, double move_r, double d)
{
    return overlap_1d(move_l + d, move_r + d, nb.l, nb.r);
}

// 對所有鄰居計算加權重疊面積（orth_overlap * 可動軸重疊 * pair_weight）
double overlap_objective_1d(const std::vector<Neighbor1D>& nbs,
                            double move_l, double move_r, double d)
{
    double total = 0.0;
    for (const auto& nb : nbs)
        total += nb.pair_weight * nb.orth_overlap
               * neighbor_1d_overlap(nb, move_l, move_r, d);
    return total;
}

// ── piecewise 分段邊界 ───────────────────────────────────────
//
// 每個鄰居區間的端點在可動軸方向上恰好是 overlap(d) 斜率改變點。

std::vector<double> build_breakpoints(const std::vector<Neighbor1D>& nbs,
                                      double move_l, double move_r,
                                      double min_move, double max_move)
{
    std::vector<double> pts;
    pts.reserve(2 * nbs.size() + 4);
    pts.push_back(min_move);
    pts.push_back(std::clamp(0.0, min_move, max_move)); // 確保加入的點在合法範圍內
    pts.push_back(max_move);

    for (const auto& nb : nbs) {
        // [move_l+d, move_r+d] 與 [nb.l, nb.r] 的接觸邊界
        const double enter = nb.l - move_r; // d 使右邊碰到 nb.l
        const double leave = nb.r - move_l; // d 使左邊離開 nb.r
        if (enter > min_move - kEps && enter < max_move + kEps) pts.push_back(enter);
        if (leave > min_move - kEps && leave < max_move + kEps) pts.push_back(leave);
    }

    std::sort(pts.begin(), pts.end());
    pts.erase(std::unique(pts.begin(), pts.end(),
                          [](double a, double b){ return std::fabs(a-b) <= 1e-10; }),
              pts.end());
    return pts;
}

// ── segment 內的二次最佳解 ───────────────────────────────────
//
// 在 [seg_lo, seg_hi] 上，overlap(d) 是一次式 slope*d + intercept，
// side bias 是線性項 side_linear_slope * d
// 目標函數 = disp_weight*d^2 + overlap_weight*(slope*d + intercept) + side_linear_slope*d
// 導數 = 2*disp_weight*d + overlap_weight*slope + side_linear_slope = 0
// → d* = -(overlap_weight*slope + side_linear_slope) / (2*disp_weight)

double solve_best_d_on_segment(const std::vector<Neighbor1D>& nbs,
                               double move_l, double move_r,
                               double seg_lo, double seg_hi,
                               const LocalMoveConfig& cfg,
                               int axis, double /*center_base*/)
{
    if (seg_hi < seg_lo + kEps) return seg_lo;

    // side bias 對 d 的線性斜率貢獻
    const double side_linear_slope =
        (cfg.side_bias_weight > kEps && cfg.side_bias_axis == axis)
        ? cfg.side_bias_weight * cfg.side_bias_sign
        : 0.0;

    // 在 segment 中點計算 overlap(d) 的一次式係數
    const double mid = 0.5 * (seg_lo + seg_hi);
    double slope     = 0.0;
    double intercept = 0.0;

    for (const auto& nb : nbs) {
        const double seg_l = move_l + mid;
        const double seg_r = move_r + mid;

        double s_d = 0.0; // overlap(d) 對 d 的斜率項
        double s_0 = 0.0; // overlap(d) 的常數項

        if (seg_r <= nb.l + kEps || seg_l >= nb.r - kEps) {
            // 無重疊
        } else if (seg_l <= nb.l + kEps && seg_r <= nb.r + kEps) {
            // 右側部分重疊：overlap = (move_r + d) - nb.l
            s_d = 1.0;
            s_0 = move_r - nb.l;
        } else if (seg_l >= nb.l - kEps && seg_r >= nb.r - kEps) {
            // 左側部分重疊：overlap = nb.r - (move_l + d)
            s_d = -1.0;
            s_0 = nb.r - move_l;
        } else {
            // 鄰居完全被覆蓋，overlap = nb.r - nb.l（常數）
            s_0 = nb.r - nb.l;
        }

        const double w = cfg.overlap_weight * nb.pair_weight * nb.orth_overlap;
        slope     += w * s_d;
        intercept += w * s_0;
    }

    const double total_slope = slope + side_linear_slope;

    if (cfg.disp_weight <= kEps) {
        // 無位移懲罰：線性函數，最小值在端點
        if (total_slope > kEps)  return seg_lo;
        if (total_slope < -kEps) return seg_hi;
        return (std::fabs(seg_lo) <= std::fabs(seg_hi)) ? seg_lo : seg_hi;
    }

    // 二次函數最小點
    double d_star = -total_slope / (2.0 * cfg.disp_weight);
    return std::clamp(d_star, seg_lo, seg_hi);
}

// ── 單軸最佳位移 ─────────────────────────────────────────────
// axis: 1=x, 2=y（與 side_bias_axis 對應）
// center_base: 移動前 module 在該軸的中心座標（x 或 y）

double optimize_one_axis(const std::vector<Neighbor1D>& nbs,
                         double move_l, double move_r,
                         double min_move_allowed, double max_move_allowed,
                         const LocalMoveConfig& cfg,
                         int axis = 0, double center_base = 0.0)
{
    // module 旋轉後尺寸超過 die：選最能縮小越界量的邊界
    if (max_move_allowed < min_move_allowed + kEps) {
        return (std::fabs(min_move_allowed) <= std::fabs(max_move_allowed))
                   ? min_move_allowed : max_move_allowed;
    }

    auto eval_obj = [&](double d) {
        const double center_after = center_base + d;
        return cfg.disp_weight * d * d
             + cfg.overlap_weight * overlap_objective_1d(nbs, move_l, move_r, d)
             + side_bias_linear(cfg, axis, center_after);
    };

    double best_d   = std::clamp(0.0, min_move_allowed, max_move_allowed);
    double best_obj = eval_obj(best_d);

    if (nbs.empty() && cfg.side_bias_weight <= kEps) return best_d;

    // side bias 無 overlap 時：breakpoints 中沒有端點能被吸引，需手動加入極端值
    std::vector<double> extra_pts;
    if (cfg.side_bias_weight > kEps && cfg.side_bias_axis == axis) {
        extra_pts.push_back(min_move_allowed);
        extra_pts.push_back(max_move_allowed);
    }

    if (!nbs.empty()) {
        const auto pts = build_breakpoints(nbs, move_l, move_r,
                                           min_move_allowed, max_move_allowed);

        auto try_d = [&](double d) {
            const double obj = eval_obj(d);
            if (obj < best_obj - kEps) { best_obj = obj; best_d = d; }
        };

        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            const double seg_lo = pts[i];
            const double seg_hi = pts[i + 1];
            if (seg_hi < seg_lo + kEps) continue;

            try_d(seg_lo);
            try_d(seg_hi);
            try_d(solve_best_d_on_segment(nbs, move_l, move_r, seg_lo, seg_hi, cfg, axis, center_base));
        }
    }

    for (double d : extra_pts) {
        const double obj = eval_obj(d);
        if (obj < best_obj - kEps) { best_obj = obj; best_d = d; }
    }

    return best_d;
}

// ── 空間 Hash Grid ────────────────────────────────────────────
//
// 每個 tier 建立一個 grid，cell_size = max_search_dist。
// 查詢時只掃 query rectangle 覆蓋的 cells，再套用精確幾何 filter，
// 將 collect / overlap 從 O(n) 全掃降至 O(候選 module 數)。

struct ModuleSpatialGrid {
    double cell_size_ = 1.0;
    int    cols_      = 0;
    int    rows_      = 0;

    std::vector<std::vector<int>> cells_;         // flat [r*cols+c] → module vector indices
    std::vector<std::vector<int>> module_cells_;  // module vector index → cell indices（供增量更新）
    mutable std::vector<int>      stamp_;
    mutable int                   cur_stamp_ = 0;

    void build(int tier, const std::vector<Module>& modules,
               const Die& die, double cell_size)
    {
        cell_size_ = cell_size > kEps ? cell_size : 1.0;
        cols_      = std::max(1, static_cast<int>(std::ceil(die.width  / cell_size_)));
        rows_      = std::max(1, static_cast<int>(std::ceil(die.height / cell_size_)));
        cells_.assign(static_cast<size_t>(rows_ * cols_), {});
        module_cells_.assign(modules.size(), {});
        stamp_.assign(modules.size(), 0);
        cur_stamp_ = 0;

        for (size_t i = 0; i < modules.size(); ++i) {
            const Module& m = modules[i];
            if (m.is_terminal || m.tier_id != tier) continue;
            insert_(static_cast<int>(i), m);
        }
    }

    void update(int mod_idx, const std::vector<Module>& modules)
    {
        for (int ci : module_cells_[static_cast<size_t>(mod_idx)]) {
            auto& cell = cells_[static_cast<size_t>(ci)];
            cell.erase(std::remove(cell.begin(), cell.end(), mod_idx), cell.end());
        }
        module_cells_[static_cast<size_t>(mod_idx)].clear();
        insert_(mod_idx, modules[static_cast<size_t>(mod_idx)]);
    }

    template<typename Fn>
    void for_each_candidate(double xmin, double ymin,
                            double xmax, double ymax, Fn fn) const
    {
        ++cur_stamp_;
        const int c0 = col_idx(xmin), c1 = col_idx(xmax);
        const int r0 = row_idx(ymin), r1 = row_idx(ymax);
        for (int r = r0; r <= r1; ++r) {
            for (int c = c0; c <= c1; ++c) {
                for (int mi : cells_[static_cast<size_t>(r * cols_ + c)]) {
                    if (stamp_[static_cast<size_t>(mi)] == cur_stamp_) continue;
                    stamp_[static_cast<size_t>(mi)] = cur_stamp_;
                    fn(mi);
                }
            }
        }
    }

private:
    int col_idx(double x) const {
        return std::clamp(static_cast<int>(std::floor(x / cell_size_)), 0, cols_ - 1);
    }
    int row_idx(double y) const {
        return std::clamp(static_cast<int>(std::floor(y / cell_size_)), 0, rows_ - 1);
    }
    void insert_(int mod_idx, const Module& m) {
        const int c0 = col_idx(m.lx()), c1 = col_idx(m.rx());
        const int r0 = row_idx(m.ly()), r1 = row_idx(m.ry());
        for (int r = r0; r <= r1; ++r) {
            for (int c = c0; c <= c1; ++c) {
                const int ci = r * cols_ + c;
                cells_[static_cast<size_t>(ci)].push_back(mod_idx);
                module_cells_[static_cast<size_t>(mod_idx)].push_back(ci);
            }
        }
    }
};

// ── 鄰居收集（grid 加速版）────────────────────────────────────

std::vector<Neighbor1D> collect_x_neighbors(const ModuleSpatialGrid&   grid,
                                            const std::vector<Module>& modules,
                                            const Module&              target,
                                            double                     max_search)
{
    std::vector<Neighbor1D> nbs;
    grid.for_each_candidate(
        target.lx() - max_search, target.ly(),
        target.rx() + max_search, target.ry(),
        [&](int mi) {
            const Module& m = modules[static_cast<size_t>(mi)];
            if (m.id == target.id || m.is_terminal) return;
            const double y_ov = overlap_1d(target.ly(), target.ry(), m.ly(), m.ry());
            if (y_ov <= kEps) return;
            if (m.rx() < target.lx() - max_search || m.lx() > target.rx() + max_search) return;
            nbs.push_back({ y_ov, m.lx(), m.rx(), target.move_weight * m.move_weight });
        });
    return nbs;
}

std::vector<Neighbor1D> collect_y_neighbors(const ModuleSpatialGrid&   grid,
                                            const std::vector<Module>& modules,
                                            const Module&              target,
                                            double                     max_search,
                                            double                     dx_applied)
{
    std::vector<Neighbor1D> nbs;
    const double slx = target.lx() + dx_applied;
    const double srx = target.rx() + dx_applied;
    grid.for_each_candidate(
        slx, target.ly() - max_search,
        srx, target.ry() + max_search,
        [&](int mi) {
            const Module& m = modules[static_cast<size_t>(mi)];
            if (m.id == target.id || m.is_terminal) return;
            const double x_ov = overlap_1d(slx, srx, m.lx(), m.rx());
            if (x_ov <= kEps) return;
            if (m.ry() < target.ly() - max_search || m.ly() > target.ry() + max_search) return;
            nbs.push_back({ x_ov, m.ly(), m.ry(), target.move_weight * m.move_weight });
        });
    return nbs;
}

double total_weighted_overlap(const ModuleSpatialGrid&   grid,
                              const std::vector<Module>& modules,
                              const Module&              target)
{
    double total = 0.0;
    grid.for_each_candidate(
        target.lx(), target.ly(), target.rx(), target.ry(),
        [&](int mi) {
            const Module& m = modules[static_cast<size_t>(mi)];
            if (m.id == target.id || m.is_terminal) return;
            total += (target.move_weight * m.move_weight) * overlap_area(target, m);
        });
    return total;
}

// ── 判斷某 tier 是否仍有 module 重疊 ────────────────────────────
bool has_tier_overlaps(const std::vector<Module>& modules, int tier)
{
    std::vector<const Module*> tier_mods;
    for (const Module& m : modules)
        if (!m.is_terminal && m.tier_id == tier)
            tier_mods.push_back(&m);

    for (size_t i = 0; i < tier_mods.size(); ++i)
        for (size_t j = i + 1; j < tier_mods.size(); ++j)
            if (overlap_area(*tier_mods[i], *tier_mods[j]) > kEps)
                return true;
    return false;
}

// ── 找出有重疊的 module ID 集合 ──────────────────────────────
//
// 回傳該 tier 中，至少與一個其他 module 發生面積重疊的 module id 集合。
std::unordered_set<int>
collect_overlap_module_ids(const std::vector<Module>& modules, int tier)
{
    std::unordered_set<int> involved;
    std::vector<const Module*> tier_mods;
    for (const Module& m : modules)
        if (!m.is_terminal && m.tier_id == tier)
            tier_mods.push_back(&m);

    for (size_t i = 0; i < tier_mods.size(); ++i) {
        for (size_t j = i + 1; j < tier_mods.size(); ++j) {
            if (overlap_area(*tier_mods[i], *tier_mods[j]) > kEps) {
                involved.insert(tier_mods[i]->id);
                involved.insert(tier_mods[j]->id);
            }
        }
    }
    return involved;
}

// ── Post-legalize WL refinement helpers ──────────────────────

constexpr double kWlForceEps      = 1e-6;  // 合力方向判斷門檻
constexpr double kGapMin          = 1e-6;  // 最小可移動量
constexpr double kHpwlRollbackEps = 1e-9;  // incident HPWL rollback 門檻

// compute_net_centroid_wl_force:
//   對每條 net（pins >= 2），計算其所有 pin 座標的幾何中心，
//   對每個可動 pin：fx[id] += (centroid_x - m.x)，fy[id] += (centroid_y - m.y)
//   terminal / fixed module 不接受力，但其座標參與 centroid 計算（含跨 tier）
void compute_net_centroid_wl_force(const std::vector<Module>& modules,
                                   const std::vector<Net>&    nets,
                                   std::vector<double>&       fx,
                                   std::vector<double>&       fy)
{
    std::fill(fx.begin(), fx.end(), 0.0);
    std::fill(fy.begin(), fy.end(), 0.0);

    for (const Net& net : nets) {
        if (net.pins.size() < 2) continue;

        double cx = 0.0, cy = 0.0;
        for (int id : net.pins) {
            cx += modules[static_cast<size_t>(id)].x;
            cy += modules[static_cast<size_t>(id)].y;
        }
        cx /= static_cast<double>(net.pins.size());
        cy /= static_cast<double>(net.pins.size());

        for (int id : net.pins) {
            const Module& m = modules[static_cast<size_t>(id)];
            if (m.is_terminal || m.is_fixed) continue;
            fx[static_cast<size_t>(id)] += cx - m.x;
            fy[static_cast<size_t>(id)] += cy - m.y;
        }
    }
}

// max_shift_x_pos: 向右最大可移動量（不與同 tier 障礙物 / die 邊界重疊）
double max_shift_x_pos(const std::vector<Module>& modules,
                       const Die& die, const Module& target)
{
    double gap = die.width - target.rx();  // die 右邊界限制
    for (const Module& m : modules) {
        if (m.id == target.id || m.tier_id != target.tier_id) continue;
        const double y_ov = overlap_1d(target.ly(), target.ry(), m.ly(), m.ry());
        if (y_ov <= kEps) continue;
        if (m.lx() >= target.rx() - kEps)  // m 在右側
            gap = std::min(gap, m.lx() - target.rx());
    }
    return std::max(0.0, gap);
}

// max_shift_x_neg: 向左最大可移動量（正值表示距離）
double max_shift_x_neg(const std::vector<Module>& modules,
                       const Die& die, const Module& target)
{
    (void)die;
    double gap = target.lx();  // die 左邊界限制
    for (const Module& m : modules) {
        if (m.id == target.id || m.tier_id != target.tier_id) continue;
        const double y_ov = overlap_1d(target.ly(), target.ry(), m.ly(), m.ry());
        if (y_ov <= kEps) continue;
        if (m.rx() <= target.lx() + kEps)  // m 在左側
            gap = std::min(gap, target.lx() - m.rx());
    }
    return std::max(0.0, gap);
}

double max_shift_y_pos(const std::vector<Module>& modules,
                       const Die& die, const Module& target)
{
    double gap = die.height - target.ry();
    for (const Module& m : modules) {
        if (m.id == target.id || m.tier_id != target.tier_id) continue;
        const double x_ov = overlap_1d(target.lx(), target.rx(), m.lx(), m.rx());
        if (x_ov <= kEps) continue;
        if (m.ly() >= target.ry() - kEps)
            gap = std::min(gap, m.ly() - target.ry());
    }
    return std::max(0.0, gap);
}

double max_shift_y_neg(const std::vector<Module>& modules,
                       const Die& die, const Module& target)
{
    (void)die;
    double gap = target.ly();
    for (const Module& m : modules) {
        if (m.id == target.id || m.tier_id != target.tier_id) continue;
        const double x_ov = overlap_1d(target.lx(), target.rx(), m.lx(), m.rx());
        if (x_ov <= kEps) continue;
        if (m.ry() <= target.ly() + kEps)
            gap = std::min(gap, target.ly() - m.ry());
    }
    return std::max(0.0, gap);
}

// hpwl_incident_sum: 計算 module_id 所連所有 net 的 HPWL 加總
//   die_w[tier]：每層 hpwl 乘數（terminal 視為 tier 0）
double hpwl_incident_sum(const std::vector<Module>&           modules,
                         const std::vector<Net>&              nets,
                         const std::vector<std::vector<int>>& module_to_nets,
                         const std::vector<double>&           die_w,
                         int                                  module_id)
{
    double total = 0.0;
    for (int ni : module_to_nets[static_cast<size_t>(module_id)]) {
        const Net& net = nets[static_cast<size_t>(ni)];
        double x_min = 1e18, x_max = -1e18;
        double y_min = 1e18, y_max = -1e18;
        std::set<int> tiers;
        for (int pid : net.pins) {
            const Module& m = modules[static_cast<size_t>(pid)];
            x_min = std::min(x_min, m.x); x_max = std::max(x_max, m.x);
            y_min = std::min(y_min, m.y); y_max = std::max(y_max, m.y);
            const int mt = m.is_terminal ? 0 : m.tier_id;
            if (mt >= 0 && mt < static_cast<int>(die_w.size()))
                tiers.insert(mt);
        }
        if (x_min > x_max || y_min > y_max) continue;
        double w_avg = 1.0;
        if (!tiers.empty()) {
            double sum_w = 0.0;
            for (int t : tiers) sum_w += die_w[static_cast<size_t>(t)];
            w_avg = sum_w / static_cast<double>(tiers.size());
        }
        total += w_avg * ((x_max - x_min) + (y_max - y_min));
    }
    return total;
}

// refine_tier_wl_centroid:
//   對已完成合法化（無 overlap）的 tier 做一輪 WL refinement：
//   near->far 順序，各 module 沿 net-centroid 合力方向做 max feasible shift；
//   移動後若 incident HPWL 變大則還原（x、y 各自判斷）。
void refine_tier_wl_centroid(PlacementEngine&          engine,
                              std::vector<Module>&       modules,
                              const std::vector<Die>&    dies,
                              int                        tier,
                              double                     bbox_cx,
                              double                     bbox_cy,
                              std::ostream*              log,
                              double                     log_phys_scale)
{
    const auto& nets = engine.nets();
    const int   nmod = static_cast<int>(modules.size());

    // 建立 module → incident net 索引
    std::vector<std::vector<int>> module_to_nets(static_cast<size_t>(nmod));
    for (int ni = 0; ni < static_cast<int>(nets.size()); ++ni) {
        for (int pid : nets[static_cast<size_t>(ni)].pins)
            module_to_nets[static_cast<size_t>(pid)].push_back(ni);
    }

    // 建立 hpwl tier weight 向量（與 compute_hpwl() 邏輯一致）
    const int nd = engine.num_dies();
    const auto& ov = engine.config().hpwl_die_weights;
    const auto& tw = engine.tier_net_weights();
    std::vector<double> die_w(static_cast<size_t>(nd), 1.0);
    if (static_cast<int>(ov.size()) == nd) {
        for (int i = 0; i < nd; ++i) die_w[i] = ov[i];
    } else if (static_cast<int>(tw.size()) == nd) {
        for (int i = 0; i < nd; ++i) die_w[i] = tw[i];
    }

    // 計算 net-centroid WL 合力
    std::vector<double> fx(static_cast<size_t>(nmod), 0.0);
    std::vector<double> fy(static_cast<size_t>(nmod), 0.0);
    compute_net_centroid_wl_force(modules, nets, fx, fy);

    // near→far 排序（針對此 tier 的可動 module；跳過 TSV proxy）
    std::vector<std::pair<double, int>> order;
    for (const Module& m : modules) {
        if (m.is_terminal || m.is_fixed || m.tier_id != tier) continue;
        if (m.is_tsv_proxy) continue;
        const double dx = m.x - bbox_cx, dy = m.y - bbox_cy;
        order.push_back({ dx * dx + dy * dy, m.id });
    }
    std::sort(order.begin(), order.end());

    const Die& die = dies[static_cast<size_t>(tier)];
    int n_moved = 0, n_reverted = 0, n_no_gap = 0;

    if (log) *log << "[wl-refine] Tier " << tier
                  << " near->far " << order.size() << " modules\n";

    for (const auto& [dist2, mid] : order) {
        Module& m = modules[static_cast<size_t>(mid)];
        const double f_x = fx[static_cast<size_t>(mid)];
        const double max_move = std::max(m.width, m.height);

        const bool allow_x = (m.boundary_side == 0) || boundary_allows_x_move(m.boundary_side);
        const bool allow_y = (m.boundary_side == 0) || boundary_allows_y_move(m.boundary_side);

        // ---- X 軸 ----
        bool x_moved = false;
        double old_x = m.x;
        if (allow_x && f_x > kWlForceEps) {
            const double gap = std::min(max_shift_x_pos(modules, die, m), max_move);
            if (gap > kGapMin) {
                const double hpwl_before = hpwl_incident_sum(modules, nets, module_to_nets, die_w, mid);
                m.x += gap;
                const double hpwl_after  = hpwl_incident_sum(modules, nets, module_to_nets, die_w, mid);
                if (hpwl_after > hpwl_before + kHpwlRollbackEps) {
                    m.x = old_x;
                    ++n_reverted;
                    if (log) *log << "  module_id=" << mid << " dx=+"
                                  << to_physical_len(gap, log_phys_scale)
                                  << " (reverted hpwl +"
                                  << to_physical_len(hpwl_after - hpwl_before, log_phys_scale)
                                  << ")\n";
                } else {
                    x_moved = true;
                }
            } else {
                ++n_no_gap;
            }
        } else if (allow_x && f_x < -kWlForceEps) {
            const double gap = std::min(max_shift_x_neg(modules, die, m), max_move);
            if (gap > kGapMin) {
                const double hpwl_before = hpwl_incident_sum(modules, nets, module_to_nets, die_w, mid);
                m.x -= gap;
                const double hpwl_after  = hpwl_incident_sum(modules, nets, module_to_nets, die_w, mid);
                if (hpwl_after > hpwl_before + kHpwlRollbackEps) {
                    m.x = old_x;
                    ++n_reverted;
                    if (log) *log << "  module_id=" << mid << " dx=-"
                                  << to_physical_len(gap, log_phys_scale)
                                  << " (reverted hpwl +"
                                  << to_physical_len(hpwl_after - hpwl_before, log_phys_scale)
                                  << ")\n";
                } else {
                    x_moved = true;
                }
            } else {
                ++n_no_gap;
            }
        }
        if (x_moved) ++n_moved;

        // ---- Y 軸（用更新後的 x 位置算鄰居）----
        bool y_moved = false;
        const double f_y2 = fy[static_cast<size_t>(mid)];
        double old_y = m.y;
        if (allow_y && f_y2 > kWlForceEps) {
            const double gap = std::min(max_shift_y_pos(modules, die, m), max_move);
            if (gap > kGapMin) {
                const double hpwl_before = hpwl_incident_sum(modules, nets, module_to_nets, die_w, mid);
                m.y += gap;
                const double hpwl_after  = hpwl_incident_sum(modules, nets, module_to_nets, die_w, mid);
                if (hpwl_after > hpwl_before + kHpwlRollbackEps) {
                    m.y = old_y;
                    ++n_reverted;
                    if (log) *log << "  module_id=" << mid << " dy=+"
                                  << to_physical_len(gap, log_phys_scale)
                                  << " (reverted hpwl +"
                                  << to_physical_len(hpwl_after - hpwl_before, log_phys_scale)
                                  << ")\n";
                } else {
                    y_moved = true;
                }
            } else {
                ++n_no_gap;
            }
        } else if (allow_y && f_y2 < -kWlForceEps) {
            const double gap = std::min(max_shift_y_neg(modules, die, m), max_move);
            if (gap > kGapMin) {
                const double hpwl_before = hpwl_incident_sum(modules, nets, module_to_nets, die_w, mid);
                m.y -= gap;
                const double hpwl_after  = hpwl_incident_sum(modules, nets, module_to_nets, die_w, mid);
                if (hpwl_after > hpwl_before + kHpwlRollbackEps) {
                    m.y = old_y;
                    ++n_reverted;
                    if (log) *log << "  module_id=" << mid << " dy=-"
                                  << to_physical_len(gap, log_phys_scale)
                                  << " (reverted hpwl +"
                                  << to_physical_len(hpwl_after - hpwl_before, log_phys_scale)
                                  << ")\n";
                } else {
                    y_moved = true;
                }
            } else {
                ++n_no_gap;
            }
        }
        if (y_moved) ++n_moved;

        if (m.boundary_side != 0) {
            snap_module_to_boundary(m, die, m.boundary_side);
        } else {
            m.x = std::max(m.width  * 0.5, std::min(die.width  - m.width  * 0.5, m.x));
            m.y = std::max(m.height * 0.5, std::min(die.height - m.height * 0.5, m.y));
        }
    }

    std::cout << "  Tier " << tier << " [wl-refine] moved=" << n_moved
              << " reverted_hpwl=" << n_reverted
              << " skipped_no_gap=" << n_no_gap << "\n";
}

// ── Sweep Counting Sort helpers ──────────────────────────────

enum class SweepSortMode {
    LxAsc, RxDesc, LyAsc, RyDesc, Dist2Asc
};

static SweepSortMode sweep_sort_mode_from_label(const std::string& label)
{
    if (label == "left->right")  return SweepSortMode::LxAsc;
    if (label == "right->left")  return SweepSortMode::RxDesc;
    if (label == "bottom->top")  return SweepSortMode::LyAsc;
    if (label == "top->bottom")  return SweepSortMode::RyDesc;
    return SweepSortMode::Dist2Asc;
}

static std::vector<int> build_sweep_order_counting(
    const std::vector<Module>& modules,
    int tier,
    SweepSortMode mode,
    double bbox_cx, double bbox_cy)
{
    std::vector<std::pair<long long, int>> keyed;
    keyed.reserve(modules.size());

    for (const Module& m : modules) {
        if (m.is_terminal || m.tier_id != tier || m.is_fixed) continue;
        long long key;
        switch (mode) {
        case SweepSortMode::LxAsc:   key = static_cast<long long>(std::llround(m.lx())); break;
        case SweepSortMode::RxDesc:  key = static_cast<long long>(std::llround(m.rx())); break;
        case SweepSortMode::LyAsc:   key = static_cast<long long>(std::llround(m.ly())); break;
        case SweepSortMode::RyDesc:  key = static_cast<long long>(std::llround(m.ry())); break;
        default: {
            const double dx = m.x - bbox_cx, dy = m.y - bbox_cy;
            key = static_cast<long long>(std::llround(dx * dx + dy * dy));
            break;
        }
        }
        keyed.push_back({key, m.id});
    }

    if (keyed.empty()) return {};

    long long min_key = keyed[0].first, max_key = keyed[0].first;
    for (const auto& [k, id] : keyed) {
        if (k < min_key) min_key = k;
        if (k > max_key) max_key = k;
    }

    const size_t K = static_cast<size_t>(max_key - min_key) + 1;
    std::vector<std::vector<int>> buckets(K);
    for (const auto& [k, id] : keyed)
        buckets[static_cast<size_t>(k - min_key)].push_back(id);

    for (auto& b : buckets)
        if (b.size() > 1) std::sort(b.begin(), b.end());

    std::vector<int> order;
    order.reserve(keyed.size());

    const bool ascending = (mode == SweepSortMode::LxAsc
                         || mode == SweepSortMode::LyAsc
                         || mode == SweepSortMode::Dist2Asc);
    if (ascending) {
        for (auto& b : buckets)
            for (int id : b) order.push_back(id);
    } else {
        for (auto it = buckets.rbegin(); it != buckets.rend(); ++it)
            for (int id : *it) order.push_back(id);
    }

    return order;
}

// ── Container Escape helpers ─────────────────────────────────

static bool is_contained_in(const Module& inner, const Module& outer)
{
    if (inner.id == outer.id) return false;
    if (inner.area() >= outer.area() - kEps) return false;
    return inner.lx() >= outer.lx() - kEps
        && inner.rx() <= outer.rx() + kEps
        && inner.ly() >= outer.ly() - kEps
        && inner.ry() <= outer.ry() + kEps;
}

static bool is_overlap_flat_in_move_box(const ModuleSpatialGrid&   grid,
                                         const std::vector<Module>& modules,
                                         const Module&              target,
                                         const Die&                 die,
                                         const LocalMoveConfig&     cfg)
{
    const double ov0 = total_weighted_overlap(grid, modules, target);
    if (ov0 <= kEps) return false;

    const double max_dx = std::min(cfg.max_move_dist, die.width  - target.rx());
    const double min_dx = std::max(-cfg.max_move_dist, -target.lx());
    const double max_dy = std::min(cfg.max_move_dist, die.height - target.ry());
    const double min_dy = std::max(-cfg.max_move_dist, -target.ly());

    const double tol = kEps * std::max(1.0, ov0);

    auto ov_at = [&](double dx, double dy) {
        Module t = target;
        t.x += dx;
        t.y += dy;
        return total_weighted_overlap(grid, modules, t);
    };

    const double probes[] = {
        ov_at(min_dx, 0.0), ov_at(max_dx, 0.0),
        ov_at(0.0, min_dy), ov_at(0.0, max_dy),
        ov_at(min_dx, min_dy), ov_at(max_dx, max_dy),
        ov_at(min_dx, max_dy), ov_at(max_dx, min_dy),
    };

    for (double ov : probes)
        if (std::fabs(ov - ov0) > tol)
            return false;
    return true;
}

static int find_container_module(const ModuleSpatialGrid&   grid,
                                  const std::vector<Module>& modules,
                                  const Module&              target)
{
    int    best_idx    = -1;
    double best_margin = std::numeric_limits<double>::infinity();

    grid.for_each_candidate(
        target.lx(), target.ly(), target.rx(), target.ry(),
        [&](int mi) {
            const Module& m = modules[static_cast<size_t>(mi)];
            if (!is_contained_in(target, m)) return;
            const double ml = target.lx() - m.lx();
            const double mr = m.rx()      - target.rx();
            const double mb = target.ly() - m.ly();
            const double mt = m.ry()      - target.ry();
            const double min_m = std::min({ ml, mr, mb, mt });
            if (min_m < best_margin) {
                best_margin = min_m;
                best_idx    = mi;
            }
        });

    return best_idx;
}

static void escape_along_nearest_edge(Module&       tm,
                                       const Module& container,
                                       const Die&    die,
                                       std::ostream* move_log,
                                       double        log_phys_scale)
{
    const double ml = tm.lx() - container.lx();
    const double mr = container.rx() - tm.rx();
    const double mb = tm.ly() - container.ly();
    const double mt = container.ry() - tm.ry();

    const double min_m = std::min({ ml, mr, mb, mt });

    constexpr double kGap = 1e-6;
    double dx = 0.0, dy = 0.0;
    const char* dir = "?";

    if (std::fabs(ml - min_m) <= kEps) {
        dx  = container.lx() - kGap - tm.rx();
        dir = "left";
    } else if (std::fabs(mr - min_m) <= kEps) {
        dx  = container.rx() + kGap - tm.lx();
        dir = "right";
    } else if (std::fabs(mb - min_m) <= kEps) {
        dy  = container.ly() - kGap - tm.ry();
        dir = "bottom";
    } else {
        dy  = container.ry() + kGap - tm.ly();
        dir = "top";
    }

    tm.x += dx;
    tm.y += dy;

    const double hw = tm.width  * 0.5;
    const double hh = tm.height * 0.5;
    tm.x = std::clamp(tm.x, hw,            die.width  - hw);
    tm.y = std::clamp(tm.y, hh,            die.height - hh);

    if (move_log)
        *move_log << "    [contain_escape] module_id=" << tm.id
                  << " dir=" << dir
                  << " dx=" << to_physical_len(dx, log_phys_scale)
                  << " dy=" << to_physical_len(dy, log_phys_scale)
                  << "\n";
}

static void try_container_escape(ModuleSpatialGrid&      grid,
                                  std::vector<Module>&    modules,
                                  int                     target_idx,
                                  const std::vector<Die>& dies,
                                  const LocalMoveConfig&  cfg,
                                  std::ostream*           move_log,
                                  double                  log_phys_scale)
{
    if (!cfg.enable_container_escape) return;

    Module& tm = modules[static_cast<size_t>(target_idx)];
    const int tier = tm.tier_id;
    if (tier < 0 || tier >= static_cast<int>(dies.size())) return;

    const Die& die = dies[static_cast<size_t>(tier)];

    if (!is_overlap_flat_in_move_box(grid, modules, tm, die, cfg)) return;

    const int container_idx = find_container_module(grid, modules, tm);
    if (container_idx < 0) return;

    const Module& container = modules[static_cast<size_t>(container_idx)];
    escape_along_nearest_edge(tm, container, die, move_log, log_phys_scale);

    if (tm.boundary_side != 0)
        snap_module_to_boundary(tm, die, tm.boundary_side);

    grid.update(target_idx, modules);
}

// ── grid-aware optimize（匿名 namespace，供 sweep 直接呼叫）──────

LocalMoveResult optimize_module_local_move_impl(
    std::vector<Module>&    modules,
    const std::vector<Die>& dies,
    int                     target_module_id,
    const LocalMoveConfig&  cfg,
    ModuleSpatialGrid&      grid,
    std::ostream*           move_log,
    double                  log_phys_scale)
{
    int target_idx = -1;
    for (size_t i = 0; i < modules.size(); ++i) {
        if (modules[i].id == target_module_id) {
            target_idx = static_cast<int>(i);
            break;
        }
    }
    if (target_idx < 0 || modules[target_idx].is_terminal) return {};
    if (modules[target_idx].is_fixed) return {};
    const Module& target = modules[target_idx];

    auto eval_orientation = [&](const Module& base, bool rotate_90) -> LocalMoveResult {
        const int tier = base.tier_id;
        if (tier < 0 || tier >= static_cast<int>(dies.size())) return {};

        const Die& die = dies[static_cast<size_t>(tier)];
        double min_dx = std::max(-cfg.max_move_dist, -base.lx());
        double max_dx = std::min( cfg.max_move_dist, die.width  - base.rx());
        double min_dy = std::max(-cfg.max_move_dist, -base.ly());
        double max_dy = std::min( cfg.max_move_dist, die.height - base.ry());

        if (base.boundary_side != 0) {
            if (boundary_is_corner(base.boundary_side)) {
                min_dx = max_dx = 0.0;
                min_dy = max_dy = 0.0;
            } else if (!boundary_allows_x_move(base.boundary_side)) {
                min_dx = max_dx = 0.0;
            } else if (!boundary_allows_y_move(base.boundary_side)) {
                min_dy = max_dy = 0.0;
            }
        }

        const double overlap_before = total_weighted_overlap(grid, modules, base);

        auto eval_order = [&](bool x_first) -> LocalMoveResult {
            LocalMoveResult r;
            r.rotate_90      = rotate_90;
            r.overlap_before = overlap_before;

            if (x_first) {
                const auto x_nbs = collect_x_neighbors(grid, modules, base, cfg.max_search_dist);
                r.dx = optimize_one_axis(x_nbs, base.lx(), base.rx(), min_dx, max_dx, cfg, 1, base.x);
                Module after_x = base;
                after_x.x += r.dx;
                r.overlap_after_x = total_weighted_overlap(grid, modules, after_x);

                const auto y_nbs = collect_y_neighbors(grid, modules, base, cfg.max_search_dist, r.dx);
                r.dy = optimize_one_axis(y_nbs, base.ly(), base.ry(), min_dy, max_dy, cfg, 2, base.y);
                Module after_xy = after_x;
                after_xy.y += r.dy;
                r.overlap_after_y = total_weighted_overlap(grid, modules, after_xy);
            } else {
                const auto y_nbs = collect_y_neighbors(grid, modules, base, cfg.max_search_dist, 0.0);
                r.dy = optimize_one_axis(y_nbs, base.ly(), base.ry(), min_dy, max_dy, cfg, 2, base.y);
                Module after_y = base;
                after_y.y += r.dy;
                r.overlap_after_x = total_weighted_overlap(grid, modules, after_y);

                Module base_shifted_y = base;
                base_shifted_y.y += r.dy;
                const auto x_nbs = collect_x_neighbors(grid, modules, base_shifted_y, cfg.max_search_dist);
                r.dx = optimize_one_axis(x_nbs, base_shifted_y.lx(), base_shifted_y.rx(),
                                         min_dx, max_dx, cfg, 1, base_shifted_y.x);
                Module after_yx = after_y;
                after_yx.x += r.dx;
                r.overlap_after_y = total_weighted_overlap(grid, modules, after_yx);
            }

            Module final_m = base;
            final_m.x += r.dx;
            final_m.y += r.dy;
            double side = 0.0;
            if (cfg.side_bias_axis == 1) side = side_bias_linear(cfg, 1, final_m.x);
            if (cfg.side_bias_axis == 2) side = side_bias_linear(cfg, 2, final_m.y);
            r.objective = cfg.disp_weight * (r.dx * r.dx + r.dy * r.dy)
                        + cfg.overlap_weight * r.overlap_after_y
                        + side;
            return r;
        };

        const LocalMoveResult xy = eval_order(true);
        const LocalMoveResult yx = eval_order(false);
        return (yx.objective < xy.objective - kEps) ? yx : xy;
    };

    const double w0 = target.width;
    const double h0 = target.height;
    const std::vector<std::pair<double,double>> shapes =
        target.is_soft ? build_soft_shape_curve(w0, h0, cfg)
                       : std::vector<std::pair<double,double>>{{w0, h0}};

    LocalMoveResult best;
    best.objective = std::numeric_limits<double>::infinity();

    for (const auto& [sw, sh] : shapes) {
        Module base0 = target;
        base0.width  = sw;
        base0.height = sh;
        LocalMoveResult r0 = eval_orientation(base0, false);
        r0.final_width  = sw;
        r0.final_height = sh;
        if (r0.objective < best.objective - kEps) best = r0;

        if (target.is_tsv_proxy) continue;

        Module base90 = base0;
        std::swap(base90.width, base90.height);
        LocalMoveResult r90 = eval_orientation(base90, true);
        r90.final_width  = sw;
        r90.final_height = sh;
        if (r90.objective < best.objective - kEps) best = r90;
    }

    Module& tm = modules[target_idx];
    if (best.final_width > kEps) {
        tm.width  = best.final_width;
        tm.height = best.final_height;
    }
    if (best.rotate_90) std::swap(tm.width, tm.height);
    tm.x += best.dx;
    tm.y += best.dy;
    tm.move_weight = std::min(cfg.max_module_weight, tm.move_weight * cfg.moved_weight_mul);

    if (tm.boundary_side != 0) {
        const int tier_snap = tm.tier_id;
        if (tier_snap >= 0 && tier_snap < static_cast<int>(dies.size()))
            snap_module_to_boundary(tm, dies[static_cast<size_t>(tier_snap)], tm.boundary_side);
    }

    try_container_escape(grid, modules, target_idx, dies, cfg, move_log, log_phys_scale);

    grid.update(target_idx, modules);

    if (move_log) {
        *move_log << "    [move_apply]"
                  << " module_id=" << tm.id
                  << " center=(" << to_physical_len(tm.x, log_phys_scale)
                  << ", " << to_physical_len(tm.y, log_phys_scale) << ")"
                  << " dx=" << to_physical_len(best.dx, log_phys_scale)
                  << " dy=" << to_physical_len(best.dy, log_phys_scale)
                  << " rot90=" << best.rotate_90;
        if (target.is_soft && best.final_width > kEps)
            *move_log << " wh=(" << to_physical_len(tm.width,  log_phys_scale)
                      << "," << to_physical_len(tm.height, log_phys_scale) << ")";
        *move_log << " weight=" << tm.move_weight << "\n";
    }

    return best;
}

} // namespace

// ============================================================
// Public API
// ============================================================

LocalMoveResult optimize_module_local_move(std::vector<Module>&    modules,
                                           const std::vector<Die>& dies,
                                           int                     target_module_id,
                                           const LocalMoveConfig&  cfg,
                                           std::ostream*           move_log,
                                           double                  log_phys_scale)
{
    int target_idx = -1;
    for (size_t i = 0; i < modules.size(); ++i) {
        if (modules[i].id == target_module_id) {
            target_idx = static_cast<int>(i);
            break;
        }
    }
    if (target_idx < 0 || modules[target_idx].is_terminal) return {};
    if (modules[target_idx].is_fixed) return {};

    const int tier = modules[target_idx].tier_id;
    if (tier < 0 || tier >= static_cast<int>(dies.size())) return {};

    ModuleSpatialGrid grid;
    grid.build(tier, modules, dies[static_cast<size_t>(tier)], cfg.max_search_dist);
    return optimize_module_local_move_impl(modules, dies, target_module_id,
                                           cfg, grid, move_log, log_phys_scale);
}

// ============================================================
// shake_nearby_rotations
// ============================================================
//
// 1. 掃描 tier 中所有 module 對，找出有重疊的 module（overlap area > 0）。
// 2. 對每個 non-terminal movable module，若其中心與任一「重疊 module」中心的距離
//    <= radius，則以 rotate_prob 的機率對其執行 90° 旋轉（以中心點為軸）。
//    旋轉後若超出 die 邊界，會將位置 clamp 回邊界內。
// 3. 回傳實際旋轉的 module 數量。
int shake_nearby_rotations(std::vector<Module>&    modules,
                           const std::vector<Die>& dies,
                           int                     tier,
                           double                  radius,
                           double                  rotate_prob,
                           unsigned                seed,
                           std::ostream*           log,
                           double                  log_phys_scale)
{
    // 找有重疊的 module id
    const auto overlap_ids = collect_overlap_module_ids(modules, tier);
    if (overlap_ids.empty()) return 0;

    // 記錄重疊 module 的中心點
    std::vector<std::pair<double, double>> ov_centers;
    ov_centers.reserve(overlap_ids.size());
    for (const Module& m : modules) {
        if (overlap_ids.count(m.id))
            ov_centers.push_back({ m.x, m.y });
    }

    const Die& die = dies[static_cast<size_t>(tier)];
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dice(0.0, 1.0);
    int rotated_count = 0;

    for (Module& m : modules) {
        if (m.is_terminal || m.tier_id != tier || m.is_fixed) continue;
        if (m.is_tsv_proxy) continue; // TSV proxy 不旋轉

        // 判斷是否在任一重疊 module 的 radius 內
        bool nearby = false;
        for (const auto& [cx, cy] : ov_centers) {
            const double dx = m.x - cx, dy = m.y - cy;
            if (dx * dx + dy * dy <= radius * radius) { nearby = true; break; }
        }
        if (!nearby) continue;

        // 機率決定是否旋轉
        if (dice(rng) > rotate_prob) continue;

        // 執行旋轉
        std::swap(m.width, m.height);

        if (m.boundary_side != 0) {
            snap_module_to_boundary(m, die, m.boundary_side);
        } else {
            const double half_w = m.width  * 0.5;
            const double half_h = m.height * 0.5;
            m.x = std::clamp(m.x, half_w,         die.width  - half_w);
            m.y = std::clamp(m.y, half_h,         die.height - half_h);
        }

        if (log) {
            *log << "    [shake_rot] module_id=" << m.id
                 << " new_size=(" << to_physical_len(m.width, log_phys_scale)
                 << "x" << to_physical_len(m.height, log_phys_scale) << ")"
                 << " center=(" << to_physical_len(m.x, log_phys_scale)
                 << ", " << to_physical_len(m.y, log_phys_scale) << ")\n";
        }
        ++rotated_count;
    }
    return rotated_count;
}

void run_legalize_heu(PlacementEngine& engine, const PartitionConfig& pcfg)
{

    // ---- log 輸出：TeeStreambuf 把所有 std::cout 同時鏡像到檔案 ----
    static const std::string kLogPath = "legalize_process.txt";
    std::ofstream log_file(kLogPath, std::ios::trunc);
    if (!log_file)
        std::cerr << "[legalize_heu] WARNING: cannot open " << kLogPath << "\n";

    struct TeeStreambuf : std::streambuf {
        std::streambuf* orig;
        std::streambuf* file;
        TeeStreambuf(std::streambuf* o, std::streambuf* f) : orig(o), file(f) {}
        int overflow(int c) override {
            if (c == EOF) return c;
            orig->sputc(static_cast<char>(c));
            if (file) file->sputc(static_cast<char>(c));
            return c;
        }
        std::streamsize xsputn(const char* s, std::streamsize n) override {
            orig->sputn(s, n);
            if (file) file->sputn(s, n);
            return n;
        }
    };
    TeeStreambuf tee(std::cout.rdbuf(),
                     log_file ? log_file.rdbuf() : nullptr);
    std::streambuf* orig_buf = std::cout.rdbuf(&tee);

    // log 內長度/位移：若有 die normalize，換算為物理座標（× 1/geometry_scale）
    const double gs     = engine.geometry_scale();
    const double inv_gs = (std::fabs(gs - 1.0) > 1e-12) ? (1.0 / gs) : 1.0;
    if (std::fabs(inv_gs - 1.0) > 1e-12)
        std::cout << "[legalize_heu] log lengths/displacements in physical coords (geometry_scale="
                  << gs << ")\n";

    // ---- 設定 ----
    // sweep 方向偏好強度：objective += weight * sign * center
    // 增大此值讓 module 更積極往 sweep 方向側靠；0 關閉
    static constexpr double kSideBiasWeight = 0.0;

    LocalMoveConfig lcfg;
    lcfg.max_search_dist   = 30.0;
    lcfg.max_move_dist     = 30.0;
    lcfg.disp_weight       = 3.0;
    lcfg.overlap_weight    = 1.0;
    lcfg.moved_weight_mul  = 3;
    lcfg.max_module_weight = 1e6;
    // soft module 等面積 shape curve
    lcfg.soft_aspect_curve_steps = 15;
    lcfg.soft_aspect_min         = 0.33;
    lcfg.soft_aspect_max         = 3.0;
    // side_bias_* 預設關閉；由 sweep lambda 依 label 填入

    auto& modules = engine.modules_mutable();
    const auto& dies = engine.dies();

    // snap 所有 boundary module 到對應邊/角（含 leg_only 路徑）
    {
        int snapped = 0;
        for (Module& m : modules) {
            if (m.is_terminal || m.boundary_side == 0) continue;
            if (m.tier_id < 0 || m.tier_id >= static_cast<int>(dies.size())) continue;
            snap_module_to_boundary(m, dies[static_cast<size_t>(m.tier_id)], m.boundary_side);
            ++snapped;
        }
        if (snapped > 0)
            std::cout << "[legalize_heu] snapped " << snapped << " boundary module(s) to die edge\n";
    }

    std::cout << "\n[legalize_heu] multi-pass local move\n";

    // per-tier 視覺化記錄器（enable_legalize_vis=false 時始終為空）
    std::optional<LegalizeFrameWriter> vis_fw;

    // 對指定排序跑一整層的 local move
    // init_weight    : sweep 開始前將該層所有 movable module 的 move_weight 重設為此值
    // moved_weight_mul : 每個 module 移動後 move_weight 乘上的倍率（覆寫 lcfg）
    auto sweep = [&](int tier, const std::string& label,
                     double init_weight,
                     double moved_weight_mul,
                     ModuleSpatialGrid& grid,
                     double bbox_cx, double bbox_cy)
    {
        for (Module& m : modules) {
            if (m.is_terminal || m.tier_id != tier) continue;
            if (m.is_fixed)
                m.move_weight = std::min(lcfg.max_module_weight,
                                         m.move_weight * moved_weight_mul);
            else
                m.move_weight = init_weight;
        }

        const auto order = build_sweep_order_counting(
            modules, tier,
            sweep_sort_mode_from_label(label),
            bbox_cx, bbox_cy);

        LocalMoveConfig sweep_cfg = lcfg;
        sweep_cfg.moved_weight_mul = moved_weight_mul;
        apply_side_bias_from_label(sweep_cfg, label, kSideBiasWeight);

        std::cout << "  Tier " << tier << " [" << label << "]"
                  << " init_w=" << init_weight
                  << " mul=" << moved_weight_mul
                  << " side_bias=" << sweep_cfg.side_bias_weight * sweep_cfg.side_bias_sign
                  << "(axis=" << sweep_cfg.side_bias_axis << ")\n";
        for (int mid : order)
            optimize_module_local_move_impl(modules, dies, mid, sweep_cfg, grid, &log_file, inv_gs);

        if (vis_fw) vis_fw->capture(modules, dies, tier, label);
    };

    for (int t = 0; t < engine.num_dies(); ++t) {
        const BBox box = compute_tier_module_bbox(modules, t);
        if (!box.valid) {
            std::cout << "  Tier " << t << ": no movable modules\n";
            continue;
        }

        const double cx = 0.5 * (box.xmin + box.xmax);
        const double cy = 0.5 * (box.ymin + box.ymax);

        std::cout << "  Tier " << t
                  << " bbox_center=(" << to_physical_len(cx, inv_gs)
                  << ", " << to_physical_len(cy, inv_gs) << ")\n";

        static constexpr int    kMaxShakeIters  = 10;
        static constexpr double kShakeRadius    = 50.0;
        static constexpr double kShakeProb      = 0.5;      // modified

        // 記錄進入本 tier 前的 module 位置快照，供 retry 還原用
        struct ModuleSnapshot { int id; double x, y, width, height; };
        auto take_snapshot = [&]() {
            std::vector<ModuleSnapshot> snap;
            for (const Module& m : modules)
                if (!m.is_terminal && m.tier_id == t)
                    snap.push_back({ m.id, m.x, m.y, m.width, m.height });
            return snap;
        };
        auto restore_snapshot = [&](const std::vector<ModuleSnapshot>& snap) {
            for (Module& m : modules) {
                if (m.is_terminal || m.tier_id != t) continue;
                for (const auto& s : snap) {
                    if (s.id == m.id) {
                        m.x = s.x; m.y = s.y;
                        m.width = s.width; m.height = s.height;
                        m.move_weight = 1.0;
                        break;
                    }
                }
            }
        };

        // 依 use_alt 決定 sweep 序列：false = 原始順序；true = 上下與左右調換
        // use_strong：weak（預設）先嘗試；weak 失敗後以 strong 重試
        auto run_one_pass = [&](int iter, bool use_alt, bool use_strong = false) {
            const double iw  = use_strong ? std::pow(3.0, iter) : std::pow(2.0, iter);
            const double mw1 = use_strong ? (3.0  + iter * 4.0) : (2.0  + iter * 2.0);
            const double mw2 = use_strong ? (9.0  + iter * 6.0) : (3.0  + iter * 3.0);
            const double mw3 = use_strong ? (20.0 + iter * 8.0) : (4.0  + iter * 4.0);
            // const double iw  = use_strong ? std::pow(1.2, iter) : std::pow(1.1, iter);
            // const double mw1 = use_strong ? (1.0  + iter * 2.0) : (1.0  + iter * 1.0);
            // const double mw2 = use_strong ? (2.0  + iter * 4.0) : (1.5  + iter * 1.0);
            // const double mw3 = use_strong ? (3.0 + iter * 6.0) : (2.0  + iter * 1.0);
            // const double iw  = use_strong ? std::pow(3.0, iter) : std::pow(3.0, iter);
            // const double mw1 = use_strong ? (3.0  + iter * 4.0) : (3.0  + iter * 4.0);
            // const double mw2 = use_strong ? (9.0  + iter * 6.0) : (9.0  + iter * 6.0);
            // const double mw3 = use_strong ? (20.0 + iter * 8.0) : (20.0  + iter * 8.0);

            ModuleSpatialGrid grid;
            grid.build(t, modules, dies[static_cast<size_t>(t)], lcfg.max_search_dist);

            sweep(t, "near->far",    iw,       mw1, grid, cx, cy);

            if (!use_alt) {
                sweep(t, "left->right",  iw * 3,  mw1, grid, cx, cy);
                sweep(t, "right->left",  iw * 3,  mw1, grid, cx, cy);
                sweep(t, "bottom->top",  iw * 9,  mw2, grid, cx, cy);
                sweep(t, "top->bottom",  iw * 9,  mw2, grid, cx, cy);
                sweep(t, "left->right",  iw * 27, mw3, grid, cx, cy);
                sweep(t, "right->left",  iw * 27, mw3, grid, cx, cy);
                sweep(t, "bottom->top",  iw * 81, mw3, grid, cx, cy);
                sweep(t, "top->bottom",  iw * 81, mw3, grid, cx, cy);
            } else {
                sweep(t, "bottom->top",  iw * 3,  mw1, grid, cx, cy);
                sweep(t, "top->bottom",  iw * 3,  mw1, grid, cx, cy);
                sweep(t, "left->right",  iw * 9,  mw2, grid, cx, cy);
                sweep(t, "right->left",  iw * 9,  mw2, grid, cx, cy);
                sweep(t, "bottom->top",  iw * 27, mw3, grid, cx, cy);
                sweep(t, "top->bottom",  iw * 27, mw3, grid, cx, cy);
                sweep(t, "left->right",  iw * 81, mw3, grid, cx, cy);
                sweep(t, "right->left",  iw * 81, mw3, grid, cx, cy);
            }
        };

        const auto initial_snap = take_snapshot();

        // 視覺化：初始化 tier 記錄器，擷取初始位置幀
        if (pcfg.enable_legalize_vis) {
            vis_fw.emplace();
            LegalizeVisConfig vcfg;
            vcfg.out_dir = pcfg.legalize_vis_dir;
            vcfg.upscale = pcfg.legalize_vis_upscale;
            vis_fw->begin_tier(t, dies[static_cast<size_t>(t)].width,
                               dies[static_cast<size_t>(t)].height, vcfg);
            vis_fw->capture(modules, dies, t, "initial");
        }

        // ---- Phase 1：原始順序（左右優先）----
        bool phase1_done = false;
        for (int iter = 0; iter < kMaxShakeIters; ++iter) {
            if (!has_tier_overlaps(modules, t)) {
                std::cout << "  Tier " << t << " [iter=" << iter
                          << "] no overlaps, done\n";
                phase1_done = true;
                break;
            }

            std::cout << "  Tier " << t << " [iter=" << iter << "] running sweeps (h-first)\n";
            run_one_pass(iter, /*use_alt=*/false);

            if (!has_tier_overlaps(modules, t)) {
                std::cout << "  Tier " << t << " [iter=" << iter
                          << "] overlaps cleared after sweep\n";
                phase1_done = true;
                break;
            }

            // 最後一輪不再 rotate（下一輪無法接續 sweep，旋轉只會留下不確定狀態）
            if (iter < kMaxShakeIters - 1) {
                const unsigned shake_seed = static_cast<unsigned>(iter * 997 + t * 31);
                std::cout << "  Tier " << t << " [shake rotations]\n";
                const int n_rot = shake_nearby_rotations(
                    modules, dies, t, kShakeRadius, kShakeProb, shake_seed, &log_file, inv_gs);
                std::cout << "  Tier " << t << " [iter=" << iter
                          << "] shake rotated=" << n_rot << " modules\n";
            }
        }

        if (!phase1_done) {
        // ---- Phase 2：還原初始位置，改用上下優先順序重試（weak）----
        std::cout << "  Tier " << t << " [retry] restoring initial positions, switching to v-first sweeps\n";
        restore_snapshot(initial_snap);

        for (int iter = 0; iter < kMaxShakeIters; ++iter) {
            if (!has_tier_overlaps(modules, t)) {
                std::cout << "  Tier " << t << " [retry iter=" << iter
                          << "] no overlaps, done\n";
                break;
            }

            std::cout << "  Tier " << t << " [retry iter=" << iter << "] running sweeps (v-first)\n";
            run_one_pass(iter, /*use_alt=*/true);

            if (!has_tier_overlaps(modules, t)) {
                std::cout << "  Tier " << t << " [retry iter=" << iter
                          << "] overlaps cleared after sweep\n";
                break;
            }

            // 最後一輪不再 rotate
            if (iter < kMaxShakeIters - 1) {
                const unsigned shake_seed = static_cast<unsigned>((iter + kMaxShakeIters) * 997 + t * 31);
                std::cout << "  Tier " << t << " [retry shake rotations]\n";
                const int n_rot = shake_nearby_rotations(
                    modules, dies, t, kShakeRadius, kShakeProb, shake_seed, &log_file, inv_gs);
                std::cout << "  Tier " << t << " [retry iter=" << iter
                          << "] shake rotated=" << n_rot << " modules\n";
            }
        }
        } // end if (!phase1_done)

        // ---- Phase 3：weak 完全失敗，以 strong 參數從初始快照重新嘗試 ----
        if (has_tier_overlaps(modules, t)) {
        std::cout << "  Tier " << t << " [strong] weak failed, restoring initial positions, retrying with strong params\n";
        restore_snapshot(initial_snap);

        // Phase 3a：strong h-first
        bool phase3_done = false;
        for (int iter = 0; iter < kMaxShakeIters; ++iter) {
            if (!has_tier_overlaps(modules, t)) {
                std::cout << "  Tier " << t << " [strong iter=" << iter
                          << "] no overlaps, done\n";
                phase3_done = true;
                break;
            }

            std::cout << "  Tier " << t << " [strong iter=" << iter << "] running sweeps (h-first)\n";
            run_one_pass(iter, /*use_alt=*/false, /*use_strong=*/true);

            if (!has_tier_overlaps(modules, t)) {
                std::cout << "  Tier " << t << " [strong iter=" << iter
                          << "] overlaps cleared after sweep\n";
                phase3_done = true;
                break;
            }

            if (iter < kMaxShakeIters - 1) {
                const unsigned shake_seed = static_cast<unsigned>((iter + kMaxShakeIters * 2) * 997 + t * 31);
                std::cout << "  Tier " << t << " [strong shake rotations]\n";
                const int n_rot = shake_nearby_rotations(
                    modules, dies, t, kShakeRadius, kShakeProb, shake_seed, &log_file, inv_gs);
                std::cout << "  Tier " << t << " [strong iter=" << iter
                          << "] shake rotated=" << n_rot << " modules\n";
            }
        }

        if (!phase3_done) {
        // Phase 3b：strong v-first
        std::cout << "  Tier " << t << " [strong-v] restoring, switching to v-first\n";
        restore_snapshot(initial_snap);

        for (int iter = 0; iter < kMaxShakeIters; ++iter) {
            if (!has_tier_overlaps(modules, t)) {
                std::cout << "  Tier " << t << " [strong-v iter=" << iter
                          << "] no overlaps, done\n";
                break;
            }

            std::cout << "  Tier " << t << " [strong-v iter=" << iter << "] running sweeps (v-first)\n";
            run_one_pass(iter, /*use_alt=*/true, /*use_strong=*/true);

            if (!has_tier_overlaps(modules, t)) {
                std::cout << "  Tier " << t << " [strong-v iter=" << iter
                          << "] overlaps cleared after sweep\n";
                break;
            }

            if (iter < kMaxShakeIters - 1) {
                const unsigned shake_seed = static_cast<unsigned>((iter + kMaxShakeIters * 3) * 997 + t * 31);
                std::cout << "  Tier " << t << " [strong-v shake rotations]\n";
                const int n_rot = shake_nearby_rotations(
                    modules, dies, t, kShakeRadius, kShakeProb, shake_seed, &log_file, inv_gs);
                std::cout << "  Tier " << t << " [strong-v iter=" << iter
                          << "] shake rotated=" << n_rot << " modules\n";
            }
        }
        } // end if (!phase3_done)
        } // end if (has_tier_overlaps) → Phase 3

        // 視覺化：tier 最終幀 + 寫 manifest
        if (vis_fw) {
            vis_fw->capture(modules, dies, t, "tier_done");
            vis_fw->end_tier();
            vis_fw.reset();
        }

    }

    // ---- Post-legalize WL refinement：所有 tier legalize 完後，再逐 tier refine ----
    if (pcfg.enable_wl_refine) {
        std::cout << "\n[legalize_heu] WL refinement pass (all tiers legalized)\n";
        for (int t = 0; t < engine.num_dies(); ++t) {
            if (!has_tier_overlaps(modules, t)) {
                const double bcx = dies[static_cast<size_t>(t)].width  * 0.5;
                const double bcy = dies[static_cast<size_t>(t)].height * 0.5;
                refine_tier_wl_centroid(engine, modules, dies, t, bcx, bcy, &log_file, inv_gs);
            } else {
                std::cout << "  Tier " << t << " [wl-refine] skipped (overlaps remain)\n";
            }
        }
    }

    std::cout.rdbuf(orig_buf);  // 還原 stdout
    if (log_file)
        std::cout << "[legalize_heu] log written -> " << kLogPath << "\n";

    // 視覺化：所有 tier 寫完後呼叫 Python 腳本合成 GIF
    if (pcfg.enable_legalize_vis) {
        const std::string cmd =
            "python3 scripts/make_legalize_gif.py "
            + pcfg.legalize_vis_dir
            + " --fps " + std::to_string(pcfg.legalize_gif_fps);
        std::cout << "[LegalizeVis] Running: " << cmd << "\n";
        if (std::system(cmd.c_str()) != 0)
            std::cerr << "[LegalizeVis] Warning: GIF script failed or not found\n";
    }
}
