// Heuristic legalization — standalone experimental flow
#include "legalize_heu.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <unordered_set>
#include <vector>

// ============================================================
// 匿名 namespace：幾何輔助型別與 1D 求解器
// ============================================================
namespace {

constexpr double kEps = 1e-12;

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
// 目標函數 = disp_weight*d^2 + overlap_weight*(slope*d + intercept)
// 導數 = 2*disp_weight*d + overlap_weight*slope = 0
// → d* = -overlap_weight*slope / (2*disp_weight)，clamp 到 [seg_lo, seg_hi]

double solve_best_d_on_segment(const std::vector<Neighbor1D>& nbs,
                               double move_l, double move_r,
                               double seg_lo, double seg_hi,
                               const LocalMoveConfig& cfg)
{
    if (seg_hi < seg_lo + kEps) return seg_lo;

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

    if (cfg.disp_weight <= kEps) {
        // 無位移懲罰：線性函數，最小值在端點
        if (slope > kEps)  return seg_lo;
        if (slope < -kEps) return seg_hi;
        return (std::fabs(seg_lo) <= std::fabs(seg_hi)) ? seg_lo : seg_hi;
    }

    // 二次函數最小點
    double d_star = -slope / (2.0 * cfg.disp_weight);
    return std::clamp(d_star, seg_lo, seg_hi);
}

// ── 單軸最佳位移 ─────────────────────────────────────────────

double optimize_one_axis(const std::vector<Neighbor1D>& nbs,
                         double move_l, double move_r,
                         double min_move_allowed, double max_move_allowed,
                         const LocalMoveConfig& cfg)
{
    // module 旋轉後尺寸超過 die：選最能縮小越界量的邊界
    if (max_move_allowed < min_move_allowed + kEps) {
        return (std::fabs(min_move_allowed) <= std::fabs(max_move_allowed))
                   ? min_move_allowed : max_move_allowed;
    }

    double best_d   = std::clamp(0.0, min_move_allowed, max_move_allowed);
    double best_obj = cfg.disp_weight * best_d * best_d
                    + cfg.overlap_weight
                      * overlap_objective_1d(nbs, move_l, move_r, best_d);
    if (nbs.empty()) return best_d;

    const auto pts = build_breakpoints(nbs, move_l, move_r,
                                       min_move_allowed, max_move_allowed);

    auto try_d = [&](double d) {
        const double obj = cfg.disp_weight * d * d
                         + cfg.overlap_weight
                           * overlap_objective_1d(nbs, move_l, move_r, d);
        if (obj < best_obj - kEps) { best_obj = obj; best_d = d; }
    };

    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const double seg_lo = pts[i];
        const double seg_hi = pts[i + 1];
        if (seg_hi < seg_lo + kEps) continue;

        try_d(seg_lo);
        try_d(seg_hi);
        try_d(solve_best_d_on_segment(nbs, move_l, move_r, seg_lo, seg_hi, cfg));
    }

    return best_d;
}

// ── 鄰居收集 ─────────────────────────────────────────────────

std::vector<Neighbor1D> collect_x_neighbors(const std::vector<Module>& modules,
                                            const Module& target,
                                            double max_search)
{
    std::vector<Neighbor1D> nbs;
    for (const Module& m : modules) {
        if (m.id == target.id || m.is_terminal || m.tier_id != target.tier_id) continue;
        const double y_ov = overlap_1d(target.ly(), target.ry(), m.ly(), m.ry());
        if (y_ov <= kEps) continue;
        if (m.rx() < target.lx() - max_search || m.lx() > target.rx() + max_search) continue;
        nbs.push_back({ y_ov, m.lx(), m.rx(), target.move_weight * m.move_weight });
    }
    return nbs;
}

std::vector<Neighbor1D> collect_y_neighbors(const std::vector<Module>& modules,
                                            const Module& target,
                                            double max_search, double dx_applied)
{
    std::vector<Neighbor1D> nbs;
    const double shifted_lx = target.lx() + dx_applied;
    const double shifted_rx = target.rx() + dx_applied;
    for (const Module& m : modules) {
        if (m.id == target.id || m.is_terminal || m.tier_id != target.tier_id) continue;
        const double x_ov = overlap_1d(shifted_lx, shifted_rx, m.lx(), m.rx());
        if (x_ov <= kEps) continue;
        if (m.ry() < target.ly() - max_search || m.ly() > target.ry() + max_search) continue;
        nbs.push_back({ x_ov, m.ly(), m.ry(), target.move_weight * m.move_weight });
    }
    return nbs;
}

// 加權重疊面積（target.move_weight * neighbor.move_weight * 面積）
double total_weighted_overlap(const std::vector<Module>& modules, const Module& target)
{
    double total = 0.0;
    for (const Module& m : modules) {
        if (m.id == target.id || m.is_terminal || m.tier_id != target.tier_id) continue;
        total += (target.move_weight * m.move_weight) * overlap_area(target, m);
    }
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

} // namespace

// ============================================================
// Public API
// ============================================================

LocalMoveResult optimize_module_local_move(std::vector<Module>&    modules,
                                           const std::vector<Die>& dies,
                                           int                     target_module_id,
                                           const LocalMoveConfig&  cfg,
                                           std::ostream*           move_log)
{
    // 找目標 module
    int target_idx = -1;
    for (size_t i = 0; i < modules.size(); ++i) {
        if (modules[i].id == target_module_id) {
            target_idx = static_cast<int>(i);
            break;
        }
    }
    if (target_idx < 0 || modules[target_idx].is_terminal) return {};
    if (modules[target_idx].is_fixed) return {}; // constraint 固定，不移動
    const Module& target = modules[target_idx];

    // 評估一種 orientation（0° 或 90°），回傳最佳位移結果
    // 分別嘗試 x→y 與 y→x 順序，取 objective 較小者
    auto eval_orientation = [&](const Module& base, bool rotate_90) -> LocalMoveResult {
        const int tier = base.tier_id;
        if (tier < 0 || tier >= static_cast<int>(dies.size())) return {};

        const Die& die = dies[static_cast<size_t>(tier)];
        const double min_dx = std::max(-cfg.max_move_dist, -base.lx());
        const double max_dx = std::min( cfg.max_move_dist, die.width  - base.rx());
        const double min_dy = std::max(-cfg.max_move_dist, -base.ly());
        const double max_dy = std::min( cfg.max_move_dist, die.height - base.ry());

        const double overlap_before = total_weighted_overlap(modules, base);

        // 給定 x_first：true = x→y，false = y→x
        auto eval_order = [&](bool x_first) -> LocalMoveResult {
            LocalMoveResult r;
            r.rotate_90     = rotate_90;
            r.overlap_before = overlap_before;

            if (x_first) {
                // 1) x 軸
                const auto x_nbs = collect_x_neighbors(modules, base, cfg.max_search_dist);
                r.dx = optimize_one_axis(x_nbs, base.lx(), base.rx(), min_dx, max_dx, cfg);
                Module after_x = base;
                after_x.x += r.dx;
                r.overlap_after_x = total_weighted_overlap(modules, after_x);

                // 2) y 軸（以 x 移動後為基礎）
                const auto y_nbs = collect_y_neighbors(modules, base, cfg.max_search_dist, r.dx);
                r.dy = optimize_one_axis(y_nbs, base.ly(), base.ry(), min_dy, max_dy, cfg);
                Module after_xy = after_x;
                after_xy.y += r.dy;
                r.overlap_after_y = total_weighted_overlap(modules, after_xy);
            } else {
                // 1) y 軸
                const auto y_nbs = collect_y_neighbors(modules, base, cfg.max_search_dist, 0.0);
                r.dy = optimize_one_axis(y_nbs, base.ly(), base.ry(), min_dy, max_dy, cfg);
                Module after_y = base;
                after_y.y += r.dy;
                r.overlap_after_x = total_weighted_overlap(modules, after_y); // 中間狀態

                // 2) x 軸（以 y 移動後為基礎）
                // 收集 x 鄰居時用 y 移動後的新 y 區間
                Module base_shifted_y = base;
                base_shifted_y.y += r.dy;
                const auto x_nbs = collect_x_neighbors(modules, base_shifted_y, cfg.max_search_dist);
                r.dx = optimize_one_axis(x_nbs, base_shifted_y.lx(), base_shifted_y.rx(),
                                         min_dx, max_dx, cfg);
                Module after_yx = after_y;
                after_yx.x += r.dx;
                r.overlap_after_y = total_weighted_overlap(modules, after_yx);
            }

            r.objective = cfg.disp_weight * (r.dx * r.dx + r.dy * r.dy)
                        + cfg.overlap_weight * r.overlap_after_y;
            return r;
        };

        const LocalMoveResult xy = eval_order(true);
        const LocalMoveResult yx = eval_order(false);
        return (yx.objective < xy.objective - kEps) ? yx : xy;
    };

    // 比較 0° 與 90° 解，取 objective 較小者（同分時偏好不旋轉）
    const LocalMoveResult result_0  = eval_orientation(target, false);
    Module target_rot = target;
    std::swap(target_rot.width, target_rot.height);
    const LocalMoveResult result_90 = eval_orientation(target_rot, true);

    const LocalMoveResult best =
        (result_90.objective < result_0.objective - kEps) ? result_90 : result_0;

    // 套用最佳解：旋轉、位移、提高 move_weight
    Module& tm = modules[target_idx];
    if (best.rotate_90) std::swap(tm.width, tm.height);
    tm.x += best.dx;
    tm.y += best.dy;
    tm.move_weight = std::min(cfg.max_module_weight,
                              tm.move_weight * cfg.moved_weight_mul);

    if (move_log) {
        *move_log << "    [move_apply]"
                  << " module_id=" << tm.id
                  << " center=(" << tm.x << ", " << tm.y << ")"
                  << " dx=" << best.dx << " dy=" << best.dy
                  << " rot90=" << best.rotate_90
                  << " weight=" << tm.move_weight << "\n";
    }

    return best;
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
                           std::ostream*           log)
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

        // 旋轉後超出 die 邊界時，clamp 中心點使其回到邊界內
        const double half_w = m.width  * 0.5;
        const double half_h = m.height * 0.5;
        m.x = std::clamp(m.x, half_w,             die.width  - half_w);
        m.y = std::clamp(m.y, half_h,             die.height - half_h);

        if (log) {
            *log << "    [shake_rot] module_id=" << m.id
                 << " new_size=(" << m.width << "x" << m.height << ")"
                 << " center=(" << m.x << ", " << m.y << ")\n";
        }
        ++rotated_count;
    }
    return rotated_count;
}

void run_legalize_heu(PlacementEngine& engine, const PartitionConfig& pcfg)
{
    (void)pcfg; // 保留給後續 heuristic 合法化使用

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

    // ---- 設定 ----
    LocalMoveConfig lcfg;
    lcfg.max_search_dist   = 30.0;
    lcfg.max_move_dist     = 30.0;
    lcfg.disp_weight       = 3.0;
    lcfg.overlap_weight    = 1.0;
    lcfg.moved_weight_mul  = 3;
    lcfg.max_module_weight = 1e6;

    auto& modules = engine.modules_mutable();
    const auto& dies = engine.dies();
    std::cout << "\n[legalize_heu] multi-pass local move\n";

    // 對指定排序跑一整層的 local move
    // init_weight    : sweep 開始前將該層所有 movable module 的 move_weight 重設為此值
    // moved_weight_mul : 每個 module 移動後 move_weight 乘上的倍率（覆寫 lcfg）
    auto sweep = [&](int tier, const std::string& label,
                     std::function<double(const Module&)> key_fn,
                     bool ascending,
                     double init_weight,
                     double moved_weight_mul)
    {
        // 重設該層 module 的初始 move_weight
        // is_fixed module 視為「已移動過的障礙物」：weight 乘上 moved_weight_mul（不受 init_weight 覆寫）
        for (Module& m : modules) {
            if (m.is_terminal || m.tier_id != tier) continue;
            if (m.is_fixed)
                m.move_weight = std::min(lcfg.max_module_weight,
                                         m.move_weight * moved_weight_mul);
            else
                m.move_weight = init_weight;
        }

        // 以當下位置建立排序（is_fixed module 不加入待處理清單）
        std::vector<std::pair<double, int>> key_id;
        for (const Module& m : modules) {
            if (m.is_terminal || m.tier_id != tier || m.is_fixed) continue;
            key_id.push_back({ key_fn(m), m.id });
        }
        if (ascending)
            std::sort(key_id.begin(), key_id.end());
        else
            std::sort(key_id.begin(), key_id.end(), std::greater<>());

        // 用此 sweep 指定的 moved_weight_mul
        LocalMoveConfig sweep_cfg = lcfg;
        sweep_cfg.moved_weight_mul = moved_weight_mul;

        std::cout << "  Tier " << tier << " [" << label << "]"
                  << " init_w=" << init_weight
                  << " mul=" << moved_weight_mul << "\n";
        for (const auto& [key, mid] : key_id)
            optimize_module_local_move(modules, dies, mid, sweep_cfg, &log_file);
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
                  << " bbox_center=(" << cx << ", " << cy << ")\n";

        static constexpr int    kMaxShakeIters  = 8;
        static constexpr double kShakeRadius    = 50.0;
        static constexpr double kShakeProb      = 0.5;

        for (int iter = 0; iter < kMaxShakeIters; ++iter) {
            // 每輪開頭先確認是否仍有重疊，若已無則提前結束
            if (!has_tier_overlaps(modules, t)) {
                std::cout << "  Tier " << t << " [iter=" << iter
                          << "] no overlaps, done\n";
                break;
            }

            std::cout << "  Tier " << t << " [iter=" << iter << "] running sweeps\n";

            // ---- Sweep passes（權重隨輪次遞增以加強排斥）----
            const double iw  = std::pow(3.0, iter);         // init_weight
            const double mw1 = 3.0  + iter * 4.0;           // moved_weight_mul（前幾輪）
            const double mw2 = 9.0  + iter * 6.0;
            const double mw3 = 20.0 + iter * 8.0;

            sweep(t, "near->far",
                  [cx, cy](const Module& m) {
                      const double dx = m.x - cx, dy = m.y - cy;
                      return dx * dx + dy * dy;
                  }, /*ascending=*/true,  iw,        mw1);

            sweep(t, "left->right",
                  [](const Module& m) { return m.lx(); }, true,  iw * 3,  mw1);
            sweep(t, "right->left",
                  [](const Module& m) { return m.rx(); }, false, iw * 3,  mw1);
            sweep(t, "bottom->top",
                  [](const Module& m) { return m.ly(); }, true,  iw * 9,  mw2);
            sweep(t, "top->bottom",
                  [](const Module& m) { return m.ry(); }, false, iw * 9,  mw2);
            sweep(t, "left->right",
                  [](const Module& m) { return m.lx(); }, true,  iw * 27, mw3);
            sweep(t, "right->left",
                  [](const Module& m) { return m.rx(); }, false, iw * 27, mw3);
            sweep(t, "bottom->top",
                  [](const Module& m) { return m.ly(); }, true,  iw * 81, mw3);
            sweep(t, "top->bottom",
                  [](const Module& m) { return m.ry(); }, false, iw * 81, mw3);

            // ---- 檢查本輪 sweep 後是否消除所有重疊 ----
            if (!has_tier_overlaps(modules, t)) {
                std::cout << "  Tier " << t << " [iter=" << iter
                          << "] overlaps cleared after sweep\n";
                break;
            }

            // ---- 還有重疊：shake 擾動（對重疊 module 附近做隨機旋轉）----
            const unsigned shake_seed = static_cast<unsigned>(iter * 997 + t * 31);
            std::cout << "  Tier " << t << " [shake rotations]\n";
            const int n_rot = shake_nearby_rotations(
                modules, dies, t, kShakeRadius, kShakeProb, shake_seed, &log_file);
            std::cout << "  Tier " << t << " [iter=" << iter
                      << "] shake rotated=" << n_rot << " modules\n";
        }
    }

    std::cout.rdbuf(orig_buf);  // 還原 stdout
    if (log_file)
        std::cout << "[legalize_heu] log written -> " << kLogPath << "\n";
}
