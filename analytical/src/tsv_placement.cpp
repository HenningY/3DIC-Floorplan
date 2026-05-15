// 3D IC Analytical Floorplanner - TSV Placement 實作
// 包含 build_tsvs、solve_tsvs、reflow_tsvs_after_legalize 及相關輔助函式
#include "floorplanner.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

// ============================================================
// build_tsvs: 依 cross-tier nets 建立 TSV 清單
//
// 規則：對每個 net，取 pin 的 tier（terminal 視為 tier 0），得 [min_tier, max_tier]。
// 若 max_tier - min_tier >= 1，則此 net 需 (max_tier - min_tier) 個 TSV，
// 分別位於 layer_index = min_tier, min_tier+1, ..., max_tier-1
// （layer k 連接 tier k 與 tier k+1）。橫跨 tier 0 與 tier 2 則在 layer 0、1 各一 TSV。
// 初始 TSV 位置設為 die 中心（與模組同一平面投影）
// ============================================================
void PlacementEngine::build_tsvs()
{
    tsvs_.clear();
    int tsv_id = 0;

    for (Net& net : nets_) {
        if (net.pins.size() < 2) continue;

        int min_t = std::numeric_limits<int>::max();
        int max_t = std::numeric_limits<int>::min();
        for (int pid : net.pins) {
            const Module& m = modules_[pid];
            int t = m.is_terminal ? 0 : m.tier_id;
            min_t = std::min(min_t, t);
            max_t = std::max(max_t, t);
        }

        if (min_t < 0) continue;   // 防呆
        if (max_t - min_t < 1) continue;  // 未跨層，不需 TSV

        net.is_cross_tier = true;
        net.min_tier = min_t;
        net.max_tier = max_t;

        // 每個 layer_index = min_t .. max_t-1 各一個 TSV
        for (int layer = min_t; layer < max_t; ++layer) {
            TSV tsv;
            tsv.id = tsv_id++;
            tsv.net_id = net.id;
            tsv.layer_index = layer;
            const Die& die = dies_[layer];
            tsv.x = die.width  * 0.5;
            tsv.y = die.height * 0.5;
            tsvs_.push_back(std::move(tsv));
        }
    }

    std::cout << "[TSV] Built " << tsvs_.size() << " TSVs from cross-tier nets "
              << "(num_dies=" << num_dies() << ").\n";
}

// ============================================================
// 輔助結構：2D 矩形框（用於 TSV BBox 計算）
// ============================================================
namespace {

struct BBox {
    double xmin =  1e18, xmax = -1e18;
    double ymin =  1e18, ymax = -1e18;
    bool valid() const { return xmin <= xmax && ymin <= ymax; }
    void expand(double x, double y) {
        xmin = std::min(xmin, x); xmax = std::max(xmax, x);
        ymin = std::min(ymin, y); ymax = std::max(ymax, y);
    }
    // 點到矩形的距離（點在框內時為 0）
    double dist(double x, double y) const {
        double dx = std::max(xmin - x, 0.0) + std::max(x - xmax, 0.0);
        double dy = std::max(ymin - y, 0.0) + std::max(y - ymax, 0.0);
        return dx + dy;
    }
};

// 由兩個 axis-aligned bbox 決定 TSV (tx, ty)：
// - 若二維有重疊：取重疊矩形中心
// - 否則：兩框共四條鉛直邊 x、四條水平邊 y，排序後取「中間兩條 x」「中間兩條 y」
//   所圍成矩形之中心（避免兩框中心相距很遠時仍取中點而落在空白處）
static void tsv_position_from_two_bboxes(const BBox& a, const BBox& b, double& tx, double& ty)
{
    const double eps = 1e-9;
    const double ix0 = std::max(a.xmin, b.xmin);
    const double ix1 = std::min(a.xmax, b.xmax);
    const double iy0 = std::max(a.ymin, b.ymin);
    const double iy1 = std::min(a.ymax, b.ymax);
    if (ix0 <= ix1 + eps && iy0 <= iy1 + eps) {
        tx = 0.5 * (ix0 + ix1);
        ty = 0.5 * (iy0 + iy1);
        return;
    }
    double xs[4] = { a.xmin, a.xmax, b.xmin, b.xmax };
    double ys[4] = { a.ymin, a.ymax, b.ymin, b.ymax };
    std::sort(xs, xs + 4);
    std::sort(ys, ys + 4);
    tx = 0.5 * (xs[1] + xs[2]);
    ty = 0.5 * (ys[1] + ys[2]);
}

// 與 tsv_position_from_two_bboxes 相同幾何，回傳可供擺放的 axis-aligned 矩形 [rx0,rx1]×[ry0,ry1]
static bool tsv_placement_rect_from_two_bboxes(const BBox& a, const BBox& b,
                                                 double& rx0, double& ry0, double& rx1, double& ry1)
{
    const double eps = 1e-9;
    const double ix0 = std::max(a.xmin, b.xmin);
    const double ix1 = std::min(a.xmax, b.xmax);
    const double iy0 = std::max(a.ymin, b.ymin);
    const double iy1 = std::min(a.ymax, b.ymax);
    if (ix0 <= ix1 + eps && iy0 <= iy1 + eps) {
        rx0 = ix0;
        rx1 = ix1;
        ry0 = iy0;
        ry1 = iy1;
    } else {
        double xs[4] = { a.xmin, a.xmax, b.xmin, b.xmax };
        double ys[4] = { a.ymin, a.ymax, b.ymin, b.ymax };
        std::sort(xs, xs + 4);
        std::sort(ys, ys + 4);
        rx0 = xs[1];
        rx1 = xs[2];
        ry0 = ys[1];
        ry1 = ys[2];
    }
    return rx0 <= rx1 + eps && ry0 <= ry1 + eps;
}

// 依 die weight 調整 TSV 初值（solve_tsvs 用）：
// - B_lower 與 B_upper 二維有交集：與 tsv_position_from_two_bboxes 相同（交集中心）
// - 無交集且 w_lo≈w_hi：仍用 tsv_position_from_two_bboxes（中間兩邊所圍矩形中心）
// - 無交集且 w_lo≠w_hi：取「中間矩形 R」與權重較大那一側 bbox 的交集之中心
static void tsv_position_from_two_bboxes_weighted(
    const BBox& b_lo, const BBox& b_hi, double w_lo, double w_hi,
    double& tx, double& ty)
{
    const double eps = 1e-9;
    const double ix0 = std::max(b_lo.xmin, b_hi.xmin);
    const double ix1 = std::min(b_lo.xmax, b_hi.xmax);
    const double iy0 = std::max(b_lo.ymin, b_hi.ymin);
    const double iy1 = std::min(b_lo.ymax, b_hi.ymax);
    if (ix0 <= ix1 + eps && iy0 <= iy1 + eps) {
        tsv_position_from_two_bboxes(b_lo, b_hi, tx, ty);
        return;
    }
    if (std::fabs(w_lo - w_hi) <= eps * std::max(1.0, std::max(w_lo, w_hi))) {
        tsv_position_from_two_bboxes(b_lo, b_hi, tx, ty);
        return;
    }
    double rx0, ry0, rx1, ry1;
    if (!tsv_placement_rect_from_two_bboxes(b_lo, b_hi, rx0, ry0, rx1, ry1)) {
        tsv_position_from_two_bboxes(b_lo, b_hi, tx, ty);
        return;
    }
    const BBox& fav = (w_lo > w_hi) ? b_lo : b_hi;
    const double ox0 = std::max(rx0, fav.xmin);
    const double ox1 = std::min(rx1, fav.xmax);
    const double oy0 = std::max(ry0, fav.ymin);
    const double oy1 = std::min(ry1, fav.ymax);
    if (ox0 <= ox1 + eps && oy0 <= oy1 + eps) {
        tx = 0.5 * (ox0 + ox1);
        ty = 0.5 * (oy0 + oy1);
    } else {
        tsv_position_from_two_bboxes(b_lo, b_hi, tx, ty);
    }
}

// R 與 bbox 交集寫入 [ox0,ox1]×[oy0,oy1]（夾在 die 內）；無交集則回傳 false
static bool rect_intersect_bbox_clamped(double rx0,
                                         double ry0,
                                         double rx1,
                                         double ry1,
                                         const BBox& b,
                                         double die_w,
                                         double die_h,
                                         double& ox0,
                                         double& oy0,
                                         double& ox1,
                                         double& oy1)
{
    const double eps = 1e-9;
    ox0 = std::max(rx0, b.xmin);
    ox1 = std::min(rx1, b.xmax);
    oy0 = std::max(ry0, b.ymin);
    oy1 = std::min(ry1, b.ymax);
    ox0 = std::max(0.0, std::min(die_w, ox0));
    ox1 = std::max(0.0, std::min(die_w, ox1));
    oy0 = std::max(0.0, std::min(die_h, oy0));
    oy1 = std::max(0.0, std::min(die_h, oy1));
    if (ox0 > ox1) std::swap(ox0, ox1);
    if (oy0 > oy1) std::swap(oy0, oy1);
    return ox0 <= ox1 + eps && oy0 <= oy1 + eps;
}

// 與 TSV 相鄰之 tier_below() 或 tier_above() 其中至少一層無此 net 的 pin 時啟用（兩層都有則不走此路）。
// n_tier 等邏輯不變；回傳目標矩形為 R∩B_lower 或 R∩B_upper（R：兩側 aggregate 皆有效時為兩框之中間矩形，
// 否則為單側 bbox）。交集失敗且對側 bbox 無效時退回 R 全範圍。
static bool tsv_gap_one_side_target_rect(const Net& net,
                                         const std::vector<Module>& modules,
                                         int num_tiers,
                                         int layer_L,
                                         const std::vector<double>& tier_w,
                                         const BBox& b_lo,
                                         const BBox& b_hi,
                                         bool lo_ok,
                                         bool hi_ok,
                                         double die_w,
                                         double die_h,
                                         double& rx0,
                                         double& ry0,
                                         double& rx1,
                                         double& ry1)
{
    const int    tier_b = layer_L;
    const int    tier_a = layer_L + 1;
    bool         pin_on_tier_below = false;
    bool         pin_on_tier_above = false;
    for (int pid : net.pins) {
        const Module& m = modules[pid];
        int           mt = m.is_terminal ? 0 : m.tier_id;
        if (mt == tier_b) pin_on_tier_below = true;
        if (mt == tier_a) pin_on_tier_above = true;
    }
    if (pin_on_tier_below && pin_on_tier_above) return false;

    std::vector<char> has(static_cast<size_t>(num_tiers), 0);
    for (int pid : net.pins) {
        const Module& m = modules[pid];
        int           mt = m.is_terminal ? 0 : m.tier_id;
        if (mt >= 0 && mt < num_tiers) has[static_cast<size_t>(mt)] = 1;
    }

    int a = -1;
    for (int t = 0; t <= layer_L; ++t) {
        if (has[static_cast<size_t>(t)]) a = t;
    }
    int b = -1;
    for (int t = layer_L + 1; t < num_tiers; ++t) {
        if (has[static_cast<size_t>(t)]) {
            b = t;
            break;
        }
    }

    int span_lo = 0, span_hi = num_tiers - 1;
    if (a >= 0 && b >= 0) {
        span_lo = std::min(a, b);
        span_hi = std::max(a, b);
    } else if (a >= 0) {
        span_lo = a;
        span_hi = layer_L;
    } else if (b >= 0) {
        span_lo = layer_L + 1;
        span_hi = b;
    } else {
        return false;
    }

    int    n_tier = -1;
    double best_w = std::numeric_limits<double>::infinity();
    for (int t = span_lo; t <= span_hi; ++t) {
        if (!has[static_cast<size_t>(t)]) continue;
        const double w = tier_w[static_cast<size_t>(t)];
        if (w < best_w - 1e-15 || (std::fabs(w - best_w) <= 1e-15 && (n_tier < 0 || t < n_tier))) {
            best_w = w;
            n_tier = t;
        }
    }
    if (n_tier < 0) return false;

    double Rx0 = 0.0, Ry0 = 0.0, Rx1 = die_w, Ry1 = die_h;
    if (lo_ok && hi_ok) {
        if (!tsv_placement_rect_from_two_bboxes(b_lo, b_hi, Rx0, Ry0, Rx1, Ry1)) return false;
    } else if (lo_ok) {
        Rx0 = b_lo.xmin;
        Rx1 = b_lo.xmax;
        Ry0 = b_lo.ymin;
        Ry1 = b_lo.ymax;
    } else if (hi_ok) {
        Rx0 = b_hi.xmin;
        Rx1 = b_hi.xmax;
        Ry0 = b_hi.ymin;
        Ry1 = b_hi.ymax;
    } else {
        return false;
    }

    Rx0 = std::max(0.0, std::min(die_w, Rx0));
    Rx1 = std::max(0.0, std::min(die_w, Rx1));
    Ry0 = std::max(0.0, std::min(die_h, Ry0));
    Ry1 = std::max(0.0, std::min(die_h, Ry1));
    if (Rx0 > Rx1) std::swap(Rx0, Rx1);
    if (Ry0 > Ry1) std::swap(Ry0, Ry1);

    const bool pick_lo = (layer_L < n_tier);
    if (pick_lo) {
        if (lo_ok && rect_intersect_bbox_clamped(Rx0, Ry0, Rx1, Ry1, b_lo, die_w, die_h, rx0, ry0, rx1,
                                                 ry1))
            return true;
        rx0 = Rx0;
        ry0 = Ry0;
        rx1 = Rx1;
        ry1 = Ry1;
        return true;
    }
    if (hi_ok && rect_intersect_bbox_clamped(Rx0, Ry0, Rx1, Ry1, b_hi, die_w, die_h, rx0, ry0, rx1, ry1))
        return true;
    rx0 = Rx0;
    ry0 = Ry0;
    rx1 = Rx1;
    ry1 = Ry1;
    return true;
}

// 相鄰 tier 缺 pin 時：見 tsv_gap_one_side_target_rect；此處取目標矩形中心。
static bool solve_tsv_gap_one_side_pin_span(const Net& net,
                                            const std::vector<Module>& modules,
                                            int num_tiers,
                                            int layer_L,
                                            const std::vector<double>& tier_w,
                                            const BBox& b_lo,
                                            const BBox& b_hi,
                                            bool lo_ok,
                                            bool hi_ok,
                                            double die_w,
                                            double die_h,
                                            double& tx,
                                            double& ty)
{
    double rx0 = 0.0, ry0 = 0.0, rx1 = 0.0, ry1 = 0.0;
    if (!tsv_gap_one_side_target_rect(net, modules, num_tiers, layer_L, tier_w, b_lo, b_hi, lo_ok,
                                       hi_ok, die_w, die_h, rx0, ry0, rx1, ry1))
        return false;
    tx = 0.5 * (rx0 + rx1);
    ty = 0.5 * (ry0 + ry1);
    tx = std::max(0.0, std::min(die_w, tx));
    ty = std::max(0.0, std::min(die_h, ty));
    return true;
}

struct PlacedRect {
    double lx, ly, rx, ry;
};

// first-fit 掃描起點：對應矩形可行區的「角落」（y 向上：ymin 為下、ymax 為上）
enum class TsvFirstFitCorner { BottomLeft, BottomRight, TopLeft, TopRight };

// 比較 b_lo 與 b_hi 兩 bbox **中心點**的相對位置（象限）→ first-fit 從對應角開始掃描（reflow TSV）
// dx = lo_x - hi_x, dy = lo_y - hi_y（y 小為下）：西南象限→左下、東南→右下、西北→左上、東北→右上
static TsvFirstFitCorner pick_corner_from_b_lo_b_hi_center_rel(const BBox& b_lo, const BBox& b_hi)
{
    const double clx = 0.5 * (b_lo.xmin + b_lo.xmax);
    const double cly = 0.5 * (b_lo.ymin + b_lo.ymax);
    const double chx = 0.5 * (b_hi.xmin + b_hi.xmax);
    const double chy = 0.5 * (b_hi.ymin + b_hi.ymax);
    const double dx  = clx - chx;
    const double dy  = cly - chy;
    const double span = std::max(b_hi.xmax - b_hi.xmin, b_hi.ymax - b_hi.ymin);
    const double eps  = std::max(1e-12, 1e-9 * std::max(1.0, span));

    if (std::fabs(dx) <= eps) {
        if (dy < -eps) return TsvFirstFitCorner::BottomLeft;  // lo 在 hi 正下方
        if (dy > eps)  return TsvFirstFitCorner::TopLeft;      // lo 在 hi 正上方
        return TsvFirstFitCorner::BottomLeft;                  // 幾乎重合
    }
    if (std::fabs(dy) <= eps) {
        if (dx < -eps) return TsvFirstFitCorner::BottomLeft;   // lo 在 hi 正左方
        if (dx > eps)  return TsvFirstFitCorner::BottomRight;  // lo 在 hi 正右方
        return TsvFirstFitCorner::BottomLeft;
    }
    if (dx < -eps && dy < -eps) return TsvFirstFitCorner::BottomLeft;   // 西南
    if (dx > eps && dy < -eps)  return TsvFirstFitCorner::BottomRight;   // 東南
    if (dx < -eps && dy > eps)  return TsvFirstFitCorner::TopLeft;      // 西北
    return TsvFirstFitCorner::TopRight;                                 // 東北
}

// 在矩形區域內（含邊界）對固定大小 hw,hh 做 first-fit；placed 為障礙 AABB。
// start_corner 決定掃描順序（從哪個角開始往內填）。
static bool first_fit_tsv_in_region(const std::vector<PlacedRect>& placed,
                                      double rx0, double ry0, double rx1, double ry1,
                                      double hw, double hh,
                                      double& out_cx, double& out_cy,
                                      TsvFirstFitCorner start_corner = TsvFirstFitCorner::BottomLeft)
{
    const double eps = 1e-9;
    double cx_lo = rx0 + hw;
    double cx_hi = rx1 - hw;
    double cy_lo = ry0 + hh;
    double cy_hi = ry1 - hh;
    if (cx_lo > cx_hi + eps || cy_lo > cy_hi + eps) {
        out_cx = 0.5 * (rx0 + rx1);
        out_cy = 0.5 * (ry0 + ry1);
        return false;
    }

    auto first_free_x_at_y_from_left = [&](double cy) -> double {
        double cx = cx_lo;
        bool   moved = true;
        while (moved) {
            moved = false;
            for (const PlacedRect& r : placed) {
                if (cy - hh >= r.ry - eps || cy + hh <= r.ly + eps) continue;
                if (cx - hw < r.rx - eps && cx + hw > r.lx + eps) {
                    cx = r.rx + hw;
                    moved = true;
                }
            }
        }
        return (cx + hw > cx_hi + eps) ? -1.0 : cx;
    };

    auto first_free_x_at_y_from_right = [&](double cy) -> double {
        double cx = cx_hi;
        bool   moved = true;
        while (moved) {
            moved = false;
            for (const PlacedRect& r : placed) {
                if (cy - hh >= r.ry - eps || cy + hh <= r.ly + eps) continue;
                if (cx - hw < r.rx - eps && cx + hw > r.lx + eps) {
                    cx = r.lx - hw;
                    moved = true;
                }
            }
        }
        return (cx - hw < cx_lo - eps) ? -1.0 : cx;
    };

    std::vector<double> ys;
    auto push_y = [&](double y) {
        if (y >= cy_lo - eps && y <= cy_hi + eps) ys.push_back(y);
    };
    push_y(cy_lo);
    push_y(cy_hi);
    for (const PlacedRect& r : placed) {
        push_y(r.ry + hh);
        push_y(r.ly - hh);
    }
    if (ys.empty()) {
        out_cx = 0.5 * (rx0 + rx1);
        out_cy = 0.5 * (ry0 + ry1);
        return false;
    }
    std::sort(ys.begin(), ys.end());
    const double tol = std::max(1e-6, hh * 1e-3);
    ys.erase(std::unique(ys.begin(), ys.end(),
                          [&](double a, double b){ return std::fabs(a - b) < tol; }),
             ys.end());

    const bool y_ascending =
        (start_corner == TsvFirstFitCorner::BottomLeft ||
         start_corner == TsvFirstFitCorner::BottomRight);
    if (!y_ascending)
        std::reverse(ys.begin(), ys.end());

    const bool from_left =
        (start_corner == TsvFirstFitCorner::BottomLeft ||
         start_corner == TsvFirstFitCorner::TopLeft);

    for (double cy : ys) {
        const double cx = from_left ? first_free_x_at_y_from_left(cy)
                                    : first_free_x_at_y_from_right(cy);
        if (cx >= 0.0) {
            out_cx = cx;
            out_cy = cy;
            return true;
        }
    }
    out_cx = 0.5 * (rx0 + rx1);
    out_cy = 0.5 * (ry0 + ry1);
    return false;
}

// L1 距離：點 (px,py) 到閉合軸對齊矩形 [rx0,rx1]×[ry0,ry1]
static double l1_dist_point_to_rect(double px, double py,
                                      double rx0, double ry0, double rx1, double ry1)
{
    double dx = 0.0;
    if (px < rx0) dx = rx0 - px;
    else if (px > rx1) dx = px - rx1;
    double dy = 0.0;
    if (py < ry0) dy = ry0 - py;
    else if (py > ry1) dy = py - ry1;
    return dx + dy;
}

// 列 cy 上，TSV 中心 x 的可行區間（與 placed 不重疊），限制在 [cx_lo, cx_hi]
static std::vector<std::pair<double, double>> free_x_intervals_for_tsv_row(
    double cy, const std::vector<PlacedRect>& placed, double hw, double hh,
    double cx_lo, double cx_hi)
{
    const double eps = 1e-9;
    std::vector<std::pair<double, double>> blocked;
    for (const PlacedRect& r : placed) {
        if (cy - hh >= r.ry - eps || cy + hh <= r.ly + eps) continue;
        double bl = r.lx - hw;
        double br = r.rx + hw;
        bl = std::max(bl, cx_lo);
        br = std::min(br, cx_hi);
        if (bl <= br + eps)
            blocked.push_back({ bl, br });
    }
    std::sort(blocked.begin(), blocked.end());
    std::vector<std::pair<double, double>> merged;
    for (const auto& seg : blocked) {
        if (merged.empty() || seg.first > merged.back().second + eps)
            merged.push_back(seg);
        else
            merged.back().second = std::max(merged.back().second, seg.second);
    }
    std::vector<std::pair<double, double>> free_iv;
    double cur = cx_lo;
    for (const auto& m : merged) {
        if (cur < m.first - eps)
            free_iv.push_back({ cur, m.first });
        cur = std::max(cur, m.second);
        if (cur > cx_hi + eps) break;
    }
    if (cur < cx_hi - eps)
        free_iv.push_back({ cur, cx_hi });
    return free_iv;
}

// 在整張 die 上找可摆 TSV 的位置，使 L1 距離到目標矩形 R 最小（區域內無空位時用）
static bool nearest_free_tsv_to_rect(const std::vector<PlacedRect>& placed,
                                       double tx0, double ty0, double tx1, double ty1,
                                       double die_w, double die_h,
                                       double hw, double hh,
                                       double& out_cx, double& out_cy)
{
    const double eps = 1e-9;
    const double cx_lo = hw;
    const double cx_hi = die_w - hw;
    const double cy_lo = hh;
    const double cy_hi = die_h - hh;
    if (cx_lo > cx_hi + eps || cy_lo > cy_hi + eps)
        return false;

    std::vector<double> ys;
    auto push_y = [&](double y) {
        if (y >= cy_lo - eps && y <= cy_hi + eps) ys.push_back(y);
    };
    push_y(cy_lo);
    push_y(cy_hi);
    for (const PlacedRect& r : placed) {
        push_y(r.ry + hh);
        push_y(r.ly - hh);
    }
    std::sort(ys.begin(), ys.end());
    const double tol = std::max(1e-6, hh * 1e-3);
    ys.erase(std::unique(ys.begin(), ys.end(),
                          [&](double a, double b){ return std::fabs(a - b) < tol; }),
             ys.end());

    double best_d = std::numeric_limits<double>::infinity();
    bool   found  = false;

    auto consider = [&](double cx, double cy) {
        const double d = l1_dist_point_to_rect(cx, cy, tx0, ty0, tx1, ty1);
        if (!found || d < best_d - 1e-12) {
            best_d = d;
            out_cx = cx;
            out_cy = cy;
            found  = true;
        } else if (std::fabs(d - best_d) < 1e-9) {
            if (cy < out_cy - 1e-9 || (std::fabs(cy - out_cy) < 1e-9 && cx < out_cx - 1e-9)) {
                out_cx = cx;
                out_cy = cy;
            }
        }
    };

    for (double cy : ys) {
        auto intervals = free_x_intervals_for_tsv_row(cy, placed, hw, hh, cx_lo, cx_hi);
        for (const auto& iv : intervals) {
            const double a = iv.first;
            const double b = iv.second;
            if (a > b + eps) continue;
            const double cand[] = {
                a,
                b,
                std::clamp(0.5 * (tx0 + tx1), a, b),
                std::clamp(tx0, a, b),
                std::clamp(tx1, a, b)
            };
            for (double cx : cand) {
                if (cx < a - eps || cx > b + eps) continue;
                consider(cx, cy);
            }
        }
    }

    return found;
}

// weighted asym 時 R∩較重側 bbox 可能只是一條線（或近似線）：先在該線附近狹帶 first-fit，
// 再沿線擴成整 die 向之狹帶；仍無則以狹帶為目標做 nearest（避免直接整個 R 內 first-fit）。
static bool reflow_weighted_pref_line_corridor_fit(const std::vector<PlacedRect>& placed,
                                                    double pref_rx0,
                                                    double pref_ry0,
                                                    double pref_rx1,
                                                    double pref_ry1,
                                                    double die_w,
                                                    double die_h,
                                                    double hw,
                                                    double hh,
                                                    double& out_cx,
                                                    double& out_cy)
{
    const double le = std::max(1e-9, 1e-7 * std::max(die_w, die_h));
    const bool   vline = (pref_rx1 - pref_rx0) <= le;
    const bool   hline = (pref_ry1 - pref_ry0) <= le;

    auto clamp_rect = [&](double& x0, double& x1, double& y0, double& y1) {
        x0 = std::max(0.0, std::min(die_w, x0));
        x1 = std::max(0.0, std::min(die_w, x1));
        y0 = std::max(0.0, std::min(die_h, y0));
        y1 = std::max(0.0, std::min(die_h, y1));
        if (x0 > x1) std::swap(x0, x1);
        if (y0 > y1) std::swap(y0, y1);
    };

    auto try_ff = [&](double x0, double x1, double y0, double y1) -> bool {
        clamp_rect(x0, x1, y0, y1);
        return first_fit_tsv_in_region(placed, x0, y0, x1, y1, hw, hh, out_cx, out_cy);
    };

    constexpr double k_strip = 2.5;

    if (!vline && !hline) {
        double px0 = pref_rx0, px1 = pref_rx1, py0 = pref_ry0, py1 = pref_ry1;
        clamp_rect(px0, px1, py0, py1);
        return first_fit_tsv_in_region(placed, px0, py0, px1, py1, hw, hh, out_cx, out_cy);
    }

    if (vline && !hline) {
        const double xm = 0.5 * (pref_rx0 + pref_rx1);
        const double x_lo = xm - k_strip * hw;
        const double x_hi = xm + k_strip * hw;
        if (try_ff(x_lo, x_hi, pref_ry0, pref_ry1)) return true;
        if (try_ff(x_lo, x_hi, hh, die_h - hh)) return true;
        double nx0 = xm - (k_strip + 1.0) * hw;
        double nx1 = xm + (k_strip + 1.0) * hw;
        double ty0 = hh, ty1 = die_h - hh;
        clamp_rect(nx0, nx1, ty0, ty1);
        if (ty0 > ty1 + 1e-12 || nx0 > nx1 + 1e-12) return false;
        return nearest_free_tsv_to_rect(placed, nx0, ty0, nx1, ty1, die_w, die_h, hw, hh, out_cx,
                                        out_cy);
    }

    if (hline && !vline) {
        const double ym = 0.5 * (pref_ry0 + pref_ry1);
        const double y_lo = ym - k_strip * hh;
        const double y_hi = ym + k_strip * hh;
        if (try_ff(pref_rx0, pref_rx1, y_lo, y_hi)) return true;
        if (try_ff(hw, die_w - hw, y_lo, y_hi)) return true;
        double my0 = ym - (k_strip + 1.0) * hh;
        double my1 = ym + (k_strip + 1.0) * hh;
        double sx0 = hw, sx1 = die_w - hw;
        clamp_rect(sx0, sx1, my0, my1);
        if (sx0 > sx1 + 1e-12 || my0 > my1 + 1e-12) return false;
        return nearest_free_tsv_to_rect(placed, sx0, my0, sx1, my1, die_w, die_h, hw, hh, out_cx,
                                        out_cy);
    }

    const double xm = 0.5 * (pref_rx0 + pref_rx1);
    const double ym = 0.5 * (pref_ry0 + pref_ry1);
    if (try_ff(xm - k_strip * hw, xm + k_strip * hw, ym - k_strip * hh, ym + k_strip * hh))
        return true;
    double p0 = xm - k_strip * hw, p1 = xm + k_strip * hw;
    double q0 = ym - k_strip * hh, q1 = ym + k_strip * hh;
    clamp_rect(p0, p1, q0, q1);
    if (p0 > p1 + 1e-12 || q0 > q1 + 1e-12) return false;
    return nearest_free_tsv_to_rect(placed, p0, q0, p1, q1, die_w, die_h, hw, hh, out_cx, out_cy);
}

// Same net、較低 interface（layer_index 較小）已擺好的 TSV 中心點一併納入 B_lower，
// 使上層 interface 的 TSV 幾何目標會「拉向」已固定的下層 TSV（例如 tier0–1 後擺 tier1–2）。
static void expand_b_lo_with_prior_same_net_tsvs(const std::vector<TSV>& all_tsvs,
                                                 const TSV&              cur,
                                                 BBox&                   b_lo)
{
    for (const TSV& o : all_tsvs) {
        if (o.net_id != cur.net_id) continue;
        if (o.layer_index >= cur.layer_index) continue;
        b_lo.expand(o.x, o.y);
    }
}

} // anonymous namespace


// ============================================================
// compute_tsv_cost: 計算所有 TSV 的 wirelength 代價
//
// 對每個 TSV，取所屬 net 上下兩側的 BBox：
//   B_lower = bbox of pins on tiers ≤ layer_index，再加上同 net 已擺在較低 interface 的 TSV 中心
//   B_upper = bbox of pins on tiers ≥ layer_index+1
// cost = w_lo * dist(tsv, B_lower) + w_hi * dist(tsv, B_upper)
// ============================================================
double PlacementEngine::compute_tsv_cost() const
{
    double total = 0.0;
    for (const TSV& tsv : tsvs_) {
        const Net& net = nets_[tsv.net_id];
        BBox b_lo, b_hi;
        for (int pid : net.pins) {
            const Module& m = modules_[pid];
            int mt = m.is_terminal ? 0 : m.tier_id;
            if (mt <= tsv.tier_below()) b_lo.expand(m.x, m.y);
            if (mt >= tsv.tier_above()) b_hi.expand(m.x, m.y);
        }
        expand_b_lo_with_prior_same_net_tsvs(tsvs_, tsv, b_lo);
        if (b_lo.valid())
            total += tsv_placement_tier_weight(tsv.tier_below()) * b_lo.dist(tsv.x, tsv.y);
        if (b_hi.valid())
            total += tsv_placement_tier_weight(tsv.tier_above()) * b_hi.dist(tsv.x, tsv.y);
    }
    return total;
}

// ============================================================
// tsv_placement_tier_weight: TSV cost 用的 tier 乘數
//   tsv_placement_cfg_.tsv_die_weights 長度 == num_dies 時覆寫；否則 .block tier_net_weights_
// ============================================================
double PlacementEngine::tsv_placement_tier_weight(int tier) const
{
    const int n = static_cast<int>(dies_.size());
    if (tier < 0 || tier >= n) return 1.0;
    const auto& ov = tsv_placement_cfg_.tsv_die_weights;
    if (static_cast<int>(ov.size()) == n)
        return ov[static_cast<size_t>(tier)];
    if (static_cast<int>(tier_net_weights_.size()) == n)
        return tier_net_weights_[static_cast<size_t>(tier)];
    return 1.0;
}

// ============================================================
// solve_tsvs: 依上下層 BBox 幾何決定 TSV 座標（無迭代優化）
//
// 對每個 TSV：
//   B_lower = bbox of pins on tiers ≤ layer_index，再加上同 net 已擺在較低 interface 的 TSV 中心
//   B_upper = bbox of pins on tiers ≥ layer_index+1
//
// 兩側皆有效時（見 tsv_position_from_two_bboxes_weighted）：
//   - 若 B_lower 與 B_upper 二維重疊 → 重疊矩形中心
//   - 若無重疊且兩側 die weight 不同 →「中間矩形 R」與權重較大側 bbox 的交集中心
//   - 若無重疊且兩側 weight 視為相同 → 維持原版中間矩形中心
// 相鄰 tier_below() 或 tier_above() 至少一層無此 net 的 pin：見 solve_tsv_gap_one_side_pin_span
//   （最近下／上 pin tier 所夾區間內找最小 weight tier n；layer_index < n 則 R∩B_lower 中心，否則
//   R∩B_upper 中心；兩側 aggregate 皆有效時 R 為兩框之中間矩形）。其餘 fallback 為單側 bbox 中心。
// 最後夾取到對應 die 邊界。
// ============================================================
void PlacementEngine::solve_tsvs(const TsvPlacementConfig& tcfg)
{
    tsv_placement_cfg_ = tcfg;

    const int N = static_cast<int>(tsvs_.size());
    if (N == 0) {
        std::cout << "[SolveTSV] No TSVs to place.\n";
        return;
    }

    std::cout << "[SolveTSV] Direct center placement for " << N << " TSVs.\n";

    const int nt = static_cast<int>(dies_.size());
    std::vector<double> tier_w(static_cast<size_t>(nt));
    for (int i = 0; i < nt; ++i)
        tier_w[static_cast<size_t>(i)] = tsv_placement_tier_weight(i);

    for (TSV& tsv : tsvs_) {
        const Net& net = nets_[tsv.net_id];
        const Die& die = dies_[tsv.layer_index];
        BBox         b_lo, b_hi;
        for (int pid : net.pins) {
            const Module& m = modules_[pid];
            int mt = m.is_terminal ? 0 : m.tier_id;
            if (mt <= tsv.tier_below()) b_lo.expand(m.x, m.y);
            if (mt >= tsv.tier_above()) b_hi.expand(m.x, m.y);
        }
        expand_b_lo_with_prior_same_net_tsvs(tsvs_, tsv, b_lo);

        double       tx = tsv.x, ty = tsv.y;
        const bool   lo_ok = b_lo.valid();
        const bool   hi_ok = b_hi.valid();

        if (!solve_tsv_gap_one_side_pin_span(net, modules_, nt, tsv.layer_index, tier_w, b_lo, b_hi,
                                             lo_ok, hi_ok, die.width, die.height, tx, ty)) {
            if (lo_ok && hi_ok) {
                const double w_lo = tsv_placement_tier_weight(tsv.tier_below());
                const double w_hi = tsv_placement_tier_weight(tsv.tier_above());
                tsv_position_from_two_bboxes_weighted(b_lo, b_hi, w_lo, w_hi, tx, ty);
            } else if (lo_ok) {
                tx = 0.5 * (b_lo.xmin + b_lo.xmax);
                ty = 0.5 * (b_lo.ymin + b_lo.ymax);
            } else if (hi_ok) {
                tx = 0.5 * (b_hi.xmin + b_hi.xmax);
                ty = 0.5 * (b_hi.ymin + b_hi.ymax);
            }
        }

        tsv.x = std::max(0.0, std::min(die.width, tx));
        tsv.y = std::max(0.0, std::min(die.height, ty));
    }

    std::cout << "[SolveTSV] Done. Final TSV cost = "
              << std::fixed << std::setprecision(2) << compute_tsv_cost() << "\n";
}

// ============================================================
// reflow_tsvs_after_legalize:
//   忽略既有 TSV 位置，依「所屬 net 全體 pin 之 bbox 周長」由小到大排序，
//   各 TSV 在 B_lower/B_upper 決定之目標區域（與 solve_tsvs 相同之兩框幾何）內 first-fit；
//   B_lower 另含同 net、較低 layer_index 且本輪已更新過的 TSV 中心（使上層 interface 與下層 TSV 對齊）。
//   若相鄰 tier_below()／tier_above() 至少一層無 net pin：與 solve_tsvs 相同，在 R∩B_lower 或 R∩B_upper
//   之矩形內 first-fit；
//   若兩層 bbox 無交集且 die weight 不同：先在 R∩較重側 bbox 上／沿該線附近狹帶找空位（交集為線時
//   不在整個 R 內 first-fit）。若仍無空位：令 ratio = w_lo / w_hi；ratio>2 或 ratio<0.5 時以較重側 bbox
//   為 nearest 目標，
//   否則（0.5≤ratio≤2）以較輕側 bbox 為 nearest 目標；其餘情況仍以 R 為 nearest 目標。
//   障礙物為同層 module + 本輪已放置之 TSV。
//   當 B_lower、B_upper 皆有效時：first-fit 掃描起點由「b_lo 中心相對 b_hi 中心」的象限決定
//   （西南→左下、東南→右下、西北→左上、東北→右上；軸上則依正負方向對應邊）。
// ============================================================
void PlacementEngine::reflow_tsvs_after_legalize(double tsv_w, double tsv_h)
{
    const int Nt = static_cast<int>(tsvs_.size());
    if (Nt == 0) {
        std::cout << "[ReflowTSV] No TSVs.\n";
        return;
    }

    const double hw = tsv_w * 0.5;
    const double hh = tsv_h * 0.5;

    // 每條 net：全體 pin 之 bbox 周長 2*(dx+dy)（供排序）
    std::vector<double> net_perim(nets_.size(), 0.0);
    for (size_t ni = 0; ni < nets_.size(); ++ni) {
        const Net& net = nets_[ni];
        double x_min = 1e18, x_max = -1e18, y_min = 1e18, y_max = -1e18;
        for (int pid : net.pins) {
            const Module& m = modules_[pid];
            x_min = std::min(x_min, m.x);
            x_max = std::max(x_max, m.x);
            y_min = std::min(y_min, m.y);
            y_max = std::max(y_max, m.y);
        }
        if (x_min <= x_max && y_min <= y_max) {
            const double dx = x_max - x_min;
            const double dy = y_max - y_min;
            net_perim[ni] = 2.0 * (dx + dy);
        }
    }

    std::vector<int> order(Nt);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int ia, int ib) {
        const double pa = net_perim[tsvs_[ia].net_id];
        const double pb = net_perim[tsvs_[ib].net_id];
        if (pa != pb) return pa < pb;
        return tsvs_[ia].id < tsvs_[ib].id;
    });

    const int nlayer = static_cast<int>(dies_.size());
    std::vector<std::vector<PlacedRect>> tsv_placed_by_layer(nlayer);
    std::vector<double> tier_w(static_cast<size_t>(nlayer));
    for (int i = 0; i < nlayer; ++i)
        tier_w[static_cast<size_t>(i)] = tsv_placement_tier_weight(i);

    std::cout << "[ReflowTSV] Post-legalize reflow " << Nt
              << " TSVs (net bbox perimeter order; in-region first-fit, else nearest off-region).\n";

    for (int ord : order) {
        TSV&         tsv = tsvs_[ord];
        const Net&   net = nets_[tsv.net_id];
        const int    layer = tsv.layer_index;
        const Die&   die = dies_[layer];

        BBox b_lo, b_hi;
        for (int pid : net.pins) {
            const Module& m = modules_[pid];
            int mt = m.is_terminal ? 0 : m.tier_id;
            if (mt <= tsv.tier_below()) b_lo.expand(m.x, m.y);
            if (mt >= tsv.tier_above()) b_hi.expand(m.x, m.y);
        }
        expand_b_lo_with_prior_same_net_tsvs(tsvs_, tsv, b_lo);

        const bool lo_ok = b_lo.valid();
        const bool hi_ok = b_hi.valid();

        double rx0 = 0.0, ry0 = 0.0, rx1 = die.width, ry1 = die.height;
        bool   have_region = false;
        bool   gap_one_side = false;

        if (tsv_gap_one_side_target_rect(net, modules_, nlayer, layer, tier_w, b_lo, b_hi, lo_ok, hi_ok,
                                         die.width, die.height, rx0, ry0, rx1, ry1)) {
            have_region   = true;
            gap_one_side = true;
        } else if (lo_ok && hi_ok) {
            have_region = tsv_placement_rect_from_two_bboxes(b_lo, b_hi, rx0, ry0, rx1, ry1);
        } else if (lo_ok) {
            rx0 = b_lo.xmin;
            rx1 = b_lo.xmax;
            ry0 = b_lo.ymin;
            ry1 = b_lo.ymax;
            have_region = true;
        } else if (hi_ok) {
            rx0 = b_hi.xmin;
            rx1 = b_hi.xmax;
            ry0 = b_hi.ymin;
            ry1 = b_hi.ymax;
            have_region = true;
        }

        if (!have_region) {
            tsv.x = std::max(hw, std::min(die.width - hw, die.width * 0.5));
            tsv.y = std::max(hh, std::min(die.height - hh, die.height * 0.5));
            tsv_placed_by_layer[layer].push_back(
                { tsv.x - hw, tsv.y - hh, tsv.x + hw, tsv.y + hh });
            continue;
        }

        rx0 = std::max(0.0, rx0);
        rx1 = std::min(die.width, rx1);
        ry0 = std::max(0.0, ry0);
        ry1 = std::min(die.height, ry1);
        if (rx0 > rx1) std::swap(rx0, rx1);
        if (ry0 > ry1) std::swap(ry0, ry1);

        double       w_lo = 1.0, w_hi = 1.0;
        bool         weighted_asym = false;
        bool         pref_rect_valid = false;
        double       pref_rx0 = 0.0, pref_ry0 = 0.0, pref_rx1 = 0.0, pref_ry1 = 0.0;
        const double epsw = 1e-9;

        if (!gap_one_side && lo_ok && hi_ok) {
            w_lo = tsv_placement_tier_weight(tsv.tier_below());
            w_hi = tsv_placement_tier_weight(tsv.tier_above());
            const double ix0 = std::max(b_lo.xmin, b_hi.xmin);
            const double ix1 = std::min(b_lo.xmax, b_hi.xmax);
            const double iy0 = std::max(b_lo.ymin, b_hi.ymin);
            const double iy1 = std::min(b_lo.ymax, b_hi.ymax);
            const bool   overlap2d = (ix0 <= ix1 + epsw && iy0 <= iy1 + epsw);
            const bool   wdiff     = std::fabs(w_lo - w_hi)
                                   > epsw * std::max(1.0, std::max(w_lo, w_hi));
            weighted_asym = !overlap2d && wdiff;
            if (weighted_asym) {
                const BBox& fav = (w_lo > w_hi) ? b_lo : b_hi;
                pref_rx0        = std::max(rx0, fav.xmin);
                pref_rx1        = std::min(rx1, fav.xmax);
                pref_ry0        = std::max(ry0, fav.ymin);
                pref_ry1        = std::min(ry1, fav.ymax);
                pref_rect_valid = (pref_rx0 <= pref_rx1 + epsw && pref_ry0 <= pref_ry1 + epsw);
            }
        }

        std::vector<PlacedRect> placed;
        placed.reserve(static_cast<int>(modules_.size()) + 16);
        for (const Module& m : modules_) {
            if (m.is_terminal) {
                if (layer != 0) continue;
            } else if (m.tier_id != layer) {
                continue;
            }
            placed.push_back({ m.lx(), m.ly(), m.rx(), m.ry() });
        }
        placed.insert(placed.end(),
                      tsv_placed_by_layer[layer].begin(),
                      tsv_placed_by_layer[layer].end());

        double cx = 0.0, cy = 0.0;
        bool   in_region = false;
        // 兩側 bbox 皆有效時：依 b_lo 中心最接近 b_hi 的哪個角，決定 first-fit 掃描起點
        const TsvFirstFitCorner ff_corner =
            (lo_ok && hi_ok) ? pick_corner_from_b_lo_b_hi_center_rel(b_lo, b_hi)
                            : TsvFirstFitCorner::BottomLeft;
        const bool skip_full_r_first_fit = !gap_one_side && weighted_asym && pref_rect_valid;
        if (skip_full_r_first_fit) {
            in_region = reflow_weighted_pref_line_corridor_fit(placed, pref_rx0, pref_ry0, pref_rx1,
                                                                pref_ry1, die.width, die.height, hw,
                                                                hh, cx, cy);
        } else {
            in_region = first_fit_tsv_in_region(placed, rx0, ry0, rx1, ry1, hw, hh, cx, cy,
                                                ff_corner);
        }
        if (!in_region) {
            double nx0 = rx0, ny0 = ry0, nx1 = rx1, ny1 = ry1;
            if (!gap_one_side && weighted_asym) {
                const bool   strong_asym = true;
                const BBox&  heavy_bbox = (w_lo >= w_hi) ? b_lo : b_hi;
                const BBox&  light_bbox = (w_lo <= w_hi) ? b_lo : b_hi;
                const BBox&  fav_fb     = strong_asym ? heavy_bbox : light_bbox;
                nx0 = std::max(0.0, std::min(die.width, fav_fb.xmin));
                nx1 = std::max(0.0, std::min(die.width, fav_fb.xmax));
                ny0 = std::max(0.0, std::min(die.height, fav_fb.ymin));
                ny1 = std::max(0.0, std::min(die.height, fav_fb.ymax));
                if (nx0 > nx1) std::swap(nx0, nx1);
                if (ny0 > ny1) std::swap(ny0, ny1);
            }
            if (!nearest_free_tsv_to_rect(placed, nx0, ny0, nx1, ny1,
                                          die.width, die.height, hw, hh, cx, cy)) {
                cx = std::max(hw, std::min(die.width - hw, die.width * 0.5));
                cy = std::max(hh, std::min(die.height - hh, die.height * 0.5));
            }
        }

        tsv.x = std::max(hw, std::min(die.width - hw, cx));
        tsv.y = std::max(hh, std::min(die.height - hh, cy));
        tsv_placed_by_layer[layer].push_back(
            { tsv.x - hw, tsv.y - hh, tsv.x + hw, tsv.y + hh });
    }

    std::cout << "[ReflowTSV] Done. TSV cost = "
              << std::fixed << std::setprecision(2) << compute_tsv_cost() << "\n";
}
