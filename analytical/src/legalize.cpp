// 3D IC Analytical Floorplanner - Recursive Bi-partitioning
//
// 流程：
//   1. partition_all_tiers()：對每個 tier 建立根節點，遞迴呼叫 partition()
//   2. partition()：
//      a. 若 module 數 ≤ leaf_threshold → 停止（葉節點）
//      b. 選切割軸（較長邊，長寬比 ≈ 1 時評估兩軸）
//      c. collect_ranked_split_candidates()：掃線得候選；切分線四捨五入為整數。
//         重試：validate 失敗，或「左右皆為 leaf」時 legalize_leaf 後仍偵測到重疊。
//      d. shift_modules()：跨線的 module/TSV 推到較大面積那側，
//         並更新座標貼齊切割線
//      e. rebalance_split_line_to_side_area_ratio()：
//         依目前左右兩側 module+TSV「面積和」比例，將切割線移到使子區域
//         幾何面積與該比例一致（例：左:右 面積 3:2 → 切割線使左寬:右寬=3:2）；
//         若移動後違反容量或子區域幾何下限則放棄調整。必要時迭代並再次 shift。
//      f. 建立子節點，遞迴
//      g. 若仍 fallback：合併為單一 leaf；凍結側可推擠往右／上（不重排），其餘 first-fit
//   3. log_partition_tree()：DFS 把 tree 印到檔案
#include "floorplanner.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

using namespace std;

// ============================================================
// 工具：計算一個矩形節點內的「可用面積」
// 可用面積 = 節點幾何面積 - 其內所有 module + TSV 的佔用面積
// ============================================================
[[maybe_unused]]
static double available_area(const PartitionNode&         node,
                             const std::vector<Module>&    modules,
                             double tsv_w, double tsv_h)
{
    double used = 0.0;
    for (int id : node.module_ids)      used += modules[id].area();
    used += node.tsv_ids.size() * tsv_w * tsv_h;
    return node.area() - used;
}

// ============================================================
// fixed_area_in_rect: 計算 fixed module 列表與指定矩形的交集面積之和
// ============================================================
static double fixed_area_in_rect(
    const std::vector<int>&    fixed_ids,
    const std::vector<Module>& modules,
    double xmin, double ymin, double xmax, double ymax)
{
    double area = 0.0;
    for (int fid : fixed_ids) {
        const Module& m = modules[fid];
        const double ix0 = std::max(m.lx(), xmin);
        const double ix1 = std::min(m.rx(), xmax);
        const double iy0 = std::max(m.ly(), ymin);
        const double iy1 = std::min(m.ry(), ymax);
        if (ix1 > ix0 + 1e-12 && iy1 > iy0 + 1e-12)
            area += (ix1 - ix0) * (iy1 - iy0);
    }
    return area;
}

// ============================================================
// collect_ranked_split_candidates:
//   與 find_best_split 相同的掃線與約束，但回傳「通過初選」的 L，
//   依 cross_area 由小到大排序（同 cross 則依 L）；鄰近重複 L 會去重。
// ============================================================
struct SplitCand {
    double L;
    double cross_area;
};

static std::vector<double> collect_ranked_split_candidates(
    PartitionNode&            node,
    const std::vector<Module>& modules,
    const std::vector<TSV>&    tsvs,
    const PartitionConfig&     pcfg,
    bool                       split_x,
    const std::vector<int>&    fixed_ids)
{
    double lo   = split_x ? node.xmin : node.ymin;
    double hi   = split_x ? node.xmax : node.ymax;
    double span = hi - lo;

    double L_min = lo + pcfg.min_split_ratio * span;
    double L_max = lo + pcfg.max_split_ratio * span;

    int    nc    = std::max(2, pcfg.num_candidates);
    double step  = (L_max - L_min) / (nc - 1);

    std::vector<SplitCand> pool;

    for (int k = 0; k < nc; ++k) {
        double L = L_min + k * step;

        int    left_mod_cnt = 0, right_mod_cnt = 0;
        double used_left = 0.0, used_right = 0.0;
        double left_max_short = 0.0, right_max_short = 0.0;

        for (int mid : node.module_ids) {
            const Module& m = modules[mid];
            double ctr = split_x ? m.x : m.y;
            double short_side = std::min(m.width, m.height);
            if (ctr <= L) {
                ++left_mod_cnt;
                used_left += m.area();
                left_max_short = std::max(left_max_short, short_side);
            } else {
                ++right_mod_cnt;
                used_right += m.area();
                right_max_short = std::max(right_max_short, short_side);
            }
        }

        if (left_mod_cnt < pcfg.min_modules_per_region ||
            right_mod_cnt < pcfg.min_modules_per_region) {
            continue;
        }

        for (int tid : node.tsv_ids) {
            double ctr = split_x ? tsvs[tid].x : tsvs[tid].y;
            double a   = pcfg.tsv_width * pcfg.tsv_height;
            if (ctr <= L) used_left  += a;
            else          used_right += a;
        }

        double left_w  = split_x ? (L - node.xmin) : node.width();
        double right_w = split_x ? (node.xmax - L) : node.width();
        double left_h  = split_x ? node.height()   : (L - node.ymin);
        double right_h = split_x ? node.height()   : (node.ymax - L);

        bool geom_ok =
            (left_w  + 1e-12 >= left_max_short)  && (left_h  + 1e-12 >= left_max_short) &&
            (right_w + 1e-12 >= right_max_short) && (right_h + 1e-12 >= right_max_short);
        if (!geom_ok) continue;

        double cross_area = 0.0;

        for (int mid : node.module_ids) {
            const Module& m  = modules[mid];
            double m_lo = split_x ? m.lx() : m.ly();
            double m_hi = split_x ? m.rx() : m.ry();
            if (m_lo < L && m_hi > L) {
                double left_part  = L - m_lo;
                double right_part = m_hi - L;
                double smaller    = std::min(left_part, right_part);
                double perp = split_x ? m.height : m.width;
                cross_area += smaller * perp;
            }
        }
        for (int tid : node.tsv_ids) {
            double tw  = pcfg.tsv_width;
            double th  = pcfg.tsv_height;
            double t_lo = split_x ? (tsvs[tid].x - tw * 0.5) : (tsvs[tid].y - th * 0.5);
            double t_hi = split_x ? (tsvs[tid].x + tw * 0.5) : (tsvs[tid].y + th * 0.5);
            if (t_lo < L && t_hi > L) {
                double smaller = std::min(L - t_lo, t_hi - L);
                double perp    = split_x ? th : tw;
                cross_area    += smaller * perp;
            }
        }

        double left_area  = split_x ? (L - node.xmin) * node.height()
                                    : node.width() * (L - node.ymin);
        double right_area = split_x ? (node.xmax - L) * node.height()
                                    : node.width() * (node.ymax - L);

        // 扣除 fixed module 在各側的佔用，得到可用面積
        const double fa_l = fixed_area_in_rect(fixed_ids, modules,
            node.xmin, node.ymin,
            split_x ? L : node.xmax,
            split_x ? node.ymax : L);
        const double fa_r = fixed_area_in_rect(fixed_ids, modules,
            split_x ? L : node.xmin,
            split_x ? node.ymin : L,
            node.xmax, node.ymax);

        bool capacity_ok = ((left_area  - fa_l) >= used_left  - 1e-9) &&
                           ((right_area - fa_r) >= used_right - 1e-9);

        if (capacity_ok)
            pool.push_back({L, cross_area});
    }

    std::sort(pool.begin(), pool.end(), [](const SplitCand& a, const SplitCand& b) {
        if (a.cross_area != b.cross_area) return a.cross_area < b.cross_area;
        return a.L < b.L;
    });

    std::vector<double> out;
    for (const auto& c : pool) {
        if (out.empty() || std::fabs(out.back() - c.L) > 1e-9)
            out.push_back(c.L);
    }
    return out;
}

// ============================================================
// validate_final_split: shift + re-balance 後，固定左右集合下檢查容量與幾何
// （與 rebalance 內邏輯一致）
// ============================================================
static bool validate_final_split(
    const PartitionNode&         node,
    const std::vector<Module>&   modules,
    const PartitionConfig&       pcfg,
    bool                         split_x,
    double                       L,
    const std::vector<int>&      left_mods,
    const std::vector<int>&      right_mods,
    const std::vector<int>&      left_tsvs,
    const std::vector<int>&      right_tsvs,
    const std::vector<int>&      fixed_ids)
{
    double used_left = 0.0, used_right = 0.0;
    for (int mid : left_mods)  used_left  += modules[mid].area();
    for (int mid : right_mods) used_right += modules[mid].area();
    const double ta = pcfg.tsv_width * pcfg.tsv_height;
    used_left  += left_tsvs.size()  * ta;
    used_right += right_tsvs.size() * ta;

    double left_area  = split_x ? (L - node.xmin) * node.height()
                                  : node.width() * (L - node.ymin);
    double right_area = split_x ? (node.xmax - L) * node.height()
                                  : node.width() * (node.ymax - L);

    // 扣除 fixed module 在各側的佔用
    const double fa_l = fixed_area_in_rect(fixed_ids, modules,
        node.xmin, node.ymin,
        split_x ? L : node.xmax,
        split_x ? node.ymax : L);
    const double fa_r = fixed_area_in_rect(fixed_ids, modules,
        split_x ? L : node.xmin,
        split_x ? node.ymin : L,
        node.xmax, node.ymax);

    if ((left_area  - fa_l) < used_left  - 1e-9 ||
        (right_area - fa_r) < used_right - 1e-9)
        return false;

    double left_max_short  = 0.0;
    double right_max_short = 0.0;
    for (int mid : left_mods) {
        const Module& m = modules[mid];
        left_max_short = std::max(left_max_short, std::min(m.width, m.height));
    }
    for (int mid : right_mods) {
        const Module& m = modules[mid];
        right_max_short = std::max(right_max_short, std::min(m.width, m.height));
    }

    const double lw = split_x ? (L - node.xmin) : node.width();
    const double rw = split_x ? (node.xmax - L) : node.width();
    const double lh = split_x ? node.height()     : (L - node.ymin);
    const double rh = split_x ? node.height()     : (node.ymax - L);

    return (lw + 1e-12 >= left_max_short)  && (lh + 1e-12 >= left_max_short) &&
           (rw + 1e-12 >= right_max_short) && (rh + 1e-12 >= right_max_short);
}

// ============================================================
// round_split_line_to_integer: 切分線四捨五入為整數，並落在 (lo, hi) 內
// ============================================================
static double round_split_line_to_integer(double L, double lo, double hi)
{
    const double eps = 1e-9;
    double       Lr  = std::round(L);
    if (Lr <= lo + eps) Lr = std::ceil(lo + eps);
    if (Lr >= hi - eps) Lr = std::floor(hi - eps);
    if (Lr <= lo || Lr >= hi) {
        Lr = std::round((lo + hi) * 0.5);
        return std::max(lo + eps, std::min(hi - eps, Lr));
    }
    return Lr;
}

// ============================================================
// rects_overlap_xy: 軸對齊矩形是否相交（面積重疊；邊貼齊不算）
// ============================================================
static bool rects_overlap_xy(
    double lax, double lay, double rax, double ray,
    double lbx, double lby, double rbx, double rby)
{
    const double eps = 1e-9;
    if (rax <= lbx + eps || rbx <= lax + eps) return false;
    if (ray <= lby + eps || rby <= lay + eps) return false;
    return true;
}

// 同一子集合內 module / TSV 是否有任意兩個幾何重疊（包含與 fixed 障礙物的碰撞）
static bool entity_group_has_overlap(
    const std::vector<int>&      mod_ids,
    const std::vector<int>&      tsv_ids,
    const std::vector<Module>&   modules,
    const std::vector<TSV>&      tsvs,
    double                       tsv_w,
    double                       tsv_h,
    const std::vector<int>&      fixed_ids)
{
    struct BB { double lx, ly, rx, ry; };
    // fixed 障礙物先進 bb（作為只參與「被撞」側，不互相檢查）
    std::vector<BB> fixed_bb;
    fixed_bb.reserve(fixed_ids.size());
    for (int fid : fixed_ids) {
        const Module& m = modules[fid];
        fixed_bb.push_back({ m.lx(), m.ly(), m.rx(), m.ry() });
    }

    std::vector<BB> bb;
    bb.reserve(mod_ids.size() + tsv_ids.size());
    for (int mid : mod_ids) {
        const Module& m = modules[mid];
        bb.push_back({ m.lx(), m.ly(), m.rx(), m.ry() });
    }
    const double hw = tsv_w * 0.5;
    const double hh = tsv_h * 0.5;
    for (int tid : tsv_ids) {
        const TSV& t = tsvs[tid];
        bb.push_back({ t.x - hw, t.y - hh, t.x + hw, t.y + hh });
    }
    // movable vs movable（含 TSV）
    for (size_t i = 0; i < bb.size(); ++i) {
        for (size_t j = i + 1; j < bb.size(); ++j) {
            if (rects_overlap_xy(
                    bb[i].lx, bb[i].ly, bb[i].rx, bb[i].ry,
                    bb[j].lx, bb[j].ly, bb[j].rx, bb[j].ry))
                return true;
        }
    }
    // movable vs fixed
    for (const auto& a : bb) {
        for (const auto& f : fixed_bb) {
            if (rects_overlap_xy(a.lx, a.ly, a.rx, a.ry,
                                 f.lx, f.ly, f.rx, f.ry))
                return true;
        }
    }
    return false;
}

static void snapshot_partition_entities(
    const PartitionNode&         node,
    const std::vector<Module>&   modules,
    const std::vector<TSV>&    tsvs,
    std::vector<double>&         mx,
    std::vector<double>&         my,
    std::vector<double>&         tx,
    std::vector<double>&         ty)
{
    mx.clear();
    my.clear();
    for (int mid : node.module_ids) {
        mx.push_back(modules[mid].x);
        my.push_back(modules[mid].y);
    }
    tx.clear();
    ty.clear();
    for (int tid : node.tsv_ids) {
        tx.push_back(tsvs[tid].x);
        ty.push_back(tsvs[tid].y);
    }
}

static void restore_partition_entities(
    const PartitionNode&       node,
    std::vector<Module>&       modules,
    std::vector<TSV>&          tsvs,
    const std::vector<double>& mx,
    const std::vector<double>& my,
    const std::vector<double>& tx,
    const std::vector<double>& ty)
{
    for (size_t i = 0; i < node.module_ids.size(); ++i) {
        int mid = node.module_ids[i];
        modules[mid].x = mx[i];
        modules[mid].y = my[i];
    }
    for (size_t i = 0; i < node.tsv_ids.size(); ++i) {
        int tid = node.tsv_ids[i];
        tsvs[tid].x = tx[i];
        tsvs[tid].y = ty[i];
    }
}

// ============================================================
// shift_modules:
//   對跨越切割線的 module/TSV，判斷哪一側面積較多，
//   整體推移至該側，並更新中心座標貼齊切割線。
// ============================================================
static void shift_modules(PartitionNode&      node,
                          std::vector<Module>& modules,
                          std::vector<TSV>&    tsvs,
                          const PartitionConfig& pcfg,
                          double               L,
                          bool                 split_x)
{
    for (int mid : node.module_ids) {
        Module& m   = modules[mid];
        double m_lo = split_x ? m.lx() : m.ly();
        double m_hi = split_x ? m.rx() : m.ry();

        if (m_lo >= L || m_hi <= L) continue;   // 不跨線

        double left_part  = L - m_lo;
        double right_part = m_hi - L;

        if (left_part >= right_part) {
            // 推到左側：右邊界 = L → 中心左移
            if (split_x) m.x = L - m.width  * 0.5;
            else         m.y = L - m.height * 0.5;
        } else {
            // 推到右側：左邊界 = L → 中心右移
            if (split_x) m.x = L + m.width  * 0.5;
            else         m.y = L + m.height * 0.5;
        }
    }

    for (int tid : node.tsv_ids) {
        TSV&   tsv  = tsvs[tid];
        double tw   = pcfg.tsv_width;
        double th   = pcfg.tsv_height;
        double t_lo = split_x ? (tsv.x - tw * 0.5) : (tsv.y - th * 0.5);
        double t_hi = split_x ? (tsv.x + tw * 0.5) : (tsv.y + th * 0.5);

        if (t_lo >= L || t_hi <= L) continue;

        double left_part  = L - t_lo;
        double right_part = t_hi - L;

        if (left_part >= right_part) {
            if (split_x) tsv.x = L - tw * 0.5;
            else         tsv.y = L - th * 0.5;
        } else {
            if (split_x) tsv.x = L + tw * 0.5;
            else         tsv.y = L + th * 0.5;
        }
    }
}

// ============================================================
// rebalance_split_line_to_side_area_ratio:
//   在選定初選 L 且已對該 L 做過一次 shift_modules() 之後呼叫。
//
//   令左側 module+TSV 面積和為 U_L、右側為 U_R（以中心點與切割線 L 判斷左右）。
//   目標：調整切割線 L*，使子區域幾何面積比等於 U_L : U_R。
//   垂直切（沿 X）：區間 [xmin,xmax] 切成兩段，使 (L*-xmin):(xmax-L*) = U_L:U_R
//     ⇒ L* = (xmin·U_R + xmax·U_L) / (U_L+U_R)
//   水平切（沿 Y）同理：L* = (ymin·U_R + ymax·U_L) / (U_L+U_R)
//
//   與等高矩形結合時，此分割亦使左右「利用率」一致。
//   注意：re-balance 階段不受 min_split_ratio / max_split_ratio 限制，
//   僅檢查容量與子區域寬高 ≥ 該側 module 短邊。
//   若新線與舊線不同，會再次 shift_modules() 並重算左右集合；最多迭代數次以收斂。
// ============================================================
static double rebalance_split_line_to_side_area_ratio(
    PartitionNode&         node,
    std::vector<Module>&   modules,
    std::vector<TSV>&      tsvs,
    const PartitionConfig& pcfg,
    bool                   split_x,
    double                 L,
    const std::vector<int>& fixed_left_mods,
    const std::vector<int>& fixed_right_mods,
    const std::vector<int>& fixed_left_tsvs,
    const std::vector<int>& fixed_right_tsvs,
    const std::vector<int>& fixed_obstacle_ids)
{
    const double lo   = split_x ? node.xmin : node.ymin;
    const double hi   = split_x ? node.xmax : node.ymax;
    const double span = hi - lo;
    (void)span;

    for (int it = 0; it < 1; ++it) {
        double used_left = 0.0, used_right = 0.0;
        for (int mid : fixed_left_mods)  used_left  += modules[mid].area();
        for (int mid : fixed_right_mods) used_right += modules[mid].area();
        used_left  += fixed_left_tsvs.size()  * pcfg.tsv_width * pcfg.tsv_height;
        used_right += fixed_right_tsvs.size() * pcfg.tsv_width * pcfg.tsv_height;

        const double denom = used_left + used_right;
        if (denom <= 1e-12)
            return L;

        double left_max_short  = 0.0;
        double right_max_short = 0.0;
        for (int mid : fixed_left_mods) {
            const Module& m = modules[mid];
            left_max_short = std::max(left_max_short, std::min(m.width, m.height));
        }
        for (int mid : fixed_right_mods) {
            const Module& m = modules[mid];
            right_max_short = std::max(right_max_short, std::min(m.width, m.height));
        }

        auto geom_ok_with = [&](double cut) {
            const double lw = split_x ? (cut - node.xmin) : node.width();
            const double rw = split_x ? (node.xmax - cut) : node.width();
            const double lh = split_x ? node.height()     : (cut - node.ymin);
            const double rh = split_x ? node.height()     : (node.ymax - cut);
            return (lw + 1e-12 >= left_max_short)  && (lh + 1e-12 >= left_max_short) &&
                   (rw + 1e-12 >= right_max_short) && (rh + 1e-12 >= right_max_short);
        };

        // 幾何可行時，初始 L_bal 就用「扣除 fixed 後的可用面積比例」決定，
        // 而不是只用幾何面積比例。
        const double geom_lo = split_x ? (node.xmin + left_max_short)
                                       : (node.ymin + left_max_short);
        const double geom_hi = split_x ? (node.xmax - right_max_short)
                                       : (node.ymax - right_max_short);
        if (geom_lo > geom_hi + 1e-9)
            return L;
        const double cut_lo = std::max(lo + 1e-9, geom_lo);
        const double cut_hi = std::min(hi - 1e-9, geom_hi);
        if (cut_lo > cut_hi + 1e-9)
            return L;

        const double target_ratio = used_left / std::max(1e-12, used_right);
        auto ratio_diff = [&](double cut) {
            const double la = split_x ? (cut - node.xmin) * node.height()
                                      : node.width() * (cut - node.ymin);
            const double ra = split_x ? (node.xmax - cut) * node.height()
                                      : node.width() * (node.ymax - cut);
            const double fa_l = fixed_area_in_rect(fixed_obstacle_ids, modules,
                node.xmin, node.ymin,
                split_x ? cut : node.xmax,
                split_x ? node.ymax : cut);
            const double fa_r = fixed_area_in_rect(fixed_obstacle_ids, modules,
                split_x ? cut : node.xmin,
                split_x ? node.ymin : cut,
                node.xmax, node.ymax);
            const double free_l = std::max(1e-12, la - fa_l);
            const double free_r = std::max(1e-12, ra - fa_r);
            return std::fabs((free_l / free_r) - target_ratio);
        };

        double L_bal = cut_lo;
        double best_diff = std::numeric_limits<double>::infinity();
        const int samples = 121;
        for (int s = 0; s < samples; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(samples - 1);
            const double cut = cut_lo + (cut_hi - cut_lo) * t;
            const double d = ratio_diff(cut);
            if (d < best_diff) {
                best_diff = d;
                L_bal = cut;
            }
        }

        bool geom_ok = geom_ok_with(L_bal);
        if (!geom_ok) {
            // 幾何不合法時，不直接 return；把切割線推到滿足 max_short 的可行範圍。
            if (geom_lo > geom_hi + 1e-9) {
                // 該固定左右分組本身幾何不可行
                return L;
            }
            L_bal = std::max(geom_lo, std::min(geom_hi, L_bal));
            geom_ok = geom_ok_with(L_bal);
        }

        // L_bal 若被幾何修正過，容量也要重算
        const double left_area  = split_x ? (L_bal - node.xmin) * node.height()
                                          : node.width() * (L_bal - node.ymin);
        const double right_area = split_x ? (node.xmax - L_bal) * node.height()
                                          : node.width() * (node.ymax - L_bal);
        // 扣除 fixed module 在各側的佔用
        const double fa_l = fixed_area_in_rect(fixed_obstacle_ids, modules,
            node.xmin, node.ymin,
            split_x ? L_bal : node.xmax,
            split_x ? node.ymax : L_bal);
        const double fa_r = fixed_area_in_rect(fixed_obstacle_ids, modules,
            split_x ? L_bal : node.xmin,
            split_x ? node.ymin : L_bal,
            node.xmax, node.ymax);
        const bool capacity_ok2 =
            (left_area  - fa_l >= used_left  - 1e-9) &&
            (right_area - fa_r >= used_right - 1e-9);

        if (!capacity_ok2 || !geom_ok)
            return L;

        if (std::fabs(L_bal - L) <= 1e-9)
            return L;

        L = L_bal;
        shift_modules(node, modules, tsvs, pcfg, L, split_x);
    }
    return L;
}

static void legalize_leaf(PartitionNode&              leaf,
                          std::vector<Module>&       modules,
                          std::vector<TSV>&          tsvs,
                          double                     tsv_w,
                          double                     tsv_h,
                          const std::vector<int>&    fixed_ids);

static void legalize_fallback_merged_firstfit(
    PartitionNode&              region,
    std::vector<Module>&       modules,
    std::vector<TSV>&          tsvs,
    const std::vector<int>&    left_mods,
    const std::vector<int>&    right_mods,
    const std::vector<int>&    left_tsvs,
    const std::vector<int>&    right_tsvs,
    bool                       keep_left,
    double                     tsv_w,
    double                     tsv_h,
    const std::vector<int>&    fixed_ids);

// ============================================================
// partition: 遞迴 Bi-partitioning 主體
// ============================================================
static void partition(PartitionNode&         node,
                      std::vector<Module>&    modules,
                      std::vector<TSV>&       tsvs,
                      const PartitionConfig&  pcfg,
                      const std::vector<int>& fixed_ids)
{
    // ---- 終止條件：module 數 ≤ leaf_threshold ----
    int mcount = static_cast<int>(node.module_ids.size());
    if (mcount <= pcfg.leaf_threshold)
        return;
    // 若切下去無法保證兩邊都 ≥ min_modules_per_region，直接停止
    if (mcount < 2 * pcfg.min_modules_per_region)
        return;

    // ---- 選切割軸 ----
    bool split_x;
    double aspect = node.width() / (node.height() + 1e-12);
    if (aspect > 1.1) {
        split_x = true;    // 寬大於高，垂直切
    } else if (aspect < 0.9) {
        split_x = false;   // 高大於寬，水平切
    } else {
        // 長寬比接近 1，選讓 module 分佈更均勻的軸
        // 策略：選 module 在該方向的 span 較大的軸
        double x_span = 0.0, y_span = 0.0;
        if (!node.module_ids.empty()) {
            double xlo = 1e18, xhi = -1e18, ylo = 1e18, yhi = -1e18;
            for (int mid : node.module_ids) {
                xlo = std::min(xlo, modules[mid].x);
                xhi = std::max(xhi, modules[mid].x);
                ylo = std::min(ylo, modules[mid].y);
                yhi = std::max(yhi, modules[mid].y);
            }
            x_span = xhi - xlo;
            y_span = yhi - ylo;
        }
        split_x = (x_span >= y_span);
    }

    // ---- 候選切割線（依 cross_area 佳→次佳）；整數切分線；
    //     重試：validate 失敗，或左右皆為 leaf 且 legalize 後仍有重疊 ----
    const double split_lo = split_x ? node.xmin : node.ymin;
    const double split_hi = split_x ? node.xmax : node.ymax;

    std::vector<double> ranked_Ls = collect_ranked_split_candidates(node, modules, tsvs, pcfg, split_x, fixed_ids);
    if (ranked_Ls.empty())
        ranked_Ls.push_back((split_lo + split_hi) * 0.5);

    std::vector<double> snap_mx, snap_my, snap_tx, snap_ty;
    snapshot_partition_entities(node, modules, tsvs, snap_mx, snap_my, snap_tx, snap_ty);

    std::vector<int> left_mods, right_mods, left_tsvs, right_tsvs;
    left_mods.reserve(node.module_ids.size());
    right_mods.reserve(node.module_ids.size());
    left_tsvs.reserve(node.tsv_ids.size());
    right_tsvs.reserve(node.tsv_ids.size());

    const int max_tries =
        std::max(1, std::min(pcfg.max_split_retries, static_cast<int>(ranked_Ls.size())));
    double L       = 0.0;
    bool   split_ok = false;
    bool   dual_leaf_legalized_in_loop = false;

    for (int attempt = 0; attempt < max_tries; ++attempt) {
        dual_leaf_legalized_in_loop = false;
        restore_partition_entities(node, modules, tsvs, snap_mx, snap_my, snap_tx, snap_ty);
        L = round_split_line_to_integer(ranked_Ls[attempt], split_lo, split_hi);

        if (attempt > 0) {
            std::cout << "[Partition] split_retry tier=T" << node.tier_id
                      << " depth=" << node.depth
                      << " candidate_index=" << attempt << "/" << max_tries
                      << " L=" << std::fixed << std::setprecision(2) << L
                      << " axis=" << (split_x ? "vertical(x)" : "horizontal(y)")
                      << "\n";
            std::cout << std::defaultfloat << std::setprecision(6);
        }

        shift_modules(node, modules, tsvs, pcfg, L, split_x);

        left_mods.clear();
        right_mods.clear();
        left_tsvs.clear();
        right_tsvs.clear();
        for (int mid : node.module_ids) {
            double ctr = split_x ? modules[mid].x : modules[mid].y;
            if (ctr <= L) left_mods.push_back(mid);
            else          right_mods.push_back(mid);
        }
        for (int tid : node.tsv_ids) {
            double ctr = split_x ? tsvs[tid].x : tsvs[tid].y;
            if (ctr <= L) left_tsvs.push_back(tid);
            else          right_tsvs.push_back(tid);
        }

        L = rebalance_split_line_to_side_area_ratio(
            node, modules, tsvs, pcfg, split_x, L,
            left_mods, right_mods, left_tsvs, right_tsvs, fixed_ids);
        L = round_split_line_to_integer(L, split_lo, split_hi);

        if (!validate_final_split(node, modules, pcfg, split_x, L,
                left_mods, right_mods, left_tsvs, right_tsvs, fixed_ids))
            continue;

        const int nL = static_cast<int>(left_mods.size());
        const int nR = static_cast<int>(right_mods.size());
        const bool both_sides_are_leaves =
            (nL <= pcfg.leaf_threshold && nR <= pcfg.leaf_threshold);

        if (both_sides_are_leaves) {
            PartitionNode tmp_l;
            PartitionNode tmp_r;
            tmp_l.tier_id  = node.tier_id;
            tmp_r.tier_id  = node.tier_id;
            tmp_l.depth    = node.depth + 1;
            tmp_r.depth    = node.depth + 1;
            if (split_x) {
                tmp_l.xmin  = node.xmin;
                tmp_l.xmax  = L;
                tmp_l.ymin  = node.ymin;
                tmp_l.ymax  = node.ymax;
                tmp_r.xmin  = L;
                tmp_r.xmax  = node.xmax;
                tmp_r.ymin  = node.ymin;
                tmp_r.ymax  = node.ymax;
            } else {
                tmp_l.xmin  = node.xmin;
                tmp_l.xmax  = node.xmax;
                tmp_l.ymin  = node.ymin;
                tmp_l.ymax  = L;
                tmp_r.xmin  = node.xmin;
                tmp_r.xmax  = node.xmax;
                tmp_r.ymin  = L;
                tmp_r.ymax  = node.ymax;
            }
            tmp_l.module_ids = left_mods;
            tmp_l.tsv_ids    = left_tsvs;
            tmp_r.module_ids = right_mods;
            tmp_r.tsv_ids    = right_tsvs;

            legalize_leaf(tmp_l, modules, tsvs, pcfg.tsv_width, pcfg.tsv_height, fixed_ids);
            legalize_leaf(tmp_r, modules, tsvs, pcfg.tsv_width, pcfg.tsv_height, fixed_ids);

            const bool ov_l =
                entity_group_has_overlap(left_mods, left_tsvs, modules, tsvs,
                    pcfg.tsv_width, pcfg.tsv_height, fixed_ids);
            const bool ov_r =
                entity_group_has_overlap(right_mods, right_tsvs, modules, tsvs,
                    pcfg.tsv_width, pcfg.tsv_height, fixed_ids);

            if (ov_l || ov_r) {
                std::cout << "[Partition] split_retry: overlap after legalize_leaf on dual-leaf split "
                          << "tier=T" << node.tier_id << " depth=" << node.depth
                          << " left_ov=" << (ov_l ? "1" : "0")
                          << " right_ov=" << (ov_r ? "1" : "0") << "\n";
                continue;
            }
            dual_leaf_legalized_in_loop = true;
        }

        split_ok = true;
        if (attempt > 0) {
            std::cout << "[Partition] split_retry success tier=T" << node.tier_id
                      << " depth=" << node.depth
                      << " attempts_used=" << (attempt + 1)
                      << " final_L=" << std::fixed << std::setprecision(2) << L << "\n";
            std::cout << std::defaultfloat << std::setprecision(6);
        }
        break;
    }

    // 若仍無合法切分，用最佳候選（或中點）強制切下去
    if (!split_ok) {
        restore_partition_entities(node, modules, tsvs, snap_mx, snap_my, snap_tx, snap_ty);
        L = round_split_line_to_integer(ranked_Ls[0], split_lo, split_hi);
        shift_modules(node, modules, tsvs, pcfg, L, split_x);

        left_mods.clear();
        right_mods.clear();
        left_tsvs.clear();
        right_tsvs.clear();
        for (int mid : node.module_ids) {
            double ctr = split_x ? modules[mid].x : modules[mid].y;
            if (ctr <= L) left_mods.push_back(mid);
            else          right_mods.push_back(mid);
        }
        for (int tid : node.tsv_ids) {
            double ctr = split_x ? tsvs[tid].x : tsvs[tid].y;
            if (ctr <= L) left_tsvs.push_back(tid);
            else          right_tsvs.push_back(tid);
        }

        L = rebalance_split_line_to_side_area_ratio(
            node, modules, tsvs, pcfg, split_x, L,
            left_mods, right_mods, left_tsvs, right_tsvs, fixed_ids);
        L = round_split_line_to_integer(L, split_lo, split_hi);
        if (max_tries > 1) {
            std::cout << "[Partition] split_retry exhausted tier=T" << node.tier_id
                      << " depth=" << node.depth
                      << " tried=" << max_tries
                      << " -> fallback first candidate L=" << std::fixed << std::setprecision(2)
                      << L << "\n";
            std::cout << std::defaultfloat << std::setprecision(6);
        }
    }

    const bool used_fallback = !split_ok;

    // ---- fallback：不分子節點，合併成單一區域；依 legalize 後是否重疊決定凍結側 + first-fit----
    if (used_fallback) {
        PartitionNode tmp_l;
        PartitionNode tmp_r;
        tmp_l.tier_id = node.tier_id;
        tmp_r.tier_id = node.tier_id;
        tmp_l.depth   = node.depth + 1;
        tmp_r.depth   = node.depth + 1;
        if (split_x) {
            tmp_l.xmin = node.xmin;
            tmp_l.xmax = L;
            tmp_l.ymin = node.ymin;
            tmp_l.ymax = node.ymax;
            tmp_r.xmin = L;
            tmp_r.xmax = node.xmax;
            tmp_r.ymin = node.ymin;
            tmp_r.ymax = node.ymax;
        } else {
            tmp_l.xmin = node.xmin;
            tmp_l.xmax = node.xmax;
            tmp_l.ymin = node.ymin;
            tmp_l.ymax = L;
            tmp_r.xmin = node.xmin;
            tmp_r.xmax = node.xmax;
            tmp_r.ymin = L;
            tmp_r.ymax = node.ymax;
        }
        tmp_l.module_ids = left_mods;
        tmp_l.tsv_ids    = left_tsvs;
        tmp_r.module_ids = right_mods;
        tmp_r.tsv_ids    = right_tsvs;

        legalize_leaf(tmp_l, modules, tsvs, pcfg.tsv_width, pcfg.tsv_height, fixed_ids);
        legalize_leaf(tmp_r, modules, tsvs, pcfg.tsv_width, pcfg.tsv_height, fixed_ids);

        const bool ov_l =
            entity_group_has_overlap(left_mods, left_tsvs, modules, tsvs,
                pcfg.tsv_width, pcfg.tsv_height, fixed_ids);
        const bool ov_r =
            entity_group_has_overlap(right_mods, right_tsvs, modules, tsvs,
                pcfg.tsv_width, pcfg.tsv_height, fixed_ids);
        const bool lok = !ov_l;
        const bool rok = !ov_r;

        node.left = nullptr;
        node.right = nullptr;

        if (lok && rok) {
            std::cout << "[Partition] fallback_merge: both sub-regions legal, merged leaf tier=T"
                      << node.tier_id << " depth=" << node.depth << "\n";
            node.skip_leaf_legalize = true;
            return;
        }
        if (lok && !rok) {
            std::cout << "[Partition] fallback_merge: keep left, first-fit right tier=T" << node.tier_id
                      << " depth=" << node.depth << "\n";
            legalize_fallback_merged_firstfit(node, modules, tsvs,
                left_mods, right_mods, left_tsvs, right_tsvs,
                true, pcfg.tsv_width, pcfg.tsv_height, fixed_ids);
            node.skip_leaf_legalize = true;
            return;
        }
        if (!lok && rok) {
            std::cout << "[Partition] fallback_merge: keep right, first-fit left tier=T" << node.tier_id
                      << " depth=" << node.depth << "\n";
            legalize_fallback_merged_firstfit(node, modules, tsvs,
                left_mods, right_mods, left_tsvs, right_tsvs,
                false, pcfg.tsv_width, pcfg.tsv_height, fixed_ids);
            node.skip_leaf_legalize = true;
            return;
        }

        std::cout << "[Partition] fallback_merge: both overlap, full-region legalize_leaf tier=T"
                  << node.tier_id << " depth=" << node.depth << "\n";
        restore_partition_entities(node, modules, tsvs, snap_mx, snap_my, snap_tx, snap_ty);
        legalize_leaf(node, modules, tsvs, pcfg.tsv_width, pcfg.tsv_height, fixed_ids);
        node.skip_leaf_legalize = true;
        return;
    }

    // ---- 建立兩個子節點並分配 module/TSV ----
    node.split_x   = split_x;
    node.split_pos = L;

    auto left_node  = std::make_unique<PartitionNode>();
    auto right_node = std::make_unique<PartitionNode>();

    left_node->tier_id  = node.tier_id;
    right_node->tier_id = node.tier_id;
    left_node->depth    = node.depth + 1;
    right_node->depth   = node.depth + 1;

    if (split_x) {
        left_node->xmin  = node.xmin;  left_node->xmax  = L;
        left_node->ymin  = node.ymin;  left_node->ymax  = node.ymax;
        right_node->xmin = L;          right_node->xmax = node.xmax;
        right_node->ymin = node.ymin;  right_node->ymax = node.ymax;
    } else {
        left_node->xmin  = node.xmin;  left_node->xmax  = node.xmax;
        left_node->ymin  = node.ymin;  left_node->ymax  = L;
        right_node->xmin = node.xmin;  right_node->xmax = node.xmax;
        right_node->ymin = L;          right_node->ymax = node.ymax;
    }

    // 使用上面 re-balance 後的左右集合，避免再重算產生不一致
    left_node->module_ids = std::move(left_mods);
    right_node->module_ids = std::move(right_mods);
    left_node->tsv_ids = std::move(left_tsvs);
    right_node->tsv_ids = std::move(right_tsvs);

    if (dual_leaf_legalized_in_loop) {
        left_node->skip_leaf_legalize  = true;
        right_node->skip_leaf_legalize = true;
    }

    // ---- 遞迴 ----
    partition(*left_node,  modules, tsvs, pcfg, fixed_ids);
    partition(*right_node, modules, tsvs, pcfg, fixed_ids);

    node.left  = std::move(left_node);
    node.right = std::move(right_node);
}

// ============================================================
// legalize_fallback_merged_firstfit:
//   fallback 後合併為父區域。
//   keep right：凍結側在 legalize 配置上僅推擠往右、往上至緊貼（不重排）；可動側由下而上、左→右 first-fit。
//   keep left：凍結側沿用 legalize 位置；可動側由上而下、右→左 first-fit；TSV 同向掃描。
// ============================================================
static void legalize_fallback_merged_firstfit(
    PartitionNode&              region,
    std::vector<Module>&       modules,
    std::vector<TSV>&          tsvs,
    const std::vector<int>&    left_mods,
    const std::vector<int>&    right_mods,
    const std::vector<int>&    left_tsvs,
    const std::vector<int>&    right_tsvs,
    bool                       keep_left,
    double                     tsv_w,
    double                     tsv_h,
    const std::vector<int>&    fixed_ids)
{
    struct Rect { double lx, ly, rx, ry; };

    auto clamp = [&](double v, double lo, double hi) -> double {
        return std::max(lo, std::min(hi, v));
    };

    const std::vector<int>& frozen_mods = keep_left ? left_mods : right_mods;
    const std::vector<int>& move_mods   = keep_left ? right_mods : left_mods;

    // keep right(top) 時：凍結側 module 會先往右/上推擠，但「凍結側的 tsv」不要 frozen，
    // 讓它們在 module 全部擺完後也一起走 first-fit。
    std::vector<int> empty_tsvs;
    std::vector<int> move_tsvs_combined;
    const std::vector<int>& frozen_tsvs =
        keep_left ? left_tsvs : empty_tsvs;

    if (keep_left) {
        move_tsvs_combined = right_tsvs;
    } else {
        move_tsvs_combined = left_tsvs;
        move_tsvs_combined.insert(move_tsvs_combined.end(),
                                   right_tsvs.begin(), right_tsvs.end());
    }
    const std::vector<int>& move_tsvs = move_tsvs_combined;

    std::vector<Rect> placed;
    placed.reserve(frozen_mods.size() + frozen_tsvs.size() + move_mods.size() + move_tsvs.size());

    // 把 fixed 障礙物先放進 placed（從 per-tier fixed list 中取與 region 相交者）
    auto add_fixed_obstacles = [&]() {
        for (int fid : fixed_ids) {
            const Module& fm = modules[fid];
            if (fm.rx() <= region.xmin + 1e-9 || fm.lx() >= region.xmax - 1e-9) continue;
            if (fm.ry() <= region.ymin + 1e-9 || fm.ly() >= region.ymax - 1e-9) continue;
            placed.push_back({ fm.lx(), fm.ly(), fm.rx(), fm.ry() });
        }
    };
    add_fixed_obstacles();

    auto first_free_x_from_left = [&](const std::vector<Rect>& p,
                                      double                     hw,
                                      double                     hh,
                                      double                     cy) -> double {
        double cx = region.xmin + hw;
        bool   moved = true;
        while (moved) {
            moved = false;
            for (const Rect& r : p) {
                if (cy - hh >= r.ry - 1e-9 || cy + hh <= r.ly + 1e-9) continue;
                if (cx - hw < r.rx - 1e-9 && cx + hw > r.lx + 1e-9) {
                    cx = r.rx + hw;
                    moved = true;
                }
            }
        }
        return (cx + hw > region.xmax + 1e-9) ? -1.0 : cx;
    };

    // 由右往左：從靠右開始，與同列障礙重疊則往左推
    auto first_free_x_from_right = [&](const std::vector<Rect>& p,
                                       double                     hw,
                                       double                     hh,
                                       double                     cy) -> double {
        double cx = region.xmax - hw;
        bool   moved = true;
        while (moved) {
            moved = false;
            for (const Rect& r : p) {
                if (cy - hh >= r.ry - 1e-9 || cy + hh <= r.ly + 1e-9) continue;
                if (cx - hw < r.rx - 1e-9 && cx + hw > r.lx + 1e-9) {
                    cx = r.lx - hw;
                    moved = true;
                }
            }
        }
        return (cx - hw < region.xmin - 1e-9) ? -1.0 : cx;
    };

    auto collect_y_levels_ascending = [&](double hh, const std::vector<Rect>& current_placed) {
        std::vector<double> ys;
        auto push_if_ok = [&](double y) {
            if (y - hh < region.ymin - 1e-9) return;
            if (y + hh > region.ymax + 1e-9) return;
            ys.push_back(y);
        };
        push_if_ok(region.ymin + hh);
        push_if_ok(region.ymax - hh);
        for (const Rect& r : current_placed) {
            push_if_ok(r.ry + hh);
            push_if_ok(r.ly - hh);
        }
        if (ys.empty()) return ys;
        std::sort(ys.begin(), ys.end());
        const double tol = std::max(1e-6, hh * 1e-3);
        ys.erase(std::unique(ys.begin(), ys.end(),
                              [&](double a, double b){ return std::fabs(a - b) < tol; }),
                 ys.end());
        return ys;
    };

    // 下→上，左→右（預設 first-fit）
    auto first_free_y_bottom_then_x_from_left = [&](const std::vector<Rect>& p, double hw, double hh)
        -> std::pair<double, double> {
        for (double cy : collect_y_levels_ascending(hh, p)) {
            double cx = first_free_x_from_left(p, hw, hh, cy);
            if (cx >= 0.0)
                return { cx, cy };
        }
        return { -1.0, -1.0 };
    };

    // keep right(top)：凍結側維持 legalize 後的相對配置，只做「往右、往上」推到緊貼障礙／邊界（不重排、不旋轉）
    if (!keep_left) {
        const double eps = 1e-7;
        for (int pass = 0; pass < 64; ++pass) {
            bool changed = false;
            for (int mid : frozen_mods) {
                Module& m = modules[mid];
                const double hw = m.width * 0.5;
                const double hh = m.height * 0.5;

                double cx_lo = region.xmin + hw;
                double cx_hi = region.xmax - hw;
                for (int oid : frozen_mods) {
                    if (oid == mid) continue;
                    const Module& o = modules[oid];
                    if (m.y + hh <= o.ly() - eps || m.y - hh >= o.ry() + eps) continue;
                    if (m.rx() <= o.lx() + eps)
                        cx_hi = std::min(cx_hi, o.lx() - hw);
                    else if (o.rx() <= m.lx() + eps)
                        cx_lo = std::max(cx_lo, o.rx() + hw);
                }
                if (cx_lo <= cx_hi + eps) {
                    const double nx = cx_hi;
                    if (std::fabs(nx - m.x) > eps) {
                        m.x = nx;
                        changed = true;
                    }
                }

                double cy_lo = region.ymin + hh;
                double cy_hi = region.ymax - hh;
                for (int oid : frozen_mods) {
                    if (oid == mid) continue;
                    const Module& o = modules[oid];
                    if (m.rx() <= o.lx() - eps || m.lx() >= o.rx() + eps) continue;
                    if (m.ry() <= o.ly() + eps)
                        cy_hi = std::min(cy_hi, o.ly() - hh);
                    else if (o.ry() <= m.ly() + eps)
                        cy_lo = std::max(cy_lo, o.ry() + hh);
                }
                if (cy_lo <= cy_hi + eps) {
                    const double ny = cy_hi;
                    if (std::fabs(ny - m.y) > eps) {
                        m.y = ny;
                        changed = true;
                    }
                }
            }
            if (!changed) break;
        }

        placed.clear();
        placed.reserve(frozen_mods.size());
        for (int mid : frozen_mods) {
            const Module& m = modules[mid];
            placed.push_back({ m.lx(), m.ly(), m.rx(), m.ry() });
        }
    } else {
        for (int mid : frozen_mods) {
            const Module& m = modules[mid];
            placed.push_back({ m.lx(), m.ly(), m.rx(), m.ry() });
        }
    }

    // 上→下，左→右
    auto first_free_y_top_then_x_from_left = [&](const std::vector<Rect>& p, double hw, double hh)
        -> std::pair<double, double> {
        std::vector<double> ys = collect_y_levels_ascending(hh, p);
        std::sort(ys.begin(), ys.end(), std::greater<double>());
        for (double cy : ys) {
            double cx = first_free_x_from_left(p, hw, hh, cy);
            if (cx >= 0.0)
                return { cx, cy };
        }
        return { -1.0, -1.0 };
    };

    // 上→下，右→左（keep_left 時可動側主要掃描）
    auto first_free_y_top_then_x_from_right = [&](const std::vector<Rect>& p, double hw, double hh)
        -> std::pair<double, double> {
        std::vector<double> ys = collect_y_levels_ascending(hh, p);
        std::sort(ys.begin(), ys.end(), std::greater<double>());
        for (double cy : ys) {
            double cx = first_free_x_from_right(p, hw, hh, cy);
            if (cx >= 0.0)
                return { cx, cy };
        }
        return { -1.0, -1.0 };
    };

    // 下→上，右→左
    auto first_free_y_bottom_then_x_from_right = [&](const std::vector<Rect>& p, double hw, double hh)
        -> std::pair<double, double> {
        for (double cy : collect_y_levels_ascending(hh, p)) {
            double cx = first_free_x_from_right(p, hw, hh, cy);
            if (cx >= 0.0)
                return { cx, cy };
        }
        return { -1.0, -1.0 };
    };

    // 可動 module 快照（供 module overlap 時還原重試）
    std::vector<double> snap_mx, snap_my, snap_mw, snap_mh;
    snap_mx.reserve(move_mods.size());
    snap_my.reserve(move_mods.size());
    snap_mw.reserve(move_mods.size());
    snap_mh.reserve(move_mods.size());
    for (int mid : move_mods) {
        const Module& m = modules[mid];
        snap_mx.push_back(m.x);
        snap_my.push_back(m.y);
        snap_mw.push_back(m.width);
        snap_mh.push_back(m.height);
    }

    // 依 first-fit 排擺可動 module，置於 placed 後方
    using PickFn = std::function<std::pair<double, double>(const std::vector<Rect>&, double, double)>;
    auto place_move_modules = [&](const std::vector<int>& mod_order, const PickFn& pick_fn) {
        // 還原可動 module 到快照
        for (size_t i = 0; i < move_mods.size(); ++i) {
            const int mid = move_mods[i];
            modules[mid].x      = snap_mx[i];
            modules[mid].y      = snap_my[i];
            modules[mid].width  = snap_mw[i];
            modules[mid].height = snap_mh[i];
        }
        // 重建 placed（fixed 障礙物 + 凍結 module）
        placed.clear();
        placed.reserve(frozen_mods.size() + frozen_tsvs.size() + move_mods.size() + move_tsvs.size());
        add_fixed_obstacles();
        for (int mid : frozen_mods) {
            const Module& m = modules[mid];
            placed.push_back({ m.lx(), m.ly(), m.rx(), m.ry() });
        }
        // 依給定模組順序 + pick 策略排可動 module
        const int k = static_cast<int>(mod_order.size());
        if (k == 0) return;

        std::vector<double> ox(k), oy(k), ow(k), oh(k);
        for (int i = 0; i < k; ++i) {
            const Module& m = modules[mod_order[i]];
            ox[i] = m.x; oy[i] = m.y; ow[i] = m.width; oh[i] = m.height;
        }

        std::vector<double> cur_x(k), cur_y(k), cur_w(k), cur_h(k);
        std::vector<double> best_x(k), best_y(k), best_w(k), best_h(k);
        double best_total_cost = std::numeric_limits<double>::infinity();
        bool   best_found = false;
        auto can_place_rect = [&](double lx, double ly, double rx, double ry) -> bool {
            for (const Rect& r : placed) {
                if (rects_overlap_xy(lx, ly, rx, ry, r.lx, r.ly, r.rx, r.ry))
                    return false;
            }
            return true;
        };
        auto placed_has_any_overlap = [&]() -> bool {
            for (size_t i = 0; i < placed.size(); ++i) {
                for (size_t j = i + 1; j < placed.size(); ++j) {
                    if (rects_overlap_xy(
                            placed[i].lx, placed[i].ly, placed[i].rx, placed[i].ry,
                            placed[j].lx, placed[j].ly, placed[j].rx, placed[j].ry))
                        return true;
                }
            }
            return false;
        };

        std::function<void(int, double)> dfs_place = [&](int depth, double acc_cost) {
            if (acc_cost >= best_total_cost - 1e-12) return;
            if (depth == k) {
                // 只用「合法無重疊」解更新 best_total_cost，避免非法解影響剪枝
                if (placed_has_any_overlap())
                    return;
                best_total_cost = acc_cost;
                best_found = true;
                best_x = cur_x; best_y = cur_y; best_w = cur_w; best_h = cur_h;
                return;
            }

            const double base_w = ow[depth];
            const double base_h = oh[depth];
            struct Cand { double cx, cy, w, h, cost; };
            std::vector<Cand> cands;
            cands.reserve(2);

            for (int rot = 0; rot <= 1; ++rot) {
                const double w_i  = (rot == 0) ? base_w : base_h;
                const double h_i  = (rot == 0) ? base_h : base_w;
                const double hw_i = w_i * 0.5;
                const double hh_i = h_i * 0.5;
                if (region.ymin + hh_i > region.ymax - hh_i + 1e-12) continue;
                const auto xy = pick_fn(placed, hw_i, hh_i);
                if (xy.first < 0.0) continue;
                const double c = std::abs(xy.first - ox[depth]) + std::abs(xy.second - oy[depth]);
                cands.push_back({ xy.first, xy.second, w_i, h_i, c });
            }

            if (cands.empty()) {
                // 幾何上無可行 first-fit 時，優先選擇「能放進 bbox」的朝向（含旋轉）。
                const double box_w = region.xmax - region.xmin;
                const double box_h = region.ymax - region.ymin;
                const bool fit0  = (base_w <= box_w + 1e-12 && base_h <= box_h + 1e-12);
                const bool fit90 = (base_h <= box_w + 1e-12 && base_w <= box_h + 1e-12);
                const bool use_rot = (!fit0 && fit90);

                const double w_sel = use_rot ? base_h : base_w;
                const double h_sel = use_rot ? base_w : base_h;
                const double hw_i = w_sel * 0.5;
                const double hh_i = h_sel * 0.5;
                const double cx = clamp(ox[depth], region.xmin + hw_i, region.xmax - hw_i);
                const double cy = clamp(oy[depth], region.ymin + hh_i, region.ymax - hh_i);
                cur_x[depth] = cx; cur_y[depth] = cy; cur_w[depth] = w_sel; cur_h[depth] = h_sel;
                if (!can_place_rect(cx - hw_i, cy - hh_i, cx + hw_i, cy + hh_i))
                    return;
                const double c = std::abs(cx - ox[depth]) + std::abs(cy - oy[depth]);
                placed.push_back({ cx - hw_i, cy - hh_i, cx + hw_i, cy + hh_i });
                dfs_place(depth + 1, acc_cost + c);
                placed.pop_back();
                return;
            }

            // 當 0°/90° 都可行時，這裡會走兩個分支，等同窮舉所有旋轉組合。
            for (const auto& cand : cands) {
                const double hw_i = cand.w * 0.5;
                const double hh_i = cand.h * 0.5;
                cur_x[depth] = cand.cx; cur_y[depth] = cand.cy; cur_w[depth] = cand.w; cur_h[depth] = cand.h;
                placed.push_back({ cand.cx - hw_i, cand.cy - hh_i, cand.cx + hw_i, cand.cy + hh_i });
                dfs_place(depth + 1, acc_cost + cand.cost);
                placed.pop_back();
            }
        };

        dfs_place(0, 0.0);
        if (!best_found) return;

        for (int i = 0; i < k; ++i) {
            Module& m = modules[mod_order[i]];
            m.x = best_x[i];
            m.y = best_y[i];
            m.width = best_w[i];
            m.height = best_h[i];
        }

        // 以最佳解重建可動 module 佔據矩形
        for (int i = 0; i < k; ++i) {
            const double hw_i = best_w[i] * 0.5;
            const double hh_i = best_h[i] * 0.5;
            placed.push_back({ best_x[i] - hw_i, best_y[i] - hh_i,
                               best_x[i] + hw_i, best_y[i] + hh_i });
        }
    };

    // module-only overlap check（包含與 fixed 障礙物的碰撞）
    auto module_has_overlap = [&]() -> bool {
        std::vector<int> all_mods;
        all_mods.reserve(left_mods.size() + right_mods.size());
        all_mods.insert(all_mods.end(), left_mods.begin(), left_mods.end());
        all_mods.insert(all_mods.end(), right_mods.begin(), right_mods.end());
        std::vector<int> empty;
        return entity_group_has_overlap(all_mods, empty, modules, tsvs, tsv_w, tsv_h, fixed_ids);
    };

    // (scan, name) 嘗試清單；每個掃描策略下窮舉所有 module 順序
    struct Attempt { PickFn pick_fn; const char* name; };
    std::vector<Attempt> attempts;
    attempts.reserve(2);

    auto push_att = [&](PickFn pf, const char* n) {
        attempts.push_back({ std::move(pf), n });
    };

    if (!keep_left) {
        // 可動側在左/下；掃描：y 下→上/上→下，x 左→右
        push_att([&](const std::vector<Rect>& p, double hw, double hh){ return first_free_y_bottom_then_x_from_left(p,hw,hh); }, "y_bot_xleft");
        push_att([&](const std::vector<Rect>& p, double hw, double hh){ return first_free_y_top_then_x_from_left   (p,hw,hh); }, "y_top_xleft");
    } else {
        // 可動側在右/上；掃描：y 上→下/下→上，x 右→左
        push_att([&](const std::vector<Rect>& p, double hw, double hh){ return first_free_y_top_then_x_from_right   (p,hw,hh); }, "y_top_xright");
        push_att([&](const std::vector<Rect>& p, double hw, double hh){ return first_free_y_bottom_then_x_from_right(p,hw,hh); }, "y_bot_xright");
    }

    // 依序嘗試掃描策略；每個策略下窮舉所有 module 順序，遇到第一個無 overlap 即停止
    bool module_sat = false;
    for (size_t i = 0; i < attempts.size() && !module_sat; ++i) {
        std::vector<int> mod_order = move_mods;
        std::sort(mod_order.begin(), mod_order.end());
        do {
            place_move_modules(mod_order, attempts[i].pick_fn);
            if (!module_has_overlap()) {
                module_sat = true;
                break;
            }
        } while (std::next_permutation(mod_order.begin(), mod_order.end()));

        if (!module_sat && i + 1 < attempts.size()) {
            std::cout << "[Partition] fallback_merge_firstfit: overlap ["
                      << attempts[i].name << "] -> retry ["
                      << attempts[i + 1].name << "] T" << region.tier_id
                      << " d=" << region.depth << "\n";
        }
    }
    if (!module_sat) {
        std::cout << "[Partition] fallback_merge_firstfit: module UNSAT (all permutations fail) "
                  << "T" << region.tier_id << " d=" << region.depth << "\n";
    }

    for (int tid : frozen_tsvs) {
        const TSV& t = tsvs[tid];
        const double hw = tsv_w * 0.5;
        const double hh = tsv_h * 0.5;
        placed.push_back({ t.x - hw, t.y - hh, t.x + hw, t.y + hh });
    }

    auto first_free_x_tsv = [&](const std::vector<Rect>& p, double hw, double hh, double cy,
                                double preferred_x) -> double {
        double cx = std::max(preferred_x, region.xmin + hw);
        bool   moved = true;
        while (moved) {
            moved = false;
            for (const Rect& r : p) {
                if (cy - hh >= r.ry - 1e-9 || cy + hh <= r.ly + 1e-9) continue;
                if (cx - hw < r.rx - 1e-9 && cx + hw > r.lx + 1e-9) {
                    cx = r.rx + hw;
                    moved = true;
                }
            }
        }
        return (cx + hw > region.xmax + 1e-9) ? -1.0 : cx;
    };

    auto collect_y_cands = [&](double hh, double orig_y, const std::vector<Rect>& current_placed) {
        std::vector<double> ys;
        auto push_if_ok = [&](double y) {
            if (y - hh < region.ymin - 1e-9) return;
            if (y + hh > region.ymax + 1e-9) return;
            ys.push_back(y);
        };
        push_if_ok(clamp(orig_y, region.ymin + hh, region.ymax - hh));
        push_if_ok(region.ymin + hh);
        push_if_ok(region.ymax - hh);
        for (const Rect& r : current_placed) {
            push_if_ok(r.ry + hh);
            push_if_ok(r.ly - hh);
        }
        if (ys.empty()) return ys;
        std::sort(ys.begin(), ys.end());
        const double tol = std::max(1e-6, hh * 1e-3);
        ys.erase(std::unique(ys.begin(), ys.end(),
                              [&](double a, double b){ return std::fabs(a - b) < tol; }),
                 ys.end());
        std::sort(ys.begin(), ys.end(),
                  [&](double a, double b){ return std::fabs(a - orig_y) < std::fabs(b - orig_y); });
        return ys;
    };

    std::vector<int> tsv_order = move_tsvs;
    if (keep_left) {
        std::sort(tsv_order.begin(), tsv_order.end(),
                  [&](int a, int b){ return tsvs[a].x > tsvs[b].x; });
    } else {
        std::sort(tsv_order.begin(), tsv_order.end(),
                  [&](int a, int b){ return tsvs[a].x < tsvs[b].x; });
    }

    for (int tid : tsv_order) {
        TSV&         tsv     = tsvs[tid];
        const double hw_t    = tsv_w * 0.5;
        const double hh_t    = tsv_h * 0.5;
        const double orig_tx = tsv.x;
        const double orig_ty = tsv.y;

        auto y_cands = collect_y_cands(hh_t, orig_ty, placed);
        if (keep_left)
            std::sort(y_cands.begin(), y_cands.end(), std::greater<double>());

        bool   ok_any    = false;
        double best_cx = -1.0, best_cy = -1.0;
        double best_cost = std::numeric_limits<double>::infinity();

        for (double cy : y_cands) {
            double cx = -1.0;
            if (keep_left) {
                cx = first_free_x_from_right(placed, hw_t, hh_t, cy);
            } else {
                cx = first_free_x_tsv(placed, hw_t, hh_t, cy, orig_tx);
                if (cx < 0.0) cx = first_free_x_tsv(placed, hw_t, hh_t, cy, region.xmin);
            }
            if (cx < 0.0) continue;

            const double cost = std::abs(cx - orig_tx) + std::abs(cy - orig_ty);
            if (cost < best_cost) {
                best_cost = cost;
                best_cx   = cx;
                best_cy   = cy;
                ok_any    = true;
            }
        }

        if (ok_any) {
            tsv.x = best_cx;
            tsv.y = best_cy;
            placed.push_back({ tsv.x - hw_t, tsv.y - hh_t, tsv.x + hw_t, tsv.y + hh_t });
        } else {
            tsv.x = clamp(tsv.x, region.xmin + hw_t, region.xmax - hw_t);
            tsv.y = clamp(tsv.y, region.ymin + hh_t, region.ymax - hh_t);
            placed.push_back({ tsv.x - hw_t, tsv.y - hh_t, tsv.x + hw_t, tsv.y + hh_t });
        }
    }
}

// ============================================================
// log_partition_tree: DFS 把 partition tree 寫到 log 檔
// 格式（縮排表示深度）：
//   [T0 depth=0] region=[0.0,268.0]x[0.0,268.0] modules=100 tsvs=10
//     split_x at x=134.00
//     [T0 depth=1 LEFT] ...
//     [T0 depth=1 RIGHT] ...
// ============================================================
static void log_tree(const PartitionNode& node, std::ofstream& ofs, const std::string& indent)
{
    ofs << indent
        << "[T" << node.tier_id << " d=" << node.depth << "]"
        << " region=[" << std::fixed << std::setprecision(1)
        << node.xmin << "," << node.xmax << "]x["
        << node.ymin << "," << node.ymax << "]"
        << " mods=" << node.module_ids.size()
        << " tsvs=" << node.tsv_ids.size();

    if (node.is_leaf()) {
        ofs << " [LEAF]\n";
        return;
    }

    ofs << " split_" << (node.split_x ? "x" : "y")
        << "=" << std::setprecision(2) << node.split_pos << "\n";

    std::string child_indent = indent + "  ";
    if (node.left)  log_tree(*node.left,  ofs, child_indent + "L ");
    if (node.right) log_tree(*node.right, ofs, child_indent + "R ");
}

// ============================================================ 0318
// legalize_leaf:
//   對單一 leaf 區域內：
//   1) 窮舉 module 的「排法」（實作為 module 逐一放置的排列順序），
//      每個位置用 **first free y**（由下往上掃 y 候選）再 **first free x**
//      （由左往右，自 leaf.xmin+hw 起），不使用 preferred 對齊原點。
//      每一步可選 0°/90° 旋轉，以 DFS 窮舉完整合法解，最後取「總 L1 位移」最小者。
//   2) module 位置確定後，逐一放 TSV：
//      對每個 TSV 在不超出邊界的候選 y 中找可行 x，
//      以與原始位置的位移量（L1）最小者作為「最鄰近空位」。
// ============================================================
static void legalize_leaf(PartitionNode&              leaf,
                           std::vector<Module>&       modules,
                           std::vector<TSV>&          tsvs,
                           double                     tsv_w,
                           double                     tsv_h,
                           const std::vector<int>&    fixed_ids)
{
    struct Rect { double lx, ly, rx, ry; };

    auto clamp = [&](double v, double lo, double hi) -> double {
        return std::max(lo, std::min(hi, v));
    };

    // Module：由左緣起算 first free x（不使用原點偏好）
    auto first_free_x_from_left = [&](const std::vector<Rect>& p,
                                      double                     hw,
                                      double                     hh,
                                      double                     cy) -> double {
        double cx = leaf.xmin + hw;
        bool moved = true;

        while (moved) {
            moved = false;
            for (const Rect& r : p) {
                if (cy - hh >= r.ry - 1e-9 || cy + hh <= r.ly + 1e-9) continue;
                if (cx - hw < r.rx - 1e-9 && cx + hw > r.lx + 1e-9) {
                    cx = r.rx + hw;
                    moved = true;
                }
            }
        }

        return (cx + hw > leaf.xmax + 1e-9) ? -1.0 : cx;
    };

    // TSV：可從偏好 x 起算再向右推
    auto first_free_x = [&](const std::vector<Rect>& placed,
                             double               hw,
                             double               hh,
                             double               cy,
                             double               preferred_x) -> double {
        double cx = std::max(preferred_x, leaf.xmin + hw);
        bool moved = true;

        while (moved) {
            moved = false;
            for (const Rect& r : placed) {
                if (cy - hh >= r.ry - 1e-9 || cy + hh <= r.ly + 1e-9) continue;
                if (cx - hw < r.rx - 1e-9 && cx + hw > r.lx + 1e-9) {
                    cx = r.rx + hw;
                    moved = true;
                }
            }
        }

        return (cx + hw > leaf.xmax + 1e-9) ? -1.0 : cx;
    };

    std::vector<Rect> placed;
    placed.reserve(leaf.module_ids.size() + leaf.tsv_ids.size());

    // 從 per-tier fixed_ids 中取與 leaf 相交者當障礙物
    auto rebuild_fixed_obstacles = [&]() {
        for (int fid : fixed_ids) {
            const Module& fm = modules[fid];
            if (fm.rx() <= leaf.xmin + 1e-9 || fm.lx() >= leaf.xmax - 1e-9) continue;
            if (fm.ry() <= leaf.ymin + 1e-9 || fm.ly() >= leaf.ymax - 1e-9) continue;
            placed.push_back({ fm.lx(), fm.ly(), fm.rx(), fm.ry() });
        }
    };
    // fixed module 先加入 placed 作為障礙，不參與 first-fit 擺放
    rebuild_fixed_obstacles();

    // Module：y 由下往上掃描（邊界與已放矩形產生的候選高度）
    auto collect_y_levels_ascending = [&](double hh, const std::vector<Rect>& current_placed) {
        std::vector<double> ys;

        auto push_if_ok = [&](double y) {
            if (y - hh < leaf.ymin - 1e-9) return;
            if (y + hh > leaf.ymax + 1e-9) return;
            ys.push_back(y);
        };

        push_if_ok(leaf.ymin + hh);
        push_if_ok(leaf.ymax - hh);
        for (const Rect& r : current_placed) {
            push_if_ok(r.ry + hh);
            push_if_ok(r.ly - hh);
        }

        if (ys.empty()) return ys;

        std::sort(ys.begin(), ys.end());
        const double tol = std::max(1e-6, hh * 1e-3);
        ys.erase(std::unique(ys.begin(), ys.end(),
                              [&](double a, double b){ return std::fabs(a - b) < tol; }),
                 ys.end());
        return ys;
    };

    auto first_free_y_then_x = [&](const std::vector<Rect>& p,
                                   double hw, double hh) -> std::pair<double, double> {
        for (double cy : collect_y_levels_ascending(hh, p)) {
            double cx = first_free_x_from_left(p, hw, hh, cy);
            if (cx >= 0.0)
                return { cx, cy };
        }
        return { -1.0, -1.0 };
    };

    // TSV：依目前 placed 產生 y 候選（按離原始 y 最近）
    auto collect_y_cands = [&](double hh, double orig_y, const std::vector<Rect>& current_placed) {
        std::vector<double> ys;

        auto push_if_ok = [&](double y) {
            if (y - hh < leaf.ymin - 1e-9) return;
            if (y + hh > leaf.ymax + 1e-9) return;
            ys.push_back(y);
        };

        // 優先加入原始 y clamp（最接近原位）
        push_if_ok(clamp(orig_y, leaf.ymin + hh, leaf.ymax - hh));

        // 也加入區域底/頂（有時候會剛好卡出走道）
        push_if_ok(leaf.ymin + hh);
        push_if_ok(leaf.ymax - hh);

        for (const Rect& r : current_placed) {
            // 使兩個矩形在 y 方向「貼邊」（理論上最容易出現可行空位）
            push_if_ok(r.ry + hh);
            push_if_ok(r.ly - hh);
        }

        if (ys.empty()) return ys;

        // 去重
        std::sort(ys.begin(), ys.end());
        const double tol = std::max(1e-6, hh * 1e-3);
        ys.erase(std::unique(ys.begin(), ys.end(),
                              [&](double a, double b){ return std::fabs(a - b) < tol; }),
                 ys.end());

        // 按離原始 y 最近排序
        std::sort(ys.begin(), ys.end(),
                  [&](double a, double b){ return std::fabs(a - orig_y) < std::fabs(b - orig_y); });
        return ys;
    };

    // ============================================================
    // A) 模組：窮舉排列順序，選位移總和最小的可行排法
    //    fixed module 已在上方加入 placed，此處僅處理可動 module
    // ============================================================
    std::vector<int> movable_leaf_ids;
    movable_leaf_ids.reserve(leaf.module_ids.size());
    for (int mid : leaf.module_ids)
        if (!modules[mid].is_fixed) movable_leaf_ids.push_back(mid);

    const int n = static_cast<int>(movable_leaf_ids.size());
    if (n > 0) {
        std::vector<int> ids = movable_leaf_ids; // leaf 內可動 module 的全域索引

        std::vector<double> orig_x(n), orig_y(n);
        std::vector<double> w0(n), h0(n); // 用來支援 90 度旋轉（寬高互換）
        for (int i = 0; i < n; ++i) {
            const Module& m = modules[ids[i]];
            orig_x[i] = m.x;
            orig_y[i] = m.y;
            w0[i]      = m.width;
            h0[i]      = m.height;
        }

        std::vector<int> perm(n);
        std::iota(perm.begin(), perm.end(), 0);

        double best_cost = std::numeric_limits<double>::infinity();
        bool   best_found = false;
        std::vector<double> best_x(n), best_y(n);
        std::vector<char>   best_rot(n, 0);

        do {
            placed.clear();
            rebuild_fixed_obstacles();
            std::vector<double> temp_x(n, 0.0), temp_y(n, 0.0);
            std::vector<char>   temp_rot(n, 0);

            // 固定 permutation：每一步 first free y → first free x（自左），
            // 旋轉 0°/90° 用 DFS 窮舉；最後在「完整合法解」中取總 L1 位移最小。
            std::function<void(int, double)> dfs_place;
            dfs_place = [&](int depth, double acc_cost) {
                if (acc_cost >= best_cost - 1e-9)
                    return;
                if (depth == n) {
                    if (acc_cost < best_cost - 1e-9) {
                        best_cost = acc_cost;
                        best_found = true;
                        best_x     = temp_x;
                        best_y     = temp_y;
                        best_rot   = temp_rot;
                    }
                    return;
                }

                const int local_i = perm[depth];
                const double ox = orig_x[local_i];
                const double oy = orig_y[local_i];

                for (int rot = 0; rot <= 1; ++rot) {
                    const double w_i  = (rot == 0) ? w0[local_i] : h0[local_i];
                    const double h_i  = (rot == 0) ? h0[local_i] : w0[local_i];
                    const double hw_i = w_i * 0.5;
                    const double hh_i = h_i * 0.5;

                    if (leaf.ymin + hh_i > leaf.ymax - hh_i + 1e-12)
                        continue;

                    const auto xy = first_free_y_then_x(placed, hw_i, hh_i);
                    const double cx = xy.first;
                    const double cy = xy.second;
                    if (cx < 0.0)
                        continue;

                    const double inc = std::abs(cx - ox) + std::abs(cy - oy);
                    placed.push_back({ cx - hw_i, cy - hh_i, cx + hw_i, cy + hh_i });
                    temp_x[local_i] = cx;
                    temp_y[local_i] = cy;
                    temp_rot[local_i] = static_cast<char>(rot);

                    dfs_place(depth + 1, acc_cost + inc);

                    placed.pop_back();
                }
            };

            dfs_place(0, 0.0);

        } while (std::next_permutation(perm.begin(), perm.end()));

        if (best_found) {
            // 寫回 module 位置，並重建 placed
            placed.clear();
            rebuild_fixed_obstacles();
            for (int i = 0; i < n; ++i) {
                Module& m = modules[ids[i]];
                m.x = best_x[i];
                m.y = best_y[i];
                const bool rot = (best_rot[i] != 0);
                m.width  = rot ? h0[i] : w0[i];
                m.height = rot ? w0[i] : h0[i];
                placed.push_back({ m.lx(), m.ly(), m.rx(), m.ry() });
            }
        } else {
            // 若完全找不到可行排法，回退到簡單的 x 升序 first-fit（至少確保在邊界內）
            std::vector<int> greedy_order = ids;
            std::sort(greedy_order.begin(), greedy_order.end(),
                      [&](int a, int b){ return modules[a].x < modules[b].x; });

            placed.clear();
            rebuild_fixed_obstacles();
            for (int mid : greedy_order) {
                Module& m = modules[mid];

                // 嘗試兩種旋轉，選擇「模組本身 displacement L1 最小」且可行者
                const double base_w = m.width;
                const double base_h = m.height;
                const double orig_x_m = m.x;
                const double orig_y_m = m.y;

                bool   ok = false;
                double best_cx = -1.0, best_cy = -1.0;
                double best_w = base_w, best_h = base_h;
                double best_cost_m = std::numeric_limits<double>::infinity();

                for (int rot = 0; rot <= 1; ++rot) {
                    const double w_i = (rot == 0) ? base_w : base_h;
                    const double h_i = (rot == 0) ? base_h : base_w;
                    const double hw_i = w_i * 0.5;
                    const double hh_i = h_i * 0.5;

                    if (leaf.ymin + hh_i > leaf.ymax - hh_i + 1e-12) continue;

                    const auto xy = first_free_y_then_x(placed, hw_i, hh_i);
                    if (xy.first < 0.0) continue;

                    const double cost_m =
                        std::abs(xy.first - orig_x_m) + std::abs(xy.second - orig_y_m);
                    if (cost_m < best_cost_m) {
                        best_cost_m = cost_m;
                        best_cx = xy.first;
                        best_cy = xy.second;
                        best_w = w_i;
                        best_h = h_i;
                        ok = true;
                    }
                }

                if (ok) {
                    m.x = best_cx;
                    m.y = best_cy;
                    m.width = best_w;
                    m.height = best_h;
                    placed.push_back({ m.lx(), m.ly(), m.rx(), m.ry() });
                } else {
                    // 幾何上無可行 first-fit 時，優先選擇能放進 leaf bbox 的朝向（含旋轉）。
                    const double box_w = leaf.xmax - leaf.xmin;
                    const double box_h = leaf.ymax - leaf.ymin;
                    const bool fit0  = (base_w <= box_w + 1e-12 && base_h <= box_h + 1e-12);
                    const bool fit90 = (base_h <= box_w + 1e-12 && base_w <= box_h + 1e-12);
                    const bool use_rot = (!fit0 && fit90);

                    const double w_sel = use_rot ? base_h : base_w;
                    const double h_sel = use_rot ? base_w : base_h;
                    const double hw_i = w_sel * 0.5;
                    const double hh_i = h_sel * 0.5;
                    m.width = w_sel;
                    m.height = h_sel;
                    m.x = clamp(m.x, leaf.xmin + hw_i, leaf.xmax - hw_i);
                    m.y = clamp(m.y, leaf.ymin + hh_i, leaf.ymax - hh_i);
                    placed.push_back({ m.lx(), m.ly(), m.rx(), m.ry() });
                }
            }
        }
    }

    // ============================================================
    // B) TSV：module 固定後，逐一放到「最鄰近且不重疊」的空位
    // ============================================================
    if (!leaf.tsv_ids.empty()) {
        std::vector<int> tsv_order = leaf.tsv_ids;
        std::sort(tsv_order.begin(), tsv_order.end(),
                  [&](int a, int b){ return tsvs[a].x < tsvs[b].x; });

        for (int tid : tsv_order) {
            TSV& tsv = tsvs[tid];

            const double hw_t = tsv_w * 0.5;
            const double hh_t = tsv_h * 0.5;
            const double orig_tx = tsv.x;
            const double orig_ty = tsv.y;

            // 先產生候選 y（依貼邊可行性），再在每個 y 下找可行 cx
            auto y_cands = collect_y_cands(hh_t, orig_ty, placed);

            bool   ok_any = false;
            double best_cx = -1.0, best_cy = -1.0;
            double best_cost = std::numeric_limits<double>::infinity();

            for (double cy : y_cands) {
                double cx = first_free_x(placed, hw_t, hh_t, cy, orig_tx);
                if (cx < 0.0) cx = first_free_x(placed, hw_t, hh_t, cy, leaf.xmin);
                if (cx < 0.0) continue;

                const double cost = std::abs(cx - orig_tx) + std::abs(cy - orig_ty);
                if (cost < best_cost) {
                    best_cost = cost;
                    best_cx = cx;
                    best_cy = cy;
                    ok_any = true;
                }
            }

            if (ok_any) {
                tsv.x = best_cx;
                tsv.y = best_cy;
                placed.push_back({ tsv.x - hw_t, tsv.y - hh_t, tsv.x + hw_t, tsv.y + hh_t });
            } else {
                // 理論上如果幾何不可行，仍至少夾到邊界，並交給後續評估
                tsv.x = clamp(tsv.x, leaf.xmin + hw_t, leaf.xmax - hw_t);
                tsv.y = clamp(tsv.y, leaf.ymin + hh_t, leaf.ymax - hh_t);
                placed.push_back({ tsv.x - hw_t, tsv.y - hh_t, tsv.x + hw_t, tsv.y + hh_t });
            }
        }
    }
}

// ============================================================
// PlacementEngine::partition_all_tiers
//   對每個 tier 各建一個根節點，遞迴 partition，最後 log 輸出。
// ============================================================
void PlacementEngine::partition_all_tiers(const PartitionConfig& pcfg)
{
    std::ofstream log_ofs;
    if (pcfg.log_tree) {
        log_ofs.open(pcfg.log_file);
        if (!log_ofs)
            std::cerr << "[Partition] Cannot open log file: " << pcfg.log_file << "\n";
    }

    int num_tiers = num_dies();
    int total_leaves = 0;

    for (int t = 0; t < num_tiers; ++t) {
        const Die& die = dies_[t];

        // 建立根節點
        PartitionNode root;
        root.tier_id = t;
        root.depth   = 0;
        root.xmin    = 0.0;   root.xmax = die.width;
        root.ymin    = 0.0;   root.ymax = die.height;

        // 分離 fixed 與 movable module：
        // fixed 不進 partition 集合，只作為全域障礙物；movable 才參與 partition
        std::vector<int> fixed_ids;
        for (const Module& m : modules_) {
            if (m.is_terminal || m.tier_id != t) continue;
            if (m.is_fixed)
                fixed_ids.push_back(m.id);
            else
                root.module_ids.push_back(m.id);
        }

        // 蒐集此 tier 的 TSV（tier below == t，即 TSV 在 tier t 與 t+1 之間的介面）
        for (const TSV& tsv : tsvs_) {
            if (tsv.layer_index == t)
                root.tsv_ids.push_back(tsv.id);
        }

        int init_mods = static_cast<int>(root.module_ids.size());
        int init_tsvs = static_cast<int>(root.tsv_ids.size());

        std::cout << "[Partition] Tier " << t
                  << ": " << init_mods << " modules, "
                  << init_tsvs << " TSVs\n";

        // ---- 遞迴 partition ----
        partition(root, modules_, tsvs_, pcfg, fixed_ids);

        // ---- Leaf legalization：在每個 leaf 內窮舉排法並避免重疊 ---- 0318
        std::function<void(PartitionNode&)> dfs_legalize;
        dfs_legalize = [&](PartitionNode& node) {
            if (node.is_leaf()) {
                if (!node.skip_leaf_legalize)
                    legalize_leaf(node, modules_, tsvs_, pcfg.tsv_width, pcfg.tsv_height, fixed_ids);
                return;
            }
            if (node.left)  dfs_legalize(*node.left);
            if (node.right) dfs_legalize(*node.right);
        };
        dfs_legalize(root);

        // ---- Post-legalize retry：擴展至 parent bbox 重排仍有重疊的 leaf ----
        //
        // 在第一輪 dfs_legalize 完成後，掃描所有仍有重疊的葉節點。
        // 對每個失敗 leaf：
        //   1. 以其 parent 節點的 bounding box 作為新的 legalization 區域
        //   2. 此 tier 中「不屬於本 leaf」的所有其他可動 module 全部加入 fixed 障礙物
        //   3. 在擴大後的區域重新呼叫 legalize_leaf
        //
        // DFS 時同步攜帶 parent bbox（四個 double），root 的 parent bbox = 自己的 bbox
        struct FailLeaf {
            PartitionNode* node;
            double par_xmin, par_ymin, par_xmax, par_ymax;
        };
        std::vector<FailLeaf> fail_leaves;

        std::function<void(PartitionNode&, double, double, double, double)> collect_fail;
        collect_fail = [&](PartitionNode& nd,
                           double pxmin, double pymin,
                           double pxmax, double pymax) {
            if (nd.is_leaf()) {
                const bool has_ov = entity_group_has_overlap(
                    nd.module_ids, nd.tsv_ids, modules_, tsvs_,
                    pcfg.tsv_width, pcfg.tsv_height, fixed_ids);
                if (has_ov)
                    fail_leaves.push_back({ &nd, pxmin, pymin, pxmax, pymax });
                return;
            }
            // 本節點的 bbox 作為子節點的 parent bbox
            if (nd.left)  collect_fail(*nd.left,  nd.xmin, nd.ymin, nd.xmax, nd.ymax);
            if (nd.right) collect_fail(*nd.right, nd.xmin, nd.ymin, nd.xmax, nd.ymax);
        };
        collect_fail(root, root.xmin, root.ymin, root.xmax, root.ymax);

        if (!fail_leaves.empty()) {
            std::cout << "[Legalize] post_retry: " << fail_leaves.size()
                      << " overlapping leaf(ves) in tier " << t << "\n";

            for (FailLeaf& fl : fail_leaves) {
                PartitionNode& leaf = *fl.node;

                // 建立 extended_fixed = 原有 fixed + 此 tier 中不屬於本 leaf 的所有可動 module
                std::vector<int> ext_fixed = fixed_ids;
                for (const Module& m : modules_) {
                    if (m.is_terminal || m.tier_id != t || m.is_fixed) continue;
                    // 判斷是否屬於本 leaf（leaf_threshold 通常很小，線性掃描即可）
                    bool in_leaf = false;
                    for (int lid : leaf.module_ids)
                        if (lid == m.id) { in_leaf = true; break; }
                    if (!in_leaf)
                        ext_fixed.push_back(m.id);
                }

                // 以 parent bbox 建立暫時葉節點
                PartitionNode parent_region;
                parent_region.tier_id    = leaf.tier_id;
                parent_region.depth      = leaf.depth;
                parent_region.xmin       = fl.par_xmin;
                parent_region.xmax       = fl.par_xmax;
                parent_region.ymin       = fl.par_ymin;
                parent_region.ymax       = fl.par_ymax;
                parent_region.module_ids = leaf.module_ids;
                parent_region.tsv_ids    = leaf.tsv_ids;

                std::cout << "[Legalize] post_retry T" << leaf.tier_id
                          << " d=" << leaf.depth
                          << " mods=" << leaf.module_ids.size()
                          << " leaf=[" << std::fixed << std::setprecision(1)
                          << leaf.xmin << "," << leaf.xmax
                          << "]x[" << leaf.ymin << "," << leaf.ymax << "]"
                          << " parent=[" << fl.par_xmin << "," << fl.par_xmax
                          << "]x[" << fl.par_ymin << "," << fl.par_ymax << "]\n"
                          << std::defaultfloat;

                legalize_leaf(parent_region, modules_, tsvs_,
                              pcfg.tsv_width, pcfg.tsv_height, ext_fixed);

                const bool still_ov = entity_group_has_overlap(
                    leaf.module_ids, leaf.tsv_ids, modules_, tsvs_,
                    pcfg.tsv_width, pcfg.tsv_height, fixed_ids);
                std::cout << "[Legalize] post_retry -> "
                          << (still_ov ? "STILL overlapping" : "SUCCESS") << "\n";
            }
        }

        // ---- 統計葉節點數 ----
        std::function<int(const PartitionNode&)> count_leaves;
        count_leaves = [&](const PartitionNode& n) -> int {
            if (n.is_leaf()) return 1;
            return count_leaves(*n.left) + count_leaves(*n.right);
        };
        int leaves = count_leaves(root);
        total_leaves += leaves;
        std::cout << "[Partition] Tier " << t << " -> " << leaves << " leaf regions\n";

        // ---- 寫 log ----
        if (pcfg.log_tree && log_ofs) {
            log_ofs << "====== Tier " << t << " ======\n";
            log_tree(root, log_ofs, "");
            log_ofs << "\n";
        }
    }

    std::cout << "[Partition] Done. Total leaf regions = " << total_leaves << "\n";
    if (pcfg.log_tree && log_ofs)
        std::cout << "[Partition] Tree log -> " << pcfg.log_file << "\n";

    // ---- 輸出 shift 後的 module / TSV 位置 ----
    // 需求：輸出格式需與 PA2 的 n100_output.txt 完全一致，
    // 因此直接重用 write_output()（其內已包含 module 列表與 NumTsvAssignments）。runtime 先填 0。
    if (pcfg.write_positions) {
        write_output(pcfg.positions_file, 0.0);
        std::cout << "[Partition] Positions -> " << pcfg.positions_file << "\n";
    }
}
