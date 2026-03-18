// 3D IC Analytical Floorplanner - Recursive Bi-partitioning
//
// 流程：
//   1. partition_all_tiers()：對每個 tier 建立根節點，遞迴呼叫 partition()
//   2. partition()：
//      a. 若 module 數 ≤ leaf_threshold → 停止（葉節點）
//      b. 選切割軸（較長邊，長寬比 ≈ 1 時評估兩軸）
//      c. find_best_split()：掃線找最小 cross-area split，須滿足面積容量限制
//      d. shift_modules()：跨線的 module/TSV 推到較大面積那側，
//         並更新座標貼齊切割線
//      e. 建立子節點，遞迴
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
// find_best_split:
//   在 [region_lo + min_ratio*span, region_lo + max_ratio*span]
//   均勻取 num_candidates 個候選切割線，對每條線評估：
//     1. cross_area：被切割線穿過的 module/TSV 中，「較小側」的面積之和
//     2. 面積容量：兩子區域的 available_area 均 ≥ 0
//   回傳令 cross_area 最小且滿足容量限制的切割座標。
//   若全部候選都違反容量，退而選最中間的（避免陷入死迴圈）。
//
// split_x = true：沿 X 方向切割（切割線平行 Y 軸，回傳 X 座標）
// split_x = false：沿 Y 方向切割（切割線平行 X 軸，回傳 Y 座標）
// ============================================================
static double find_best_split(PartitionNode&            node,
                              const std::vector<Module>& modules,
                              const std::vector<TSV>&    tsvs,
                              const PartitionConfig&     pcfg,
                              bool                       split_x)
{
    double lo   = split_x ? node.xmin : node.ymin;
    double hi   = split_x ? node.xmax : node.ymax;
    double span = hi - lo;

    double L_min = lo + pcfg.min_split_ratio * span;
    double L_max = lo + pcfg.max_split_ratio * span;

    // 均勻取候選切割線
    int    nc    = std::max(2, pcfg.num_candidates);
    double step  = (L_max - L_min) / (nc - 1);

    double best_L      = (lo + hi) * 0.5;
    double best_cross  = 1e30;
    bool   found_valid = false;

    for (int k = 0; k < nc; ++k) {
        double L = L_min + k * step;

        // ----- 先以中心點分類左右，並檢查最少 module 數 -----
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

        // 強制每個子區域至少有 pcfg.min_modules_per_region 個 modules
        if (left_mod_cnt < pcfg.min_modules_per_region ||
            right_mod_cnt < pcfg.min_modules_per_region) {
            continue;
        }

        // TSV 面積與左右 used 累加（TSV 不計入「至少幾個 module」的門檻）
        for (int tid : node.tsv_ids) {
            double ctr = split_x ? tsvs[tid].x : tsvs[tid].y;
            double a   = pcfg.tsv_width * pcfg.tsv_height;
            if (ctr <= L) used_left  += a;
            else          used_right += a;
        }

        // ----- 子區域幾何尺寸限制：寬/高不得小於子區域內任何 module 的短邊 -----
        // 左/右子區域尺寸
        double left_w  = split_x ? (L - node.xmin) : node.width();
        double right_w = split_x ? (node.xmax - L) : node.width();
        double left_h  = split_x ? node.height()   : (L - node.ymin);
        double right_h = split_x ? node.height()   : (node.ymax - L);

        bool geom_ok =
            (left_w  + 1e-12 >= left_max_short)  && (left_h  + 1e-12 >= left_max_short) &&
            (right_w + 1e-12 >= right_max_short) && (right_h + 1e-12 >= right_max_short);
        if (!geom_ok) continue;

        // ----- 計算 cross-boundary area -----
        double cross_area = 0.0;

        for (int mid : node.module_ids) {
            const Module& m  = modules[mid];
            double m_lo = split_x ? m.lx() : m.ly();
            double m_hi = split_x ? m.rx() : m.ry();
            if (m_lo < L && m_hi > L) {
                // 被切割線穿過
                double left_part  = L - m_lo;
                double right_part = m_hi - L;
                double smaller    = std::min(left_part, right_part);
                // cross area = smaller_dim × perpendicular_dim
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

        // ----- 檢查面積容量 -----
        double left_area  = split_x ? (L - node.xmin) * node.height()
                                    : node.width() * (L - node.ymin);
        double right_area = split_x ? (node.xmax - L) * node.height()
                                    : node.width() * (node.ymax - L);

        bool capacity_ok = (left_area  >= used_left ) &&
                           (right_area >= used_right);

        if (capacity_ok && cross_area < best_cross) {
            best_cross = cross_area;
            best_L     = L;
            found_valid = true;
        }
    }

    // 若無滿足容量的選項，退而選中點（避免無限遞迴）
    if (!found_valid)
        best_L = (lo + hi) * 0.5;

    return best_L;
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
// partition: 遞迴 Bi-partitioning 主體
// ============================================================
static void partition(PartitionNode&         node,
                      std::vector<Module>&    modules,
                      std::vector<TSV>&       tsvs,
                      const PartitionConfig&  pcfg)
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

    // ---- 找最佳切割線 ----
    double L = find_best_split(node, modules, tsvs, pcfg, split_x);

    // ---- 第一次：以 best L 做 push-back，確立左右集合 ----
    shift_modules(node, modules, tsvs, pcfg, L, split_x);

    // ---- 依目前 L 分配左右集合（用中心點）----
    std::vector<int> left_mods, right_mods, left_tsvs, right_tsvs;
    left_mods.reserve(node.module_ids.size());
    right_mods.reserve(node.module_ids.size());
    left_tsvs.reserve(node.tsv_ids.size());
    right_tsvs.reserve(node.tsv_ids.size());

    double used_left = 0.0, used_right = 0.0;
    for (int mid : node.module_ids) {
        double ctr = split_x ? modules[mid].x : modules[mid].y;
        if (ctr <= L) { left_mods.push_back(mid);  used_left  += modules[mid].area(); }
        else          { right_mods.push_back(mid); used_right += modules[mid].area(); }
    }
    for (int tid : node.tsv_ids) {
        double ctr = split_x ? tsvs[tid].x : tsvs[tid].y;
        double a   = pcfg.tsv_width * pcfg.tsv_height;
        if (ctr <= L) { left_tsvs.push_back(tid);  used_left  += a; }
        else          { right_tsvs.push_back(tid); used_right += a; }
    }

    // ---- Re-balance：調整切割線到左右使用率相等的位置 ----
    // 目標：used_left / area_left == used_right / area_right
    // 對垂直切：area_left  = (L-xmin)*H, area_right = (xmax-L)*H
    // 解得：L = (used_left*xmax + used_right*xmin) / (used_left+used_right)
    // 水平切同理（以 ymin/ymax）
    double lo   = split_x ? node.xmin : node.ymin;
    double hi   = split_x ? node.xmax : node.ymax;
    double span = hi - lo;
    double L_min = lo + pcfg.min_split_ratio * span;
    double L_max = lo + pcfg.max_split_ratio * span;

    double denom = used_left + used_right;
    if (denom > 1e-12) {
        double L_bal = split_x
            ? (used_left * node.xmax + used_right * node.xmin) / denom
            : (used_left * node.ymax + used_right * node.ymin) / denom;
        L_bal = std::max(L_min, std::min(L_max, L_bal));

        // 檢查容量限制（兩邊 area 必須 ≥ used）
        double left_area  = split_x ? (L_bal - node.xmin) * node.height()
                                    : node.width() * (L_bal - node.ymin);
        double right_area = split_x ? (node.xmax - L_bal) * node.height()
                                    : node.width() * (node.ymax - L_bal);
        bool ok = (left_area >= used_left) && (right_area >= used_right);

        // Re-balance 也要通過幾何尺寸限制（子區域寬/高 ≥ 該側 module 短邊最大值）
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

        double left_w  = split_x ? (L_bal - node.xmin) : node.width();
        double right_w = split_x ? (node.xmax - L_bal) : node.width();
        double left_h  = split_x ? node.height()       : (L_bal - node.ymin);
        double right_h = split_x ? node.height()       : (node.ymax - L_bal);
        bool geom_ok =
            (left_w  + 1e-12 >= left_max_short)  && (left_h  + 1e-12 >= left_max_short) &&
            (right_w + 1e-12 >= right_max_short) && (right_h + 1e-12 >= right_max_short);

        if (ok && geom_ok && std::fabs(L_bal - L) > 1e-9) {
            L = L_bal;
            // 重新 push-back 到新的切割線，並重算左右集合（保持幾何一致）
            shift_modules(node, modules, tsvs, pcfg, L, split_x);
            left_mods.clear(); right_mods.clear(); left_tsvs.clear(); right_tsvs.clear();
            used_left = 0.0; used_right = 0.0;
            for (int mid : node.module_ids) {
                double ctr = split_x ? modules[mid].x : modules[mid].y;
                if (ctr <= L) { left_mods.push_back(mid);  used_left  += modules[mid].area(); }
                else          { right_mods.push_back(mid); used_right += modules[mid].area(); }
            }
            for (int tid : node.tsv_ids) {
                double ctr = split_x ? tsvs[tid].x : tsvs[tid].y;
                double a   = pcfg.tsv_width * pcfg.tsv_height;
                if (ctr <= L) { left_tsvs.push_back(tid);  used_left  += a; }
                else          { right_tsvs.push_back(tid); used_right += a; }
            }
        }
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

    // ---- 遞迴 ----
    partition(*left_node,  modules, tsvs, pcfg);
    partition(*right_node, modules, tsvs, pcfg);

    node.left  = std::move(left_node);
    node.right = std::move(right_node);
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

        // 蒐集此 tier 的可移動 module
        for (const Module& m : modules_) {
            if (!m.is_terminal && m.tier_id == t)
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
        partition(root, modules_, tsvs_, pcfg);

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
