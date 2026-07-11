// 3D IC Analytical Floorplanner - 位移分析工具實作
#include "util.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

// stb_image_write 實作在 floorplanner.cpp 中已 define；
// 這裡只引入宣告即可（連結時共用符號，不重複 define）
#include "stb_image_write.h"
std::vector<ModuleSnapshot> record_positions(const PlacementEngine& engine)
{
    std::vector<ModuleSnapshot> snap;
    if constexpr (!ENABLE_DISPLACEMENT_REPORT) return snap;

    // 預先計算每個 module 連接的 net 數量
    const auto& nets    = engine.nets();
    const auto& modules = engine.modules();
    std::vector<int> net_count(modules.size(), 0);
    for (const Net& n : nets)
        for (int pid : n.pins)
            if (pid >= 0 && pid < static_cast<int>(net_count.size()))
                ++net_count[pid];

    for (const Module& m : modules) {
        if (m.is_terminal) continue;
        snap.push_back({ m.id, m.name, m.x, m.y, m.area(), net_count[m.id] });
    }
    return snap;
}

void write_displacement_report(
    const std::vector<ModuleSnapshot>& before,
    const std::vector<ModuleSnapshot>& after,
    const std::string&                 filepath,
    double                             inv_geometry_scale)
{
    if constexpr (!ENABLE_DISPLACEMENT_REPORT) return;
    if (before.empty() || after.empty()) return;

    const double s  = inv_geometry_scale;
    const double s2 = s * s;

    struct Entry { std::string name; double disp; double ax, ay, bx, by; double area; int num_nets; };
    std::vector<Entry> entries;
    entries.reserve(before.size());

    for (const auto& b : before) {
        for (const auto& a : after) {
            if (a.id != b.id) continue;
            const double dx   = a.x - b.x;
            const double dy   = a.y - b.y;
            const double dist = std::sqrt(dx * dx + dy * dy) * s;
            entries.push_back({
                b.name, dist,
                a.x * s, a.y * s, b.x * s, b.y * s,
                b.area * s2, b.num_nets });
            break;
        }
    }

    auto print_section = [&](std::ofstream& ofs, const std::string& header,
                              std::function<bool(const Entry&, const Entry&)> cmp)
    {
        std::sort(entries.begin(), entries.end(), cmp);
        ofs << header << "\n";
        for (const auto& e : entries) {
            ofs << e.name
                << "  " << e.disp
                << "  " << e.area
                << "  " << e.num_nets
                << "  " << (e.disp * e.area)
                << "  " << (e.disp * e.num_nets)
                << "  " << e.ax << "  " << e.ay
                << "  " << e.bx << "  " << e.by
                << "\n";
        }
    };

    const std::string col_header =
        "# module_name  displacement  area  num_nets  disp*area  disp*num_nets"
        "  after_x  after_y  before_x  before_y";

    std::ofstream ofs(filepath);
    if (!ofs) {
        std::cerr << "[Displacement] Cannot open " << filepath << "\n";
        return;
    }
    ofs << std::fixed;

    // --- 第一段：依 displacement 排序 ---
    print_section(ofs,
        "# [Sort by displacement]\n" + col_header,
        [](const Entry& a, const Entry& b){ return a.disp > b.disp; });

    ofs << "\n";

    // --- 第二段：依 displacement * area 排序 ---
    print_section(ofs,
        "# [Sort by displacement * area]\n" + col_header,
        [](const Entry& a, const Entry& b){ return (a.disp * a.area) > (b.disp * b.area); });

    ofs << "\n";

    // --- 第三段：依 displacement * num_nets 排序 ---
    print_section(ofs,
        "# [Sort by displacement * num_nets]\n" + col_header,
        [](const Entry& a, const Entry& b){ return (a.disp * a.num_nets) > (b.disp * b.num_nets); });

    double disp_sum = 0.0;
    for (const auto& e : entries)
        disp_sum += e.disp;
    const double avg_disp = entries.empty() ? 0.0 : disp_sum / static_cast<double>(entries.size());
    ofs << "\n# average_displacement: " << avg_disp << "\n";

    std::cout << "[Displacement] Report -> " << filepath
              << "  (" << entries.size() << " modules, avg=" << avg_disp << ")\n";
}

// ============================================================
// compute_intra_die_hpwl
// ============================================================
IntraDieStats compute_intra_die_hpwl(const PlacementEngine& engine)
{
    const auto& nets    = engine.nets();
    const auto& modules = engine.modules();
    const auto& weights = engine.tier_net_weights();
    const int   nd      = engine.num_dies();

    IntraDieStats s;
    s.total_hpwl  = 0.0;
    s.tier_hpwl.assign(nd, 0.0);
    s.tier_count.assign(nd, 0);

    for (const Net& net : nets) {
        // 判斷所有 pin 是否都在同一層
        int  common_tier = -2;
        bool cross       = false;
        for (int pid : net.pins) {
            const Module& m = modules[pid];
            int t = m.is_terminal ? 0 : m.tier_id;
            if (common_tier == -2) { common_tier = t; }
            else if (common_tier != t) { cross = true; break; }
        }
        if (cross || common_tier < 0) continue;

        // 計算此 net 的 2D HPWL
        double x_min =  1e18, x_max = -1e18;
        double y_min =  1e18, y_max = -1e18;
        for (int pid : net.pins) {
            const Module& m = modules[pid];
            x_min = std::min(x_min, m.x);
            x_max = std::max(x_max, m.x);
            y_min = std::min(y_min, m.y);
            y_max = std::max(y_max, m.y);
        }
        if (x_min > x_max) continue;

        // die weight（來源與 compute_hpwl() 相同）
        double w = 1.0;
        if (common_tier >= 0 && common_tier < nd &&
            static_cast<int>(weights.size()) == nd)
            w = weights[static_cast<size_t>(common_tier)];

        const double hpwl = w * ((x_max - x_min) + (y_max - y_min));
        s.total_hpwl += hpwl;
        if (common_tier >= 0 && common_tier < nd) {
            s.tier_hpwl[common_tier]  += hpwl;
            s.tier_count[common_tier] += 1;
        }
    }
    return s;
}

// ============================================================
// print_intra_die_stats
// ============================================================
void print_intra_die_stats(const IntraDieStats& stats, std::ostream& os)
{
    os << "  Intra-die HPWL (same-tier nets only): "
       << stats.total_hpwl << "\n";
    const int nd = static_cast<int>(stats.tier_hpwl.size());
    for (int t = 0; t < nd; ++t) {
        const int    cnt = stats.tier_count[t];
        const double h   = stats.tier_hpwl[t];
        os << "    Die " << t
           << "  HPWL=" << h
           << "  nets=" << cnt
           << "  avg=" << (cnt > 0 ? h / cnt : 0.0) << "\n";
    }
}

// ============================================================
// print_overlap_report
// ============================================================
void print_overlap_report(const PlacementEngine& engine,
                          double                 tsv_width,
                          double                 tsv_height,
                          std::ostream&          os)
{
    struct ItemRect {
        char   kind; // 'M' = Module, 'T' = TSV
        int    id;
        double lx, ly, rx, ry;
    };

    auto overlaps = [](const ItemRect& a, const ItemRect& b) -> bool {
        constexpr double eps = 1e-9;
        if (a.rx <= b.lx + eps || b.rx <= a.lx + eps) return false;
        if (a.ry <= b.ly + eps || b.ry <= a.ly + eps) return false;
        return true;
    };

    const double tsv_hw = tsv_width  * 0.5;
    const double tsv_hh = tsv_height * 0.5;

    for (int t = 0; t < engine.num_dies(); ++t) {
        std::vector<ItemRect> items;

        for (const Module& m : engine.modules()) {
            if (!m.is_terminal && m.tier_id == t)
                items.push_back({ 'M', m.id, m.lx(), m.ly(), m.rx(), m.ry() });
        }
        for (const TSV& tsv : engine.tsvs()) {
            if (tsv.layer_index == t)
                items.push_back({ 'T', tsv.id,
                                   tsv.x - tsv_hw, tsv.y - tsv_hh,
                                   tsv.x + tsv_hw, tsv.y + tsv_hh });
        }

        int ov_count = 0;
        for (size_t i = 0; i < items.size(); ++i)
            for (size_t j = i + 1; j < items.size(); ++j)
                if (overlaps(items[i], items[j])) ++ov_count;

        if (ov_count == 0)
            os << "  Tier " << t << " overlaps: none\n";
        else
            os << "  Tier " << t << " overlaps: " << ov_count << " pairs\n";
    }
}

// ============================================================
// BinEdgeDemands primitives
// ============================================================

void bin_edge_clear(BinEdgeDemands& d, const std::vector<Die>& dies)
{
    const int nd = static_cast<int>(dies.size());
    d.H.resize(static_cast<size_t>(nd));
    d.V.resize(static_cast<size_t>(nd));
    for (int t = 0; t < nd; ++t) {
        const Die& die = dies[t];
        d.H[t].assign(static_cast<size_t>((die.bin_rows + 1) * die.bin_cols), 0.0);
        d.V[t].assign(static_cast<size_t>((die.bin_cols + 1) * die.bin_rows), 0.0);
    }
}

void bin_edge_accumulate_pair(const Die& die,
                               double x1, double y1,
                               double x2, double y2,
                               std::vector<double>& H,
                               std::vector<double>& V)
{
    const double bw = die.bin_w;
    const double bh = die.bin_h;
    const int    R  = die.bin_rows;
    const int    C  = die.bin_cols;

    const double xlo = std::max(0.0, std::min(x1, x2));
    const double xhi = std::min(die.width,  std::max(x1, x2));
    const double ylo = std::max(0.0, std::min(y1, y2));
    const double yhi = std::min(die.height, std::max(y1, y2));

    const int hc_lo = static_cast<int>(std::floor(xlo / bw));
    const int hc_hi = static_cast<int>(std::ceil(xhi / bw)) - 1;
    const int hr_lo = static_cast<int>(std::ceil(ylo / bh));
    const int hr_hi = static_cast<int>(std::floor(yhi / bh));
    const int vr_lo = static_cast<int>(std::floor(ylo / bh));
    const int vr_hi = static_cast<int>(std::ceil(yhi / bh)) - 1;
    const int vc_lo = static_cast<int>(std::ceil(xlo / bw));
    const int vc_hi = static_cast<int>(std::floor(xhi / bw));

    const int    col_span = std::max(1, hc_hi - hc_lo + 1);
    const int    row_span = std::max(1, vr_hi - vr_lo + 1);
    const double h_delta  = 1.0 / col_span;
    const double v_delta  = 1.0 / row_span;
    const double h_pair_delta = h_delta * 0.5;
    const double v_pair_delta = v_delta * 0.5;

    // 若兩 pin 在同一 bin row（hr_lo > hr_hi），累加該 bin 的上下兩條邊
    if (hr_lo > hr_hi) {
        const int bin_row = static_cast<int>(std::floor(0.5 * (ylo + yhi) / bh));
        const int hr_bot  = std::clamp(bin_row, 0, R);
        const int hr_top  = std::clamp(bin_row + 1, 0, R);
        const int hc0 = std::clamp(hc_lo, 0, C - 1);
        const int hc1 = std::clamp(hc_hi, 0, C - 1);
        for (int hc = hc0; hc <= hc1; ++hc) {
            H[static_cast<size_t>(hr_bot * C + hc)] += h_pair_delta;
            if (hr_top != hr_bot)
                H[static_cast<size_t>(hr_top * C + hc)] += h_pair_delta;
        }
    } else {
        const int hr0 = std::clamp(hr_lo, 0, R);
        const int hr1 = std::clamp(hr_hi, 0, R);
        const int hc0 = std::clamp(hc_lo, 0, C - 1);
        const int hc1 = std::clamp(hc_hi, 0, C - 1);
        for (int hr = hr0; hr <= hr1; ++hr)
            for (int hc = hc0; hc <= hc1; ++hc)
                H[static_cast<size_t>(hr * C + hc)] += h_delta;
    }

    // 若兩 pin 在同一 bin col（vc_lo > vc_hi），累加該 bin 的左右兩條邊
    if (vc_lo > vc_hi) {
        const int bin_col  = static_cast<int>(std::floor(0.5 * (xlo + xhi) / bw));
        const int vc_left  = std::clamp(bin_col, 0, C);
        const int vc_right = std::clamp(bin_col + 1, 0, C);
        const int vr0 = std::clamp(vr_lo, 0, R - 1);
        const int vr1 = std::clamp(vr_hi, 0, R - 1);
        for (int vr = vr0; vr <= vr1; ++vr) {
            V[static_cast<size_t>(vc_left  * R + vr)] += v_pair_delta;
            if (vc_right != vc_left)
                V[static_cast<size_t>(vc_right * R + vr)] += v_pair_delta;
        }
    } else {
        const int vc0 = std::clamp(vc_lo, 0, C);
        const int vc1 = std::clamp(vc_hi, 0, C);
        const int vr0 = std::clamp(vr_lo, 0, R - 1);
        const int vr1 = std::clamp(vr_hi, 0, R - 1);
        for (int vc = vc0; vc <= vc1; ++vc)
            for (int vr = vr0; vr <= vr1; ++vr)
                V[static_cast<size_t>(vc * R + vr)] += v_delta;
    }
}

void bin_edge_accumulate_clique(const Die& die,
                                 const std::vector<std::pair<double,double>>& pts,
                                 std::vector<double>& H,
                                 std::vector<double>& V)
{
    const auto np = pts.size();
    for (size_t i = 0; i < np; ++i)
        for (size_t j = i + 1; j < np; ++j)
            bin_edge_accumulate_pair(die,
                                     pts[i].first, pts[i].second,
                                     pts[j].first, pts[j].second,
                                     H, V);
}

void bin_edge_accumulate_x_chain(const Die& die,
                                  std::vector<std::pair<double,double>> pts,
                                  std::vector<double>& H,
                                  std::vector<double>& V)
{
    if (pts.size() < 2) return;
    std::sort(pts.begin(), pts.end());
    for (size_t i = 0; i + 1 < pts.size(); ++i)
        bin_edge_accumulate_pair(die,
                                 pts[i].first,     pts[i].second,
                                 pts[i+1].first,   pts[i+1].second,
                                 H, V);
}

static void collect_net_tier_points(
    const Net& net, int tier,
    const std::vector<Module>& modules,
    const std::vector<TSV>* tsvs_or_null,
    std::vector<std::pair<double,double>>& out)
{
    for (int pid : net.pins) {
        const Module& m = modules[static_cast<size_t>(pid)];
        const int mt = m.is_terminal ? 0 : m.tier_id;
        if (mt == tier) out.push_back({m.x, m.y});
    }
    if (tsvs_or_null) {
        for (const TSV& tsv : *tsvs_or_null) {
            if (tsv.net_id != net.id) continue;
            if (tsv.tier_below() == tier) out.push_back({tsv.x, tsv.y});
            else if (tsv.tier_above() == tier) out.push_back({tsv.x, tsv.y});
        }
    }
}

void bin_edge_cells_from_hv(const Die& die,
                              const std::vector<double>& H,
                              const std::vector<double>& V,
                              std::vector<double>& out_cell_avg)
{
    const int R = die.bin_rows;
    const int C = die.bin_cols;
    out_cell_avg.resize(static_cast<size_t>(R * C), 0.0);
    for (int r = 0; r < R; ++r) {
        for (int c = 0; c < C; ++c) {
            const double bot   = H[static_cast<size_t>(r       * C + c)];
            const double top   = H[static_cast<size_t>((r + 1) * C + c)];
            const double left  = V[static_cast<size_t>(c       * R + r)];
            const double right = V[static_cast<size_t>((c + 1) * R + r)];
            out_cell_avg[static_cast<size_t>(r * C + c)] = (bot + top + left + right) * 0.25;
        }
    }
}

BinEdgeDemands build_bin_edge_baseline_modules_only(const PlacementEngine& engine)
{
    const auto& nets    = engine.nets();
    const auto& modules = engine.modules();
    const auto& dies    = engine.dies();
    const int   nd      = engine.num_dies();

    BinEdgeDemands dem;
    bin_edge_clear(dem, dies);

    for (int t = 0; t < nd; ++t) {
        const Die& die = dies[t];
        for (const Net& net : nets) {
            if (net.pins.size() < 2) continue;
            std::vector<std::pair<double,double>> pts;
            collect_net_tier_points(net, t, modules, nullptr, pts);
            if (pts.size() >= 2)
                bin_edge_accumulate_x_chain(die, std::move(pts), dem.H[t], dem.V[t]);
        }
    }
    return dem;
}

// ============================================================
// bin_edge_stats_from_demands：從已建好的 BinEdgeDemands 計算統計量
// 避免重複遍歷 net；供 solve() 的 RC-alpha 判斷使用
// tier_max / top1%_mean 以單條 H/V edge 為單位；tier_cell_avg 供推力與 PPM
// ============================================================
static void edge_demand_stats(const std::vector<double>& H,
                              const std::vector<double>& V,
                              double& out_max,
                              double& out_top10p_mean)
{
    std::vector<double> edges;
    edges.reserve(H.size() + V.size());
    edges.insert(edges.end(), H.begin(), H.end());
    edges.insert(edges.end(), V.begin(), V.end());
    if (edges.empty()) {
        out_max = 0.0;
        out_top10p_mean = 0.0;
        return;
    }
    double mx = 0.0;
    for (double v : edges) mx = std::max(mx, v);
    out_max = mx;

    std::sort(edges.begin(), edges.end());
    const int total = static_cast<int>(edges.size());
    const int top_k = std::max(1, static_cast<int>(
        std::ceil(0.01 * static_cast<double>(total))));
    double top_sum = 0.0;
    for (int i = total - top_k; i < total; ++i)
        top_sum += edges[static_cast<size_t>(i)];
    out_top10p_mean = top_sum / static_cast<double>(top_k);
}

static BinEdgeCongestionStats::TierMaxEdgeLoc edge_demand_max_loc(
    const Die& die,
    const std::vector<double>& H,
    const std::vector<double>& V)
{
    BinEdgeCongestionStats::TierMaxEdgeLoc loc;
    const int R = die.bin_rows;
    const int C = die.bin_cols;
    double mx = -1.0;

    for (int hr = 0; hr <= R; ++hr) {
        for (int hc = 0; hc < C; ++hc) {
            const double v = H[static_cast<size_t>(hr * C + hc)];
            if (v > mx) {
                mx = v;
                loc.is_horizontal = true;
                loc.row = hr;
                loc.col = hc;
            }
        }
    }
    for (int vc = 0; vc <= C; ++vc) {
        for (int vr = 0; vr < R; ++vr) {
            const double v = V[static_cast<size_t>(vc * R + vr)];
            if (v > mx) {
                mx = v;
                loc.is_horizontal = false;
                loc.row = vr;
                loc.col = vc;
            }
        }
    }
    return loc;
}

BinEdgeCongestionStats bin_edge_stats_from_demands(const BinEdgeDemands& dem,
                                                    const std::vector<Die>& dies)
{
    const int nd = static_cast<int>(dies.size());
    BinEdgeCongestionStats result;
    result.tier_cell_avg.resize(static_cast<size_t>(nd));
    result.tier_max.resize(static_cast<size_t>(nd), 0.0);
    result.tier_top10p_mean.resize(static_cast<size_t>(nd), 0.0);
    result.tier_max_edge_loc.resize(static_cast<size_t>(nd));

    std::vector<double> all_edges;

    for (int t = 0; t < nd && t < static_cast<int>(dem.H.size()); ++t) {
        const Die& die = dies[static_cast<size_t>(t)];
        const auto& H = dem.H[static_cast<size_t>(t)];
        const auto& V = dem.V[static_cast<size_t>(t)];
        bin_edge_cells_from_hv(die, H, V,
                               result.tier_cell_avg[static_cast<size_t>(t)]);

        edge_demand_stats(H, V,
                          result.tier_max[static_cast<size_t>(t)],
                          result.tier_top10p_mean[static_cast<size_t>(t)]);
        result.tier_max_edge_loc[static_cast<size_t>(t)] =
            edge_demand_max_loc(die, H, V);

        if (result.tier_max[static_cast<size_t>(t)] > result.global_max) {
            result.global_max = result.tier_max[static_cast<size_t>(t)];
            result.global_max_tier = t;
            result.global_max_edge_loc = result.tier_max_edge_loc[static_cast<size_t>(t)];
        }

        all_edges.insert(all_edges.end(), H.begin(), H.end());
        all_edges.insert(all_edges.end(), V.begin(), V.end());
    }

    if (!all_edges.empty()) {
        std::sort(all_edges.begin(), all_edges.end());
        const int total = static_cast<int>(all_edges.size());
        const int top_k = std::max(1, static_cast<int>(
            std::ceil(0.01 * static_cast<double>(total))));
        double top_sum = 0.0;
        for (int i = total - top_k; i < total; ++i)
            top_sum += all_edges[static_cast<size_t>(i)];
        result.global_top10p_mean = top_sum / static_cast<double>(top_k);
    }

    return result;
}

// ============================================================
// build_bin_edge_demands_with_tsv：module pin + TSV 合併後做 x-chain
// 從零建 map，不先呼叫 baseline，避免 module-module 段被重複累加。
// ============================================================
BinEdgeDemands build_bin_edge_demands_with_tsv(const PlacementEngine& engine)
{
    const auto& nets    = engine.nets();
    const auto& modules = engine.modules();
    const auto& tsvs    = engine.tsvs();
    const auto& dies    = engine.dies();
    const int   nd      = engine.num_dies();

    BinEdgeDemands dem;
    bin_edge_clear(dem, dies);

    for (int t = 0; t < nd; ++t) {
        const Die& die = dies[static_cast<size_t>(t)];
        for (const Net& net : nets) {
            if (net.pins.size() < 2) continue;
            std::vector<std::pair<double,double>> pts;
            collect_net_tier_points(net, t, modules, &tsvs, pts);
            if (pts.size() >= 2)
                bin_edge_accumulate_x_chain(die, std::move(pts),
                    dem.H[static_cast<size_t>(t)], dem.V[static_cast<size_t>(t)]);
        }
    }

    return dem;
}

// ============================================================
// compute_bin_edge_congestion：委派 x-chain 路徑
// ============================================================
BinEdgeCongestionStats compute_bin_edge_congestion(const PlacementEngine& engine)
{
    const auto& dies = engine.dies();
    BinEdgeDemands dem = engine.tsvs().empty()
        ? build_bin_edge_baseline_modules_only(engine)
        : build_bin_edge_demands_with_tsv(engine);
    return bin_edge_stats_from_demands(dem, dies);
}

void print_bin_edge_congestion_summary(const BinEdgeCongestionStats& s, std::ostream& os)
{
    auto fmt_edge_loc = [](const BinEdgeCongestionStats::TierMaxEdgeLoc& loc) -> std::string {
        if (loc.is_horizontal)
            return "H-edge row=" + std::to_string(loc.row)
                 + " col=" + std::to_string(loc.col);
        return "V-edge row=" + std::to_string(loc.row)
             + " col=" + std::to_string(loc.col);
    };

    os << "  [Congestion edge] Global max=" << s.global_max
       << "  top1%_mean=" << s.global_top10p_mean;
    if (s.global_max_tier >= 0
        && static_cast<size_t>(s.global_max_tier) < s.tier_max_edge_loc.size())
        os << "  at tier " << s.global_max_tier << " "
           << fmt_edge_loc(s.global_max_edge_loc);
    os << "\n";

    for (int t = 0; t < static_cast<int>(s.tier_cell_avg.size()); ++t) {
        os << "    Die " << t
           << "  edge_max=" << s.tier_max[t]
           << "  edge_top1%_mean=" << s.tier_top10p_mean[t];
        if (static_cast<size_t>(t) < s.tier_max_edge_loc.size())
            os << "  max_at=" << fmt_edge_loc(s.tier_max_edge_loc[static_cast<size_t>(t)]);
        os << "\n";
    }
}

// Seaborn "rocket" colormap：u∈[0,1] → 深黑紫→紫紅→橙→亮黃
// u > 1.0 輸出白色（飽和），u < 0 夾為 0
// 控制點來自 seaborn/matplotlib rocket palette（8 stops 線性插值）
static void congestion_rainbow_rgb(double u, unsigned char out[3])
{
    if (u > 1.0) { out[0] = out[1] = out[2] = 255; return; }
    u = std::max(u, 0.0);

    struct Stop { float u, r, g, b; };
    static constexpr Stop stops[] = {
        {0.000f,   3,   2,  22},   // 近黑/深藍紫
        {0.125f,  38,  14,  65},   // 深紫
        {0.250f,  85,  14,  92},   // 深品紅紫
        {0.375f, 145,  18,  98},   // 深品紅
        {0.500f, 198,  35,  80},   // 緋紅/品紅
        {0.625f, 228,  72,  58},   // 橙紅
        {0.750f, 242, 128,  85},   // 淺橙紅/鮭魚
        {0.875f, 250, 188, 148},   // 淺鮭魚桃色
        {1.000f, 254, 232, 213},   // 接近白色奶油
    };
    constexpr int N = static_cast<int>(sizeof(stops) / sizeof(stops[0]));

    int i = 0;
    while (i < N - 2 && u > stops[i + 1].u) ++i;
    const Stop& lo = stops[i];
    const Stop& hi = stops[i + 1];
    const float t = (static_cast<float>(u) - lo.u) / (hi.u - lo.u);
    out[0] = static_cast<unsigned char>(lo.r + t * (hi.r - lo.r) + 0.5f);
    out[1] = static_cast<unsigned char>(lo.g + t * (hi.g - lo.g) + 0.5f);
    out[2] = static_cast<unsigned char>(lo.b + t * (hi.b - lo.b) + 0.5f);
}

// 3×5 bitmap font，字符 '0'–'9' 和 '%'（索引 10）
static const uint8_t kDigitFont[11][5] = {
    {0b111,0b101,0b101,0b101,0b111}, // 0
    {0b010,0b110,0b010,0b010,0b111}, // 1
    {0b111,0b001,0b111,0b100,0b111}, // 2
    {0b111,0b001,0b111,0b001,0b111}, // 3
    {0b101,0b101,0b111,0b001,0b001}, // 4
    {0b111,0b100,0b111,0b001,0b111}, // 5
    {0b111,0b100,0b111,0b101,0b111}, // 6
    {0b111,0b001,0b001,0b001,0b001}, // 7
    {0b111,0b101,0b111,0b101,0b111}, // 8
    {0b111,0b101,0b111,0b001,0b111}, // 9
    {0b101,0b101,0b010,0b101,0b101}, // %
};

static void draw_char(std::vector<uint8_t>& img, int W, int H,
                      int x0, int y0, int ci,
                      uint8_t r, uint8_t g, uint8_t b)
{
    if (ci < 0 || ci > 10) return;
    for (int row = 0; row < 5; ++row) {
        const uint8_t bits = kDigitFont[ci][row];
        for (int col = 0; col < 3; ++col) {
            const int px = x0 + col;
            const int py = y0 + row;
            if (px < 0 || px >= W || py < 0 || py >= H) continue;
            if (bits & (1u << (2 - static_cast<unsigned>(col)))) {
                const size_t idx = (static_cast<size_t>(py) * static_cast<size_t>(W) + static_cast<size_t>(px)) * 3u;
                img[idx+0] = r; img[idx+1] = g; img[idx+2] = b;
            }
        }
    }
}

static void draw_label(std::vector<uint8_t>& img, int W, int H,
                       int x0, int y0, const char* text)
{
    for (int i = 0; text[i] != '\0'; ++i) {
        const char c = text[i];
        if (c >= '0' && c <= '9')
            draw_char(img, W, H, x0 + i * 4, y0, c - '0', 30, 30, 30);
    }
}

void write_bin_edge_congestion_maps(const PlacementEngine&        engine,
                                    const BinEdgeCongestionStats& stats,
                                    const std::string&            base_filename,
                                    int                           upscale,
                                    double                        value_max)
{
    if (upscale < 1) upscale = 1;

    const auto& dies = engine.dies();
    const int nd = engine.num_dies();

    if (static_cast<int>(stats.tier_cell_avg.size()) != nd) return;

    const int bar_w   = 28;
    const int label_w = 36;
    const int gap     = 6;
    const int pad     = 20;

    for (int t = 0; t < nd; ++t) {
        const Die&   die  = dies[static_cast<size_t>(t)];
        const int    R    = die.bin_rows;
        const int    C    = die.bin_cols;
        const auto&  cell = stats.tier_cell_avg[static_cast<size_t>(t)];

        const int grid_w  = C * upscale;
        const int grid_h  = R * upscale;
        const int inner_w = grid_w + gap + bar_w + label_w;
        const int inner_h = grid_h;
        const int total_w = inner_w + 2 * pad;
        const int total_h = inner_h + 2 * pad;

        std::vector<uint8_t> img(static_cast<size_t>(total_w * total_h) * 3u, 255u);

        auto set_px = [&](int px, int py, uint8_t r, uint8_t g, uint8_t b) {
            px += pad;
            py += pad;
            if (px < pad || px >= pad + inner_w || py < pad || py >= pad + inner_h) return;
            const size_t idx = (static_cast<size_t>(py) * static_cast<size_t>(total_w) + static_cast<size_t>(px)) * 3u;
            img[idx+0] = r; img[idx+1] = g; img[idx+2] = b;
        };

        // 左側 grid
        for (int py = 0; py < grid_h; ++py) {
            const int r_chip = (R - 1) - (py / upscale);
            for (int px = 0; px < grid_w; ++px) {
                const int    c_chip = px / upscale;
                const double raw    = cell[static_cast<size_t>(r_chip * C + c_chip)];
                const double u      = raw / value_max;
                uint8_t rgb[3];
                congestion_rainbow_rgb(u, rgb);
                set_px(px, py, rgb[0], rgb[1], rgb[2]);
            }
        }

        // 右側 colorbar（top = high value = red）
        const int bar_x = grid_w + gap;
        for (int py = 0; py < grid_h; ++py) {
            const double u = 1.0 - static_cast<double>(py) / static_cast<double>(grid_h - 1);
            uint8_t rgb[3];
            congestion_rainbow_rgb(u, rgb);
            for (int bx = 0; bx < bar_w; ++bx)
                set_px(bar_x + bx, py, rgb[0], rgb[1], rgb[2]);
        }

        // tick 刻度線 + 數字標籤（0 ~ value_max）
        const int label_x = bar_x + bar_w + 3;
        for (int i = 0; i < 5; ++i) {
            const int tick_val = static_cast<int>(value_max * (4 - i) / 4.0 + 0.5);
            const double frac = static_cast<double>(i) / 4.0;
            const int    py   = static_cast<int>(frac * (grid_h - 1) + 0.5);
            for (int bx = 0; bx < bar_w; ++bx)
                set_px(bar_x + bx, py, 30, 30, 30);
            char label[8];
            std::snprintf(label, sizeof(label), "%d", tick_val);
            draw_label(img, total_w, total_h, label_x + pad, py - 2 + pad, label);
        }

        const std::string path = base_filename + "_congestion_tier"
                                 + std::to_string(t) + ".jpg";
        if (!stbi_write_jpg(path.c_str(), total_w, total_h, 3, img.data(), 90)) {
            std::cerr << "[Congestion map] Cannot write " << path << "\n";
            continue;
        }

        std::cout << "[Congestion map] " << path
                  << "  (" << total_w << "x" << total_h
                  << "  scale=0-" << static_cast<int>(value_max)
                  << "  edge_max=" << stats.tier_max[static_cast<size_t>(t)] << ")\n";
    }
}

// ============================================================
// squarify_modules
// ============================================================
void squarify_modules(PlacementEngine& engine)
{
    int count = 0;
    for (Module& m : engine.modules_mutable()) {
        if (m.is_terminal || m.is_fixed) continue;
        const double side = std::sqrt(m.width * m.height);
        m.width  = side;
        m.height = side;
        ++count;
    }
    std::cout << "[squarify] " << count << " modules squarified\n";
}

// ============================================================
// BOUNDARY constraint geometry helpers
// ============================================================

bool boundary_is_corner(int side)
{
    return side >= 5 && side <= 8;
}

bool boundary_allows_x_move(int side)
{
    return side == 3 || side == 4; // TOP or BOTTOM
}

bool boundary_allows_y_move(int side)
{
    return side == 1 || side == 2; // LEFT or RIGHT
}

void init_module_on_boundary(Module& m, const Die& die, int side)
{
    const double hw = m.width  * 0.5;
    const double hh = m.height * 0.5;

    switch (side) {
    case 1: // LEFT
        m.x = hw;
        m.y = die.height * 0.5;
        break;
    case 2: // RIGHT
        m.x = die.width - hw;
        m.y = die.height * 0.5;
        break;
    case 3: // TOP
        m.x = die.width  * 0.5;
        m.y = die.height - hh;
        break;
    case 4: // BOTTOM
        m.x = die.width  * 0.5;
        m.y = hh;
        break;
    case 5: // TOP-LEFT
        m.x = hw;
        m.y = die.height - hh;
        break;
    case 6: // TOP-RIGHT
        m.x = die.width - hw;
        m.y = die.height - hh;
        break;
    case 7: // BOTTOM-LEFT
        m.x = hw;
        m.y = hh;
        break;
    case 8: // BOTTOM-RIGHT
        m.x = die.width - hw;
        m.y = hh;
        break;
    default:
        break;
    }
    // clamp free axis inside die (in case module is very large)
    m.x = std::max(hw, std::min(die.width  - hw, m.x));
    m.y = std::max(hh, std::min(die.height - hh, m.y));
}

void snap_module_to_boundary(Module& m, const Die& die, int side)
{
    const double hw = m.width  * 0.5;
    const double hh = m.height * 0.5;

    // constrain axis: snap tightly to required die edge(s)
    if (side == 1 || side == 5 || side == 7) // left edge
        m.x = hw;
    if (side == 2 || side == 6 || side == 8) // right edge
        m.x = die.width - hw;
    if (side == 3 || side == 5 || side == 6) // top edge
        m.y = die.height - hh;
    if (side == 4 || side == 7 || side == 8) // bottom edge
        m.y = hh;

    // free axis: just clamp to die
    m.x = std::max(hw, std::min(die.width  - hw, m.x));
    m.y = std::max(hh, std::min(die.height - hh, m.y));
}

void apply_boundary_move_mask(int side, double& gx, double& gy)
{
    // zero the gradient on the frozen axis
    if (!boundary_allows_x_move(side)) gx = 0.0;
    if (!boundary_allows_y_move(side)) gy = 0.0;
}

// ============================================================
// LegalizeFrameWriter 實作
// ============================================================
namespace {

// 將 die+module 渲染成 RGB 像素緩衝（y 翻轉：影像頂 = die y_max）
// 配色對齊 floorplan-viewer-extension color_mode='alt'
std::vector<uint8_t> render_legalize_frame_rgb(
    const std::vector<Module>& modules,
    const std::vector<Die>&    dies,
    int tier, int pix_w, int pix_h)
{
    constexpr uint8_t BG_R = 255, BG_G = 255, BG_B = 255;
    constexpr uint8_t BDR_R = 0, BDR_G = 0, BDR_B = 0;
    constexpr uint8_t MOD_R = 202, MOD_G = 208, MOD_B = 224;
    constexpr double  MOD_ALPHA = 0.72;

    const size_t total = static_cast<size_t>(pix_w * pix_h) * 3u;
    std::vector<uint8_t> img(total);
    for (size_t i = 0; i < total; i += 3) {
        img[i]     = BG_R;
        img[i + 1] = BG_G;
        img[i + 2] = BG_B;
    }

    auto set_px = [&](int px, int py, uint8_t r, uint8_t g, uint8_t b) {
        if (px < 0 || px >= pix_w || py < 0 || py >= pix_h) return;
        const size_t o = (static_cast<size_t>(py) * static_cast<size_t>(pix_w)
                         + static_cast<size_t>(px)) * 3u;
        img[o] = r; img[o+1] = g; img[o+2] = b;
    };

    auto blend_px = [&](int px, int py, uint8_t r, uint8_t g, uint8_t b, double alpha) {
        if (px < 0 || px >= pix_w || py < 0 || py >= pix_h) return;
        const size_t o = (static_cast<size_t>(py) * static_cast<size_t>(pix_w)
                         + static_cast<size_t>(px)) * 3u;
        const double inv = 1.0 - alpha;
        img[o]     = static_cast<uint8_t>(std::lround(img[o]     * inv + r * alpha));
        img[o + 1] = static_cast<uint8_t>(std::lround(img[o + 1] * inv + g * alpha));
        img[o + 2] = static_cast<uint8_t>(std::lround(img[o + 2] * inv + b * alpha));
    };

    // 找 die
    const Die* dp = nullptr;
    for (const Die& d : dies) { if (d.id == tier) { dp = &d; break; } }
    if (!dp) return img;

    const double sx = pix_w / dp->width;
    const double sy = pix_h / dp->height;

    // die 邊框 2px
    for (int px = 0; px < pix_w; ++px) {
        for (int b = 0; b < 2; ++b) {
            set_px(px, b,          BDR_R, BDR_G, BDR_B);
            set_px(px, pix_h-1-b,  BDR_R, BDR_G, BDR_B);
        }
    }
    for (int py = 0; py < pix_h; ++py) {
        for (int b = 0; b < 2; ++b) {
            set_px(b,         py, BDR_R, BDR_G, BDR_B);
            set_px(pix_w-1-b, py, BDR_R, BDR_G, BDR_B);
        }
    }

    // module 矩形
    for (const Module& m : modules) {
        if (m.is_terminal || m.tier_id != tier) continue;

        const double lx = m.x - m.width  * 0.5;
        const double ly = m.y - m.height * 0.5;
        const double rx = lx + m.width;
        const double ry = ly + m.height;

        const int px0 = std::max(0, static_cast<int>(std::floor(lx * sx)));
        const int px1 = std::min(pix_w - 1, static_cast<int>(std::ceil(rx * sx)) - 1);
        // y 翻轉：die y_max → 影像頂（py 小）
        const int py0 = std::max(0, pix_h - static_cast<int>(std::ceil(ry * sy)));
        const int py1 = std::min(pix_h - 1, pix_h - 1 - static_cast<int>(std::floor(ly * sy)));

        if (px0 > px1 || py0 > py1) continue;

        // 填充
        for (int py = py0; py <= py1; ++py)
            for (int px = px0; px <= px1; ++px)
                blend_px(px, py, MOD_R, MOD_G, MOD_B, MOD_ALPHA);

        // 外框 1px
        for (int px = px0; px <= px1; ++px) {
            set_px(px, py0, BDR_R, BDR_G, BDR_B);
            set_px(px, py1, BDR_R, BDR_G, BDR_B);
        }
        for (int py = py0; py <= py1; ++py) {
            set_px(px0, py, BDR_R, BDR_G, BDR_B);
            set_px(px1, py, BDR_R, BDR_G, BDR_B);
        }
    }

    return img;
}

} // anonymous namespace

void LegalizeFrameWriter::begin_tier(
    int tier, double die_w, double die_h,
    const LegalizeVisConfig& cfg)
{
    tier_id_    = tier;
    cfg_        = cfg;
    frame_seq_  = 0;
    frame_names_.clear();

    // 計算像素尺寸（寬限 [64,1200]，高度等比）
    const double w_px = std::clamp(die_w * cfg.upscale, 64.0, 1200.0);
    const double h_px = (die_w > 0.0 && die_h > 0.0)
                      ? std::clamp(die_h / die_w * w_px, 1.0, 1200.0)
                      : w_px;
    pix_w_ = static_cast<int>(std::round(w_px));
    pix_h_ = static_cast<int>(std::round(h_px));

    // 建立輸出目錄 <out_dir>/tier<t>/
    std::filesystem::create_directories(
        cfg.out_dir + "/tier" + std::to_string(tier));
}

void LegalizeFrameWriter::capture(
    const std::vector<Module>& modules,
    const std::vector<Die>&    dies,
    int tier, const std::string& tag)
{
    if (pix_w_ <= 0 || pix_h_ <= 0) return;

    // 組檔名（tag 直接用，允許含 ->）
    std::ostringstream name;
    name << "frame_" << std::setfill('0') << std::setw(5) << frame_seq_
         << "_" << tag << ".png";
    const std::string fname = name.str();
    const std::string path  = cfg_.out_dir + "/tier" + std::to_string(tier_id_)
                            + "/" + fname;

    auto rgb = render_legalize_frame_rgb(modules, dies, tier, pix_w_, pix_h_);
    if (!stbi_write_png(path.c_str(), pix_w_, pix_h_, 3,
                        rgb.data(), pix_w_ * 3)) {
        std::cerr << "[LegalizeVis] Cannot write PNG: " << path << "\n";
    }

    frame_names_.push_back(fname);
    ++frame_seq_;
}

void LegalizeFrameWriter::end_tier()
{
    // 寫 manifest.json 供 Python 腳本穩定讀取幀順序
    const std::string mpath =
        cfg_.out_dir + "/tier" + std::to_string(tier_id_) + "/manifest.json";
    std::ofstream mf(mpath);
    if (!mf) {
        std::cerr << "[LegalizeVis] Cannot write manifest: " << mpath << "\n";
        return;
    }
    mf << "{\n  \"tier\": " << tier_id_ << ",\n  \"frames\": [\n";
    for (size_t i = 0; i < frame_names_.size(); ++i) {
        mf << "    \"" << frame_names_[i] << "\"";
        if (i + 1 < frame_names_.size()) mf << ",";
        mf << "\n";
    }
    mf << "  ]\n}\n";
    std::cout << "[LegalizeVis] Tier " << tier_id_
              << "  " << frame_seq_ << " frames  -> "
              << cfg_.out_dir + "/tier" + std::to_string(tier_id_) << "/\n";
}

// ============================================================
// Tier 面積使用率
// ============================================================
std::vector<double> compute_tier_module_utilization(const PlacementEngine& engine,
                                                    double tsv_width,
                                                    double tsv_height,
                                                    const std::vector<int>* estimated_tsv_count_per_tier)
{
    const int nd = engine.num_dies();
    const auto& modules = engine.modules();
    const auto& dies    = engine.dies();
    const double tsv_area = (tsv_width > 0.0 && tsv_height > 0.0)
                            ? tsv_width * tsv_height : 0.0;

    std::vector<double> util(static_cast<size_t>(nd), 0.0);
    for (const Module& m : modules) {
        if (m.is_terminal) continue;
        const int t = m.tier_id;
        if (t < 0 || t >= nd) continue;
        util[static_cast<size_t>(t)] += m.area();
    }
    if (tsv_area > 0.0) {
        if (estimated_tsv_count_per_tier
            && static_cast<int>(estimated_tsv_count_per_tier->size()) == nd) {
            for (int t = 0; t < nd; ++t)
                util[static_cast<size_t>(t)]
                    += static_cast<double>((*estimated_tsv_count_per_tier)[static_cast<size_t>(t)])
                       * tsv_area;
        } else {
            for (const TSV& tsv : engine.tsvs()) {
                const int t = tsv.tier_below();
                if (t < 0 || t >= nd) continue;
                util[static_cast<size_t>(t)] += tsv_area;
            }
        }
    }
    for (int t = 0; t < nd; ++t) {
        const Die& die = dies[static_cast<size_t>(t)];
        const double die_area = die.width * die.height;
        util[static_cast<size_t>(t)] = (die_area > 0.0) ? util[static_cast<size_t>(t)] / die_area : 0.0;
    }
    return util;
}

bool any_tier_exceeds_module_util(const PlacementEngine& engine, double threshold,
                                    double tsv_width, double tsv_height)
{
    const auto util = compute_tier_module_utilization(engine, tsv_width, tsv_height, nullptr);
    for (double u : util)
        if (u > threshold) return true;
    return false;
}

std::vector<int> estimate_tsv_count_per_tier(const PlacementEngine& engine)
{
    const int nd = engine.num_dies();
    const auto& nets    = engine.nets();
    const auto& modules = engine.modules();
    std::vector<int> cnt(static_cast<size_t>(nd), 0);

    for (const Net& net : nets) {
        if (net.pins.size() < 2) continue;

        int min_t = std::numeric_limits<int>::max();
        int max_t = std::numeric_limits<int>::min();
        for (int pid : net.pins) {
            const Module& m = modules[static_cast<size_t>(pid)];
            const int t = m.is_terminal ? 0 : m.tier_id;
            min_t = std::min(min_t, t);
            max_t = std::max(max_t, t);
        }
        if (min_t < 0 || max_t - min_t < 1) continue;

        for (int layer = min_t; layer < max_t; ++layer)
            ++cnt[static_cast<size_t>(layer)];
    }
    return cnt;
}

bool check_tier_module_tsv_area_limit(const PlacementEngine& engine,
                                      double tsv_width,
                                      double tsv_height,
                                      double max_ratio,
                                      std::ostream& err)
{
    const auto tsv_cnt = estimate_tsv_count_per_tier(engine);
    const auto util    = compute_tier_module_utilization(
        engine, tsv_width, tsv_height, &tsv_cnt);

    bool ok = true;
    for (int t = 0; t < static_cast<int>(util.size()); ++t) {
        if (util[static_cast<size_t>(t)] <= max_ratio + 1e-12) continue;
        ok = false;
        err << "[Error] Tier " << t << " module+TSV area "
            << std::fixed << std::setprecision(1)
            << util[static_cast<size_t>(t)] * 100.0
            << "% exceeds " << max_ratio * 100.0 << "% limit"
            << " (tsv_count=" << tsv_cnt[static_cast<size_t>(t)] << ")\n";
    }
    return ok;
}
