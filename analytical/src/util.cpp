// 3D IC Analytical Floorplanner - 位移分析工具實作
#include "util.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
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
    const std::string&                 filepath)
{
    if constexpr (!ENABLE_DISPLACEMENT_REPORT) return;
    if (before.empty() || after.empty()) return;

    struct Entry { std::string name; double disp; double ax, ay, bx, by; double area; int num_nets; };
    std::vector<Entry> entries;
    entries.reserve(before.size());

    for (const auto& b : before) {
        for (const auto& a : after) {
            if (a.id != b.id) continue;
            const double dx   = a.x - b.x;
            const double dy   = a.y - b.y;
            const double dist = std::sqrt(dx * dx + dy * dy);
            entries.push_back({ b.name, dist, a.x, a.y, b.x, b.y, b.area, b.num_nets });
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

    std::cout << "[Displacement] Report -> " << filepath
              << "  (" << entries.size() << " modules)\n";
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

    const double dx = std::fabs(x1 - x2);
    const double dy = std::fabs(y1 - y2);

    const int    col_span = std::max(1, static_cast<int>(std::ceil(dx / bw)));
    const int    row_span = std::max(1, static_cast<int>(std::ceil(dy / bh)));
    const double h_delta  = 1.0 / col_span;
    const double v_delta  = 1.0 / row_span;

    const int hc_lo = static_cast<int>(std::floor(xlo / bw));
    const int hc_hi = static_cast<int>(std::ceil(xhi / bw)) - 1;
    const int hr_lo = static_cast<int>(std::ceil(ylo / bh));
    const int hr_hi = static_cast<int>(std::floor(yhi / bh));
    const int vr_lo = static_cast<int>(std::floor(ylo / bh));
    const int vr_hi = static_cast<int>(std::ceil(yhi / bh)) - 1;
    const int vc_lo = static_cast<int>(std::ceil(xlo / bw));
    const int vc_hi = static_cast<int>(std::floor(xhi / bw));

    for (int hr = hr_lo; hr <= hr_hi && hr >= 0 && hr <= R; ++hr)
        for (int hc = hc_lo; hc <= hc_hi && hc >= 0 && hc < C; ++hc)
            H[static_cast<size_t>(hr * C + hc)] += h_delta;

    for (int vc = vc_lo; vc <= vc_hi && vc >= 0 && vc <= C; ++vc)
        for (int vr = vr_lo; vr <= vr_hi && vr >= 0 && vr < R; ++vr)
            V[static_cast<size_t>(vc * R + vr)] += v_delta;
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
            for (int pid : net.pins) {
                const Module& m   = modules[pid];
                const int     mt  = m.is_terminal ? 0 : m.tier_id;
                if (mt == t) pts.push_back({ m.x, m.y });
            }
            if (pts.size() >= 2)
                bin_edge_accumulate_clique(die, pts, dem.H[t], dem.V[t]);
        }
    }
    return dem;
}

// ============================================================
// compute_bin_edge_congestion
// ============================================================
BinEdgeCongestionStats compute_bin_edge_congestion(const PlacementEngine& engine)
{
    const auto& nets    = engine.nets();
    const auto& modules = engine.modules();
    const auto& tsvs    = engine.tsvs();
    const auto& dies    = engine.dies();
    const int   nd      = engine.num_dies();

    BinEdgeCongestionStats result;
    result.tier_cell_avg.resize(nd);
    result.tier_max.resize(nd, 0.0);
    result.tier_top10p_mean.resize(nd, 0.0);

    for (int t = 0; t < nd; ++t) {
        const Die& die = dies[t];
        const int  R   = die.bin_rows;
        const int  C   = die.bin_cols;
        const double bw = die.bin_w;
        const double bh = die.bin_h;

        // H[hr * C + hc]：水平邊段 demand，hr∈[0,R]，hc∈[0,C-1]
        // V[vc * R + vr]：垂直邊段 demand，vc∈[0,C]，vr∈[0,R-1]
        std::vector<double> H(static_cast<size_t>((R + 1) * C), 0.0);
        std::vector<double> V(static_cast<size_t>((C + 1) * R), 0.0);

        for (const Net& net : nets) {
            if (net.pins.size() < 2) continue;

            // 收集此 net 在 tier t 的端點（含 TSV 虛擬端點）
            std::vector<std::pair<double,double>> pts;

            for (int pid : net.pins) {
                const Module& m = modules[pid];
                const int mt = m.is_terminal ? 0 : m.tier_id;
                if (mt == t)
                    pts.push_back({ m.x, m.y });
            }

            for (const TSV& tsv : tsvs) {
                if (tsv.net_id != net.id) continue;
                if (tsv.tier_below() == t) pts.push_back({ tsv.x, tsv.y });
                if (tsv.tier_above() == t) pts.push_back({ tsv.x, tsv.y });
            }

            if (pts.size() < 2) continue;

            // 所有無序 pair
            for (size_t i = 0; i < pts.size(); ++i) {
                for (size_t j = i + 1; j < pts.size(); ++j) {
                    const double x1 = pts[i].first,  y1 = pts[i].second;
                    const double x2 = pts[j].first,  y2 = pts[j].second;

                    // bbox（clamp 到 die 範圍）
                    const double xlo = std::max(0.0, std::min(x1, x2));
                    const double xhi = std::min(die.width,  std::max(x1, x2));
                    const double ylo = std::max(0.0, std::min(y1, y2));
                    const double yhi = std::min(die.height, std::max(y1, y2));

                    const double dx = std::fabs(x1 - x2);
                    const double dy = std::fabs(y1 - y2);

                    // span：橫跨多少格（至少 1）
                    const int col_span = std::max(1, static_cast<int>(std::ceil(dx / bw)));
                    const int row_span = std::max(1, static_cast<int>(std::ceil(dy / bh)));

                    const double h_delta = 1.0 / col_span;
                    const double v_delta = 1.0 / row_span;

                    // bbox 對應的 col/row 索引範圍
                    // 水平邊段 (hr, hc)：hr 對應 y 方向的邊線，hc 對應 x 方向的格子
                    const int hc_lo = static_cast<int>(std::floor(xlo / bw));
                    const int hc_hi = static_cast<int>(std::ceil(xhi / bw)) - 1;
                    // 水平邊線 hr∈[hr_lo, hr_hi]（bbox ylo 下方到 yhi 上方的橫線）
                    const int hr_lo = static_cast<int>(std::ceil(ylo / bh));
                    const int hr_hi = static_cast<int>(std::floor(yhi / bh));

                    // 垂直邊段 (vr, vc)：vc 對應 x 方向的邊線，vr 對應 y 方向的格子
                    const int vr_lo = static_cast<int>(std::floor(ylo / bh));
                    const int vr_hi = static_cast<int>(std::ceil(yhi / bh)) - 1;
                    const int vc_lo = static_cast<int>(std::ceil(xlo / bw));
                    const int vc_hi = static_cast<int>(std::floor(xhi / bw));

                    // 累加水平邊段
                    for (int hr = hr_lo; hr <= hr_hi && hr >= 0 && hr <= R; ++hr) {
                        for (int hc = hc_lo; hc <= hc_hi && hc >= 0 && hc < C; ++hc) {
                            H[static_cast<size_t>(hr * C + hc)] += h_delta;
                        }
                    }

                    // 累加垂直邊段
                    for (int vc = vc_lo; vc <= vc_hi && vc >= 0 && vc <= C; ++vc) {
                        for (int vr = vr_lo; vr <= vr_hi && vr >= 0 && vr < R; ++vr) {
                            V[static_cast<size_t>(vc * R + vr)] += v_delta;
                        }
                    }
                }
            }
        }

        // 計算每個 cell 的四邊平均
        auto& cell_avg = result.tier_cell_avg[t];
        cell_avg.resize(static_cast<size_t>(R * C), 0.0);

        double mx = 0.0;
        for (int r = 0; r < R; ++r) {
            for (int c = 0; c < C; ++c) {
                // cell (r,c) 的四條邊：
                //   bottom H: hr=r,   hc=c
                //   top    H: hr=r+1, hc=c
                //   left   V: vc=c,   vr=r
                //   right  V: vc=c+1, vr=r
                const double bot   = H[static_cast<size_t>(r       * C + c)];
                const double top   = H[static_cast<size_t>((r + 1) * C + c)];
                const double left  = V[static_cast<size_t>(c       * R + r)];
                const double right = V[static_cast<size_t>((c + 1) * R + r)];
                const double avg   = (bot + top + left + right) * 0.25;

                cell_avg[static_cast<size_t>(r * C + c)] = avg;
                if (avg > mx) mx = avg;
            }
        }
        result.tier_max[t] = mx;
        if (mx > result.global_max) result.global_max = mx;

        // 前 10% 高壅塞 bin 的平均：排序後取末尾 ceil(10% * total) 個
        const int total_bins = R * C;
        if (total_bins > 0) {
            std::vector<double> sorted_vals(cell_avg.begin(), cell_avg.end());
            std::sort(sorted_vals.begin(), sorted_vals.end());
            const int top_k = std::max(1, static_cast<int>(
                std::ceil(0.1 * static_cast<double>(total_bins))));
            double top_sum = 0.0;
            for (int i = total_bins - top_k; i < total_bins; ++i)
                top_sum += sorted_vals[static_cast<size_t>(i)];
            result.tier_top10p_mean[t] = top_sum / top_k;
        }
    }

    // global top-10% mean = 跨所有 tier 的 cell_avg 合併後取前 10%
    {
        std::vector<double> all_vals;
        for (int t = 0; t < nd; ++t)
            all_vals.insert(all_vals.end(),
                            result.tier_cell_avg[t].begin(),
                            result.tier_cell_avg[t].end());
        const int total = static_cast<int>(all_vals.size());
        if (total > 0) {
            std::sort(all_vals.begin(), all_vals.end());
            const int top_k = std::max(1, static_cast<int>(
                std::ceil(0.1 * static_cast<double>(total))));
            double top_sum = 0.0;
            for (int i = total - top_k; i < total; ++i)
                top_sum += all_vals[static_cast<size_t>(i)];
            result.global_top10p_mean = top_sum / top_k;
        }
    }

    return result;
}

void print_bin_edge_congestion_summary(const BinEdgeCongestionStats& s, std::ostream& os)
{
    os << "  [Congestion] Global max=" << s.global_max
       << "  top10%_mean=" << s.global_top10p_mean << "\n";
    for (int t = 0; t < static_cast<int>(s.tier_cell_avg.size()); ++t) {
        os << "    Die " << t
           << "  max=" << s.tier_max[t]
           << "  top10%_mean=" << s.tier_top10p_mean[t] << "\n";
    }
}

// 將 u∈[0,1] 映射為 RGB：淺青白→深藍黑（壅塞由小到大）
static void congestion_to_rgb(double u, unsigned char out[3])
{
    u = std::clamp(u, 0.0, 1.0);
    // Light: ~(245, 250, 255)  Deep: ~(8, 18, 45)
    const double r0 = 245.0, g0 = 250.0, b0 = 255.0;
    const double r1 = 8.0,   g1 = 18.0,  b1 = 45.0;
    out[0] = static_cast<unsigned char>(r0 + (r1 - r0) * u + 0.5);
    out[1] = static_cast<unsigned char>(g0 + (g1 - g0) * u + 0.5);
    out[2] = static_cast<unsigned char>(b0 + (b1 - b0) * u + 0.5);
}

void write_bin_edge_congestion_maps(const PlacementEngine&        engine,
                                    const BinEdgeCongestionStats& stats,
                                    const std::string&            base_filename,
                                    int                           upscale)
{
    if (upscale < 1) upscale = 1;

    const auto& dies = engine.dies();
    const int nd   = engine.num_dies();

    if (static_cast<int>(stats.tier_cell_avg.size()) != nd) return;

    for (int t = 0; t < nd; ++t) {
        const Die&   die       = dies[t];
        const int    R         = die.bin_rows;
        const int    C         = die.bin_cols;
        const auto& cell       = stats.tier_cell_avg[static_cast<size_t>(t)];
        const double vmax      = stats.tier_max[t];
        const int    PixW      = C * upscale;
        const int    PixH      = R * upscale;

        const std::string path = base_filename + "_congestion_tier"
                                 + std::to_string(t) + ".ppm";

        std::ofstream ofs(path, std::ios::binary);
        if (!ofs) {
            std::cerr << "[Congestion map] Cannot open " << path << "\n";
            continue;
        }

        ofs << "P6\n" << PixW << " " << PixH << "\n255\n";

        std::vector<unsigned char> scanline(static_cast<size_t>(PixW) * 3u);

        for (int py = 0; py < PixH; ++py) {
            // Die：row 較小者對應較小的 y（近 die 底部）；影像上方對應較大的 row（近 die 頂）
            const int r_chip = (R - 1) - (py / upscale);
            for (int px = 0; px < PixW; ++px) {
                const int           c_chip = px / upscale;
                const double        raw    = cell[static_cast<size_t>(r_chip * C + c_chip)];
                const double        norm   =
                    (vmax > 1e-18) ? std::clamp(raw / vmax, 0.0, 1.0) : 0.0;
                unsigned char       rgb[3];
                congestion_to_rgb(norm, rgb);
                scanline[static_cast<size_t>(px) * 3u + 0u] = rgb[0];
                scanline[static_cast<size_t>(px) * 3u + 1u] = rgb[1];
                scanline[static_cast<size_t>(px) * 3u + 2u] = rgb[2];
            }
            ofs.write(reinterpret_cast<const char*>(scanline.data()),
                      static_cast<std::streamsize>(scanline.size()));
        }

        std::cout << "[Congestion map] " << path << "  (" << PixW << "x" << PixH
                  << "  tier_max=" << vmax << ")\n";
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
// LegalizeFrameWriter 實作
// ============================================================
namespace {

// 將 die+module 渲染成 RGB 像素緩衝（y 翻轉：影像頂 = die y_max）
// 背景淺灰、die 黑框 2px、所有 module 米色填充 + 1px 黑框
std::vector<uint8_t> render_legalize_frame_rgb(
    const std::vector<Module>& modules,
    const std::vector<Die>&    dies,
    int tier, int pix_w, int pix_h)
{
    // 背景：淺灰
    constexpr uint8_t BG_R = 220, BG_G = 220, BG_B = 220;
    // 邊框：黑
    constexpr uint8_t BDR_R = 0, BDR_G = 0, BDR_B = 0;
    // module 填充：米色
    constexpr uint8_t MOD_R = 248, MOD_G = 228, MOD_B = 185;

    const size_t total = static_cast<size_t>(pix_w * pix_h) * 3u;
    std::vector<uint8_t> img(total);
    // 填背景
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
                set_px(px, py, MOD_R, MOD_G, MOD_B);

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
