// 3D IC Analytical Floorplanner - 位移分析工具實作
#include "util.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
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
