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
