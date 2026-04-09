// 3D IC Analytical Floorplanner - 輸入檔案解析器
// 支援 .block 與 .nets 格式（與 PA2 輸入規格相容）
#include "floorplanner.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

// ============================================================
// parse_blocks: 解析 .block 檔案
//
// 格式：
//   NumDie: <N>
//   Outline: <W> <H>    (重複 N 次)
//   NumBlocks: <B>
//   NumTerminals: <T>
//
//   <name> <width> <height> <tier_id>    (方塊，共 B 個)
//
//   <name> terminal <x> <y>              (端點，共 T 個，固定位置)
// ============================================================
bool PlacementEngine::parse_blocks(const std::string& filename)
{
    std::ifstream fin(filename);
    if (!fin) {
        std::cerr << "[Error] Cannot open block file: " << filename << "\n";
        return false;
    }

    std::string line, token;
    int num_dies = 0, num_blocks = 0, num_terminals = 0;

    // ---- 解析標頭 ----
    // 先讀 NumDie
    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        ss >> token;
        if (token == "NumDie:") {
            ss >> num_dies;
            break;
        }
    }

    // 讀取每個 Die 的 Outline（依序，第 i 個 Outline 對應 Die i）
    std::vector<double> die_widths(num_dies), die_heights(num_dies);
    int die_idx = 0;
    while (die_idx < num_dies && std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        ss >> token;
        if (token == "Outline:") {
            ss >> die_widths[die_idx] >> die_heights[die_idx];
            ++die_idx;
        }
    }

    // 初始化 Die 結構（第一個 Die 的尺寸，各層相同）
    setup_dies(num_dies, die_widths[0], die_heights[0]);

    // 讀 NumBlocks 和 NumTerminals
    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        ss >> token;
        if (token == "NumBlocks:")   ss >> num_blocks;
        if (token == "NumTerminals:") {
            ss >> num_terminals;
            break;
        }
    }

    // ---- 解析方塊與 terminal ----
    modules_.clear();
    name_to_id_.clear();

    int id_counter = 0;
    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream ss(line);
        std::string name;
        ss >> name;
        if (name.empty()) continue;

        std::string second_token;
        ss >> second_token;

        Module m;
        m.id   = id_counter;
        m.name = name;

        if (second_token == "terminal") {
            // Terminal：固定端點，座標為絕對位置
            double tx = 0.0, ty = 0.0;
            ss >> tx >> ty;
            m.width       = 0.0;
            m.height      = 0.0;
            m.x           = tx;
            m.y           = ty;
            m.tier_id     = -1;   // terminal 不屬於特定 Die
            m.is_terminal = true;
        } else {
            // Block：可移動方塊
            double w = std::stod(second_token);
            double h = 0.0;
            int    t = 0;
            ss >> h >> t;
            m.width       = w;
            m.height      = h;
            m.x           = die_widths[t]  * 0.5;  // 預設置中
            m.y           = die_heights[t] * 0.5;
            m.tier_id     = t;
            m.is_terminal = false;
        }

        name_to_id_[name] = id_counter;
        modules_.push_back(std::move(m));
        ++id_counter;
    }

    std::cout << "[Parser] Loaded " << num_dies      << " dies, "
              << num_blocks   << " blocks, "
              << num_terminals << " terminals.\n";

    // 驗證讀取數量
    int actual_blocks    = 0, actual_terminals = 0;
    for (const Module& m : modules_) {
        if (m.is_terminal) ++actual_terminals;
        else               ++actual_blocks;
    }
    if (actual_blocks != num_blocks || actual_terminals != num_terminals) {
        std::cerr << "[Warning] Expected " << num_blocks << " blocks and "
                  << num_terminals << " terminals, "
                  << "but got " << actual_blocks << " blocks and "
                  << actual_terminals << " terminals.\n";
    }

    return true;
}

// ============================================================
// parse_constraints: 解析 .constraint 檔案
//
// 格式（每行一條約束，# 開頭為註解）：
//   FIXED <module_name> <x_ll> <y_ll> <x_ur> <y_ur>
//
// 效果：
//   - 找到對應 Module（必須是非 terminal 的 block）
//   - 以 (x_ll, y_ll, x_ur, y_ur) 決定固定位置與尺寸（因 ll/ur 隱含旋轉方向）
//   - 設定 is_fixed = true；x/y 設為中心；width/height 設為 ur-ll
// ============================================================
bool PlacementEngine::parse_constraints(const std::string& filename)
{
    std::ifstream fin(filename);
    if (!fin) {
        std::cerr << "[Constraint] Cannot open constraint file: " << filename << "\n";
        return false;
    }

    int count = 0;
    std::string line;
    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string keyword;
        ss >> keyword;
        if (keyword != "FIXED") continue;

        std::string name;
        double x_ll, y_ll, x_ur, y_ur;
        if (!(ss >> name >> x_ll >> y_ll >> x_ur >> y_ur)) {
            std::cerr << "[Constraint] Malformed FIXED line: " << line << "\n";
            continue;
        }

        auto it = name_to_id_.find(name);
        if (it == name_to_id_.end()) {
            std::cerr << "[Constraint] Unknown module name: " << name << "\n";
            continue;
        }

        Module& m = modules_[it->second];
        if (m.is_terminal) {
            std::cerr << "[Constraint] Cannot fix terminal: " << name << "\n";
            continue;
        }

        if (x_ur < x_ll) std::swap(x_ll, x_ur);
        if (y_ur < y_ll) std::swap(y_ll, y_ur);

        m.width    = x_ur - x_ll;
        m.height   = y_ur - y_ll;
        m.x        = 0.5 * (x_ll + x_ur);
        m.y        = 0.5 * (y_ll + y_ur);
        m.is_fixed = true;

        ++count;
        std::cout << "[Constraint] FIXED " << name
                  << "  tier=" << m.tier_id
                  << "  ll=(" << x_ll << "," << y_ll << ")"
                  << "  ur=(" << x_ur << "," << y_ur << ")\n";
    }

    std::cout << "[Constraint] Applied " << count << " FIXED constraint(s).\n";
    return true;
}

// ============================================================
// parse_nets: 解析 .nets 檔案
//
// 格式：
//   NumNets: <N>
//   NetDegree: <D>
//   <pin1_name>
//   <pin2_name>
//   ... (共 D 個)
// ============================================================
bool PlacementEngine::parse_nets(const std::string& filename)
{
    std::ifstream fin(filename);
    if (!fin) {
        std::cerr << "[Error] Cannot open nets file: " << filename << "\n";
        return false;
    }

    std::string line, token;
    int num_nets = 0;

    // 讀取 NumNets
    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        ss >> token;
        if (token == "NumNets:") {
            ss >> num_nets;
            break;
        }
    }

    nets_.clear();
    nets_.reserve(num_nets);
    int net_id = 0;

    while (std::getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        ss >> token;
        if (token != "NetDegree:") continue;

        int degree = 0;
        ss >> degree;

        Net net;
        net.id   = net_id;
        net.name = "net" + std::to_string(net_id);
        ++net_id;

        for (int d = 0; d < degree; ++d) {
            // 讀取每個 pin 名稱
            while (std::getline(fin, line)) {
                if (line.empty() || line[0] == '#') continue;
                std::istringstream pin_ss(line);
                std::string pin_name;
                pin_ss >> pin_name;
                if (pin_name.empty()) continue;

                auto it = name_to_id_.find(pin_name);
                if (it != name_to_id_.end()) {
                    net.pins.push_back(it->second);
                } else {
                    std::cerr << "[Warning] Unknown pin name: " << pin_name << "\n";
                }
                break;
            }
        }

        if (net.pins.size() >= 2) {
            nets_.push_back(std::move(net));
        }
    }

    std::cout << "[Parser] Loaded " << nets_.size() << " nets "
              << "(expected " << num_nets << ").\n";
    return true;
}
