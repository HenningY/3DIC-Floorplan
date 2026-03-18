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
