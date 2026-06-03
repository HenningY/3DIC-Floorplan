// 3D IC Analytical Floorplanner - 位移分析工具
#pragma once

#include "floorplanner.h"

#include <iostream>
#include <string>
#include <vector>

// 設為 true 開啟位移分析；false 則所有函式為 no-op
static constexpr bool ENABLE_DISPLACEMENT_REPORT = true;

struct ModuleSnapshot {
    int         id;
    std::string name;
    double      x, y;
    double      area;
    int         num_nets;
};

// 擷取目前所有 movable modules 的中心座標
std::vector<ModuleSnapshot> record_positions(const PlacementEngine& engine);

// 計算 before/after 兩次快照的 Euclidean 位移，依大小排序後寫至 filepath
void write_displacement_report(
    const std::vector<ModuleSnapshot>& before,
    const std::vector<ModuleSnapshot>& after,
    const std::string&                 filepath);

// ============================================================
// Intra-die HPWL：只計算所有 pin 都在同一層的 net
// ============================================================
struct IntraDieStats {
    double              total_hpwl;  // 全部層加總（已乘 die weight）
    std::vector<double> tier_hpwl;   // 每層加權 HPWL
    std::vector<int>    tier_count;  // 每層 intra-die net 數量
};

IntraDieStats compute_intra_die_hpwl(const PlacementEngine& engine);

// 在 std::ostream 上列印 IntraDieStats 的逐層明細
void print_intra_die_stats(const IntraDieStats& stats, std::ostream& os = std::cout);

// ============================================================
// 正方化：將所有 non-terminal movable module 的長寬改成等面積的正方形
// 只改 width / height，不改中心座標 (x, y)。
// ============================================================
void squarify_modules(PlacementEngine& engine);

// ============================================================
// Overlap 檢查：module 與 TSV 的重疊對數統計
// ============================================================
// 印出每一層的 overlap pair 數量（module-module / module-TSV / TSV-TSV 全部算）
// tsv_width / tsv_height：TSV 的物理尺寸（用來建矩形邊界）
void print_overlap_report(const PlacementEngine& engine,
    double                 tsv_width,
    double                 tsv_height,
    std::ostream&          os = std::cout);

// ============================================================
// Bin-edge Congestion 指標
//
// 以「全域布線 bounding-box + 線段 demand」為每個 die 的
// (R+1)*C 水平邊段與 (C+1)*R 垂直邊段累積 demand，
// 再對每個 cell 取四邊 demand 平均作為壅塞度。
//
// net 端點展開規則（per tier）：
//   - 一般 pin：若 module.tier_id == t（terminal 視為 tier 0）
//   - TSV：對每個 tsv，在 tier_below() 與 tier_above() 各放一個虛擬端點
//     （因此一顆 TSV 在相鄰兩 tier 各加一份 (tsv.x, tsv.y)）
//   - 前提：需先呼叫 engine.build_tsvs()；若未呼叫則跨層 net 不含 TSV 端點
//
// 對 tier t 中屬於同一 net 的 Pts，取所有無序 pair i<j：
//   col_span = max(1, ceil(|dx| / bin_w))
//   row_span = max(1, ceil(|dy| / bin_h))
//   對 pair bbox 內每條水平邊段 += 1/col_span
//   對 pair bbox 內每條垂直邊段 += 1/row_span
//
// cell_avg[r,c] = (top_H + bot_H + left_V + right_V) / 4
// ============================================================
struct BinEdgeCongestionStats {
    // 每個 tier 的 cell 壅塞度，row-major，長度 = bin_rows * bin_cols
    std::vector<std::vector<double>> tier_cell_avg;
    // 各 tier 的 cell 最大壅塞值
    std::vector<double> tier_max;
    // 各 tier 前 10% 高壅塞 bin 的平均壅塞值
    std::vector<double> tier_top10p_mean;
    // 全域最大
    double global_max        = 0.0;
    // 全域前 10% 高壅塞 bin 的平均壅塞值
    double global_top10p_mean = 0.0;
};

BinEdgeCongestionStats compute_bin_edge_congestion(const PlacementEngine& engine);

// ============================================================
// BinEdgeDemands：可增量更新的 H/V edge demand 陣列
//
// 格式與 compute_bin_edge_congestion 內部完全一致：
//   H[t][hr * C + hc]  hr∈[0,R], hc∈[0,C-1]
//   V[t][vc * R + vr]  vc∈[0,C], vr∈[0,R-1]
// ============================================================
struct BinEdgeDemands {
    // 每層各一個 H/V 向量
    std::vector<std::vector<double>> H;  // 長度 (R+1)*C，per tier
    std::vector<std::vector<double>> V;  // 長度 (C+1)*R，per tier
};

// 配置並清零（依 dies 決定每層大小）
void bin_edge_clear(BinEdgeDemands& d, const std::vector<Die>& dies);

// 對單一 2-pin (x1,y1)↔(x2,y2) 在 die d 的 H/V 累加 demand
void bin_edge_accumulate_pair(const Die& die,
                               double x1, double y1,
                               double x2, double y2,
                               std::vector<double>& H,
                               std::vector<double>& V);

// 對所有無序 pair i<j 呼叫 bin_edge_accumulate_pair
void bin_edge_accumulate_clique(const Die& die,
                                 const std::vector<std::pair<double,double>>& pts,
                                 std::vector<double>& H,
                                 std::vector<double>& V);

// 從 H/V 推算每個 cell 的四邊平均 cell_avg（長度 R*C，row-major）
void bin_edge_cells_from_hv(const Die& die,
                              const std::vector<double>& H,
                              const std::vector<double>& V,
                              std::vector<double>& out_cell_avg);

// 建立「僅含 module/terminal pin、不含 TSV」的 baseline demands
// （per-tier 切片，與 compute_bin_edge_congestion 之 module 端點部分行為一致）
BinEdgeDemands build_bin_edge_baseline_modules_only(const PlacementEngine& engine);

// 印每層 max / top-10% mean 以及全域摘要（不 dump 完整格子）
void print_bin_edge_congestion_summary(const BinEdgeCongestionStats& s,
    std::ostream& os = std::cout);

// 將各 tier 的 cell 壅塞度畫成 PPM（RGB）；淺色＝壅塞低、深色＝壅塞高（每 die 對 [0,tier_max] 線性對應）
// upscale：每個 bin 在影像中佔 upscale×upscale 像素（預設 8）
// y 軸對齊：影像上方 = die y 較大一側（row index 較大的 bin）
void write_bin_edge_congestion_maps(const PlacementEngine&        engine,
                                    const BinEdgeCongestionStats& stats,
                                    const std::string&            base_filename,
                                    int                           upscale = 8);

// ============================================================
// Legalize 過程視覺化
//
// 用法（legalize_heu.cpp 的 tier 迴圈內）：
//   LegalizeFrameWriter fw;
//   fw.begin_tier(t, die_w, die_h, cfg);
//   fw.capture(modules, dies, t, "initial");
//   // ... each sweep ...
//   fw.capture(modules, dies, t, "near->far");
//   fw.capture(modules, dies, t, "tier_done");
//   fw.end_tier();   // 寫 manifest.json
// ============================================================
struct LegalizeVisConfig {
    std::string out_dir = "legalize_frames";
    int         upscale = 4;   // die 座標 → 像素倍率
};

class LegalizeFrameWriter {
public:
    void begin_tier(int tier, double die_w, double die_h,
                    const LegalizeVisConfig& cfg);

    // 渲染目前 module 位置並寫成 PNG
    // tag 用於檔名（允許含 '->' 等字元）
    void capture(const std::vector<Module>& modules,
                 const std::vector<Die>&    dies,
                 int tier, const std::string& tag);

    // 寫出 manifest.json，記錄本 tier 所有幀的順序
    void end_tier();

    int frame_count() const { return frame_seq_; }

private:
    int               tier_id_   = 0;
    int               pix_w_     = 0;
    int               pix_h_     = 0;
    LegalizeVisConfig cfg_;
    int               frame_seq_ = 0;
    std::vector<std::string> frame_names_;  // 僅檔名（無路徑），供 manifest
};