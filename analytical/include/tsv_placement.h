// 3D IC Analytical Floorplanner - TSV 相關資料結構
// TSV: 矽穿孔（Through-Silicon Via）型別定義與擺放求解器超參數
#pragma once

#include <vector>

// ============================================================
// TSV: 矽穿孔，連接相鄰兩層 (tier N 與 tier N+1)
// 若 net 橫跨 tier 0 與 tier 2，則需 2 個 TSV：layer 0 (tier0↔tier1)、layer 1 (tier1↔tier2)
// ============================================================
struct TSV {
    int    id;           // 全域 TSV 索引
    double x;            // 中心 X（可移動，TSV placement 優化）
    double y;            // 中心 Y（可移動）
    int    net_id;       // 所屬 net
    int    layer_index;  // 0 = 介於 tier0 與 tier1 之間，1 = 介於 tier1 與 tier2 之間
    // layer_index 即「下方 tier」的編號，TSV 連接 tier (layer_index) 與 tier (layer_index+1)
    int tier_below() const { return layer_index; }
    int tier_above() const { return layer_index + 1; }
};

// ============================================================
// TsvPlacementConfig: TSV Placement 獨立求解器的超參數
// ============================================================
struct TsvPlacementConfig {
    int    max_iterations = 1000;    // 最大迭代次數
    double init_step_size = 2.0;     // 初始步長
    double step_decay     = 0.998;   // 步長衰減（每次迭代乘以此值）
    double momentum       = 0.9;     // Nesterov 動量係數

    // TSV 物理尺寸（用於密度排斥的影響範圍，單位與 die 座標相同）
    double tsv_width  = 4.0;
    double tsv_height = 4.0;

    // 密度排斥力：使 TSV 往 module 較稀疏的空白區域移動
    // tsv_lambda 乘以密度 overflow 產生排斥梯度
    double tsv_lambda = 0.5;

    // 密度感測平滑半徑（0 = 自動設為 bin_w）
    double tsv_sigma  = 0.0;

    // 印出 progress 的間隔（0 = 不印）
    int    print_interval = 100;

    // TSV placement / compute_tsv_cost：每層 die 乘數（長度須等於 num_dies 才生效）。
    // 若為空，則與 .block 的 Weight:（PlacementEngine::tier_net_weights_）一致。
    std::vector<double> tsv_die_weights;
};
