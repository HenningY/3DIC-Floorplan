// 3D IC Analytical Floorplanner - 主程式入口
// 用法：./analytical <block_file> <net_file> [output_file]
#include "floorplanner.h"

#include <iostream>
#include <string>
#include <chrono>

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <block_file> <net_file> [output_file]\n";
        return 1;
    }

    const std::string block_file  = argv[1];
    const std::string nets_file   = argv[2];
    const std::string output_file = (argc >= 4) ? argv[3] : "output.txt";

    // ---- 設定超參數 ----
    PlacementConfig cfg;
    cfg.max_iterations  = 5000;    // 最大迭代次數 default 3000
    cfg.gamma           = 5.0;    // LSE 平滑參數（越大越接近真實 HPWL） default 5.0
    cfg.init_step_size  = 3.0;     // 初始步長 default 2.0
    cfg.step_decay      = 0.999;  // 步長衰減（越小衰減越快） default 0.9998
    cfg.momentum        = 0.9;     // Nesterov 動量係數 default 0.9
    cfg.target_density  = 0.85;     // 每層目標密度 default 0.85
    cfg.bin_resolution  = 64;      // Bin 格數（每層 16x16）
    cfg.convergence_tol = 1e-5;    // 收斂容忍度 default 1e-5

    // ---- λ 遞增排程 ----
    cfg.lambda_init_mult       = 200.0; // 初始倍率（從極小值出發，讓 WL 先主導）default 0.001
    cfg.lambda_increase_rate   = 1.001;   // 每次更新倍增 20% default 1.2
    cfg.lambda_update_interval = 20;    // 每 20 次迭代更新一次
    // 1.2^k = 200/0.001 → k ≈ 66 updates = 1320 iters 後達到上限，之後保持穩定
    cfg.lambda_max_mult        = 300.0; // λ 倍率上限

    // ---- 動態平滑半徑 σ ----
    cfg.sigma_start_frac = 0.4;   // 初始 = 20% die 寬（=53.6 for 268）
    cfg.sigma_end_frac   = 0.04;   // 最終 = 5% die 寬（=18.8，略大於 bin 寬 16.75）

    // 每層 Die 的密度懲罰係數基礎值（與 lambda_mult 相乘）
    cfg.tier_lambdas    = {0.0087, 0.0087, 0.009};

    // LSE wirelength：module pin 權重 > terminal pin 權重（可調）
    cfg.wl_pin_weight_module   = 1.0;
    cfg.wl_pin_weight_terminal = 1.0;

    // ---- 初始化引擎 ----
    PlacementEngine engine(cfg);

    // ---- 解析輸入 ----
    auto t0 = std::chrono::high_resolution_clock::now();

    if (!engine.parse_blocks(block_file)) return 1;
    if (!engine.parse_nets(nets_file))    return 1;

    // ---- 初始化位置 ----
    engine.initialize_positions();

    // ---- 執行優化 ----
    std::cout << "\n[Solve] Starting analytical placement...\n";
    engine.solve();

    // ---- 建立 TSV 清單 ----
    engine.build_tsvs();

    // ---- TSV Analytical Placement ----
    TsvPlacementConfig tcfg;
    tcfg.max_iterations = 1000;
    tcfg.init_step_size = 2.0;
    tcfg.step_decay     = 0.998;
    tcfg.momentum       = 0.9;
    tcfg.tsv_width      = 3.0;   // TSV 物理寬度（die 座標單位）
    tcfg.tsv_height     = 3.0;   // TSV 物理高度
    tcfg.tsv_lambda     = 0.3;   // 密度排斥力強度
    tcfg.tsv_sigma      = 0.0;   // 0 = 自動設為 bin_w
    tcfg.print_interval = 200;   // 每 200 次印一次進度
    engine.solve_tsvs(tcfg);

    // ---- Recursive Bi-partitioning（Legalization 前準備）----
    PartitionConfig pcfg;
    pcfg.leaf_threshold  = 8;     // ≤ 8 個 module 的區域停止遞迴
    pcfg.min_modules_per_region = 3; // partition 後每個子區域至少 3 個 modules
    pcfg.min_split_ratio = 0.4;   // 切割比例限制 [0.1, 0.9]
    pcfg.max_split_ratio = 0.6;
    pcfg.num_candidates  = 64;    // 掃線候選切割數
    pcfg.tsv_width       = 3.0;   // TSV 物理尺寸（與 solve_tsvs 一致）
    pcfg.tsv_height      = 3.0;
    pcfg.log_tree        = true;
    pcfg.log_file        = output_file + "_partition_tree.txt";
    pcfg.write_positions = true;
    pcfg.positions_file  = output_file + "_partition_positions.txt";

    // ---- Recursive Bi-partitioning and Legalization ----
    engine.partition_all_tiers(pcfg);

    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t1 - t0).count();

    // ---- 最終統計 ----
    std::cout << "\n========== Summary ==========\n";
    std::cout << "  Total time   : " << elapsed << " s\n";
    std::cout << "  Final HPWL   : " << engine.compute_hpwl() << "\n";
    std::cout << "  Num dies     : " << engine.num_dies() << "\n";
    std::cout << "  Num modules  : " << engine.modules().size() << "\n";
    std::cout << "  Num nets     : " << engine.nets().size() << "\n";
    std::cout << "  Num TSVs     : " << engine.tsvs().size() << "\n";
    std::cout << "  TSV cost     : " << engine.compute_tsv_cost() << "\n";

    // Die 使用率統計
    for (int t = 0; t < engine.num_dies(); ++t) {
        double total_area = 0.0;
        for (const Module& m : engine.modules()) {
            if (!m.is_terminal && m.tier_id == t)
                total_area += m.area();
        }
        const Die& die = engine.dies()[t];
        double util = total_area / (die.width * die.height) * 100.0;
        std::cout << "  Die " << t << " utilization: "
                  << total_area << " / "
                  << (die.width * die.height) << " = "
                  << util << "%\n";
    }

    // ---- 寫出結果（含 runtime）----
    engine.write_output(output_file, elapsed);

    // ---- 輸出各 tier 的 bin 密度圖（供除錯檢查）----
    // 產生 <output_file>_density_tier0.txt / tier1.txt / ...
    engine.write_density_map(output_file);

    return 0;
}
