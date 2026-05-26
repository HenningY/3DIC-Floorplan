// 3D IC Analytical Floorplanner - main entry point
// Usage: ./analytical <block_file> <net_file> [output_file] [constraint_file]
#include "floorplanner.h"
#include "legalize_heu.h"
#include "util.h"

#include <iostream>
#include <string>
#include <chrono>

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <block_file> <net_file> [output_file] [constraint_file]\n";
        return 1;
    }

    const std::string block_file       = argv[1];
    const std::string nets_file        = argv[2];
    const std::string output_file      = (argc >= 4) ? argv[3] : "output.txt";
    const std::string constraint_file  = (argc >= 5) ? argv[4] : "";

    // ---- set hyperparameters ----
    PlacementConfig cfg;
    cfg.max_iterations  = 10000;     // 最大迭代次數 default 3000
    cfg.gamma           = 5.0;      // LSE 平滑參數（越大越接近真實 HPWL） default 5.0
    cfg.init_step_size  = 1.0;      // 初始步長 default 2.0, n100: 3.0
    cfg.step_decay      = 0.999;    // 步長衰減（越小衰減越快） default 0.9998, 0.999
    cfg.momentum        = 0.9;      // Nesterov 動量係數 default 0.9
    cfg.target_density  = 0.9;      // 每層目標密度 default 0.85
    cfg.bin_resolution  = 64;       // Bin 格數（每層 16x16） n100: 64
    cfg.convergence_tol = 1e-3;     // 收斂容忍度 default 1e-5
    cfg.rotation_start_iter = 0;    // Analytical 過程旋轉優化起始 iter 數（0 = 停用）
    cfg.rotation_interval   = 0;    // Analytical 過程旋轉優化 iter 間隔（0 = 停用）

    // ---- λ increasing schedule ----
    cfg.lambda_init_mult       = 200.0;     // 初始倍率（從極小值出發，讓 WL 先主導）default 0.001, n100: 200.0
    cfg.lambda_increase_rate   = 1.001;     // 每次更新倍增 20% default 1.2, n100: 1.001
    cfg.lambda_update_interval = 20;        // 每 20 次迭代更新一次
    // 1.2^k = 200/0.001 → k ≈ 66 updates = 1320 iters 後達到上限，之後保持穩定
    cfg.lambda_max_mult        = 300.0;     // λ 倍率上限, n100: 300.0

    // ---- dynamic smoothing radius σ ----
    cfg.sigma_start_frac = 0.04;     // 初始 = 20% die 寬（=53.6 for 268） n100: 0.4
    cfg.sigma_end_frac   = 0.04;    // 最終 = 5% die 寬（=18.8，略大於 bin 寬 16.75） n100: 0.04

    // base value of density penalty coefficient for each tier (multiplied by lambda_mult)
    cfg.tier_lambdas    = {0.0087, 0.0087, 0.009};

    // ---- convergence tolerance ----
    cfg.convergence_overflow_stable_steps = 3;
    cfg.convergence_max_overflow_delta_tol = 0.001;
    cfg.convergence_total_overflow_delta_tol = 1.5;

    // ---- routing congestion ----
    cfg.routing_congestion_alpha = 0;
    cfg.routing_capacity_C = 1.0;
    cfg.routing_sigmoid_eps = 1.0;
    cfg.routing_bbox_margin_bins = 1;
    cfg.routing_congestion_start_iter = 2000;
    cfg.routing_congestion_refresh_interval = 100;

    // ---- die geometry normalize ----
    // 依全 die 最小邊長判斷是否縮放，僅等比縮放幾何（die/module/TSV），超參數不變
    cfg.enable_die_normalize      = true;   // false = 關閉，與舊行為完全一致
    cfg.die_normalize_target      = 280.0;  // 目標最短邊（die 座標單位）
    cfg.die_normalize_min_extent  = 200.0;  // min_edge < 此值時放大
    cfg.die_normalize_max_extent  = 400.0;  // min_edge > 此值時縮小

    // ---- analytical iter 追蹤寫檔：true 時與 [Iter ...] 同頻率寫入非 terminal module 外框 ----
    const bool dump_analytical_iter_trace = false;
    if (dump_analytical_iter_trace) {
        cfg.dump_analytical_iter_trace = true;
        cfg.analytical_iter_trace_path = output_file + "_analytical_iter.txt";
    }

    // ---- initialize engine ----
    PlacementEngine engine(cfg);

    // ---- parse input ----
    auto t0 = std::chrono::high_resolution_clock::now();

    if (!engine.parse_blocks(block_file)) return 1;
    if (!engine.parse_nets(nets_file))    return 1;

    // compute_hpwl() 每層乘數：預設用 .block 的 Weight:。
    // 僅覆寫「報告用 HPWL」時在 parse 之後呼叫：engine.set_hpwl_die_weight_override({...});
    // analytical（LSE）線長權重始終只用 .block 的 Weight:。

    // ---- 讀取 constraint 檔（選填），套用前必須在 parse_blocks 之後 ----
    if (!constraint_file.empty())
        engine.parse_constraints(constraint_file);

    // ---- 幾何 Normalize（在 constraint 之後、initialize_positions 之前）----
    // 必須在 constraint 之後：FIXED 座標需與 block 同一座標系一起縮放
    engine.maybe_normalize_geometry();

    // engine.set_hpwl_die_weight_override({1, 1, 1});

    // ---- initialize positions ----
    engine.initialize_positions();

    // ---- (optional) square modules: convert all modules to equal-area squares before analytical ----
    // true = 啟用；false = 使用原始長寬
    const bool squarify = false;
    if (squarify) squarify_modules(engine);

    // ---- execute optimization ----
    std::cout << "\n[Solve] Starting analytical placement...\n";
    engine.solve();

    // ---- record module positions after analytical ----
    const auto snap_analytical = record_positions(engine);

    // ---- build TSV list ----
    engine.build_tsvs();

    // ---- TSV Placement ----
    // 物理 TSV 尺寸（die 座標單位），在 normalize 縮放空間中等比放大
    const double base_tsv_w = 3;
    const double base_tsv_h = 3;
    const double gs = engine.geometry_scale();  // 1.0 若未觸發 normalize

    TsvPlacementConfig tcfg;
    tcfg.tsv_width      = base_tsv_w * gs;  // scaled 座標空間的 TSV 寬
    tcfg.tsv_height     = base_tsv_h * gs;  // scaled 座標空間的 TSV 高
    // TSV cost 每層乘數：留空則與 .block 的 Weight: 相同；若要覆寫例如：
    tcfg.tsv_die_weights = {1, 1, 1};
    engine.solve_tsvs(tcfg);

    // ---- Recursive Bi-partitioning（Legalization 前準備）---- 目前沒用到
    PartitionConfig pcfg;
    pcfg.leaf_threshold  = 8;     // ≤ 8 個 module 的區域停止遞迴
    pcfg.min_modules_per_region = 1; // partition 後每個子區域至少 2 個 modules
    pcfg.min_split_ratio = 0.4;   // 切割比例限制 [0.4, 0.6]
    pcfg.max_split_ratio = 0.6;
    pcfg.num_candidates  = 64;    // 掃線候選切割數
    pcfg.max_split_retries = 32;
    pcfg.tsv_width       = base_tsv_w * gs;  // scaled 座標空間
    pcfg.tsv_height      = base_tsv_h * gs;
    // true：TSV reflow 依 congestion map 選低壅塞插槽；false 維持 first-fit（預設）
    pcfg.tsv_reflow_congestion_order = true;
    // 壅塞選位時的邊界懲罰權重：離合法 bbox 越近 cost 越大；0 則停用邊界懲罰
    pcfg.tsv_reflow_bbox_edge_weight = 0;
    pcfg.log_tree        = true;
    // pcfg.log_file        = output_file + "_partition_tree.txt";
    pcfg.write_positions = true;
    // pcfg.positions_file  = output_file + "_partition_positions.txt";

    // ---- Heuristic legalization flow (WIP) ----
    run_legalize_heu(engine, pcfg);

    // ---- Recursive Bi-partitioning and Legalization ---- 目前沒用到
    // engine.partition_all_tiers(pcfg);

    // ---- TSV：sort by net bbox length, first-fit / congestion-order in two bbox geometric regions ----
    engine.reflow_tsvs_after_legalize(pcfg.tsv_width, pcfg.tsv_height, pcfg.tsv_reflow_congestion_order, pcfg.tsv_reflow_bbox_edge_weight);

    // ---- 記錄 legalization 完成後的 module 位置，並輸出位移報告 ----
    // const auto snap_legal = record_positions(engine);
    // write_displacement_report(snap_analytical, snap_legal, output_file + "_displacement.txt");

    // ---- 還原物理座標（必須在 Summary / HPWL / write_output / congestion 之前）----
    engine.restore_geometry();

    auto t1 = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t1 - t0).count();

    // ---- 最終統計 ----
    // true  = 只計算「整條 net 的所有 pin 都在同一層」的 HPWL（跨層 net 排除）
    // false = 計算完整 HPWL（含跨層 net，行為與原本相同）
    const bool intra_die_hpwl_only = false;

    std::cout << "\n========== Summary ==========\n";
    std::cout << "  Total time   : " << elapsed << " s\n";
    if (intra_die_hpwl_only) {
        print_intra_die_stats(compute_intra_die_hpwl(engine));
    } else {
        std::cout << "  Final HPWL   : " << engine.compute_hpwl() << "\n";
    }
    std::cout << "  Num dies     : " << engine.num_dies() << "\n";
    std::cout << "  Num modules  : " << engine.modules().size() << "\n";
    std::cout << "  Num nets     : " << engine.nets().size() << "\n";
    std::cout << "  Num TSVs     : " << engine.tsvs().size() << "\n";
    std::cout << "  TSV cost     : " << engine.compute_tsv_cost() << "\n";

    // ---- 最終重疊檢查（summary 印出每一層 overlap pairs）----
    // 還原後使用物理 TSV 尺寸
    print_overlap_report(engine, base_tsv_w, base_tsv_h);

    // ---- Bin-edge Congestion 指標 + 視覺化（每個 die 一張 PPM）----
    {
        BinEdgeCongestionStats cstats = compute_bin_edge_congestion(engine);
        print_bin_edge_congestion_summary(cstats);
        write_bin_edge_congestion_maps(engine, cstats, output_file, /*upscale=*/8);
    }

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
    // engine.write_density_map(output_file);

    return 0;
}
