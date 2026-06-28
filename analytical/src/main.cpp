// 3D IC Analytical Floorplanner - main entry point
// Usage: ./analytical <block_file> <net_file> [output_file] [constraint_file]
#include "floorplanner.h"
#include "legalize_heu.h"
#include "util.h"

#include <iomanip>
#include <iostream>
#include <string>
#include <chrono>

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <block_file> <net_file> [output_file] [constraint_file]"
                  << " [--leg_only] [--wl lse|wa]\n";
        return 1;
    }

    const std::string block_file = argv[1];
    const std::string nets_file  = argv[2];

    // 從 argv[3] 開始：-- 開頭視為 flag，否則依序填充 output_file / constraint_file
    bool            leg_only        = false;
    WirelengthModel wl_model        = WirelengthModel::LSE;
    std::string     output_file     = "output.txt";
    std::string     constraint_file = "";
    int             positional_idx  = 0;  // 0 = output, 1 = constraint

    for (int i = 3; i < argc; ++i) {
        const std::string opt = argv[i];
        if (opt == "--leg_only") {
            leg_only = true;
        } else if (opt == "--wl" && i + 1 < argc) {
            const std::string val = argv[++i];
            if (val == "wa" || val == "WA")
                wl_model = WirelengthModel::WA;
            else if (val == "lse" || val == "LSE")
                wl_model = WirelengthModel::LSE;
            else {
                std::cerr << "[Error] Unknown wirelength model '" << val
                          << "'. Use 'lse' or 'wa'.\n";
                return 1;
            }
        } else if (opt.size() >= 2 && opt[0] == '-' && opt[1] == '-') {
            std::cerr << "[Error] Unknown option: " << opt << "\n";
            return 1;
        } else {
            // 非 flag 的引數依序為 output_file, constraint_file
            if (positional_idx == 0)      output_file     = opt;
            else if (positional_idx == 1) constraint_file = opt;
            ++positional_idx;
        }
    }

    std::cout << "[Config] mode = " << (leg_only ? "legalize-only" : "full pipeline") << "\n";
    if (!leg_only)
        std::cout << "[Config] wirelength_model = "
                  << (wl_model == WirelengthModel::LSE ? "LSE" : "WA") << "\n";

    // ---- set hyperparameters ----
    PlacementConfig cfg;
    cfg.max_iterations  = 10000;     // 最大迭代次數 default 3000
    cfg.wirelength_model = wl_model;              // 線長平滑模型：LSE 或 WA（由 --wl 指定）
    cfg.gamma_lse       = 5.0;      // LSE 平滑參數 γ（越小越接近真實 HPWL）
    cfg.gamma_wa        = 10.0;     // WA 平滑參數 γ（ePlace；通常與 gamma_lse 獨立調整）
    cfg.init_step_size  = 1.0;      // 初始步長 default 2.0, < 1.0 >
    cfg.step_decay      = 0.999;    // 步長衰減（越小衰減越快） default 0.9998, < 0.999 >
    cfg.momentum        = 0.9;      // Nesterov 動量係數 default 0.9
    cfg.target_density  = 0.9;      // 每層目標密度 default 0.85
    cfg.bin_resolution  = 64;       // Bin 格數（每層 16x16） n100: 64
    cfg.convergence_tol = 1e-3;     // 收斂容忍度 default 1e-5
    cfg.rotation_start_iter = 0;    // Analytical 過程旋轉優化起始 iter 數（0 = 停用）
    cfg.rotation_interval   = 0;    // Analytical 過程旋轉優化 iter 間隔（0 = 停用）
    cfg.repulse_projection_alpha = 0.5;     // 每次修正 violation 量的比例（0~1）；0.5 為軟修正
    cfg.repulse_projection_passes = 5;      // 每 iter 做幾輪 Gauss-Seidel pass

    // ---- λ increasing schedule ----
    cfg.lambda_init_mult       = 200.0;     // 初始倍率（從極小值出發，讓 WL 先主導）default 0.001, n100: 200.0
    cfg.lambda_increase_rate   = 1.001;     // 每次更新倍增 20% default 1.2, n100: 1.001
    cfg.lambda_update_interval = 20;        // 每 20 次迭代更新一次
    // 1.2^k = 200/0.001 → k ≈ 66 updates = 1320 iters 後達到上限，之後保持穩定
    cfg.lambda_max_mult        = 300.0;     // λ 倍率上限, n100: 300.0

    // ---- dynamic smoothing radius σ ----
    cfg.sigma_start_frac = 0.04;     // 初始 = 20% die 寬（=53.6 for 268） n100: 0.04
    cfg.sigma_end_frac   = 0.04;    // 最終 = 5% die 寬（=18.8，略大於 bin 寬 16.75） n100: 0.04

    // base value of density penalty coefficient for each tier (multiplied by lambda_mult)
    // 實際層數在 parse_blocks 後才能確定；這裡設定「模板值」，
    // parse 後會依層數從後面取（#die < len）或從前面補第一個值（#die > len）
    const std::vector<double> tier_lambdas_template = {0.0087, 0.0087, 0.009};
    cfg.tier_lambdas = tier_lambdas_template;  // 先填入；parse 後再調整

    // ---- convergence tolerance ----
    cfg.convergence_overflow_stable_steps = 3;
    cfg.convergence_max_overflow_delta_tol = 0.001;
    cfg.convergence_total_overflow_delta_tol = 1.5;

    // ---- routing congestion（density-style 局部場模型）----
    cfg.routing_congestion_alpha = 5;
    cfg.routing_capacity_C = 1.0;
    cfg.routing_congestion_start_on_overflow_stable = true;
    cfg.routing_congestion_overflow_stable_tol = 0.005;
    cfg.routing_congestion_refresh_interval = 10;

    // ---- per-tier adaptive congestion alpha ----
    cfg.routing_congestion_max = 0.0; //(0 = 關閉)
    cfg.routing_congestion_alpha_boost_rate = 1.15;
    cfg.routing_congestion_alpha_max_mult = 50;
    // 小 module 比例門檻：area < tier_max_area / divisor 的 module 視為小 module
    // 比例 <= gate_min_ratio（預設 20%）時停用 routing_congestion_max
    // gate_divisor <= 0 停用此門檻
    cfg.routing_congestion_gate_divisor   = 10.0;
    cfg.routing_congestion_gate_min_ratio = 0.30;

    // phase 2 analytical tsv
    cfg.analytical_tsv_max_iterations = 1000;
    // 任一 tier module 面積使用率超過此值則跳過 Phase 2（走 solve_tsvs + reflow）；<=0 停用，一律 Phase 2
    cfg.tier_area_util_phase2_max = 0.80;

    // ---- die geometry normalize ----
    // 依全 die 最小邊長判斷是否縮放，僅等比縮放幾何（die/module/TSV），超參數不變
    cfg.enable_die_normalize      = true;   // false = 關閉，與舊行為完全一致
    cfg.die_normalize_target      = 280.0;  // 目標最短邊（die 座標單位）
    cfg.die_normalize_min_extent  = 200.0;  // min_edge < 此值時放大
    cfg.die_normalize_max_extent  = 400.0;  // min_edge > 此值時縮小

    // ---- analytical iter 追蹤寫檔：每 N iter 記錄非 terminal module 外框至獨立檔案 ----
    const bool   dump_analytical_iter_trace = true;
    const int    analytical_iter_trace_interval = 1000;
    if (dump_analytical_iter_trace) {
        cfg.dump_analytical_iter_trace = true;
        cfg.analytical_iter_trace_interval = analytical_iter_trace_interval;
        cfg.analytical_iter_trace_path = output_file + "_module_positions.txt";
    }

    // ---- initialize engine ----
    PlacementEngine engine(cfg);

    // ---- parse input ----
    auto t0 = std::chrono::high_resolution_clock::now();

    if (!engine.parse_blocks(block_file, leg_only)) return 1;
    if (!engine.parse_nets(nets_file))               return 1;

    // ---- 依實際 die 數量調整 tier_lambdas ----
    // #die < template length → 取後 n 個；#die > template length → 前面補第一個值
    {
        const int nd  = engine.num_dies();
        const auto& tmpl = tier_lambdas_template;
        const int tlen = static_cast<int>(tmpl.size());
        std::vector<double> adjusted;
        if (nd <= tlen) {
            // 從模板末尾取 nd 個
            adjusted.assign(tmpl.end() - nd, tmpl.end());
        } else {
            // 前面補 (nd - tlen) 個 tmpl[0]
            adjusted.assign(static_cast<size_t>(nd - tlen), tmpl[0]);
            adjusted.insert(adjusted.end(), tmpl.begin(), tmpl.end());
        }
        engine.set_tier_lambdas(adjusted);
        std::cout << "[Config] tier_lambdas adjusted for " << nd << " dies: [";
        for (int i = 0; i < nd; ++i) std::cout << (i ? ", " : "") << adjusted[i];
        std::cout << "]\n";
    }

    // compute_hpwl() 每層乘數：預設用 .block 的 Weight:。
    // 僅覆寫「報告用 HPWL」時在 parse 之後呼叫：engine.set_hpwl_die_weight_override({...});
    // analytical（LSE）線長權重始終只用 .block 的 Weight:。

    // ---- 讀取 constraint 檔（選填），套用前必須在 parse_blocks 之後 ----
    if (!constraint_file.empty())
        engine.parse_constraints(constraint_file);

    // ---- 幾何 Normalize（在 constraint 之後、initialize_positions 之前）----
    // 必須在 constraint 之後：FIXED 座標需與 block 同一座標系一起縮放
    engine.maybe_normalize_geometry();

    constexpr double base_tsv_w = 3;
    constexpr double base_tsv_h = 3;
    const double gs = engine.geometry_scale();

    // ---- 面積可行性檢查（module + 估算 TSV）----
    {
        if (!check_tier_module_tsv_area_limit(
                engine, base_tsv_w * gs, base_tsv_h * gs, 1.0)) {
            std::cerr << "[Error] Aborting: tier area exceeds 100% before placement.\n";
            return 1;
        }
        const auto tsv_cnt = estimate_tsv_count_per_tier(engine);
        const auto util    = compute_tier_module_utilization(
            engine, base_tsv_w * gs, base_tsv_h * gs, &tsv_cnt);
        std::cout << "[Check] tier module+tsv util (pre-placement):";
        for (int t = 0; t < static_cast<int>(util.size()); ++t)
            std::cout << " t" << t << "=" << std::fixed << std::setprecision(1)
                      << util[static_cast<size_t>(t)] * 100.0 << "%";
        std::cout << "\n";
    }

    // engine.set_hpwl_die_weight_override({1, 1, 1});

    if (!leg_only) {
        // ---- initialize positions ----
        engine.initialize_positions();

        // ---- (optional) square modules: convert all modules to equal-area squares before analytical ----
        // true = 啟用；false = 使用原始長寬
        const bool squarify = false;
        if (squarify) squarify_modules(engine);

        // ---- execute optimization ----
        std::cout << "\n[Solve] Starting analytical placement...\n";
        engine.solve();
    }

    // ---- record module positions after analytical ----
    const auto snap_analytical = record_positions(engine);

    // ---- build TSV list ----
    engine.build_tsvs();

    // ---- TSV Placement / Phase 2 ----
    // 物理 TSV 尺寸（die 座標單位），在 normalize 縮放空間中等比放大
    TsvPlacementConfig tcfg;
    tcfg.tsv_width      = base_tsv_w * gs;
    tcfg.tsv_height     = base_tsv_h * gs;
    tcfg.tsv_die_weights = {1, 1, 1};

    // ---- Phase 2 決策：依各 tier 面積使用率自動選流程 ----
    {
        const auto util = compute_tier_module_utilization(engine, tcfg.tsv_width, tcfg.tsv_height);
        const double thr = cfg.tier_area_util_phase2_max;
        std::cout << "[Config] tier module+tsv util:";
        for (int t = 0; t < static_cast<int>(util.size()); ++t)
            std::cout << " t" << t << "=" << std::fixed << std::setprecision(1)
                      << util[static_cast<size_t>(t)] * 100.0 << "%";
        if (thr > 0.0)
            std::cout << " (threshold=" << std::setprecision(1) << thr * 100.0 << "%)";
        std::cout << "\n";

        bool skip_phase2 = false;
        if (thr > 0.0) {
            for (int t = 0; t < static_cast<int>(util.size()); ++t) {
                if (util[static_cast<size_t>(t)] > thr) {
                    std::cout << "[Config] Phase2 skipped: tier " << t
                              << " util " << std::setprecision(1)
                              << util[static_cast<size_t>(t)] * 100.0 << "% > "
                              << thr * 100.0 << "% -> legalize+reflow\n";
                    skip_phase2 = true;
                    break;
                }
            }
        }
        cfg.enable_analytical_tsv_phase = !skip_phase2;
    }

    cfg.analytical_tsv_width          = base_tsv_w * gs;
    cfg.analytical_tsv_height         = base_tsv_h * gs;
    cfg.analytical_tsv_warmup         = true;   // Phase 2 前先 solve_tsvs() 初始化位置

    if (cfg.enable_analytical_tsv_phase) {
        if (cfg.analytical_tsv_warmup)
            engine.solve_tsvs(tcfg);         // 暖機：幾何初始化 TSV 位置
        engine.solve_tsv_phase();            // Phase 2 joint analytical
        // 跳過 post-analytical solve_tsvs()
    } else {
        engine.solve_tsvs(tcfg);             // 舊流程
    }

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
    pcfg.enable_wl_refine = true;

    // Legalize 視覺化（設 false 可零額外 I/O）
    pcfg.enable_legalize_vis   = false;
    pcfg.legalize_vis_dir      = "legalize_frames";
    pcfg.legalize_vis_upscale  = 4;
    pcfg.legalize_gif_fps      = 5;

    // Phase 2 有跑 → unified legalize（TSV 作為 proxy module）；否則走 reflow
    pcfg.legalize_tsv_as_module = cfg.enable_analytical_tsv_phase;

    // ---- Heuristic legalization flow ----
    if (pcfg.legalize_tsv_as_module) {
        engine.inject_tsv_proxies_for_legalize(pcfg.tsv_width, pcfg.tsv_height);
    }
    run_legalize_heu(engine, pcfg);

    if (pcfg.legalize_tsv_as_module) {
        engine.commit_tsv_proxies_from_legalize();
    } else {
        // 舊流程：legalize module → reflow TSV
        engine.reflow_tsvs_after_legalize(pcfg.tsv_width, pcfg.tsv_height,
                                          pcfg.tsv_reflow_congestion_order,
                                          pcfg.tsv_reflow_bbox_edge_weight);
    }
    // engine.reflow_tsvs_after_legalize(pcfg.tsv_width, pcfg.tsv_height,
    //     pcfg.tsv_reflow_congestion_order,
    //     pcfg.tsv_reflow_bbox_edge_weight);
    
    // ---- Recursive Bi-partitioning and Legalization ---- 目前沒用到
    // engine.partition_all_tiers(pcfg);

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

    // ---- Bin-edge Congestion 指標 + 視覺化（每個 die 一張 JPG，彩虹色 0-100）----
    {
        BinEdgeCongestionStats cstats = compute_bin_edge_congestion(engine);
        print_bin_edge_congestion_summary(cstats);
        // write_bin_edge_congestion_maps(engine, cstats, output_file, /*upscale=*/8);
    }

    // Die 使用率統計（module + TSV，還原後物理座標）
    {
        const auto utils = compute_tier_module_utilization(engine, base_tsv_w, base_tsv_h);
        const auto& dies = engine.dies();
        for (int t = 0; t < engine.num_dies(); ++t) {
            const Die& die = dies[static_cast<size_t>(t)];
            const double die_area = die.width * die.height;
            const double total_area = utils[static_cast<size_t>(t)] * die_area;
            std::cout << "  Die " << t << " utilization (module+tsv): "
                      << total_area << " / "
                      << die_area << " = "
                      << std::fixed << std::setprecision(1)
                      << utils[static_cast<size_t>(t)] * 100.0 << "%\n";
        }
    }

    // ---- 寫出結果（含 runtime）----
    engine.write_output(output_file, elapsed);

    // ---- 輸出各 tier 的 bin 密度圖（供除錯檢查）----
    // 產生 <output_file>_density_tier0.txt / tier1.txt / ...
    // engine.write_density_map(output_file);

    return 0;
}
