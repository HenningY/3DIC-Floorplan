// 3D IC Analytical Floorplanner - Header
// 支援異質整合的多層 Die 解析式擺放引擎
// 使用 LSE Wirelength Model + Bin-based Density Model
// 優化方法: Nesterov's Accelerated Gradient (NAG)
#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <limits>

// ============================================================
// Module: 代表一個硬體方塊或 I/O 端點
// ============================================================
struct Module {
    int         id;           // 全域索引
    std::string name;         // 方塊名稱 (如 "sb0", "p1")
    double      width;        // 寬度
    double      height;       // 高度
    double      x;            // 中心座標 X (可移動)
    double      y;            // 中心座標 Y (可移動)
    int         tier_id;      // 所屬 Die（0-indexed），terminal 為 -1
    bool        is_terminal;  // 是否為固定 I/O 端點
    bool        is_fixed = false; // 是否被 constraint 固定（位置不可移動）
    bool        is_soft  = false; // true = soft module（.block 行尾 s）；預設 hard
    double      move_weight = 1.0; // heuristic legalize 權重（越大越不希望與其重疊）
    bool        is_tsv_proxy = false; // true = 由 TSV 暫時注入的 proxy，legalize 後移除
    int         tsv_proxy_id = -1;   // 對應 tsvs_[tsv_proxy_id]

    double area() const { return width * height; }
    double lx()   const { return x - width  * 0.5; }   // 左邊界
    double ly()   const { return y - height * 0.5; }   // 下邊界
    double rx()   const { return x + width  * 0.5; }   // 右邊界
    double ry()   const { return y + height * 0.5; }   // 上邊界
};

// ============================================================
// Net: 連線，包含所有連接 Module 的索引
// ============================================================
struct Net {
    int              id;
    std::string      name;           // Net 名稱（如 "net33"）
    std::vector<int> pins;           // Module 索引（同時含 terminal 和 block）
    bool             is_cross_tier = false;  // 可移動方塊是否跨越多個 tier
    int              min_tier = -1;  // 此 net 中 pin 的最小 tier（terminal 視為 0）
    int              max_tier = -1;  // 此 net 中 pin 的最大 tier（由 build_tsvs 等填寫）
};

#include "tsv_placement.h"

// ============================================================
// Bin: 密度格的最小單元
// ============================================================
struct Bin {
    double cx;             // Bin 中心 X
    double cy;             // Bin 中心 Y
    double w;              // Bin 寬
    double h;              // Bin 高
    double density;        // 當前密度 ρ_ab
    double target_density; // 目標密度 ρ_t（通常 0.8~1.0）
};

// ============================================================
// Die: 一層晶粒的幾何資訊與密度格
// ============================================================
struct Die {
    int    id;
    double width;         // Die 寬度
    double height;        // Die 高度
    double lambda;        // 密度懲罰係數 λ（每層可不同）

    int    bin_rows;      // 密度格行數
    int    bin_cols;      // 密度格列數
    double bin_w;         // 每格寬
    double bin_h;         // 每格高

    std::vector<Bin> bins;  // 一維展開的 bins，index = row * bin_cols + col

    // 根據座標取 Bin 索引（若超界回傳 -1）
    int bin_index(double x, double y) const {
        int col = static_cast<int>((x / width)  * bin_cols);
        int row = static_cast<int>((y / height) * bin_rows);
        col = std::max(0, std::min(bin_cols - 1, col));
        row = std::max(0, std::min(bin_rows - 1, row));
        return row * bin_cols + col;
    }
};

// Wirelength surrogate model 選擇
// LSE: Log-Sum-Exp（原預設）; WA: Weighted-Average (ePlace)
enum class WirelengthModel { LSE, WA };

// ============================================================
// BinEdgeDemands：可增量更新的 H/V edge demand 陣列
//   H[t][hr * C + hc]  hr∈[0,R], hc∈[0,C-1]
//   V[t][vc * R + vr]  vc∈[0,C], vr∈[0,R-1]
// ============================================================
struct BinEdgeDemands {
    std::vector<std::vector<double>> H;  // 長度 (R+1)*C，per tier
    std::vector<std::vector<double>> V;  // 長度 (C+1)*R，per tier
};

// ============================================================
// PlacementConfig: 優化超參數集合
// ============================================================
struct PlacementConfig {
    int    max_iterations   = 3000;    // 最大迭代次數
    WirelengthModel wirelength_model = WirelengthModel::LSE; // 線長平滑模型
    double gamma_lse        = 10.0;    // LSE 平滑參數 γ（越大越接近真實 HPWL）
    double gamma_wa         = 10.0;    // WA 平滑參數 γ（ePlace；γ→0 趨近真實 HPWL）
    double init_step_size   = 1.0;     // 初始步長
    double step_decay       = 0.9999;  // 步長衰減率（每次迭代乘以此值）
    double momentum         = 0.9;     // Nesterov 動量係數
    double target_density   = 0.85;     // 目標密度
    int    bin_resolution   = 16;      // 每層 bin 數量 (bin_resolution x bin_resolution)
    double convergence_tol  = 1e-4;    // 收斂容忍度（相對 HPWL 變化）

    // ---- 提早停止：密度 overflow 穩定判斷（每 50 iter 記錄一次時比較）----
    // 若連續 convergence_overflow_stable_steps 次（每次相隔 50 iter）皆滿足：
    //   |Δ max_overflow| < convergence_max_overflow_delta_tol
    //   且 |Δ TotalOverflow| < convergence_total_overflow_delta_tol
    // 則視為 overflow 已穩定（搭配 iter 下限與 rel_change 使用）。
    int    convergence_overflow_stable_steps      = 3;     // 連續 n 次「每 50 iter」檢查
    double convergence_max_overflow_delta_tol     = 0.001; // max_overflow 相鄰兩次變化上限
    double convergence_total_overflow_delta_tol   = 1.5;   // TotalOverflow 相鄰兩次變化上限

    // ---- λ 遞增排程 ----
    // lambda_mult 每 lambda_update_interval 次迭代乘以 lambda_increase_rate，
    // 但不超過 lambda_max_mult（防止密度力爆炸）
    double lambda_init_mult      = 0.01;   // λ 初始倍率（從小開始）
    double lambda_increase_rate  = 1.2;    // 每次更新的倍增比率
    int    lambda_update_interval = 20;    // 更新間隔（iterations）
    double lambda_max_mult        = 200.0; // λ 倍率上限（防止密度力主導崩潰）

    // ---- 動態平滑半徑 σ ----
    // 初始 σ 為 die 寬度的 sigma_start_frac，線性退火至 sigma_end_frac
    double sigma_start_frac = 0.20;  // 初始平滑半徑（die 寬度的 20%）
    double sigma_end_frac   = 0.07;  // 最終平滑半徑（die 寬度的 7%，≥ bin 寬）

    // 每層的密度懲罰係數 λ；若向量長度 < num_dies 則用最後一個值填充
    std::vector<double> tier_lambdas = {0.01, 0.01, 0.01};

    // ---- 週期性旋轉優化（以 bin density overflow 做代理指標）----
    // rotation_start_iter : 從第幾個 iter 開始嘗試旋轉（0 = 停用）
    // rotation_interval   : 之後每隔幾個 iter 做一次（0 = 停用）
    int rotation_start_iter = 4000;
    int rotation_interval   = 500;

    // ---- analytical 進度追蹤寫檔（與 [Iter ...] 同頻率，每 50 iter）----
    // dump_analytical_iter_trace = true 時，每次印 [Iter N] 同時把該輪所有**非 terminal**
    // module 的 name llx lly urx ury 附加寫入 analytical_iter_trace_path（同一檔案累加）。
    bool        dump_analytical_iter_trace = false;
    std::string analytical_iter_trace_path;

    // 僅影響 compute_hpwl() 報告／評估用乘數（長度須等於 num_dies 才會生效）：
    // 若為空，compute_hpwl 使用 .block 的 Weight:（tier_net_weights_）。
    // LSE / analytical 線長梯度始終只用 .block 的 Weight:，不受此欄位影響。
    std::vector<double> hpwl_die_weights;

    // ---- Routability-Driven Congestion Penalty（density-style 局部場模型）----
    // alpha = 0 → 整個壅塞懲罰關閉（early-return，不影響速度）。
    // 模型：先建 H/V edge demand 快取（update_routing_congestion_map），
    //       再對附近 module 以 bell 核計算斥力（H→gy，V→gx）。
    double routing_congestion_alpha      = 0.5;  // 懲罰強度；0 = 關閉
    double routing_capacity_C            = 1.0;  // 每條邊的 routing capacity（全域常數）

    // 週期性計算：
    //   iter >= routing_congestion_start_iter
    //   且 (iter - routing_congestion_start_iter) % routing_congestion_refresh_interval == 0
    // 時刷新 map 並重算梯度；其餘 iter 直接將 gx_rc/gy_rc 清零。
    int routing_congestion_start_iter       = 1000;   // 從第幾個 iter 開始啟用（n）
    int routing_congestion_refresh_interval = 100;   // 每幾個 iter 重算一次（m）

    // ---- Per-tier 自適應壅塞強度 ----
    // routing_congestion_max > 0 時啟用：每隔 routing_congestion_refresh_interval 次
    // 從快取 map 算各層 edge max（所有 H/V edge demand 的最大值）：
    //   tier_max >  routing_congestion_max → 該層 alpha 倍率 × boost_rate
    //   tier_max <= routing_congestion_max → 該層 alpha 倍率 ÷ boost_rate（最小 1.0）
    // 設 0 關閉自適應（tier_rc_alpha_mult_ 全程維持 1.0）。
    double routing_congestion_max            = 15.0;   // 每層 edge max 的目標上限（0=停用）
    double routing_congestion_alpha_boost_rate = 5; // 每次調整的倍率幅度
    double routing_congestion_alpha_max_mult   = 1000.0;// alpha 倍率上限（防止爆炸）

    // ---- Congestion 啟用門檻（小 module 比例）----
    // area < 各層最大 module 面積 / divisor 的 module 視為「小 module」
    // 全部可動 module 中小 module 比例 <= gate_min_ratio 時，將 routing_congestion_max 強制設為 0（停用）
    // gate_divisor <= 0 → 停用此門檻，直接使用 routing_congestion_max 設定值
    double routing_congestion_gate_divisor   = 0.0;   // <=0 停用門檻
    double routing_congestion_gate_min_ratio = 0.20;  // 預設 20%


    // ---- Phase 2 Analytical TSV ----
    // 面積使用率門檻：build_tsvs() 後若任一 tier 的 module 面積佔比 > 此值，跳過 Phase 2
    // （改走 solve_tsvs + reflow_tsvs_after_legalize）；<=0 停用檢查，一律走 Phase 2。
    double tier_area_util_phase2_max      = 0.70;
    // true：solve() 後 build_tsvs() 然後再跑一輪 joint analytical（module + TSV 一起移動）
    bool   enable_analytical_tsv_phase    = false;
    int    analytical_tsv_max_iterations  = 1000;  // Phase 2 最大迭代次數
    double analytical_tsv_width           = 3.0;   // TSV 小矩形寬（density/congestion bell 半徑用）
    double analytical_tsv_height          = 3.0;   // TSV 小矩形高
    bool   analytical_tsv_warmup          = true;  // Phase 2 前是否先 solve_tsvs() 初始化位置
    // Phase 2 congestion 對 TSV 上下層各算一次後累加，可能力道偏大；此係數縮放 TSV congestion 梯度
    double analytical_tsv_congestion_scale = 0.5;

    // ---- 幾何 Normalize（縮放 die/module 到目標邊長再還原）----
    // 依全 die 最小邊長判斷是否觸發縮放，僅等比縮放幾何資料；超參數不變
    bool   enable_die_normalize      = false;  // 總開關
    double die_normalize_target      = 268.0;  // 目標最短邊（scaled 後 min_edge ≈ target）
    double die_normalize_min_extent  = 100.0;  // min_edge < 此值時放大（Case A）
    double die_normalize_max_extent  = 400.0;  // min_edge > 此值時縮小（Case B）
};


// ============================================================
// PartitionConfig: Recursive Bi-partitioning 超參數
// ============================================================
struct PartitionConfig {
    int    leaf_threshold  = 8;     // 區域內 module 數 ≤ 此值時停止遞迴
    int    min_modules_per_region = 3; // 每個子區域最少 module 數（避免切到太碎）
    double min_split_ratio = 0.1;   // 切割位置不能靠近邊界的最小比例
    double max_split_ratio = 0.9;   // 對稱的最大比例
    int    num_candidates  = 32;    // 掃線候選切割位置數量
    // validate 失敗，或（左右皆為 leaf 時）legalize 後仍有重疊，則改試下一候選切分線
    int    max_split_retries = 32;
    double tsv_width       = 4.0;   // TSV 物理寬（面積計算用）
    double tsv_height      = 4.0;   // TSV 物理高（面積計算用）
    // true：TSV reflow 改用 bin-edge congestion map 選低壅塞插槽（增量更新）；false 維持 first-fit
    bool   tsv_reflow_congestion_order = false;
    // 壅塞次序選位時，離合法 bbox 邊界越近越貴：mean(1/(min_dist+eps)) 乘此權重後加入 total score
    // ≤0 → 只看壅塞，無邊界懲罰（相容舊行為）
    double tsv_reflow_bbox_edge_weight = 1.0;
    bool   log_tree        = true;  // 是否把 partition tree 寫到 log 檔
    std::string log_file   = "partition_tree.txt";

    // 將 partition 後（push-back / flush 到切割線）的 module/TSV 位置寫檔
    bool        write_positions = true;
    std::string positions_file  = "partition_positions.txt";

    // true：legalization 後在無 overlap 的 tier 執行 WL refinement（net-centroid 拉力）
    bool enable_wl_refine = true;

    // true：legalize 前把 TSV 注入為 proxy module，一起做 overlap 消除；
    //        legalize 後把座標寫回 tsvs_ 並移除 proxy，跳過 reflow_tsvs_after_legalize。
    // false：舊流程（legalize module → reflow TSV）
    bool legalize_tsv_as_module = true;

    // normalized space 中，module 最短邊超過此值視為「大型 module」，啟用 strong legalize
    double legalize_large_module_min_edge = 60.0;

    // Legalize 視覺化（enable=false 時零額外 I/O）
    bool        enable_legalize_vis   = false;
    std::string legalize_vis_dir      = "legalize_frames";
    int         legalize_vis_upscale  = 4;
    int         legalize_gif_fps      = 5;   // 傳給 Python（imageio duration = 1/fps）
};

// ============================================================
// PartitionNode: Recursive Partition Tree 的節點
//   每個節點記錄一個矩形子區域以及其內容
// ============================================================
struct PartitionNode {
    // 幾何邊界
    double xmin, ymin, xmax, ymax;
    int    tier_id;
    int    depth;             // 遞迴深度（根 = 0）

    // 本節點包含的 module/TSV 索引（模組中心在此矩形內）
    std::vector<int> module_ids;  // 指向全域 modules_ 的索引
    std::vector<int> tsv_ids;     // 指向全域 tsvs_ 的索引

    // 子節點（葉節點兩者皆為 nullptr）
    std::unique_ptr<PartitionNode> left;   // 切割線左側 / 下側
    std::unique_ptr<PartitionNode> right;  // 切割線右側 / 上側

    // 切割資訊（葉節點無意義）
    bool   split_x   = true;   // true = 垂直切割（沿 X 軸），false = 水平切割
    double split_pos = 0.0;    // 切割線的座標值

    // 若在 partition 內已對 leaf 做過 legalize_leaf，dfs_legalize 不再重複呼叫
    bool skip_leaf_legalize = false;

    bool is_leaf() const { return !left && !right; }
    double width()  const { return xmax - xmin; }
    double height() const { return ymax - ymin; }
    double area()   const { return width() * height(); }
};

// ============================================================
// RepulsionGroup: 斥力群組（來自 constraint 檔的 REPULSE 行）
// 群組內所有成員兩兩互斥，analytical 階段施加反平方斥力梯度
// 支援跨 tier（以 2D 平面座標計算，不考慮層間 Z 距離）
// terminal 可加入群組：自身不移動（不累加梯度），但會推開群組內其他可動 module
// ============================================================
struct RepulsionGroup {
    std::vector<int> module_ids; // 參與的 module 索引（含 terminal，解析時不過濾）
    double           strength;   // 斥力強度係數 k（由 constraint 檔指定）
};

// ============================================================
// PlacementEngine: 解析式擺放主引擎
// ============================================================
class PlacementEngine {
public:
    // 建構子：指定層數與每層的 Die 尺寸
    PlacementEngine(const PlacementConfig& cfg = PlacementConfig());

    // ---- 輸入解析 ----
    // placed_format=true: block 行格式為 "name tier llx lly urx ury [s]"（初始位置已給定）
    // placed_format=false（預設）: 原始格式 "name w h tier [s]"
    bool parse_blocks(const std::string& filename, bool placed_format = false);
    bool parse_nets(const std::string& filename);

    // ---- 主要介面 ----
    // 將所有 module 初始化至各自 Die 的中心（加入隨機擾動）
    void initialize_positions();

    // 執行解析式優化主迴圈
    void solve();

    // ---- Constraint ----
    // 讀取 .constraint 檔，格式：FIXED <name> <x_ll> <y_ll> <x_ur> <y_ur>
    // 會設定對應 Module 的 is_fixed=true，並更新 x/y/width/height（以 ll/ur 決定）
    bool parse_constraints(const std::string& filename);

    // ---- TSV ----
    // 依 cross-tier nets 建立 TSV 清單（需在 parse + module 位置固定後呼叫）
    // 規則：net 橫跨 tier [min_tier, max_tier] 時，在 layer 0..(max_tier-min_tier-1) 各放一個 TSV
    void build_tsvs();

    // TSV Placement 求解器
    // 所有 Module 座標固定，只更新 TSV (x,y)
    // 目標：最小化 dist(TSV, B_lower) + dist(TSV, B_upper)，同時避開 module 密集區
    void solve_tsvs(const TsvPlacementConfig& tcfg = TsvPlacementConfig());

    // 計算目前 TSV 的總 wirelength cost（評估用）
    // = Σ_tsv [ w_lo*dist(tsv,B_lower) + w_hi*dist(tsv,B_upper) ]；
    // w_lo/w_hi 見最近一次 solve_tsvs 的 TsvPlacementConfig::tsv_die_weights（空則用 .block）
    double compute_tsv_cost() const;

    // Legalization 完成後：依所屬 net 全體 bbox 之周長（小→大）排序，
    // 將各 TSV 以 B_lower/B_upper 幾何區域為範圍做 first-fit；區域內無空位則改在整張 die 上
    // 找 L1 距離至該目標矩形最近之空位。
    // use_congestion_order=true：在同一合法區域內改以 bin-edge congestion map 選最低壅塞插槽，
    //   並在每次放置後增量更新受影響 tier 的 demand；fallback（無候選）時仍退化為 nearest。
    void reflow_tsvs_after_legalize(double tsv_w, double tsv_h,
                                    bool use_congestion_order = false,
                                    double congestion_bbox_edge_weight = 1.0);

    // ---- 輸出 ----
    // 寫出符合 PA2 格式的結果檔：
    //   Line 1: total_cost
    //   Line 2: hpwl
    //   Line 3: bounding_box_area
    //   Line 4: bbox_w  bbox_h
    //   Line 5: runtime (秒)
    //   Line 6+: <name> <die_id> <x_ll> <y_ll> <x_ur> <y_ur>
    void write_output(const std::string& filename, double runtime = 0.0) const;
    // 輸出各 tier 的 bin 密度圖（數值表格 + ASCII + PNG）
    // 每層：<base>_density_tier<N>.txt、<base>_density_tier<N>.png（每 bin 一像素）
    void write_density_map(const std::string& base_filename) const;
    double compute_hpwl() const;  // 計算真實 HPWL（評估用）

    // ---- 公開的優化計算（供測試與除錯）----
    double compute_lse_wirelength() const;
    // gx_tsv/gy_tsv: 若非 nullptr 且 analytical_tsv_active_，累加 per-tier TSV 梯度（長度=tsvs_.size()）
    void   calculate_wirelength_gradient(std::vector<double>& gx,
                                         std::vector<double>& gy,
                                         std::vector<double>* gx_tsv = nullptr,
                                         std::vector<double>* gy_tsv = nullptr) const;

    void   update_density_map();
    // gx_tsv/gy_tsv: 若非 nullptr 且 analytical_tsv_active_，輸出 TSV 密度梯度（長度=tsvs_.size()）
    void   calculate_density_gradient(std::vector<double>& gx,
                                      std::vector<double>& gy,
                                      std::vector<double>* gx_tsv = nullptr,
                                      std::vector<double>* gy_tsv = nullptr) const;

    // ---- Recursive Partitioning（Legalization 準備）----
    // 對所有 tier 做 Recursive Bi-partitioning，
    // 結果用 partition_all_tiers() 統一觸發並寫 log
    void partition_all_tiers(const struct PartitionConfig& pcfg);

    // ---- Phase 2 Analytical TSV ----
    // 以 TSV 為 movable entity 繼續 analytical（需先 build_tsvs()）
    void solve_tsv_phase();

    // ---- 訪問器 ----
    const std::vector<Module>& modules()  const { return modules_; }
    std::vector<Module>&       modules_mutable() { return modules_; }
    const std::vector<Net>&    nets()     const { return nets_; }
    const std::vector<Die>&    dies()     const { return dies_; }
    const std::vector<TSV>&    tsvs()     const { return tsvs_; }
    // net_id → [tsv 索引] 正向索引；build_tsvs() 後有效
    const std::vector<std::vector<int>>& net_to_tsvs() const { return net_to_tsvs_; }
    int                        num_dies() const { return static_cast<int>(dies_.size()); }

    // 由 .block 的 Weight: 行讀入；每層 die 一個係數（供之後 weighted net 使用）
    const std::vector<double>& tier_net_weights() const { return tier_net_weights_; }

    // 取得當前 PlacementConfig（供外部模組如 routing_congestion 讀取超參數）
    const PlacementConfig& config() const { return cfg_; }

    // ---- Routing Congestion Map（density-style 場模型）----
    // 建立/刷新 H/V edge demand 快取（供 calculate_routing_congestion_gradient 使用）
    void update_routing_congestion_map();
    // 讀取快取（供 routing_congestion.cpp 的 gradient 計算讀取）
    const BinEdgeDemands& routing_edge_demands() const { return rc_edge_demands_; }
    // 讀取當前迭代的平滑半徑 σ（供 routing_congestion.cpp 決定影響半徑）
    double smooth_sigma() const { return smooth_sigma_; }

    // 僅覆寫 compute_hpwl() 的每層乘數；analytical（LSE）仍只用 .block 的 Weight:
    void set_hpwl_die_weight_override(std::vector<double> w) { cfg_.hpwl_die_weights = std::move(w); }
    // 覆寫各 tier 的基礎密度懲罰係數（parse 後依實際 die 數量調整用）
    void set_tier_lambdas(std::vector<double> v) { cfg_.tier_lambdas = std::move(v); }

    // 覆寫 TSV placement 用的每層乘數（可不經 solve_tsvs 先設）；tsv_die_weights 空則跟 .block
    void set_tsv_placement_config(const TsvPlacementConfig& cfg) { tsv_placement_cfg_ = cfg; }
    const TsvPlacementConfig& tsv_placement_config() const { return tsv_placement_cfg_; }

    // ---- TSV Proxy for Unified Legalization ----
    // 把每個 TSV 以小矩形（tsv_w × tsv_h）注入 modules_ 作為 proxy（tier_id = tier_below）
    // 供 run_legalize_heu 與真實 module 一起做 overlap 消除
    void inject_tsv_proxies_for_legalize(double tsv_w, double tsv_h);
    // legalize 結束後把 proxy 的 (x,y) 寫回 tsvs_，再從 modules_ 移除所有 proxy
    void commit_tsv_proxies_from_legalize();

    // ---- 幾何 Normalize ----
    // parse_blocks + parse_constraints 之後呼叫；依 PlacementConfig 判斷是否縮放
    // 回傳 true 表示有縮放（geometry_scale_ != 1）
    bool maybe_normalize_geometry();
    // 將所有 die/module/tsv 幾何資料乘以 factor（並重建 bin 網格）
    void apply_geometry_scale(double factor);
    // 還原到物理座標（geometry_scale_ 歸 1）；在 Summary/HPWL/write_output 之前呼叫
    void restore_geometry();
    // 查詢目前有效縮放比（供 main 計算 scaled TSV 寬高）
    double geometry_scale() const { return geometry_scale_; }

private:
    PlacementConfig cfg_;

    std::vector<Module> modules_;
    std::vector<Net>    nets_;
    std::vector<Die>    dies_;
    std::vector<TSV>    tsvs_;  // 跨層 net 產生的 TSV，供後續 TSV placement 使用
    std::vector<RepulsionGroup> repulsion_groups_; // REPULSE constraint 群組

    // 每層 die 的 net 權重（.block 中 Weight: w0 w1 ...；預設全為 1.0）
    std::vector<double> tier_net_weights_;

    // ---- 幾何 Normalize 狀態 ----
    double geometry_scale_ = 1.0;              // 目前縮放比（1 = 未縮放）
    std::vector<double> orig_die_w_;           // parse 後原始 Outline 寬
    std::vector<double> orig_die_h_;           // parse 後原始 Outline 高

    // 最近一次 solve_tsvs(tcfg) 的設定（含 tsv_die_weights）；供 compute_tsv_cost 等使用
    TsvPlacementConfig tsv_placement_cfg_;

    // 名稱 → 模組索引的映射（解析 .nets 用）
    std::unordered_map<std::string, int> name_to_id_;

    // Nesterov 加速梯度的歷史狀態
    std::vector<double> prev_x_, prev_y_;         // x_{k-1}（module）
    std::vector<double> vel_x_,  vel_y_;          // 動量向量
    std::vector<double> prev_tsv_x_, prev_tsv_y_; // x_{k-1}（TSV，Phase 2 用）

    // ---- Routing Congestion Map 快取 ----
    BinEdgeDemands rc_edge_demands_;

    // ---- 動態狀態（每次迭代更新）----
    // 當前 σ 平滑半徑，供 update_density_map / calculate_density_gradient 使用
    double smooth_sigma_ = 0.0;
    // 當前 λ 倍率，在 solve() 內每 lambda_update_interval 次迭代遞增
    double lambda_mult_  = 1.0;
    // NAG 步長：Phase 1 結束時寫入，Phase 2 從此值繼續（不再重置為 init_step_size）
    double nag_step_     = 0.0;
    // 跨 Phase 累積迭代次數（σ / λ / RC 排程與 log 皆用此值，Phase 2 接續 Phase 1）
    int    nag_iter_     = 0;
    // Per-tier routing congestion alpha 動態倍率（routing_congestion_max > 0 時更新）
    std::vector<double> tier_rc_alpha_mult_;

    // Phase 2 Analytical TSV 啟用旗標（由 solve_tsv_phase() 設定/清除）
    bool analytical_tsv_active_ = false;

    // TSV proxy 注入後記錄的 modules_ 索引，供 commit_tsv_proxies_from_legalize 使用
    std::vector<int> proxy_module_indices_;

    // net_id → [tsv 在 tsvs_ 的索引]，在 build_tsvs() 後建立；
    // 大小 = nets_.size()，非 cross-tier net 的 entry 為空 vector。
    // 供 WL 梯度 / HPWL / congestion map 用，避免對每個 (net,tier) 全掃 tsvs_。
    std::vector<std::vector<int>> net_to_tsvs_;

    // ---- 內部輔助函式 ----

    // 初始化 Die 的 Bin 網格（每層可獨立指定寬高）
    void setup_dies(int num_dies,
                    const std::vector<double>& die_ws,
                    const std::vector<double>& die_hs);

    // 二次型 Bell 函數（平滑緊支撐密度核）
    // Φ(d, r) = 1 - (d/r)^2   if |d| <= r
    //         = 0              otherwise
    static double bell_func(double d, double r) {
        if (std::fabs(d) >= r) return 0.0;
        double u = d / r;
        return 1.0 - u * u;
    }

    // Bell 函數對 d 的偏微分
    // ∂Φ/∂d = -2d/r^2   if |d| <= r
    //       = 0          otherwise
    static double bell_grad(double d, double r) {
        if (std::fabs(d) >= r) return 0.0;
        return -2.0 * d / (r * r);
    }

    // 邊界夾取：確保模組中心在 Die 內部（留 0.5 邊距）
    void clamp_to_die(Module& m, const Die& die) const;
    // 邊界夾取：確保 TSV 中心在 tier_below die 內部（留半寬邊距）
    void clamp_tsv_to_die(TSV& tsv) const;

    // Phase 2 NAG 主迴圈（include_tsv=true 時 TSV 一起移動）
    void run_nag_loop(int max_iter, bool include_tsv);

    // 記錄每次迭代的 HPWL 用於收斂判斷
    double prev_hpwl_ = std::numeric_limits<double>::max();

    // 僅供 compute_hpwl：cfg 覆寫優先，否則 .block 的 tier_net_weights_
    double hpwl_die_weight(int tier) const;

    // analytical / LSE：僅用 .block 的 tier_net_weights_（不受 hpwl_die_weights 覆寫）
    double analytical_tier_net_weight(int tier) const;

    // LSE：依 net 所跨 tier 對 analytical_tier_net_weight 取算術平均
    double net_wirelength_die_weight(const Net& net) const;

    // TSV cost：tier 乘數（tsv_die_weights 若滿長度則覆寫，否則 tier_net_weights_）
    double tsv_placement_tier_weight(int tier) const;

    // REPULSE constraint：對所有斥力群組計算 pairwise inverse-square 梯度
    // 跨 tier 以 2D 平面座標計算；is_terminal / is_fixed 的 module 不受力
    void calculate_repulsion_gradient(std::vector<double>& gx,
                                      std::vector<double>& gy) const;
};
