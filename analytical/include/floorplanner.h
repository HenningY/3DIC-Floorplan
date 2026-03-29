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

// ============================================================
// PlacementConfig: 優化超參數集合
// ============================================================
struct PlacementConfig {
    int    max_iterations   = 3000;    // 最大迭代次數
    double gamma            = 10.0;    // LSE 平滑參數 γ
    double init_step_size   = 1.0;     // 初始步長
    double step_decay       = 0.9999;  // 步長衰減率（每次迭代乘以此值）
    double momentum         = 0.9;     // Nesterov 動量係數
    double target_density   = 0.85;     // 目標密度
    int    bin_resolution   = 16;      // 每層 bin 數量 (bin_resolution x bin_resolution)
    double convergence_tol  = 1e-4;    // 收斂容忍度（相對 HPWL 變化）

    // ---- LSE wirelength：pin 權重（可移動 module vs 固定 terminal）----
    // 在 softmax 近似中，terminal 權重較小 → 對 bbox 的影響較弱，
    // 可移動 module 之間的「相對」吸引力會強於 module–terminal。
    // 需滿足：wl_pin_weight_terminal <= wl_pin_weight_module
    double wl_pin_weight_module   = 1.0;
    double wl_pin_weight_terminal = 0.2;

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
    bool   log_tree        = true;  // 是否把 partition tree 寫到 log 檔
    std::string log_file   = "partition_tree.txt";

    // 將 partition 後（push-back / flush 到切割線）的 module/TSV 位置寫檔
    bool        write_positions = true;
    std::string positions_file  = "partition_positions.txt";
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
// PlacementEngine: 解析式擺放主引擎
// ============================================================
class PlacementEngine {
public:
    // 建構子：指定層數與每層的 Die 尺寸
    PlacementEngine(const PlacementConfig& cfg = PlacementConfig());

    // ---- 輸入解析 ----
    bool parse_blocks(const std::string& filename);
    bool parse_nets(const std::string& filename);

    // ---- 主要介面 ----
    // 將所有 module 初始化至各自 Die 的中心（加入隨機擾動）
    void initialize_positions();

    // 執行解析式優化主迴圈
    void solve();

    // ---- TSV ----
    // 依 cross-tier nets 建立 TSV 清單（需在 parse + module 位置固定後呼叫）
    // 規則：net 橫跨 tier [min_tier, max_tier] 時，在 layer 0..(max_tier-min_tier-1) 各放一個 TSV
    void build_tsvs();

    // TSV Placement 求解器
    // 所有 Module 座標固定，只更新 TSV (x,y)
    // 目標：最小化 dist(TSV, B_lower) + dist(TSV, B_upper)，同時避開 module 密集區
    void solve_tsvs(const TsvPlacementConfig& tcfg = TsvPlacementConfig());

    // 計算目前 TSV 的總 wirelength cost（評估用）
    // = Σ_tsv [dist(tsv, bbox_lower) + dist(tsv, bbox_upper)]
    double compute_tsv_cost() const;

    // ---- 輸出 ----
    // 寫出符合 PA2 格式的結果檔：
    //   Line 1: total_cost
    //   Line 2: hpwl
    //   Line 3: bounding_box_area
    //   Line 4: bbox_w  bbox_h
    //   Line 5: runtime (秒)
    //   Line 6+: <name> <die_id> <x_ll> <y_ll> <x_ur> <y_ur>
    void write_output(const std::string& filename, double runtime = 0.0) const;
    // 輸出各 tier 的 bin 密度圖（數值表格 + ASCII 視覺化）
    // 每層輸出 <base>_density_tier<N>.txt
    void write_density_map(const std::string& base_filename) const;
    double compute_hpwl() const;  // 計算真實 HPWL（評估用）

    // ---- 公開的優化計算（供測試與除錯）----
    double compute_lse_wirelength() const;
    void   calculate_wirelength_gradient(std::vector<double>& gx,
                                         std::vector<double>& gy) const;

    void   update_density_map();
    void   calculate_density_gradient(std::vector<double>& gx,
                                      std::vector<double>& gy) const;

    // ---- Recursive Partitioning（Legalization 準備）----
    // 對所有 tier 做 Recursive Bi-partitioning，
    // 結果用 partition_all_tiers() 統一觸發並寫 log
    void partition_all_tiers(const struct PartitionConfig& pcfg);

    // ---- 訪問器 ----
    const std::vector<Module>& modules()  const { return modules_; }
    const std::vector<Net>&    nets()     const { return nets_; }
    const std::vector<Die>&    dies()     const { return dies_; }
    const std::vector<TSV>&    tsvs()     const { return tsvs_; }
    int                        num_dies() const { return static_cast<int>(dies_.size()); }

private:
    PlacementConfig cfg_;

    std::vector<Module> modules_;
    std::vector<Net>    nets_;
    std::vector<Die>    dies_;
    std::vector<TSV>    tsvs_;  // 跨層 net 產生的 TSV，供後續 TSV placement 使用

    // 名稱 → 模組索引的映射（解析 .nets 用）
    std::unordered_map<std::string, int> name_to_id_;

    // Nesterov 加速梯度的歷史狀態
    std::vector<double> prev_x_, prev_y_;   // x_{k-1}
    std::vector<double> vel_x_,  vel_y_;    // 動量向量

    // ---- 動態狀態（每次迭代更新）----
    // 當前 σ 平滑半徑，供 update_density_map / calculate_density_gradient 使用
    double smooth_sigma_ = 0.0;
    // 當前 λ 倍率，在 solve() 內每 lambda_update_interval 次迭代遞增
    double lambda_mult_  = 1.0;

    // ---- 內部輔助函式 ----

    // 初始化 Die 的 Bin 網格
    void setup_dies(int num_dies, double die_w, double die_h);

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

    // 記錄每次迭代的 HPWL 用於收斂判斷
    double prev_hpwl_ = std::numeric_limits<double>::max();
};
