// 3D IC Analytical Floorplanner - 核心引擎實作
// 實作 LSE Wirelength + Bin Density + Nesterov NAG 優化
#include "floorplanner.h"

#include <algorithm>
#include <cmath>
#include <cassert>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <random>
#include <stdexcept>

// ============================================================
// 建構子
// ============================================================
PlacementEngine::PlacementEngine(const PlacementConfig& cfg)
    : cfg_(cfg)
{
}

// ============================================================
// setup_dies: 根據解析結果初始化各 Die 的 Bin 網格
// 在 parse_blocks() 完成後由內部呼叫
// ============================================================
void PlacementEngine::setup_dies(int num_dies, double die_w, double die_h)
{
    dies_.resize(num_dies);

    for (int t = 0; t < num_dies; ++t) {
        Die& d      = dies_[t];
        d.id        = t;
        d.width     = die_w;
        d.height    = die_h;
        d.bin_rows  = cfg_.bin_resolution;
        d.bin_cols  = cfg_.bin_resolution;
        d.bin_w     = die_w  / d.bin_cols;
        d.bin_h     = die_h  / d.bin_rows;

        // 密度懲罰係數（每層可不同）
        if (t < static_cast<int>(cfg_.tier_lambdas.size()))
            d.lambda = cfg_.tier_lambdas[t];
        else
            d.lambda = cfg_.tier_lambdas.back();

        // 初始化所有 Bin
        d.bins.resize(d.bin_rows * d.bin_cols);
        for (int r = 0; r < d.bin_rows; ++r) {
            for (int c = 0; c < d.bin_cols; ++c) {
                Bin& b        = d.bins[r * d.bin_cols + c];
                b.cx          = (c + 0.5) * d.bin_w;
                b.cy          = (r + 0.5) * d.bin_h;
                b.w           = d.bin_w;
                b.h           = d.bin_h;
                b.density     = 0.0;
                b.target_density = cfg_.target_density;
            }
        }
    }
}

// ============================================================
// initialize_positions:
//   將每個可移動方塊置於所屬 Die 中心，
//   加上 1% Die 寬度的均勻隨機擾動以打破對稱性
// ============================================================
void PlacementEngine::initialize_positions()
{
    if (dies_.empty()) {
        throw std::runtime_error("Dies not initialized. Call parse_blocks() first.");
    }

    std::mt19937                          rng(42);  // 固定種子，結果可重現
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    for (Module& m : modules_) {
        if (m.is_terminal) continue;   // terminal 位置固定不動

        const Die& die = dies_[m.tier_id];
        double cx      = die.width  * 0.5;
        double cy      = die.height * 0.5;
        double jitter  = die.width  * 0.01;  // 1% Die 寬度的擾動幅度

        m.x = cx + jitter * dist(rng);
        m.y = cy + jitter * dist(rng);

        // 確保初始位置合法
        clamp_to_die(m, die);
    }

    int n = static_cast<int>(modules_.size());
    prev_x_.assign(n, 0.0);
    prev_y_.assign(n, 0.0);
    vel_x_.assign(n,  0.0);
    vel_y_.assign(n,  0.0);

    for (int i = 0; i < n; ++i) {
        prev_x_[i] = modules_[i].x;
        prev_y_[i] = modules_[i].y;
    }

    prev_hpwl_ = std::numeric_limits<double>::max();
    std::cout << "[Init] Positions initialized. HPWL = "
              << std::fixed << std::setprecision(2)
              << compute_hpwl() << "\n";
}

// ============================================================
// solve: Nesterov Accelerated Gradient 主迴圈
//
//   每次迭代流程：
//   1. 更新動態參數（σ 線性退火；λ 每 interval 次迭代 ×1.2）
//   2. 由當前位置 x_k 計算 NAG lookahead y_k
//   3. 在 y_k 處更新密度圖並計算兩路梯度
//   4. RMS 正規化，使 WL 與 Density 梯度量級一致後相加
//   5. 更新位置並夾取至 Die 邊界
// ============================================================
void PlacementEngine::solve()
{
    const int    max_iter = cfg_.max_iterations;
    const double mu       = cfg_.momentum;
    double       step     = cfg_.init_step_size;
    const double decay    = cfg_.step_decay;
    const double tol      = cfg_.convergence_tol;

    int n = static_cast<int>(modules_.size());

    // λ 從 init_mult 出發，每 interval 次迭代乘以 increase_rate
    lambda_mult_ = cfg_.lambda_init_mult;

    // σ 平滑半徑：以第一個 Die 的寬度為基準（各層同尺寸）
    const double die_w       = dies_.empty() ? 268.0 : dies_[0].width;
    const double sigma_start = cfg_.sigma_start_frac * die_w;
    const double sigma_end   = cfg_.sigma_end_frac   * die_w;

    std::vector<double> gx_wl(n), gy_wl(n);   // Wirelength 梯度
    std::vector<double> gx_d(n),  gy_d(n);    // Density 梯度
    std::vector<double> gx(n),    gy(n);       // 合併梯度
    std::vector<double> look_x(n), look_y(n);  // NAG lookahead 暫存

    for (int iter = 0; iter < max_iter; ++iter) {

        // ---- Step 1a: σ 線性退火 ----
        // 前半段快速退火（cosine annealing 的線性近似）
        double t      = static_cast<double>(iter) / max_iter;
        smooth_sigma_ = sigma_start + (sigma_end - sigma_start) * t;

        // ---- Step 1b: λ 遞增排程（每 interval 次 ×rate，上限 lambda_max_mult）----
        if (iter > 0 && iter % cfg_.lambda_update_interval == 0) {
            lambda_mult_ = std::min(lambda_mult_ * cfg_.lambda_increase_rate,
                                    cfg_.lambda_max_mult);
        }

        // ---- Step 2: NAG lookahead y_k = x_k + μ*(x_k - x_{k-1}) ----
        for (int i = 0; i < n; ++i) {
            if (modules_[i].is_terminal) {
                look_x[i] = modules_[i].x;
                look_y[i] = modules_[i].y;
                continue;
            }
            look_x[i] = modules_[i].x + mu * (modules_[i].x - prev_x_[i]);
            look_y[i] = modules_[i].y + mu * (modules_[i].y - prev_y_[i]);
        }

        // 暫存 x_k，再把 module 座標移到 lookahead 位置以計算梯度
        for (int i = 0; i < n; ++i) {
            prev_x_[i]    = modules_[i].x;
            prev_y_[i]    = modules_[i].y;
            modules_[i].x = look_x[i];
            modules_[i].y = look_y[i];
        }

        // ---- Step 3: 計算兩路梯度 ----
        update_density_map();                          // 使用 smooth_sigma_
        calculate_wirelength_gradient(gx_wl, gy_wl);
        calculate_density_gradient(gx_d,  gy_d);      // 使用 smooth_sigma_

        // ---- Step 4: RMS 正規化 → 梯度量級對齊 ----
        // 分別計算 WL 與 Density 梯度的 RMS（僅統計可移動模組）
        double wl_sq = 0.0, d_sq = 0.0;
        int    movable = 0;
        for (int i = 0; i < n; ++i) {
            if (modules_[i].is_terminal) continue;
            wl_sq += gx_wl[i]*gx_wl[i] + gy_wl[i]*gy_wl[i];
            d_sq  += gx_d[i] *gx_d[i]  + gy_d[i] *gy_d[i];
            ++movable;
        }
        double wl_rms = std::sqrt(wl_sq / (2.0 * movable + 1e-12));
        double d_rms  = std::sqrt(d_sq  / (2.0 * movable + 1e-12));

        // scale_d：將 density 梯度縮放到與 WL 梯度相同的 RMS 量級
        // 再由 λ 控制兩者的相對權重，避免不同量級造成 λ 失去意義
        double scale_d = (d_rms > 1e-12) ? (wl_rms / d_rms) : 0.0;

        // ---- Step 5: 合併梯度並更新位置 ----
        for (int i = 0; i < n; ++i) {
            if (modules_[i].is_terminal) continue;

            // 每層 λ = 基礎值 × 當前倍率
            double lam = dies_[modules_[i].tier_id].lambda * lambda_mult_;

            // WL 梯度 + 正規化後的 Density 梯度
            gx[i] = gx_wl[i] + lam * scale_d * gx_d[i];
            gy[i] = gy_wl[i] + lam * scale_d * gy_d[i];

            modules_[i].x = look_x[i] - step * gx[i];
            modules_[i].y = look_y[i] - step * gy[i];

            clamp_to_die(modules_[i], dies_[modules_[i].tier_id]);
        }

        step *= decay;

        // ---- Step 6: 收斂監控（每 50 次）----
        if (iter % 50 == 0) {
            double hpwl = compute_hpwl();

            double max_overflow = 0.0;
            for (const Die& die : dies_)
                for (const Bin& b : die.bins)
                    max_overflow = std::max(max_overflow,
                                            b.density - b.target_density);

            double rel_change = std::fabs(prev_hpwl_ - hpwl) /
                                (prev_hpwl_ + 1e-12);

            std::cout << "[Iter " << std::setw(5) << iter << "]"
                      << " HPWL="       << std::fixed      << std::setprecision(1) << hpwl
                      << " Overflow="   << std::setprecision(3) << max_overflow
                      << " λ_mult="     << std::setprecision(4) << lambda_mult_
                      << " σ="          << std::setprecision(1) << smooth_sigma_
                      << " WLrms="      << std::scientific  << std::setprecision(2) << wl_rms
                      << " Drms="       << d_rms
                      << " step="       << step
                      << "\n";

            if (iter > 100 && rel_change < tol && max_overflow < 0.1) {
                std::cout << "[Converged] iter=" << iter
                          << " HPWL=" << std::fixed << hpwl << "\n";
                break;
            }
            prev_hpwl_ = hpwl;
        }
    }

    std::cout << "\n[Final] HPWL = " << std::fixed
              << std::setprecision(2) << compute_hpwl() << "\n";
}

// ============================================================
// build_tsvs: 依 cross-tier nets 建立 TSV 清單
//
// 規則：對每個 net，取 pin 的 tier（terminal 視為 tier 0），得 [min_tier, max_tier]。
// 若 max_tier - min_tier >= 1，則此 net 需 (max_tier - min_tier) 個 TSV，
// 分別位於 layer_index = min_tier, min_tier+1, ..., max_tier-1
// （layer k 連接 tier k 與 tier k+1）。橫跨 tier 0 與 tier 2 則在 layer 0、1 各一 TSV。
// 初始 TSV 位置設為 die 中心（與模組同一平面投影）
// ============================================================
void PlacementEngine::build_tsvs()
{
    tsvs_.clear();
    int tsv_id = 0;

    for (Net& net : nets_) {
        if (net.pins.size() < 2) continue;

        int min_t = std::numeric_limits<int>::max();
        int max_t = std::numeric_limits<int>::min();
        for (int pid : net.pins) {
            const Module& m = modules_[pid];
            int t = m.is_terminal ? 0 : m.tier_id;
            min_t = std::min(min_t, t);
            max_t = std::max(max_t, t);
        }

        if (min_t < 0) continue;   // 防呆
        if (max_t - min_t < 1) continue;  // 未跨層，不需 TSV

        net.is_cross_tier = true;
        net.min_tier = min_t;
        net.max_tier = max_t;

        // 每個 layer_index = min_t .. max_t-1 各一個 TSV
        for (int layer = min_t; layer < max_t; ++layer) {
            TSV tsv;
            tsv.id = tsv_id++;
            tsv.net_id = net.id;
            tsv.layer_index = layer;
            const Die& die = dies_[layer];
            tsv.x = die.width  * 0.5;
            tsv.y = die.height * 0.5;
            tsvs_.push_back(std::move(tsv));
        }
    }

    std::cout << "[TSV] Built " << tsvs_.size() << " TSVs from cross-tier nets "
              << "(num_dies=" << num_dies() << ").\n";
}

// ============================================================
// 輔助結構：2D 矩形框（用於 TSV BBox 計算）
// ============================================================
namespace {

struct BBox {
    double xmin =  1e18, xmax = -1e18;
    double ymin =  1e18, ymax = -1e18;
    bool valid() const { return xmin <= xmax && ymin <= ymax; }
    void expand(double x, double y) {
        xmin = std::min(xmin, x); xmax = std::max(xmax, x);
        ymin = std::min(ymin, y); ymax = std::max(ymax, y);
    }
    // 點到矩形的距離（點在框內時為 0）
    double dist(double x, double y) const {
        double dx = std::max(xmin - x, 0.0) + std::max(x - xmax, 0.0);
        double dy = std::max(ymin - y, 0.0) + std::max(y - ymax, 0.0);
        return dx + dy;
    }
};

} // anonymous namespace

// ============================================================
// compute_tsv_cost: 計算所有 TSV 的 wirelength 代價
//
// 對每個 TSV，取所屬 net 上下兩側的 BBox：
//   B_lower = bbox of pins on tiers ≤ layer_index
//   B_upper = bbox of pins on tiers ≥ layer_index+1
// cost = dist(tsv, B_lower) + dist(tsv, B_upper)
// ============================================================
double PlacementEngine::compute_tsv_cost() const
{
    double total = 0.0;
    for (const TSV& tsv : tsvs_) {
        const Net& net = nets_[tsv.net_id];
        BBox b_lo, b_hi;
        for (int pid : net.pins) {
            const Module& m = modules_[pid];
            int mt = m.is_terminal ? 0 : m.tier_id;
            if (mt <= tsv.tier_below()) b_lo.expand(m.x, m.y);
            if (mt >= tsv.tier_above()) b_hi.expand(m.x, m.y);
        }
        if (b_lo.valid()) total += b_lo.dist(tsv.x, tsv.y);
        if (b_hi.valid()) total += b_hi.dist(tsv.x, tsv.y);
    }
    return total;
}

// ============================================================
// solve_tsvs: 直接以上下層 BBox 中心決定 TSV 座標（無迭代優化）
//
// 對每個 TSV：
//   B_lower = bbox of pins on tiers ≤ layer_index
//   B_upper = bbox of pins on tiers ≥ layer_index+1
//
// 中心點定義：
//   C_lower = center(B_lower), C_upper = center(B_upper)
//   TSV 位置 = (C_lower + C_upper) / 2
//
// 若只有一側 bbox 有效，則使用該側中心；最後夾取到對應 die 邊界。
// ============================================================
void PlacementEngine::solve_tsvs(const TsvPlacementConfig& /*tcfg*/)
{
    const int N = static_cast<int>(tsvs_.size());
    if (N == 0) {
        std::cout << "[SolveTSV] No TSVs to place.\n";
        return;
    }

    std::cout << "[SolveTSV] Direct center placement for " << N << " TSVs.\n";

    for (TSV& tsv : tsvs_) {
        const Net& net = nets_[tsv.net_id];
        BBox b_lo, b_hi;
        for (int pid : net.pins) {
            const Module& m = modules_[pid];
            int mt = m.is_terminal ? 0 : m.tier_id;
            if (mt <= tsv.tier_below()) b_lo.expand(m.x, m.y);
            if (mt >= tsv.tier_above()) b_hi.expand(m.x, m.y);
        }

        double tx = tsv.x, ty = tsv.y;
        const bool lo_ok = b_lo.valid();
        const bool hi_ok = b_hi.valid();

        if (lo_ok && hi_ok) {
            const double cxl = 0.5 * (b_lo.xmin + b_lo.xmax);
            const double cyl = 0.5 * (b_lo.ymin + b_lo.ymax);
            const double cxh = 0.5 * (b_hi.xmin + b_hi.xmax);
            const double cyh = 0.5 * (b_hi.ymin + b_hi.ymax);
            tx = 0.5 * (cxl + cxh);
            ty = 0.5 * (cyl + cyh);
        } else if (lo_ok) {
            tx = 0.5 * (b_lo.xmin + b_lo.xmax);
            ty = 0.5 * (b_lo.ymin + b_lo.ymax);
        } else if (hi_ok) {
            tx = 0.5 * (b_hi.xmin + b_hi.xmax);
            ty = 0.5 * (b_hi.ymin + b_hi.ymax);
        }

        const Die& die = dies_[tsv.layer_index];
        tsv.x = std::max(0.0, std::min(die.width,  tx));
        tsv.y = std::max(0.0, std::min(die.height, ty));
    }

    std::cout << "[SolveTSV] Done. Final TSV cost = "
              << std::fixed << std::setprecision(2) << compute_tsv_cost() << "\n";
}

// ============================================================
// compute_lse_wirelength:
//   LSE 近似 HPWL 的目標函數值
//   WL(e) = γ * [ln Σ exp(xi/γ) + ln Σ exp(-xi/γ)
//              + ln Σ exp(yi/γ) + ln Σ exp(-yi/γ)]
// ============================================================
double PlacementEngine::compute_lse_wirelength() const
{
    double total = 0.0;
    const double g = cfg_.gamma;

    auto pin_wl_w = [this](int pid) -> double {
        return modules_[pid].is_terminal ? cfg_.wl_pin_weight_terminal
                                         : cfg_.wl_pin_weight_module;
    };

    for (const Net& net : nets_) {
        if (net.pins.size() < 2) continue;

        double sum_exp_x  = 0.0, sum_exp_nx = 0.0;
        double sum_exp_y  = 0.0, sum_exp_ny = 0.0;

        // 數值穩定：減去最大/最小值後再求 exp
        double max_x = -1e18, min_x = 1e18;
        double max_y = -1e18, min_y = 1e18;
        for (int id : net.pins) {
            max_x = std::max(max_x, modules_[id].x);
            min_x = std::min(min_x, modules_[id].x);
            max_y = std::max(max_y, modules_[id].y);
            min_y = std::min(min_y, modules_[id].y);
        }

        for (int id : net.pins) {
            const double wi = pin_wl_w(id);
            double xi = modules_[id].x;
            double yi = modules_[id].y;
            sum_exp_x  += wi * std::exp((xi - max_x) / g);
            sum_exp_nx += wi * std::exp((min_x - xi) / g);
            sum_exp_y  += wi * std::exp((yi - max_y) / g);
            sum_exp_ny += wi * std::exp((min_y - yi) / g);
        }

        // WL_e = γ * [ln(Σe^{xi/γ}) + ln(Σe^{-xi/γ}) + ...]
        //      = γ * [max_x/γ + ln(Σe^{(xi-max_x)/γ}) + ...]  等效形式
        total += g * (std::log(sum_exp_x)  + max_x  / g
                    + std::log(sum_exp_nx) + (-min_x) / g
                    + std::log(sum_exp_y)  + max_y  / g
                    + std::log(sum_exp_ny) + (-min_y) / g);
    }
    return total;
}

// ============================================================
// calculate_wirelength_gradient:
//   ∂WL/∂xj = exp(xj/γ)/Σexp(xi/γ) - exp(-xj/γ)/Σexp(-xi/γ)
//   同理 y 方向
// ============================================================
void PlacementEngine::calculate_wirelength_gradient(
    std::vector<double>& gx, std::vector<double>& gy) const
{
    std::fill(gx.begin(), gx.end(), 0.0);
    std::fill(gy.begin(), gy.end(), 0.0);

    const double g = cfg_.gamma;

    auto pin_wl_w = [this](int pid) -> double {
        return modules_[pid].is_terminal ? cfg_.wl_pin_weight_terminal
                                         : cfg_.wl_pin_weight_module;
    };

    for (const Net& net : nets_) {
        if (net.pins.size() < 2) continue;

        // 數值穩定：找各方向最大最小值
        double max_x = -1e18, min_x = 1e18;
        double max_y = -1e18, min_y = 1e18;
        for (int id : net.pins) {
            max_x = std::max(max_x, modules_[id].x);
            min_x = std::min(min_x, modules_[id].x);
            max_y = std::max(max_y, modules_[id].y);
            min_y = std::min(min_y, modules_[id].y);
        }

        // 加權分母 Z = Σ w_i exp(...)
        double Z_px = 0.0, Z_nx = 0.0;
        double Z_py = 0.0, Z_ny = 0.0;
        for (int id : net.pins) {
            const double wi = pin_wl_w(id);
            Z_px += wi * std::exp((modules_[id].x - max_x) / g);
            Z_nx += wi * std::exp((min_x - modules_[id].x) / g);
            Z_py += wi * std::exp((modules_[id].y - max_y) / g);
            Z_ny += wi * std::exp((min_y - modules_[id].y) / g);
        }

        // 計算每個 pin 的梯度貢獻
        for (int id : net.pins) {
            if (modules_[id].is_terminal) continue;  // terminal 固定，跳過

            const double wj = pin_wl_w(id);
            double ex  = std::exp((modules_[id].x - max_x) / g);
            double enx = std::exp((min_x - modules_[id].x) / g);
            double ey  = std::exp((modules_[id].y - max_y) / g);
            double eny = std::exp((min_y - modules_[id].y) / g);

            // Weighted LSE: ∂WL/∂xj = wj * (ex/Z_px - enx/Z_nx)（γ=g 時與目標一致）
            gx[id] += wj * (ex / Z_px - enx / Z_nx);
            gy[id] += wj * (ey / Z_py - eny / Z_ny);
        }
    }
}

// ============================================================
// update_density_map:
//   對每個可移動方塊，計算其對鄰近 Bin 的密度貢獻
//   密度貢獻 = (module 面積 / bin 面積) * Φx * Φy
//   Φ(d, r) 為二次型 Bell 函數（緊支撐平滑核）
//
//   影響半徑 r = 模組半寬 + smooth_sigma_
//   smooth_sigma_ 由 solve() 在每次迭代前設定，
//   初始值大（20% die 寬），逐漸退火至 7%，
//   使早期密度廣泛擴散、後期精細收斂
// ============================================================
void PlacementEngine::update_density_map()
{
    for (Die& die : dies_)
        for (Bin& b : die.bins) b.density = 0.0;

    for (const Module& m : modules_) {
        if (m.is_terminal) continue;

        Die& die = dies_[m.tier_id];

        // 影響半徑：模組半寬 + 當前平滑 σ（確保至少涵蓋一個 bin 寬度）
        double rx = m.width  * 0.5 + std::max(smooth_sigma_, die.bin_w);
        double ry = m.height * 0.5 + std::max(smooth_sigma_, die.bin_h);

        // 正規化係數：Bell 核在 [-rx,rx]×[-ry,ry] 的積分面積 = 4*rx*ry*(∫Φ)^2
        // Φ(d,r) = 1-(d/r)^2 在 [-r,r] 的積分 = 4r/3，故 2D 積分 = 16*rx*ry/9
        // 令 A_ratio = m.area() / (16*rx*ry/9) 使所有 bin 的密度總和 = m.area()/bin_area，
        // 即核函數面積守恆（density 值落在合理的 0~1 量級，而非百倍以上的數值）
        double A_ratio = m.area() * (9.0 / 16.0) / (rx * ry);

        int c_min = std::max(0, static_cast<int>((m.x - rx) / die.bin_w));
        int c_max = std::min(die.bin_cols - 1,
                             static_cast<int>((m.x + rx) / die.bin_w));
        int r_min = std::max(0, static_cast<int>((m.y - ry) / die.bin_h));
        int r_max = std::min(die.bin_rows - 1,
                             static_cast<int>((m.y + ry) / die.bin_h));

        for (int r = r_min; r <= r_max; ++r) {
            for (int c = c_min; c <= c_max; ++c) {
                Bin& b       = die.bins[r * die.bin_cols + c];
                double phi_x = bell_func(m.x - b.cx, rx);
                double phi_y = bell_func(m.y - b.cy, ry);
                b.density   += A_ratio * phi_x * phi_y;
            }
        }
    }
}

// ============================================================
// calculate_density_gradient:
//   密度目標函數 D = Σ_b (ρ_b - ρ_t)^2
//   ∂D/∂xi = Σ_b 2*(ρ_b - ρ_t) * ∂ρ_b/∂xi
//   ∂ρ_b/∂xi = A_ratio * ∂Φx/∂xi * Φy
//   其中 A_ratio = Ai * (9/16) / (rx * ry)（面積守恆正規化）
//
//   影響半徑與 update_density_map 保持完全一致，
//   使梯度與密度圖精確對應
// ============================================================
void PlacementEngine::calculate_density_gradient(
    std::vector<double>& gx, std::vector<double>& gy) const
{
    int n = static_cast<int>(modules_.size());
    std::fill(gx.begin(), gx.end(), 0.0);
    std::fill(gy.begin(), gy.end(), 0.0);

    for (int i = 0; i < n; ++i) {
        const Module& m = modules_[i];
        if (m.is_terminal) continue;

        const Die& die = dies_[m.tier_id];

        // 與 update_density_map 完全相同的影響半徑與正規化係數
        double rx = m.width  * 0.5 + std::max(smooth_sigma_, die.bin_w);
        double ry = m.height * 0.5 + std::max(smooth_sigma_, die.bin_h);
        double A_ratio = m.area() * (9.0 / 16.0) / (rx * ry);

        int c_min = std::max(0, static_cast<int>((m.x - rx) / die.bin_w));
        int c_max = std::min(die.bin_cols - 1,
                             static_cast<int>((m.x + rx) / die.bin_w));
        int r_min = std::max(0, static_cast<int>((m.y - ry) / die.bin_h));
        int r_max = std::min(die.bin_rows - 1,
                             static_cast<int>((m.y + ry) / die.bin_h));

        for (int r = r_min; r <= r_max; ++r) {
            for (int c = c_min; c <= c_max; ++c) {
                const Bin& b    = die.bins[r * die.bin_cols + c];
                double overflow = b.density - b.target_density;
                if (overflow <= 0.0) continue;  // 未超載不產生排斥力

                double dx_m   = m.x - b.cx;
                double dy_m   = m.y - b.cy;
                double phi_x  = bell_func(dx_m, rx);
                double phi_y  = bell_func(dy_m, ry);
                double dphi_x = bell_grad(dx_m, rx);
                double dphi_y = bell_grad(dy_m, ry);

                double coeff = 2.0 * overflow * A_ratio;
                gx[i] += coeff * dphi_x * phi_y;
                gy[i] += coeff * phi_x  * dphi_y;
            }
        }
    }
}

// ============================================================
// compute_hpwl: 計算精確 HPWL（含 terminal、module、TSV）
//
// 3D IC 正確計算方式：對每條 net，逐層累加 HPWL。
// 對第 t 層，納入：
//   - 位於該層的 module 中心（tier_id == t）
//   - terminal（is_terminal == true，全部視為 tier 0）
//   - 連接第 t 層與第 t+1 層的 TSV（layer_index == t）：出現在 tier t 的 bbox
//   - 連接第 t-1 層與第 t 層的 TSV（layer_index == t-1）：出現在 tier t 的 bbox
// 即每個 TSV 同時計入其 tier_below 與 tier_above 兩層的 bbox 中，
// 反映「此層需要在 pin 與 TSV 之間佈線」的真實線長。
// 跨越多層的 net 每一層都會各算一次 HPWL 後累加。
// ============================================================
double PlacementEngine::compute_hpwl() const
{
    double total = 0.0;
    const int num_tiers = static_cast<int>(dies_.size());

    for (const Net& net : nets_) {
        for (int t = 0; t < num_tiers; ++t) {
            double x_min = 1e18, x_max = -1e18;
            double y_min = 1e18, y_max = -1e18;

            auto expand = [&](double x, double y) {
                x_min = std::min(x_min, x);
                x_max = std::max(x_max, x);
                y_min = std::min(y_min, y);
                y_max = std::max(y_max, y);
            };

            // 納入此層的 module 與 terminal（terminal 視為 tier 0）
            for (int id : net.pins) {
                const Module& m = modules_[id];
                int mt = m.is_terminal ? 0 : m.tier_id;
                if (mt == t) expand(m.x, m.y);
            }

            // 納入與此層相鄰的 TSV：
            //   tier_below() == t → 從 tier t 往上的 TSV，屬於 tier t 的出口
            //   tier_above() == t → 從 tier t-1 往上的 TSV，屬於 tier t 的入口
            for (const TSV& tsv : tsvs_) {
                if (tsv.net_id != net.id) continue;
                if (tsv.tier_below() == t || tsv.tier_above() == t)
                    expand(tsv.x, tsv.y);
            }

            // 此層有至少兩個點才會有非零線長
            if (x_min <= x_max && y_min <= y_max)
                total += (x_max - x_min) + (y_max - y_min);
        }
    }
    return total;
}

// ============================================================
// clamp_to_die: 將模組中心夾取至 Die 邊界內部
// ============================================================
void PlacementEngine::clamp_to_die(Module& m, const Die& die) const
{
    double half_w = m.width  * 0.5;
    double half_h = m.height * 0.5;
    m.x = std::max(half_w, std::min(die.width  - half_w, m.x));
    m.y = std::max(half_h, std::min(die.height - half_h, m.y));
}

// ============================================================
// write_output: 輸出符合 PA2 格式的擺放結果
//
// 格式：
//   Line 1: total_cost（此處以 HPWL 代替，無 TSV 項）
//   Line 2: hpwl
//   Line 3: bounding_box_area  (bbox_w * bbox_h)
//   Line 4: bbox_w  bbox_h     (所有可移動方塊的最大外框)
//   Line 5: runtime (秒)
//   Line 6+: <name> <die_id> <x_ll> <y_ll> <x_ur> <y_ur>
// ============================================================
void PlacementEngine::write_output(const std::string& filename,
                                   double runtime) const
{
    FILE* fp = std::fopen(filename.c_str(), "w");
    if (!fp) {
        std::cerr << "[Error] Cannot open output file: " << filename << "\n";
        return;
    }

    // ---- 計算全域 bounding box（僅含可移動方塊）----
    double g_xmin =  1e18, g_xmax = -1e18;
    double g_ymin =  1e18, g_ymax = -1e18;
    for (const Module& m : modules_) {
        if (m.is_terminal) continue;
        g_xmin = std::min(g_xmin, m.lx());
        g_xmax = std::max(g_xmax, m.rx());
        g_ymin = std::min(g_ymin, m.ly());
        g_ymax = std::max(g_ymax, m.ry());
    }
    // 整數化邊界
    int bbox_w = static_cast<int>(std::ceil(g_xmax) - std::floor(g_xmin));
    int bbox_h = static_cast<int>(std::ceil(g_ymax) - std::floor(g_ymin));
    double bbox_area = static_cast<double>(bbox_w) * bbox_h;

    double hpwl       = compute_hpwl();  // 含 terminal、module、TSV 的 bbox
    double total_cost = hpwl;

    // ---- 寫出前五行統計資訊 ----
    std::fprintf(fp, "%f\n",   total_cost);
    std::fprintf(fp, "%f\n",   hpwl);
    std::fprintf(fp, "%f\n",   bbox_area);
    std::fprintf(fp, "%d %d\n", bbox_w, bbox_h);
    std::fprintf(fp, "%f\n",   runtime);

    // ---- 逐一輸出每個可移動方塊 ----
    // <name> <die_id> <x_ll> <y_ll> <x_ur> <y_ur>（整數座標）
    for (const Module& m : modules_) {
        if (m.is_terminal) continue;
        int x_ll = static_cast<int>(std::round(m.lx()));
        int y_ll = static_cast<int>(std::round(m.ly()));
        int x_ur = static_cast<int>(std::round(m.rx()));
        int y_ur = static_cast<int>(std::round(m.ry()));
        std::fprintf(fp, "%s %d %d %d %d %d\n",
                     m.name.c_str(), m.tier_id,
                     x_ll, y_ll, x_ur, y_ur);
    }

    // ---- TSV Assignments ----
    // 格式：NumTsvAssignments <N>
    //       <net_name> tier<N>-<N+1> <x> <y>
    if (!tsvs_.empty()) {
        std::fprintf(fp, "NumTsvAssignments %d\n",
                     static_cast<int>(tsvs_.size()));
        for (const TSV& tsv : tsvs_) {
            const Net& net = nets_[tsv.net_id];
            std::fprintf(fp, "%s tier%d-%d %f %f\n",
                         net.name.c_str(),
                         tsv.tier_below(), tsv.tier_above(),
                         tsv.x, tsv.y);
        }
    }

    std::fclose(fp);
    std::cout << "[Output] Written to " << filename << "\n";
}

// ============================================================
// write_density_map: 將各 tier 的 bin 密度圖輸出成文字檔
//
// 每層產生一個獨立檔案：<base_filename>_density_tier<N>.txt
// 內容包含：
//   1. 基本資訊（tier id、bin 解析度、目標密度）
//   2. ASCII 視覺化（row 由上到下，即 y 由大到小排列）
//      字元對應密度區間：
//        ' ' = 0.0  ~ 0.30 * target  (近乎空）
//        '.' = 0.30 ~ 0.70 * target  (稀疏)
//        'o' = 0.70 ~ 1.00 * target  (接近目標)
//        '+' = 1.00 ~ 1.30 * target  (輕微超標)
//        '#' = 1.30 ~ 2.00 * target  (嚴重超標)
//        '@' = 2.00+       * target  (爆炸)
//   3. 數值表格（每格保留兩位小數，空格分隔）
//   4. 每列最大密度（方便找熱點）
// ============================================================
void PlacementEngine::write_density_map(const std::string& base_filename) const
{
    // 每層密度映射用的字元等級
    // 依照 density / target_density 的比值分段
    auto density_char = [](double rho, double target) -> char {
        double r = rho / (target > 1e-9 ? target : 1.0);
        if (r < 0.30) return ' ';
        if (r < 0.70) return '.';
        if (r < 1.00) return 'o';
        if (r < 1.30) return '+';
        if (r < 2.00) return '#';
        return '@';
    };

    for (const Die& die : dies_) {
        // 組合輸出檔名
        std::string fname = base_filename + "_density_tier"
                          + std::to_string(die.id) + ".txt";
        FILE* fp = std::fopen(fname.c_str(), "w");
        if (!fp) {
            std::cerr << "[Error] Cannot open density map file: " << fname << "\n";
            continue;
        }

        const int R = die.bin_rows;
        const int C = die.bin_cols;
        // 以第一個真實 bin 的 target_density 為代表值
        double target = (die.bins.empty()) ? cfg_.target_density
                                           : die.bins[0].target_density;

        // ---- 基本資訊 ----
        std::fprintf(fp, "=== Tier %d Density Map ===\n", die.id);
        std::fprintf(fp, "Die size  : %.1f x %.1f\n", die.width, die.height);
        std::fprintf(fp, "Bin grid  : %d rows x %d cols  (bin=%.2f x %.2f)\n",
                     R, C, die.bin_w, die.bin_h);
        std::fprintf(fp, "Target ρ  : %.3f\n\n", target);

        // ---- ASCII 視覺化（y 由大到小，即頂行 = 最高 y）----
        std::fprintf(fp, "ASCII visualization (row = y-axis, col = x-axis):\n");
        std::fprintf(fp, "  ' '=0~0.3t  '.'=0.3~0.7t  'o'=0.7~1.0t  "
                         "'+'=1.0~1.3t  '#'=1.3~2.0t  '@'=2.0t+\n");

        // 欄位座標尺標（每 8 格一個標記）
        std::fprintf(fp, "  X: ");
        for (int c = 0; c < C; ++c) {
            if (c % 8 == 0) std::fprintf(fp, "|");
            else            std::fprintf(fp, " ");
        }
        std::fprintf(fp, "\n");

        for (int r = R - 1; r >= 0; --r) {
            std::fprintf(fp, "%3d |", r);
            for (int c = 0; c < C; ++c) {
                double rho = die.bins[r * C + c].density;
                std::fprintf(fp, "%c", density_char(rho, target));
            }
            // 該列最大密度
            double row_max = 0.0;
            for (int c = 0; c < C; ++c)
                row_max = std::max(row_max, die.bins[r * C + c].density);
            std::fprintf(fp, "| max=%.3f\n", row_max);
        }
        std::fprintf(fp, "\n");

        // ---- 數值表格（y 由大到小）----
        std::fprintf(fp, "Numeric density table (row = y-axis, each cell = bin density):\n");
        // 標頭列
        std::fprintf(fp, "%5s", "y\\x");
        for (int c = 0; c < C; ++c) std::fprintf(fp, " %5d", c);
        std::fprintf(fp, " | row_max\n");
        std::fprintf(fp, "%5s", "-----");
        for (int c = 0; c < C; ++c) std::fprintf(fp, "------");
        std::fprintf(fp, "-|--------\n");

        for (int r = R - 1; r >= 0; --r) {
            std::fprintf(fp, "%5d", r);
            double row_max = 0.0;
            for (int c = 0; c < C; ++c) {
                double rho = die.bins[r * C + c].density;
                std::fprintf(fp, " %5.2f", rho);
                row_max = std::max(row_max, rho);
            }
            std::fprintf(fp, " | %6.3f\n", row_max);
        }

        // ---- 全域統計 ----
        double total_rho = 0.0, max_rho = 0.0, over_bins = 0;
        for (const Bin& b : die.bins) {
            total_rho += b.density;
            max_rho    = std::max(max_rho, b.density);
            if (b.density > b.target_density) ++over_bins;
        }
        std::fprintf(fp, "\nStats: avg=%.4f  max=%.4f  over-target bins=%d / %d\n",
                     total_rho / (R * C), max_rho,
                     static_cast<int>(over_bins), R * C);

        std::fclose(fp);
        std::cout << "[DensityMap] Tier " << die.id << " -> " << fname << "\n";
    }
}
