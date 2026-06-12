// 3D IC Analytical Floorplanner - 核心引擎實作
// 實作 LSE Wirelength + Bin Density + Nesterov NAG 優化
#include "floorplanner.h"
#include "routing_congestion.h"
#include "util.h"

#include <algorithm>
#include <cmath>
#include <cassert>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

namespace {

// density_norm ∈ [0,1]，越大越深（與 congestion PPM 同色調：淺青白→深藍黑）
void density_png_u_to_rgb(double u, unsigned char rgb[3])
{
    u = std::clamp(u, 0.0, 1.0);
    const double r0 = 245.0, g0 = 250.0, b0 = 255.0;
    const double r1 = 8.0,   g1 = 18.0,  b1 = 45.0;
    rgb[0] = static_cast<unsigned char>(r0 + (r1 - r0) * u + 0.5);
    rgb[1] = static_cast<unsigned char>(g0 + (g1 - g0) * u + 0.5);
    rgb[2] = static_cast<unsigned char>(b0 + (b1 - b0) * u + 0.5);
}

} // namespace
// ============================================================
PlacementEngine::PlacementEngine(const PlacementConfig& cfg)
    : cfg_(cfg)
{
}

// ============================================================
// setup_dies: 根據解析結果初始化各 Die 的 Bin 網格
// 在 parse_blocks() 完成後由內部呼叫
// ============================================================
void PlacementEngine::setup_dies(int num_dies,
                                 const std::vector<double>& die_ws,
                                 const std::vector<double>& die_hs)
{
    dies_.resize(num_dies);

    for (int t = 0; t < num_dies; ++t) {
        Die& d      = dies_[t];
        double die_w = die_ws[t];
        double die_h = die_hs[t];
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
// 幾何 Normalize 實作
// ============================================================

// 計算 scale = s * 10^(-n)（s 正整數、n 正整數），使 min_edge 縮放後落在合理範圍。
// Case A（太小，min_edge < min_thresh）：
//   固定 n=0（scale = s），搜尋正整數 s 使 min_edge*s 最接近 target。
// Case B（太大，min_edge > max_thresh）兩段式：
//   Step 1 — 找最小正整數 n，使 min_edge * 10^(-n) < min_thresh
//            （即 -(n-1) 時還不小於 min_thresh，-n 時才第一次跨過）
//   Step 2 — 在 base = min_edge * 10^(-n) 的基礎上，找正整數 s 使
//            base * s 落在 [min_thresh, max_thresh] 且最接近 target；
//            若無整數 s 能滿足區間，退而選讓誤差最小的 s。
// 回傳 (scale, active=true) 或 (1.0, false)。
static std::pair<double,bool> compute_normalize_scale(double min_edge,
                                                       double target,
                                                       double min_thresh,
                                                       double max_thresh)
{
    if (min_edge >= min_thresh - 1e-12 && min_edge <= max_thresh + 1e-12)
        return { 1.0, false };

    const bool too_small = min_edge < min_thresh;
    double best_scale = 1.0;
    double best_err   = std::numeric_limits<double>::infinity();

    if (too_small) {
        // Case A：scale = s（整數倍放大）
        const int s_max = static_cast<int>(std::ceil(target / min_edge)) * 4 + 2;
        for (int s = 1; s <= s_max; ++s) {
            const double err = std::fabs(min_edge * s - target);
            if (err < best_err) {
                best_err   = err;
                best_scale = static_cast<double>(s);
            }
        }
        std::cout << "[Normalize] trigger=small  min_edge=" << min_edge
                  << "  best_s=" << static_cast<int>(std::round(best_scale))
                  << "  n=0\n";
    } else {
        // Case B Step 1：找最小正整數 n 使 min_edge * 10^(-n) < min_thresh
        constexpr int n_max = 15;
        int    chosen_n    = -1;
        double chosen_pow  = 1.0;  // 10^(-n)
        double pow10       = 0.1;  // 10^(-1)
        for (int n = 1; n <= n_max; ++n, pow10 *= 0.1) {
            if (min_edge * pow10 < min_thresh - 1e-12) {
                chosen_n   = n;
                chosen_pow = pow10;
                break;
            }
        }

        if (chosen_n < 0) {
            // 超出 n_max 還找不到：用最後一個 pow10 當 fallback
            chosen_n   = n_max;
            chosen_pow = pow10;
        }

        // Case B Step 2：在 base = min_edge * 10^(-n) 上找 s，
        // 優先選 base*s ∈ [min_thresh, max_thresh] 且最接近 target 的 s；
        // 若無法落在區間，退而選誤差最小的 s。
        const double base   = min_edge * chosen_pow;
        const int    s_min  = std::max(1, static_cast<int>(std::floor(min_thresh / base)));
        const int    s_max2 = static_cast<int>(std::ceil(max_thresh / base)) + 1;

        // 先搜 [min_thresh, max_thresh] 內的 s
        for (int s = s_min; s <= s_max2; ++s) {
            const double cand = base * s;
            if (cand < min_thresh - 1e-9 || cand > max_thresh + 1e-9) continue;
            const double err = std::fabs(cand - target);
            if (err < best_err) {
                best_err   = err;
                best_scale = chosen_pow * s;
            }
        }

        // 若找不到落在區間內的 s，退而全局搜（不限 [min, max]）
        if (best_scale == 1.0 && best_err == std::numeric_limits<double>::infinity()) {
            const int s_fb_max = static_cast<int>(std::ceil(target / base)) * 4 + 2;
            for (int s = 1; s <= s_fb_max; ++s) {
                const double err = std::fabs(base * s - target);
                if (err < best_err) {
                    best_err   = err;
                    best_scale = chosen_pow * s;
                }
            }
        }

        const double scaled = min_edge * best_scale;
        std::cout << "[Normalize] trigger=large  min_edge=" << min_edge
                  << "  n=" << chosen_n << "  base=" << base
                  << "  s=" << static_cast<int>(std::round(best_scale / chosen_pow))
                  << "  scale=" << best_scale
                  << "  -> scaled_min_edge=" << scaled << "\n";
    }
    return { best_scale, true };
}

void PlacementEngine::apply_geometry_scale(double factor)
{
    if (std::fabs(factor - 1.0) < 1e-15) return;

    const int nd = static_cast<int>(dies_.size());
    std::vector<double> scaled_w(nd), scaled_h(nd);
    for (int t = 0; t < nd; ++t) {
        scaled_w[t] = dies_[t].width  * factor;
        scaled_h[t] = dies_[t].height * factor;
    }
    setup_dies(nd, scaled_w, scaled_h);

    for (Module& m : modules_) {
        m.x      *= factor;
        m.y      *= factor;
        m.width  *= factor;
        m.height *= factor;
    }
    for (TSV& tsv : tsvs_) {
        tsv.x *= factor;
        tsv.y *= factor;
    }
}

void PlacementEngine::restore_geometry()
{
    if (std::fabs(geometry_scale_ - 1.0) < 1e-15) return;

    const double inv = 1.0 / geometry_scale_;

    // die：從原始 Outline 重建（最精確，避免浮點累積誤差）
    const int nd = static_cast<int>(orig_die_w_.size());
    if (nd > 0 && nd == static_cast<int>(dies_.size())) {
        setup_dies(nd, orig_die_w_, orig_die_h_);
    } else {
        apply_geometry_scale(inv);  // fallback
        geometry_scale_ = 1.0;
        return;
    }

    for (Module& m : modules_) {
        m.x      *= inv;
        m.y      *= inv;
        m.width  *= inv;
        m.height *= inv;
    }
    for (TSV& tsv : tsvs_) {
        tsv.x *= inv;
        tsv.y *= inv;
    }

    geometry_scale_ = 1.0;
    std::cout << "[Normalize] Geometry restored to physical coordinates.\n";
}

bool PlacementEngine::maybe_normalize_geometry()
{
    const auto& c = cfg_;
    if (!c.enable_die_normalize) return false;
    if (dies_.empty()) return false;

    // 計算全 die 所有邊長的最小值
    double min_edge = std::numeric_limits<double>::infinity();
    for (const Die& d : dies_) {
        min_edge = std::min(min_edge, std::min(d.width, d.height));
    }

    // 存原始 Outline（供 restore 精確還原）
    orig_die_w_.resize(dies_.size());
    orig_die_h_.resize(dies_.size());
    for (int t = 0; t < static_cast<int>(dies_.size()); ++t) {
        orig_die_w_[t] = dies_[t].width;
        orig_die_h_[t] = dies_[t].height;
    }

    auto [scale, active] = compute_normalize_scale(
        min_edge, c.die_normalize_target,
        c.die_normalize_min_extent, c.die_normalize_max_extent);

    if (!active) {
        std::cout << "[Normalize] trigger=none  min_edge=" << min_edge
                  << "  in [" << c.die_normalize_min_extent
                  << ", " << c.die_normalize_max_extent << "], no scaling.\n";
        return false;
    }

    geometry_scale_ = scale;
    std::cout << "[Normalize] Applying scale=" << scale
              << "  min_edge " << min_edge << " -> " << min_edge * scale << "\n";
    apply_geometry_scale(scale);
    return true;
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
        if (m.is_fixed)    continue;   // 被 constraint 固定的 module 保持原位

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

    const int nd = static_cast<int>(dies_.size());
    tier_rc_alpha_mult_.assign(static_cast<size_t>(nd), 0.0);

    // ---- log 目前 wirelength model ----
    {
        const bool use_wa = (cfg_.wirelength_model == WirelengthModel::WA);
        std::cout << "[Solve] wirelength_model="
                  << (use_wa ? "WA" : "LSE")
                  << "  gamma=" << (use_wa ? cfg_.gamma_wa : cfg_.gamma_lse)
                  << "\n";
    }

    // σ 平滑半徑：以各層 Die 寬度最大值為基準
    double die_w = 268.0;
    if (!dies_.empty()) {
        die_w = 0.0;
        for (const Die& d : dies_) die_w = std::max(die_w, d.width);
    }
    const double sigma_start = cfg_.sigma_start_frac * die_w;
    const double sigma_end   = cfg_.sigma_end_frac   * die_w;

    std::vector<double> gx_wl(n), gy_wl(n);   // Wirelength 梯度
    std::vector<double> gx_d(n),  gy_d(n);    // Density 梯度
    std::vector<double> gx_rep(n), gy_rep(n); // Repulsion 梯度（REPULSE constraint）
    std::vector<double> gx_rc(n),  gy_rc(n);  // Routing congestion 梯度（週期性更新）
    std::vector<double> gx(n),    gy(n);       // 合併梯度
    std::vector<double> look_x(n), look_y(n);  // NAG lookahead 暫存

    double prev_max_overflow   = 0.0;
    double prev_total_overflow = 0.0;
    bool   have_prev_overflow  = false;
    int    overflow_stable_streak = 0;

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
            // terminal 和 fixed module 均保持原位不做 lookahead 偏移
            if (modules_[i].is_terminal || modules_[i].is_fixed) {
                look_x[i] = modules_[i].x;
                look_y[i] = modules_[i].y;
                continue;
            }
            look_x[i] = modules_[i].x + mu * (modules_[i].x - prev_x_[i]);
            look_y[i] = modules_[i].y + mu * (modules_[i].y - prev_y_[i]);
        }

        // 暫存 x_k，再把 module 座標移到 lookahead 位置以計算梯度
        // fixed module 也暫存但不搬移（density map 和 WL gradient 仍需感知其位置）
        for (int i = 0; i < n; ++i) {
            prev_x_[i]    = modules_[i].x;
            prev_y_[i]    = modules_[i].y;
            modules_[i].x = look_x[i];
            modules_[i].y = look_y[i];
        }

        // ---- Step 3: 計算梯度（fixed module 位置已正確反映在 look_x/y 中）----
        update_density_map();
        calculate_wirelength_gradient(gx_wl, gy_wl);
        calculate_density_gradient(gx_d,  gy_d);
        calculate_repulsion_gradient(gx_rep, gy_rep); // REPULSE constraint 斥力

        // Routing congestion 梯度：alpha>0 或 adaptive 模式時依週期排程重算，否則清零
        if ((cfg_.routing_congestion_alpha > 0.0 || cfg_.routing_congestion_max > 0.0)
            && iter >= cfg_.routing_congestion_start_iter
            && ((iter - cfg_.routing_congestion_start_iter)
                    % cfg_.routing_congestion_refresh_interval == 0))
        {
            // Per-tier adaptive alpha：先更新倍率再決定是否需要算梯度
            if (cfg_.routing_congestion_max > 0.0) {
                BinEdgeCongestionStats cstats = compute_bin_edge_congestion(*this);
                std::cout << "[RC-alpha] iter=" << iter;
                for (int t = 0; t < nd; ++t) {
                    const size_t ti = static_cast<size_t>(t);
                    const double tmax = t < static_cast<int>(cstats.tier_max.size())
                                        ? cstats.tier_max[ti] : 0.0;
                    if (tmax > cfg_.routing_congestion_max) {
                        const double prev = tier_rc_alpha_mult_[ti];
                        tier_rc_alpha_mult_[ti] = std::min(
                            (prev < 1.0 ? 1.0 : prev * cfg_.routing_congestion_alpha_boost_rate),
                            cfg_.routing_congestion_alpha_max_mult);
                    } else {
                        const double prev = tier_rc_alpha_mult_[ti];
                        tier_rc_alpha_mult_[ti] = std::max(
                            1.0, prev / cfg_.routing_congestion_alpha_boost_rate);
                    }
                    std::cout << " t" << t << "=" << std::fixed << std::setprecision(3)
                              << tmax << (tmax > cfg_.routing_congestion_max ? "!(a=" : " (a=")
                              << std::setprecision(2) << tier_rc_alpha_mult_[ti] << ")";
                }
                std::cout << "\n";
            }

            // 若所有層 alpha 倍率皆為 0 則跳過梯度計算
            const bool any_active = [&] {
                if (cfg_.routing_congestion_max <= 0.0) return cfg_.routing_congestion_alpha > 0.0;
                for (int t = 0; t < nd; ++t)
                    if (tier_rc_alpha_mult_[static_cast<size_t>(t)] > 0.0) return true;
                return false;
            }();

            std::fill(gx_rc.begin(), gx_rc.end(), 0.0);
            std::fill(gy_rc.begin(), gy_rc.end(), 0.0);
            if (any_active)
                calculate_routing_congestion_gradient(*this, gx_rc, gy_rc);
        } else {
            std::fill(gx_rc.begin(), gx_rc.end(), 0.0);
            std::fill(gy_rc.begin(), gy_rc.end(), 0.0);
        }

        // ---- Step 4: RMS 正規化（只統計真正可動模組）----
        // wl_rms 為基準；density 與 routing congestion 梯度均以 wl_rms / x_rms 縮放，
        // 使三路梯度的位移量尺度一致，再分別由 lambda（density）與 alpha（congestion）控制力道。
        double wl_sq = 0.0, d_sq = 0.0, rc_sq = 0.0;
        int    movable = 0;
        for (int i = 0; i < n; ++i) {
            if (modules_[i].is_terminal || modules_[i].is_fixed) continue;
            wl_sq += gx_wl[i]*gx_wl[i] + gy_wl[i]*gy_wl[i];
            d_sq  += gx_d[i] *gx_d[i]  + gy_d[i] *gy_d[i];
            rc_sq += gx_rc[i]*gx_rc[i] + gy_rc[i]*gy_rc[i];
            ++movable;
        }
        const double denom   = 2.0 * movable + 1e-12;
        const double wl_rms  = std::sqrt(wl_sq / denom);
        const double d_rms   = std::sqrt(d_sq  / denom);
        const double rc_rms  = std::sqrt(rc_sq / denom);

        const double scale_d  = (d_rms  > 1e-12) ? (wl_rms / d_rms)  : 0.0;
        const double scale_rc = (rc_rms > 1e-12) ? (wl_rms / rc_rms) : 0.0;

        // ---- Step 5: 合併梯度並更新位置（fixed module 不更新）----
        for (int i = 0; i < n; ++i) {
            if (modules_[i].is_terminal || modules_[i].is_fixed) continue;

            double lam = dies_[modules_[i].tier_id].lambda * lambda_mult_;
            const double rc_mult = tier_rc_alpha_mult_.empty() ? 1.0
                : tier_rc_alpha_mult_[static_cast<size_t>(modules_[i].tier_id)];
            // adaptive 模式：rc_mult 本身即為有效 alpha（0 = 關閉，1+ = 啟動並遞增）
            // 非 adaptive 模式：沿用固定的 routing_congestion_alpha × rc_mult
            const double rc_alpha = (cfg_.routing_congestion_max > 0.0)
                ? rc_mult
                : cfg_.routing_congestion_alpha * rc_mult;

            gx[i] = gx_wl[i] + gx_rep[i]
                    + rc_alpha * scale_rc * gx_rc[i]
                    + lam * scale_d * gx_d[i];
            gy[i] = gy_wl[i] + gy_rep[i]
                    + rc_alpha * scale_rc * gy_rc[i]
                    + lam * scale_d * gy_d[i];

            modules_[i].x = look_x[i] - step * gx[i];
            modules_[i].y = look_y[i] - step * gy[i];

            clamp_to_die(modules_[i], dies_[modules_[i].tier_id]);
        }

        step *= decay;

        // ---- Step 5b: 週期性旋轉優化（pairwise 重疊面積）----
        // 在 rotation_start_iter 之後，每隔 rotation_interval 次對所有可動 module
        // 試旋轉 90°（以中心點為軸），比較旋轉前後與同 tier 所有其他 module 的
        // 重疊面積總和，取較小者保留。
        if (cfg_.rotation_start_iter > 0
         && cfg_.rotation_interval   > 0
         && iter >= cfg_.rotation_start_iter
         && (iter - cfg_.rotation_start_iter) % cfg_.rotation_interval == 0)
        {
            // 計算 m 與同 tier 所有其他 module 的重疊面積加總
            auto pairwise_overlap = [&](const Module& m) -> double {
                constexpr double eps = 1e-12;
                double total = 0.0;
                for (const Module& o : modules_) {
                    if (o.id == m.id || o.tier_id != m.tier_id) continue;
                    const double ox = std::max(0.0, std::min(m.rx(), o.rx())
                                                  - std::max(m.lx(), o.lx()));
                    const double oy = std::max(0.0, std::min(m.ry(), o.ry())
                                                  - std::max(m.ly(), o.ly()));
                    total += ox * oy;
                }
                return total + eps; // eps 防止浮點相等時不必要旋轉
            };

            int rotated_count = 0;
            for (Module& m : modules_) {
                if (m.is_terminal || m.is_fixed) continue;
                const Die& die = dies_[static_cast<size_t>(m.tier_id)];

                // 旋轉後（swap 長寬）若 footprint 會超出 die，則不嘗試旋轉
                const double hw_after = m.height * 0.5; // swap 後的半寬 = 原半高
                const double hh_after = m.width  * 0.5; // swap 後的半高 = 原半寬
                if (m.x < hw_after || m.x > die.width  - hw_after
                 || m.y < hh_after || m.y > die.height - hh_after)
                    continue;

                const double ov_before = pairwise_overlap(m);
                std::swap(m.width, m.height);
                const double ov_after  = pairwise_overlap(m);

                if (ov_after >= ov_before)
                    std::swap(m.width, m.height); // 無改善，還原
                else
                    ++rotated_count;
            }
            std::cout << "[Rotation] iter=" << iter
                      << " rotated_modules=" << rotated_count << "\n";
        }

        // ---- Step 6: 收斂監控（每 50 次）----
        if (iter % 50 == 0) {
            double hpwl = compute_hpwl();

            double max_overflow    = 0.0;
            double total_overflow  = 0.0; // 所有 bin 的正超出量加總
            for (const Die& die : dies_) {
                for (const Bin& b : die.bins) {
                    const double ex = b.density - b.target_density;
                    max_overflow   = std::max(max_overflow, ex);
                    total_overflow += std::max(0.0, ex);
                }
            }

            double rel_change = std::fabs(prev_hpwl_ - hpwl) /
                                (prev_hpwl_ + 1e-12);

            // 相鄰兩次「每 50 iter」記錄點的 overflow 變化量（連續 n 次皆小才累積）
            if (have_prev_overflow) {
                const double d_max = std::fabs(max_overflow - prev_max_overflow);
                const double d_tot = std::fabs(total_overflow - prev_total_overflow);
                if (d_max < cfg_.convergence_max_overflow_delta_tol
                    && d_tot < cfg_.convergence_total_overflow_delta_tol)
                    ++overflow_stable_streak;
                else
                    overflow_stable_streak = 0;
            }
            prev_max_overflow   = max_overflow;
            prev_total_overflow = total_overflow;
            have_prev_overflow    = true;

            std::cout << "[Iter " << std::setw(5) << iter << "]"
                      << " HPWL="       << std::fixed      << std::setprecision(1) << hpwl
                      << " Overflow="   << std::setprecision(3) << max_overflow
                      << " TotalOverflow=" << std::setprecision(3) << total_overflow
                      << " λ_mult="     << std::setprecision(4) << lambda_mult_
                      << " σ="          << std::setprecision(1) << smooth_sigma_
                      << " WLrms="      << std::scientific  << std::setprecision(2) << wl_rms
                      << " Drms="       << d_rms
                      << " step="       << step
                      << "\n";

            if (cfg_.dump_analytical_iter_trace
                && !cfg_.analytical_iter_trace_path.empty()) {
                std::ofstream ofs(cfg_.analytical_iter_trace_path,
                                  (iter == 0) ? (std::ios::out | std::ios::trunc)
                                              : (std::ios::out | std::ios::app));
                if (ofs) {
                    ofs << std::fixed << std::setprecision(6);
                    ofs << "[Iter " << iter << "]\n";
                    for (const Module& m : modules_) {
                        if (m.is_terminal) continue;
                        ofs << m.name << ' ' << m.lx() << ' ' << m.ly() << ' '
                            << m.rx() << ' ' << m.ry() << '\n';
                    }
                    ofs << '\n';
                } else if (iter == 0) {
                    std::cerr << "[Solve] WARNING: cannot open analytical trace file: "
                              << cfg_.analytical_iter_trace_path << "\n";
                }
            }

            const bool overflow_stable =
                (cfg_.convergence_overflow_stable_steps > 0
                 && overflow_stable_streak >= cfg_.convergence_overflow_stable_steps);

            if (iter > 1000 && rel_change < tol && overflow_stable) {
                std::cout << "[Converged] iter=" << iter
                          << " HPWL=" << std::fixed << hpwl << "\n";
                break;
            }
            prev_hpwl_ = hpwl;
        }
    }

    const double hpwl_scaled = compute_hpwl();
    const double gs          = geometry_scale_;
    std::cout << "\n[Final] HPWL = " << std::fixed << std::setprecision(2) << hpwl_scaled;
    if (std::fabs(gs - 1.0) > 1e-12)
        std::cout << "  (actual HPWL = " << (hpwl_scaled / gs) << ", scale:" << gs << ")";
    std::cout << "\n";
}



// ============================================================
// compute_lse_wirelength:
//   LSE 近似 HPWL 的目標函數值
//   WL(e) = γ * [ln Σ exp(xi/γ) + ln Σ exp(-xi/γ)
//              + ln Σ exp(yi/γ) + ln Σ exp(-yi/γ)]
//   每條 net 再乘以 net_wirelength_die_weight(net)（與梯度一致）
// ============================================================
double PlacementEngine::compute_lse_wirelength() const
{
    double total = 0.0;
    const double g = cfg_.gamma_lse;

    for (const Net& net : nets_) {
        if (net.pins.size() < 2) continue;

        const double w_net = net_wirelength_die_weight(net);

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
            double xi = modules_[id].x;
            double yi = modules_[id].y;
            sum_exp_x  += std::exp((xi - max_x) / g);
            sum_exp_nx += std::exp((min_x - xi) / g);
            sum_exp_y  += std::exp((yi - max_y) / g);
            sum_exp_ny += std::exp((min_y - yi) / g);
        }

        // WL_e = γ * [ln(Σe^{xi/γ}) + ln(Σe^{-xi/γ}) + ...]
        //      = γ * [max_x/γ + ln(Σe^{(xi-max_x)/γ}) + ...]  等效形式
        const double wl_e = g * (std::log(sum_exp_x)  + max_x  / g
                    + std::log(sum_exp_nx) + (-min_x) / g
                    + std::log(sum_exp_y)  + max_y  / g
                    + std::log(sum_exp_ny) + (-min_y) / g);
        total += w_net * wl_e;
    }
    return total;
}

// ============================================================
// calculate_wirelength_gradient:
//   依 cfg_.wirelength_model 選擇 LSE 或 WA 梯度。
//
//   LSE: ∂WL/∂xj = w_net * (exp(xj/γ)/Σexp(xi/γ) - exp(-xj/γ)/Σexp(-xi/γ))
//
//   WA (ePlace):
//     x̄+ = Σ xi*exp(xi/γ) / Σ exp(xi/γ)   ≈ max(xi)
//     x̄- = Σ xi*exp(-xi/γ) / Σ exp(-xi/γ)  ≈ min(xi)
//     ∂WL/∂xj = (1/γ) * [ (exp_p/A)*(γ+xj-x̄+) - (exp_n/C)*(γ-xj+x̄-) ]
//
//   兩者均使用 max/min shift 以確保數值穩定；w_net 為跨 tier die weight 平均。
// ============================================================
void PlacementEngine::calculate_wirelength_gradient(
    std::vector<double>& gx, std::vector<double>& gy) const
{
    std::fill(gx.begin(), gx.end(), 0.0);
    std::fill(gy.begin(), gy.end(), 0.0);

    const bool use_wa = (cfg_.wirelength_model == WirelengthModel::WA);
    const double g = use_wa ? cfg_.gamma_wa : cfg_.gamma_lse;

    for (const Net& net : nets_) {
        if (net.pins.size() < 2) continue;

        const double w_net = net_wirelength_die_weight(net);

        // ---- 共用前處理：max/min shift 與基本 exp 分母 ----
        double max_x = -1e18, min_x = 1e18;
        double max_y = -1e18, min_y = 1e18;
        for (int id : net.pins) {
            max_x = std::max(max_x, modules_[id].x);
            min_x = std::min(min_x, modules_[id].x);
            max_y = std::max(max_y, modules_[id].y);
            min_y = std::min(min_y, modules_[id].y);
        }

        // A = Σ exp((xi-max_x)/g)，C = Σ exp((min_x-xi)/g)
        // B, D 為 WA 額外需要的加權和（LSE 不用，但統一計算以共用迴圈）
        double A_x = 0.0, C_x = 0.0;
        double A_y = 0.0, C_y = 0.0;
        double B_x = 0.0, D_x = 0.0;  // WA: Σ xi*exp_p, Σ xi*exp_n
        double B_y = 0.0, D_y = 0.0;

        for (int id : net.pins) {
            const double xi = modules_[id].x;
            const double yi = modules_[id].y;
            const double ep_x = std::exp((xi - max_x) / g);
            const double en_x = std::exp((min_x - xi) / g);
            const double ep_y = std::exp((yi - max_y) / g);
            const double en_y = std::exp((min_y - yi) / g);
            A_x += ep_x;  C_x += en_x;
            A_y += ep_y;  C_y += en_y;
            if (use_wa) {
                B_x += xi * ep_x;  D_x += xi * en_x;
                B_y += yi * ep_y;  D_y += yi * en_y;
            }
        }

        if (use_wa) {
            // WA: x̄+ = B/A, x̄- = D/C（shift 後仍等價）
            const double x_bar_p = B_x / A_x;
            const double x_bar_n = D_x / C_x;
            const double y_bar_p = B_y / A_y;
            const double y_bar_n = D_y / C_y;
            const double inv_g = 1.0 / g;

            for (int id : net.pins) {
                if (modules_[id].is_terminal) continue;
                const double xi = modules_[id].x;
                const double yi = modules_[id].y;
                const double ep_x = std::exp((xi - max_x) / g);
                const double en_x = std::exp((min_x - xi) / g);
                const double ep_y = std::exp((yi - max_y) / g);
                const double en_y = std::exp((min_y - yi) / g);
                // ∂WL_x/∂xj = (1/γ)*[ (ep/A)*(γ+xj-x̄+) - (en/C)*(γ-xj+x̄-) ]
                gx[id] += w_net * inv_g * (
                    (ep_x / A_x) * (g + xi - x_bar_p)
                  - (en_x / C_x) * (g - xi + x_bar_n)
                );
                gy[id] += w_net * inv_g * (
                    (ep_y / A_y) * (g + yi - y_bar_p)
                  - (en_y / C_y) * (g - yi + y_bar_n)
                );
            }
        } else {
            // LSE: ∂WL/∂xj = w_net * (ep/A - en/C)
            for (int id : net.pins) {
                if (modules_[id].is_terminal) continue;
                const double xi = modules_[id].x;
                const double yi = modules_[id].y;
                const double ep_x = std::exp((xi - max_x) / g);
                const double en_x = std::exp((min_x - xi) / g);
                const double ep_y = std::exp((yi - max_y) / g);
                const double en_y = std::exp((min_y - yi) / g);
                gx[id] += w_net * (ep_x / A_x - en_x / C_x);
                gy[id] += w_net * (ep_y / A_y - en_y / C_y);
            }
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
// calculate_repulsion_gradient:
//   對每個 RepulsionGroup 掃所有無序 pair (a, b)：
//
//     E_rep = k / (dx^2 + dy^2 + ε)
//     ∂E/∂xa = -k * 2*dx / (dx^2 + dy^2 + ε)^2  → 推離 b（取正讓梯度下降時往斥力方向走）
//
//   跨 tier 僅用 2D 平面座標（x, y）計算，不考慮 Z 距離。
//   is_terminal 與 is_fixed 的 module 不施加梯度（不可動）。
//   epsilon = 1.0（與 die 座標同量綱，防止距離為 0 時梯度爆炸）。
// ============================================================
void PlacementEngine::calculate_repulsion_gradient(
    std::vector<double>& gx, std::vector<double>& gy) const
{
    constexpr double eps = 1.0;

    std::fill(gx.begin(), gx.end(), 0.0);
    std::fill(gy.begin(), gy.end(), 0.0);

    for (const RepulsionGroup& grp : repulsion_groups_) {
        const auto& ids = grp.module_ids;
        const double k  = grp.strength * 10000.0;

        for (int a = 0; a < static_cast<int>(ids.size()); ++a) {
            for (int b = a + 1; b < static_cast<int>(ids.size()); ++b) {
                const int ia = ids[a];
                const int ib = ids[b];

                // 不可動的 module：仍貢獻斥力給對方（作為障礙），自身梯度不更新
                const Module& ma = modules_[ia];
                const Module& mb = modules_[ib];

                const double dx = ma.x - mb.x;
                const double dy = ma.y - mb.y;
                const double d2 = dx * dx + dy * dy + eps;
                const double coeff = k * 2.0 / (d2 * d2);

                // 可動 module 才累加梯度（梯度方向 = 推離對方）
                if (!ma.is_terminal && !ma.is_fixed) {
                    gx[ia] += coeff * dx;
                    gy[ia] += coeff * dy;
                }
                if (!mb.is_terminal && !mb.is_fixed) {
                    gx[ib] -= coeff * dx;
                    gy[ib] -= coeff * dy;
                }
            }
        }
    }
}

// ============================================================
// hpwl_die_weight: 僅供 compute_hpwl 的每層乘數
//   cfg_.hpwl_die_weights 長度 == num_dies 時優先；否則用 .block 的 tier_net_weights_
// ============================================================
double PlacementEngine::hpwl_die_weight(int tier) const
{
    const int n = static_cast<int>(dies_.size());
    if (tier < 0 || tier >= n) return 1.0;
    const auto& ov = cfg_.hpwl_die_weights;
    if (static_cast<int>(ov.size()) == n)
        return ov[static_cast<size_t>(tier)];
    if (static_cast<int>(tier_net_weights_.size()) == n)
        return tier_net_weights_[static_cast<size_t>(tier)];
    return 1.0;
}

// ============================================================
// analytical_tier_net_weight: LSE / analytical 專用，僅讀 .block 的 tier_net_weights_
// ============================================================
double PlacementEngine::analytical_tier_net_weight(int tier) const
{
    const int n = static_cast<int>(dies_.size());
    if (tier < 0 || tier >= n) return 1.0;
    if (static_cast<int>(tier_net_weights_.size()) == n)
        return tier_net_weights_[static_cast<size_t>(tier)];
    return 1.0;
}

// ============================================================
// net_wirelength_die_weight: 該 net 在 LSE 中的 die weight 乘數
//   收集所有 pin 所在 tier（terminal 視為 tier 0），對相異 tier 的
//   analytical_tier_net_weight 取算術平均；單層 net 即為該層 .block weight。
// ============================================================
double PlacementEngine::net_wirelength_die_weight(const Net& net) const
{
    std::set<int> tiers;
    for (int id : net.pins) {
        const Module& m = modules_[id];
        const int     mt = m.is_terminal ? 0 : m.tier_id;
        tiers.insert(mt);
    }
    if (tiers.empty()) return 1.0;
    double sum_w = 0.0;
    for (int t : tiers)
        sum_w += analytical_tier_net_weight(t);
    return sum_w / static_cast<double>(tiers.size());
}

// ============================================================
// compute_hpwl: 計算精確 HPWL（含 terminal、module、TSV）
//
// 無 TSV（例如尚未 build_tsvs）：
//   每條 net 所有 pin 壓成單一 2D bbox，半周長 (Δx+Δy) 乘以
//   「該 net 有 pin 的各層」之 hpwl_die_weight 的算術平均 (Σw / n)。
//
// 有 TSV：逐層 bbox（module + terminal@tier0 + 該層相關 TSV），
//   每層 (Δx+Δy) * hpwl_die_weight(t) 後累加。
// ============================================================
double PlacementEngine::compute_hpwl() const
{
    double total = 0.0;
    const int num_tiers = static_cast<int>(dies_.size());

    if (tsvs_.empty()) {
        for (const Net& net : nets_) {
            double x_min = 1e18, x_max = -1e18;
            double y_min = 1e18, y_max = -1e18;
            std::set<int> tiers_seen;

            for (int id : net.pins) {
                const Module& m = modules_[id];
                x_min = std::min(x_min, m.x);
                x_max = std::max(x_max, m.x);
                y_min = std::min(y_min, m.y);
                y_max = std::max(y_max, m.y);
                const int mt = m.is_terminal ? 0 : m.tier_id;
                tiers_seen.insert(mt);
            }

            if (x_min <= x_max && y_min <= y_max) {
                const double flat_hpwl = (x_max - x_min) + (y_max - y_min);
                double w_avg = 1.0;
                if (!tiers_seen.empty()) {
                    double sum_w = 0.0;
                    for (int t : tiers_seen)
                        sum_w += hpwl_die_weight(t);
                    w_avg = sum_w / static_cast<double>(tiers_seen.size());
                }
                total += w_avg * flat_hpwl;
            }
        }
        return total;
    }

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

            for (int id : net.pins) {
                const Module& m = modules_[id];
                const int mt = m.is_terminal ? 0 : m.tier_id;
                if (mt == t) expand(m.x, m.y);
            }

            for (const TSV& tsv : tsvs_) {
                if (tsv.net_id != net.id) continue;
                if (tsv.tier_below() == t || tsv.tier_above() == t)
                    expand(tsv.x, tsv.y);
            }

            if (x_min <= x_max && y_min <= y_max) {
                const double layer_hpwl = (x_max - x_min) + (y_max - y_min);
                total += hpwl_die_weight(t) * layer_hpwl;
            }
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
//   Line 4: bbox_w  bbox_h     (所有可移動方塊的最大外框，浮點寬高)
//   Line 5: runtime (秒)
//   Line 6+: <name> <die_id> <x_ll> <y_ll> <x_ur> <y_ur>（浮點座標）
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
    double bbox_w = 0.0, bbox_h = 0.0, bbox_area = 0.0;
    if (g_xmin <= g_xmax && g_ymin <= g_ymax) {
        bbox_w    = g_xmax - g_xmin;
        bbox_h    = g_ymax - g_ymin;
        bbox_area = bbox_w * bbox_h;
    }

    double hpwl       = compute_hpwl();  // 含 terminal、module、TSV 的 bbox
    double total_cost = hpwl;

    // ---- 寫出前五行統計資訊 ----
    std::fprintf(fp, "%f\n",   total_cost);
    std::fprintf(fp, "%f\n",   hpwl);
    std::fprintf(fp, "%f\n",   bbox_area);
    std::fprintf(fp, "%.15g %.15g\n", bbox_w, bbox_h);
    std::fprintf(fp, "%f\n",   runtime);

    // ---- 逐一輸出每個可移動方塊 ----
    // <name> <die_id> <x_ll> <y_ll> <x_ur> <y_ur>
    for (const Module& m : modules_) {
        if (m.is_terminal) continue;
        std::fprintf(fp, "%s %d %.15g %.15g %.15g %.15g\n",
                     m.name.c_str(), m.tier_id,
                     m.lx(), m.ly(), m.rx(), m.ry());
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
// write_density_map: 將各 tier 的 bin 密度圖輸出成文字檔與 PNG
//
// 每層產生：
//   <base_filename>_density_tier<N>.txt（同上原有格式）
//   <base_filename>_density_tier<N>.png — RGB，寬=C、高=R（每 bin 一像素），
//      影像頂端對應 chip 較大 y（與 ASCII 區塊一致）；該層最大密度對應最深色。
// .txt 內容包含：
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

        double max_rho_bin = 0.0;
        for (const Bin& b : die.bins)
            max_rho_bin = std::max(max_rho_bin, b.density);
        const double vmax_png = std::max(max_rho_bin, 1e-18);

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

        // ---- PNG：每 bin 一像素（寬=C、高=R）；頂列 = chip y 較大側（與 ASCII 區塊一致）
        {
            std::vector<unsigned char> png(static_cast<size_t>(R * C * 3u));
            for (int py = 0; py < R; ++py) {
                const int chip_r = R - 1 - py;
                for (int px = 0; px < C; ++px) {
                    const double rho =
                        die.bins[static_cast<size_t>(chip_r * C + px)].density;
                    unsigned char rgb[3];
                    density_png_u_to_rgb(rho / vmax_png, rgb);
                    const size_t o =
                        (static_cast<size_t>(py) * static_cast<size_t>(C)
                         + static_cast<size_t>(px)) * 3u;
                    png[o + 0] = rgb[0];
                    png[o + 1] = rgb[1];
                    png[o + 2] = rgb[2];
                }
            }
            std::string png_path = base_filename + "_density_tier"
                                   + std::to_string(die.id) + ".png";
            if (!stbi_write_png(png_path.c_str(), C, R, 3, png.data(),
                                static_cast<int>(C * 3))) {
                std::cerr << "[DensityMap] Cannot write PNG: " << png_path << "\n";
            } else {
                std::cout << "[DensityMap] Tier " << die.id << " -> " << png_path
                          << "  (" << C << "x" << R << " px, 1 bin = 1 px)\n";
            }
        }

        std::cout << "[DensityMap] Tier " << die.id << " -> " << fname << "\n";
    }
}
