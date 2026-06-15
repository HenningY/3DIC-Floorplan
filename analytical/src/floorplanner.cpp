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
// clamp_tsv_to_die: TSV 中心夾取到 tier_below die 邊界（留半寬邊距）
// ============================================================
void PlacementEngine::clamp_tsv_to_die(TSV& tsv) const
{
    const Die& die = dies_[static_cast<size_t>(tsv.tier_below())];
    const double hw = cfg_.analytical_tsv_width  * 0.5;
    const double hh = cfg_.analytical_tsv_height * 0.5;
    tsv.x = std::max(hw, std::min(die.width  - hw, tsv.x));
    tsv.y = std::max(hh, std::min(die.height - hh, tsv.y));
}

// ============================================================
// run_nag_loop: Nesterov Accelerated Gradient 主迴圈
//   include_tsv=false → Phase 1（純 module，維持舊行為）
//   include_tsv=true  → Phase 2（module + TSV joint）
// ============================================================
void PlacementEngine::run_nag_loop(int max_iter, bool include_tsv)
{
    const double mu    = cfg_.momentum;
    double       step  = include_tsv && nag_step_ > 0.0
                         ? nag_step_
                         : cfg_.init_step_size;
    const double decay = cfg_.step_decay;
    const double tol   = cfg_.convergence_tol;

    int n  = static_cast<int>(modules_.size());
    int nt = include_tsv ? static_cast<int>(tsvs_.size()) : 0;

    const int nd = static_cast<int>(dies_.size());

    // σ 平滑半徑：以各層 Die 寬度最大值為基準
    double die_w = 268.0;
    if (!dies_.empty()) {
        die_w = 0.0;
        for (const Die& d : dies_) die_w = std::max(die_w, d.width);
    }
    const double sigma_start = cfg_.sigma_start_frac * die_w;
    const double sigma_end   = cfg_.sigma_end_frac   * die_w;
    const int    sigma_budget = cfg_.max_iterations + cfg_.analytical_tsv_max_iterations;

    // Module 梯度向量
    std::vector<double> gx_wl(n), gy_wl(n);
    std::vector<double> gx_d(n),  gy_d(n);
    std::vector<double> gx_rep(n), gy_rep(n);
    std::vector<double> gx_rc(n),  gy_rc(n);
    std::vector<double> gx(n),    gy(n);
    std::vector<double> look_x(n), look_y(n);

    // TSV 梯度向量（Phase 2 用）
    std::vector<double> gx_tsv_wl(static_cast<size_t>(nt)),
                        gy_tsv_wl(static_cast<size_t>(nt));
    std::vector<double> gx_tsv_d(static_cast<size_t>(nt)),
                        gy_tsv_d(static_cast<size_t>(nt));
    std::vector<double> gx_tsv_rc(static_cast<size_t>(nt)),
                        gy_tsv_rc(static_cast<size_t>(nt));
    std::vector<double> look_tsv_x(static_cast<size_t>(nt)),
                        look_tsv_y(static_cast<size_t>(nt));

    double prev_max_overflow   = 0.0;
    double prev_total_overflow = 0.0;
    bool   have_prev_overflow  = false;
    int    overflow_stable_streak = 0;

    // Phase 2：所有非 terminal / 非 fixed 的 module 都可移動
    std::vector<bool> phase2_movable(static_cast<size_t>(n), false);
    if (include_tsv) {
        int n_movable = 0;
        for (int i = 0; i < n; ++i) {
            const Module& m = modules_[i];
            if (m.is_terminal || m.is_fixed || m.tier_id < 0) continue;
            phase2_movable[static_cast<size_t>(i)] = true;
            ++n_movable;
        }
        std::cout << "[SolveTSV phase] all modules movable: " << n_movable << "/" << n << "\n";
    }

    for (int iter = 0; iter < max_iter; ++iter) {

        // ---- Step 1a: σ 線性退火（跨 Phase 累積 nag_iter_，Phase 2 不接續從 0 重算）----
        if (sigma_budget > 0) {
            const double t = std::min(1.0,
                static_cast<double>(nag_iter_) / static_cast<double>(sigma_budget));
            smooth_sigma_ = sigma_start + (sigma_end - sigma_start) * t;
        }

        // ---- Step 1b: λ 遞增排程 ----
        if (nag_iter_ > 0 && nag_iter_ % cfg_.lambda_update_interval == 0) {
            lambda_mult_ = std::min(lambda_mult_ * cfg_.lambda_increase_rate,
                                    cfg_.lambda_max_mult);
        }

        // ---- Step 2: NAG lookahead（module）----
        // Phase 2 時大 module 凍結（look = 當前座標）；小 module 同 Phase 1 做 NAG lookahead
        if (include_tsv) {
            for (int i = 0; i < n; ++i) {
                if (phase2_movable[static_cast<size_t>(i)]) {
                    look_x[i] = modules_[i].x + mu * (modules_[i].x - prev_x_[i]);
                    look_y[i] = modules_[i].y + mu * (modules_[i].y - prev_y_[i]);
                } else {
                    look_x[i] = modules_[i].x;
                    look_y[i] = modules_[i].y;
                }
            }
            for (int i = 0; i < n; ++i) {
                if (!phase2_movable[static_cast<size_t>(i)]) continue;
                prev_x_[i]    = modules_[i].x;
                prev_y_[i]    = modules_[i].y;
                modules_[i].x = look_x[i];
                modules_[i].y = look_y[i];
            }
        } else {
            for (int i = 0; i < n; ++i) {
                if (modules_[i].is_terminal || modules_[i].is_fixed) {
                    look_x[i] = modules_[i].x;
                    look_y[i] = modules_[i].y;
                    continue;
                }
                look_x[i] = modules_[i].x + mu * (modules_[i].x - prev_x_[i]);
                look_y[i] = modules_[i].y + mu * (modules_[i].y - prev_y_[i]);
            }
            for (int i = 0; i < n; ++i) {
                prev_x_[i]    = modules_[i].x;
                prev_y_[i]    = modules_[i].y;
                modules_[i].x = look_x[i];
                modules_[i].y = look_y[i];
            }
        }

        // TSV NAG lookahead（Phase 2）
        if (include_tsv) {
            for (int i = 0; i < nt; ++i) {
                look_tsv_x[static_cast<size_t>(i)] =
                    tsvs_[static_cast<size_t>(i)].x
                    + mu * (tsvs_[static_cast<size_t>(i)].x - prev_tsv_x_[static_cast<size_t>(i)]);
                look_tsv_y[static_cast<size_t>(i)] =
                    tsvs_[static_cast<size_t>(i)].y
                    + mu * (tsvs_[static_cast<size_t>(i)].y - prev_tsv_y_[static_cast<size_t>(i)]);
            }
            for (int i = 0; i < nt; ++i) {
                prev_tsv_x_[static_cast<size_t>(i)] = tsvs_[static_cast<size_t>(i)].x;
                prev_tsv_y_[static_cast<size_t>(i)] = tsvs_[static_cast<size_t>(i)].y;
                tsvs_[static_cast<size_t>(i)].x = look_tsv_x[static_cast<size_t>(i)];
                tsvs_[static_cast<size_t>(i)].y = look_tsv_y[static_cast<size_t>(i)];
            }
        }

        // ---- Step 3: 計算梯度 ----
        update_density_map();
        if (include_tsv) {
            calculate_wirelength_gradient(gx_wl, gy_wl, &gx_tsv_wl, &gy_tsv_wl);
            calculate_density_gradient(gx_d, gy_d, &gx_tsv_d, &gy_tsv_d);
        } else {
            calculate_wirelength_gradient(gx_wl, gy_wl);
            calculate_density_gradient(gx_d, gy_d);
        }
        calculate_repulsion_gradient(gx_rep, gy_rep);

        // Routing congestion 梯度
        if ((cfg_.routing_congestion_alpha > 0.0 || cfg_.routing_congestion_max > 0.0)
            && nag_iter_ >= cfg_.routing_congestion_start_iter
            && ((nag_iter_ - cfg_.routing_congestion_start_iter)
                    % cfg_.routing_congestion_refresh_interval == 0))
        {
            update_routing_congestion_map();

            if (cfg_.routing_congestion_max > 0.0) {
                BinEdgeCongestionStats cstats =
                    bin_edge_stats_from_demands(rc_edge_demands_, dies_);
                std::cout << "[RC-alpha] iter=" << nag_iter_;
                for (int t = 0; t < nd; ++t) {
                    const size_t ti = static_cast<size_t>(t);
                    const double tmax = t < static_cast<int>(cstats.tier_max.size())
                                        ? cstats.tier_max[ti] : 0.0;
                    const double ttop = t < static_cast<int>(cstats.tier_top10p_mean.size())
                                        ? cstats.tier_top10p_mean[ti] : 0.0;
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
                    std::cout << " t" << t << "=max " << std::fixed << std::setprecision(3)
                              << tmax << "/top10% " << std::setprecision(3) << ttop
                              << (tmax > cfg_.routing_congestion_max ? "!(a=" : " (a=")
                              << std::setprecision(2) << tier_rc_alpha_mult_[ti] << ")";
                }
                std::cout << "\n";
            }

            const bool any_active = [&] {
                if (cfg_.routing_congestion_max <= 0.0) return cfg_.routing_congestion_alpha > 0.0;
                for (int t = 0; t < nd; ++t)
                    if (tier_rc_alpha_mult_[static_cast<size_t>(t)] > 0.0) return true;
                return false;
            }();

            std::fill(gx_rc.begin(), gx_rc.end(), 0.0);
            std::fill(gy_rc.begin(), gy_rc.end(), 0.0);
            if (include_tsv) {
                std::fill(gx_tsv_rc.begin(), gx_tsv_rc.end(), 0.0);
                std::fill(gy_tsv_rc.begin(), gy_tsv_rc.end(), 0.0);
            }
            if (any_active) {
                if (include_tsv)
                    calculate_routing_congestion_gradient(*this, gx_rc, gy_rc,
                                                          &gx_tsv_rc, &gy_tsv_rc);
                else
                    calculate_routing_congestion_gradient(*this, gx_rc, gy_rc);
            }
        } else {
            std::fill(gx_rc.begin(), gx_rc.end(), 0.0);
            std::fill(gy_rc.begin(), gy_rc.end(), 0.0);
            if (include_tsv) {
                std::fill(gx_tsv_rc.begin(), gx_tsv_rc.end(), 0.0);
                std::fill(gy_tsv_rc.begin(), gy_tsv_rc.end(), 0.0);
            }
        }

        // ---- Step 4: RMS 正規化 ----
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

        // ---- Step 5a: 更新 module 位置
        //   Phase 1：所有可動 module 更新
        //   Phase 2：所有非 terminal / 非 fixed module 均更新（phase2_movable 已全設為 true）
        {
            const bool phase1 = !include_tsv;
            for (int i = 0; i < n; ++i) {
                if (modules_[i].is_terminal || modules_[i].is_fixed) continue;
                if (!phase1 && !phase2_movable[static_cast<size_t>(i)]) continue;

                double lam = dies_[static_cast<size_t>(modules_[i].tier_id)].lambda * lambda_mult_;
                const double rc_mult = tier_rc_alpha_mult_.empty() ? 1.0
                    : tier_rc_alpha_mult_[static_cast<size_t>(modules_[i].tier_id)];
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

                clamp_to_die(modules_[i], dies_[static_cast<size_t>(modules_[i].tier_id)]);
            }
        }

        // ---- Step 5b: 更新 TSV 位置（Phase 2）----
        if (include_tsv) {
            for (int i = 0; i < nt; ++i) {
                const size_t si = static_cast<size_t>(i);
                TSV& tsv = tsvs_[si];
                const Die& die_tb = dies_[static_cast<size_t>(tsv.tier_below())];
                const double lam_tsv = die_tb.lambda * lambda_mult_;

                // rc_mult：取 tier_below 與 tier_above 的最大值
                const double rc_mult_lo = tier_rc_alpha_mult_.empty() ? 1.0
                    : tier_rc_alpha_mult_[static_cast<size_t>(tsv.tier_below())];
                const double rc_mult_hi = tier_rc_alpha_mult_.empty() ? 1.0
                    : tier_rc_alpha_mult_[static_cast<size_t>(tsv.tier_above())];
                const double rc_mult_tsv = std::max(rc_mult_lo, rc_mult_hi);
                const double rc_alpha_tsv = (cfg_.routing_congestion_max > 0.0)
                    ? rc_mult_tsv
                    : cfg_.routing_congestion_alpha * rc_mult_tsv;
                const double tsv_cong_scale = cfg_.analytical_tsv_congestion_scale;

                const double total_gx = gx_tsv_wl[si]
                    + lam_tsv * scale_d  * gx_tsv_d[si]
                    + rc_alpha_tsv * scale_rc * tsv_cong_scale * gx_tsv_rc[si];
                const double total_gy = gy_tsv_wl[si]
                    + lam_tsv * scale_d  * gy_tsv_d[si]
                    + rc_alpha_tsv * scale_rc * tsv_cong_scale * gy_tsv_rc[si];

                tsv.x = look_tsv_x[si] - step * total_gx;
                tsv.y = look_tsv_y[si] - step * total_gy;
                clamp_tsv_to_die(tsv);
            }
        }

        step *= decay;

        // ---- Step 5c: 週期性旋轉優化（Phase 1 Only，Phase 2 跳過）----
        if (!include_tsv
         && cfg_.rotation_start_iter > 0
         && cfg_.rotation_interval   > 0
         && nag_iter_ >= cfg_.rotation_start_iter
         && (nag_iter_ - cfg_.rotation_start_iter) % cfg_.rotation_interval == 0)
        {
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
                return total + eps;
            };

            int rotated_count = 0;
            for (Module& m : modules_) {
                if (m.is_terminal || m.is_fixed) continue;
                const Die& die = dies_[static_cast<size_t>(m.tier_id)];
                const double hw_after = m.height * 0.5;
                const double hh_after = m.width  * 0.5;
                if (m.x < hw_after || m.x > die.width  - hw_after
                 || m.y < hh_after || m.y > die.height - hh_after)
                    continue;
                const double ov_before = pairwise_overlap(m);
                std::swap(m.width, m.height);
                const double ov_after  = pairwise_overlap(m);
                if (ov_after >= ov_before) std::swap(m.width, m.height);
                else ++rotated_count;
            }
            std::cout << "[Rotation] iter=" << nag_iter_
                      << " rotated_modules=" << rotated_count << "\n";
        }

        ++nag_iter_;

        // ---- Step 6: 收斂監控（每 50 次）----
        if (nag_iter_ % 50 == 0) {
            double hpwl = compute_hpwl();

            double max_overflow    = 0.0;
            double total_overflow  = 0.0;
            for (const Die& die : dies_) {
                for (const Bin& b : die.bins) {
                    const double ex = b.density - b.target_density;
                    max_overflow   = std::max(max_overflow, ex);
                    total_overflow += std::max(0.0, ex);
                }
            }

            double rel_change = std::fabs(prev_hpwl_ - hpwl) / (prev_hpwl_ + 1e-12);

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

            std::cout << "[Iter " << std::setw(5) << nag_iter_ << "]"
                      << " HPWL="          << std::fixed      << std::setprecision(1) << hpwl
                      << " Overflow="      << std::setprecision(3) << max_overflow
                      << " TotalOverflow=" << std::setprecision(3) << total_overflow
                      << " λ_mult="        << std::setprecision(4) << lambda_mult_
                      << " σ="             << std::setprecision(1) << smooth_sigma_
                      << " WLrms="         << std::scientific  << std::setprecision(2) << wl_rms
                      << " Drms="          << d_rms
                      << " step="          << step
                      << "\n";

            if (!include_tsv
                && cfg_.dump_analytical_iter_trace
                && !cfg_.analytical_iter_trace_path.empty()) {
                std::ofstream ofs(cfg_.analytical_iter_trace_path,
                                  (nag_iter_ == 50) ? (std::ios::out | std::ios::trunc)
                                                    : (std::ios::out | std::ios::app));
                if (ofs) {
                    ofs << std::fixed << std::setprecision(6);
                    ofs << "[Iter " << nag_iter_ << "]\n";
                    for (const Module& m : modules_) {
                        if (m.is_terminal) continue;
                        ofs << m.name << ' ' << m.lx() << ' ' << m.ly() << ' '
                            << m.rx() << ' ' << m.ry() << '\n';
                    }
                    ofs << '\n';
                } else if (nag_iter_ == 50) {
                    std::cerr << "[Solve] WARNING: cannot open analytical trace file: "
                              << cfg_.analytical_iter_trace_path << "\n";
                }
            }

            const bool overflow_stable =
                (cfg_.convergence_overflow_stable_steps > 0
                 && overflow_stable_streak >= cfg_.convergence_overflow_stable_steps);

            if (!include_tsv
                && nag_iter_ > 1000 && rel_change < tol && overflow_stable) {
                std::cout << "[Converged] iter=" << nag_iter_
                          << " HPWL=" << std::fixed << hpwl << "\n";
                break;
            }
            prev_hpwl_ = hpwl;
        }
    }

    const double hpwl_scaled = compute_hpwl();
    const double gs          = geometry_scale_;
    const char*  phase_tag   = include_tsv ? "[SolveTSV phase]" : "[Final]";
    std::cout << "\n" << phase_tag << " HPWL = "
              << std::fixed << std::setprecision(2) << hpwl_scaled;
    if (std::fabs(gs - 1.0) > 1e-12)
        std::cout << "  (actual = " << (hpwl_scaled / gs) << ", scale:" << gs << ")";
    std::cout << "\n";
    nag_step_ = step;
}

// ============================================================
// solve: Phase 1 module-only analytical
// ============================================================
void PlacementEngine::solve()
{
    analytical_tsv_active_ = false;
    lambda_mult_ = cfg_.lambda_init_mult;
    nag_iter_    = 0;
    nag_step_    = 0.0;
    const int nd = static_cast<int>(dies_.size());
    tier_rc_alpha_mult_.assign(static_cast<size_t>(nd), 0.0);

    {
        const bool use_wa = (cfg_.wirelength_model == WirelengthModel::WA);
        std::cout << "[Solve] wirelength_model="
                  << (use_wa ? "WA" : "LSE")
                  << "  gamma=" << (use_wa ? cfg_.gamma_wa : cfg_.gamma_lse)
                  << "\n";
    }

    // ---- Congestion 小 module 比例門檻：比例 <= gate_min_ratio 時停用 routing_congestion_max ----
    if (cfg_.routing_congestion_gate_divisor > 0.0 && cfg_.routing_congestion_max > 0.0) {
        const int nm = static_cast<int>(modules_.size());
        std::vector<double> tier_max_area(static_cast<size_t>(nd), 0.0);
        for (const Module& m : modules_) {
            if (m.is_terminal || m.tier_id < 0 || m.tier_id >= nd) continue;
            const double a = m.area();
            if (a > tier_max_area[static_cast<size_t>(m.tier_id)])
                tier_max_area[static_cast<size_t>(m.tier_id)] = a;
        }
        int small_count = 0, total_count = 0;
        for (const Module& m : modules_) {
            if (m.is_terminal || m.is_fixed || m.tier_id < 0 || m.tier_id >= nd) continue;
            ++total_count;
            const double thresh = tier_max_area[static_cast<size_t>(m.tier_id)]
                                  / cfg_.routing_congestion_gate_divisor;
            if (m.area() < thresh) ++small_count;
        }
        const double ratio = (total_count > 0)
            ? static_cast<double>(small_count) / static_cast<double>(total_count)
            : 0.0;
        std::cout << "[Solve] congestion gate: small=" << small_count
                  << "/" << total_count
                  << " (" << std::fixed << std::setprecision(1) << ratio * 100.0 << "%)"
                  << " threshold=" << cfg_.routing_congestion_gate_min_ratio * 100.0 << "%\n";
        if (ratio <= cfg_.routing_congestion_gate_min_ratio) {
            std::cout << "[Solve] congestion disabled (small module ratio too low)\n";
            cfg_.routing_congestion_max = 0.0;
        }
        (void)nm;
    }

    run_nag_loop(cfg_.max_iterations, false);
}

// ============================================================
// solve_tsv_phase: Phase 2 joint analytical（module + TSV）
// ============================================================
void PlacementEngine::solve_tsv_phase()
{
    if (tsvs_.empty()) {
        std::cout << "[SolveTSV phase] No TSVs - skipping.\n";
        return;
    }

    std::cout << "[SolveTSV phase] Starting Phase 2 joint analytical ("
              << tsvs_.size() << " TSVs)...\n";
    std::cout << "[SolveTSV phase] Continuing from Phase 1 end:"
              << " iter=" << nag_iter_
              << " step=" << std::fixed << std::setprecision(4) << nag_step_
              << " λ_mult=" << std::setprecision(4) << lambda_mult_
              << " σ=" << std::setprecision(1) << smooth_sigma_
              << "\n";

    analytical_tsv_active_ = true;

    // 初始化 TSV NAG 動量
    const int nt = static_cast<int>(tsvs_.size());
    prev_tsv_x_.resize(static_cast<size_t>(nt));
    prev_tsv_y_.resize(static_cast<size_t>(nt));
    for (int i = 0; i < nt; ++i) {
        prev_tsv_x_[static_cast<size_t>(i)] = tsvs_[static_cast<size_t>(i)].x;
        prev_tsv_y_[static_cast<size_t>(i)] = tsvs_[static_cast<size_t>(i)].y;
    }

    run_nag_loop(cfg_.analytical_tsv_max_iterations, true);

    analytical_tsv_active_ = false;
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
// 共用 LSE/WA 梯度累加核心：對一組 endpoints 計算 WL 梯度
// pts[k] = {x, y}; module_ids[k] >= 0 表示 module pin（累加到 gx/gy），< 0 表示 TSV
// tsv_ids[k] >= 0 時累加到 gx_tsv/gy_tsv（忽略 nullptr）
static void accumulate_wl_gradient(
    const std::vector<std::pair<double,double>>& pts,
    const std::vector<int>&   module_ids,   // -1 = TSV
    const std::vector<int>&   tsv_ids,      // -1 = module
    double w, bool use_wa, double g,
    std::vector<double>& gx, std::vector<double>& gy,
    std::vector<double>* gx_tsv, std::vector<double>* gy_tsv)
{
    if (pts.size() < 2) return;

    double max_x = -1e18, min_x = 1e18;
    double max_y = -1e18, min_y = 1e18;
    for (const auto& p : pts) {
        max_x = std::max(max_x, p.first);
        min_x = std::min(min_x, p.first);
        max_y = std::max(max_y, p.second);
        min_y = std::min(min_y, p.second);
    }

    double A_x = 0.0, C_x = 0.0, A_y = 0.0, C_y = 0.0;
    double B_x = 0.0, D_x = 0.0, B_y = 0.0, D_y = 0.0;
    for (const auto& p : pts) {
        const double xi = p.first, yi = p.second;
        const double ep_x = std::exp((xi - max_x) / g);
        const double en_x = std::exp((min_x - xi) / g);
        const double ep_y = std::exp((yi - max_y) / g);
        const double en_y = std::exp((min_y - yi) / g);
        A_x += ep_x; C_x += en_x; A_y += ep_y; C_y += en_y;
        if (use_wa) {
            B_x += xi * ep_x; D_x += xi * en_x;
            B_y += yi * ep_y; D_y += yi * en_y;
        }
    }

    const size_t np = pts.size();
    if (use_wa) {
        const double x_bar_p = B_x / A_x, x_bar_n = D_x / C_x;
        const double y_bar_p = B_y / A_y, y_bar_n = D_y / C_y;
        const double inv_g = 1.0 / g;
        for (size_t k = 0; k < np; ++k) {
            const double xi = pts[k].first, yi = pts[k].second;
            const double ep_x = std::exp((xi - max_x) / g);
            const double en_x = std::exp((min_x - xi) / g);
            const double ep_y = std::exp((yi - max_y) / g);
            const double en_y = std::exp((min_y - yi) / g);
            const double dgx = w * inv_g * (
                (ep_x / A_x) * (g + xi - x_bar_p) - (en_x / C_x) * (g - xi + x_bar_n));
            const double dgy = w * inv_g * (
                (ep_y / A_y) * (g + yi - y_bar_p) - (en_y / C_y) * (g - yi + y_bar_n));
            if (module_ids[k] >= 0) {
                gx[static_cast<size_t>(module_ids[k])] += dgx;
                gy[static_cast<size_t>(module_ids[k])] += dgy;
            } else if (tsv_ids[k] >= 0 && gx_tsv && gy_tsv) {
                (*gx_tsv)[static_cast<size_t>(tsv_ids[k])] += dgx;
                (*gy_tsv)[static_cast<size_t>(tsv_ids[k])] += dgy;
            }
        }
    } else {
        for (size_t k = 0; k < np; ++k) {
            const double xi = pts[k].first, yi = pts[k].second;
            const double ep_x = std::exp((xi - max_x) / g);
            const double en_x = std::exp((min_x - xi) / g);
            const double ep_y = std::exp((yi - max_y) / g);
            const double en_y = std::exp((min_y - yi) / g);
            const double dgx = w * (ep_x / A_x - en_x / C_x);
            const double dgy = w * (ep_y / A_y - en_y / C_y);
            if (module_ids[k] >= 0) {
                gx[static_cast<size_t>(module_ids[k])] += dgx;
                gy[static_cast<size_t>(module_ids[k])] += dgy;
            } else if (tsv_ids[k] >= 0 && gx_tsv && gy_tsv) {
                (*gx_tsv)[static_cast<size_t>(tsv_ids[k])] += dgx;
                (*gy_tsv)[static_cast<size_t>(tsv_ids[k])] += dgy;
            }
        }
    }
}

void PlacementEngine::calculate_wirelength_gradient(
    std::vector<double>& gx, std::vector<double>& gy,
    std::vector<double>* gx_tsv, std::vector<double>* gy_tsv) const
{
    std::fill(gx.begin(), gx.end(), 0.0);
    std::fill(gy.begin(), gy.end(), 0.0);
    if (gx_tsv) std::fill(gx_tsv->begin(), gx_tsv->end(), 0.0);
    if (gy_tsv) std::fill(gy_tsv->begin(), gy_tsv->end(), 0.0);

    const bool use_wa = (cfg_.wirelength_model == WirelengthModel::WA);
    const double g = use_wa ? cfg_.gamma_wa : cfg_.gamma_lse;
    const int num_tiers = static_cast<int>(dies_.size());

    // ---- Phase 2 per-tier 模式 ----
    if (analytical_tsv_active_ && !tsvs_.empty() && gx_tsv && gy_tsv) {
        for (int ni = 0; ni < static_cast<int>(nets_.size()); ++ni) {
            const Net& net = nets_[static_cast<size_t>(ni)];
            if (net.pins.size() < 2) continue;

            // 利用 net_to_tsvs_ 索引：只有有 TSV 的 cross-tier net 才需要 per-tier 展開
            const auto& tsv_ids_for_net = net_to_tsvs_.empty()
                ? std::vector<int>{}
                : net_to_tsvs_[static_cast<size_t>(ni)];

            // 確定此 net 涉及的 tier 範圍（只遍歷有意義的 tier）
            const int t_lo = net.is_cross_tier ? net.min_tier : 0;
            const int t_hi = net.is_cross_tier ? net.max_tier : (num_tiers - 1);

            for (int t = t_lo; t <= t_hi; ++t) {
                const double w_tier = analytical_tier_net_weight(t);

                std::vector<std::pair<double,double>> pts;
                std::vector<int> mod_ids, tsv_ids_vec;

                // module/terminal pins on tier t
                for (int pid : net.pins) {
                    const Module& m = modules_[pid];
                    const int mt = m.is_terminal ? 0 : m.tier_id;
                    if (mt != t) continue;
                    if (m.is_terminal) {
                        pts.push_back({m.x, m.y});
                        mod_ids.push_back(-1);
                        tsv_ids_vec.push_back(-1);
                    } else {
                        pts.push_back({m.x, m.y});
                        mod_ids.push_back(pid);
                        tsv_ids_vec.push_back(-1);
                    }
                }

                // TSV 虛擬端點：只查屬於此 net 的 TSV（已由索引給出）
                for (int ti : tsv_ids_for_net) {
                    const TSV& tsv = tsvs_[static_cast<size_t>(ti)];
                    if (tsv.tier_below() == t || tsv.tier_above() == t) {
                        pts.push_back({tsv.x, tsv.y});
                        mod_ids.push_back(-1);
                        tsv_ids_vec.push_back(ti);
                    }
                }

                accumulate_wl_gradient(pts, mod_ids, tsv_ids_vec, w_tier,
                                       use_wa, g, gx, gy, gx_tsv, gy_tsv);
            }
        }
        return;
    }

    // ---- Phase 1：全域 all-pin（維持原有行為）----
    for (const Net& net : nets_) {
        if (net.pins.size() < 2) continue;

        const double w_net = net_wirelength_die_weight(net);

        double max_x = -1e18, min_x = 1e18;
        double max_y = -1e18, min_y = 1e18;
        for (int id : net.pins) {
            max_x = std::max(max_x, modules_[id].x);
            min_x = std::min(min_x, modules_[id].x);
            max_y = std::max(max_y, modules_[id].y);
            min_y = std::min(min_y, modules_[id].y);
        }

        double A_x = 0.0, C_x = 0.0;
        double A_y = 0.0, C_y = 0.0;
        double B_x = 0.0, D_x = 0.0;
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
// inject_tsv_proxies_for_legalize:
//   把每個 TSV 以小矩形（tsv_w × tsv_h）注入 modules_ 作為 proxy module，
//   tier_id = tsv.tier_below()（tier_below = layer_index）。
//   供 run_legalize_heu 與真實 module 一起做 overlap 消除。
// ============================================================
void PlacementEngine::inject_tsv_proxies_for_legalize(double tsv_w, double tsv_h)
{
    proxy_module_indices_.clear();
    if (tsvs_.empty()) return;

    // 取現有最大 id（proxy id 從此後遞增）
    int next_id = 0;
    for (const Module& m : modules_)
        next_id = std::max(next_id, m.id + 1);

    std::cout << "[InjectProxy] Injecting " << tsvs_.size()
              << " TSV proxies for unified legalization.\n";

    for (int k = 0; k < static_cast<int>(tsvs_.size()); ++k) {
        const TSV& tsv = tsvs_[static_cast<size_t>(k)];
        const int tier = tsv.tier_below();
        const Die& die = dies_[static_cast<size_t>(tier)];

        Module proxy;
        proxy.id            = next_id++;
        proxy.name          = "__tsv_" + std::to_string(k) + "__";
        proxy.width         = tsv_w;
        proxy.height        = tsv_h;
        proxy.x             = tsv.x;
        proxy.y             = tsv.y;
        proxy.tier_id       = tier;
        proxy.is_terminal   = false;
        proxy.is_fixed      = false;
        proxy.is_soft       = false;
        proxy.move_weight   = 1.0;
        proxy.is_tsv_proxy  = true;
        proxy.tsv_proxy_id  = k;

        // 夾取初始位置在 die 邊界內
        proxy.x = std::max(tsv_w * 0.5, std::min(die.width  - tsv_w * 0.5, proxy.x));
        proxy.y = std::max(tsv_h * 0.5, std::min(die.height - tsv_h * 0.5, proxy.y));

        proxy_module_indices_.push_back(static_cast<int>(modules_.size()));
        modules_.push_back(proxy);
    }
}

// ============================================================
// commit_tsv_proxies_from_legalize:
//   把 proxy 的 (x,y) 寫回 tsvs_，再從 modules_ 移除所有 proxy。
// ============================================================
void PlacementEngine::commit_tsv_proxies_from_legalize()
{
    int synced = 0;
    for (const Module& m : modules_) {
        if (!m.is_tsv_proxy || m.tsv_proxy_id < 0) continue;
        TSV& tsv = tsvs_[static_cast<size_t>(m.tsv_proxy_id)];
        tsv.x = m.x;
        tsv.y = m.y;
        ++synced;
    }

    // 移除所有 proxy（從後往前 erase 保持索引正確性）
    std::vector<Module> kept;
    kept.reserve(modules_.size() - static_cast<size_t>(synced));
    for (const Module& m : modules_)
        if (!m.is_tsv_proxy) kept.push_back(m);
    modules_ = std::move(kept);
    proxy_module_indices_.clear();

    std::cout << "[CommitProxy] Synced " << synced << " TSV proxy positions back to tsvs_.\n";
}

// ============================================================
// update_density_map:
//   對每個可移動方塊，計算其對鄰近 Bin 的密度貢獻
//   密度貢獻 = (module 面積 / bin 面積) * Φx * Φy
void PlacementEngine::update_routing_congestion_map()
{
    if (analytical_tsv_active_ && !tsvs_.empty())
        rc_edge_demands_ = build_bin_edge_demands_with_tsv(*this);
    else
        rc_edge_demands_ = build_bin_edge_baseline_modules_only(*this);
}

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

        Die& die = dies_[static_cast<size_t>(m.tier_id)];

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
                Bin& b       = die.bins[static_cast<size_t>(r * die.bin_cols + c)];
                double phi_x = bell_func(m.x - b.cx, rx);
                double phi_y = bell_func(m.y - b.cy, ry);
                b.density   += A_ratio * phi_x * phi_y;
            }
        }
    }

    // Phase 2：TSV 以小矩形貢獻到 tier_below density
    if (analytical_tsv_active_) {
        const double tsv_w = cfg_.analytical_tsv_width;
        const double tsv_h = cfg_.analytical_tsv_height;
        const double tsv_area = tsv_w * tsv_h;

        for (const TSV& tsv : tsvs_) {
            Die& die = dies_[static_cast<size_t>(tsv.tier_below())];

            double rx = tsv_w * 0.5 + std::max(smooth_sigma_, die.bin_w);
            double ry = tsv_h * 0.5 + std::max(smooth_sigma_, die.bin_h);
            double A_ratio = tsv_area * (9.0 / 16.0) / (rx * ry);

            int c_min = std::max(0, static_cast<int>((tsv.x - rx) / die.bin_w));
            int c_max = std::min(die.bin_cols - 1,
                                 static_cast<int>((tsv.x + rx) / die.bin_w));
            int r_min = std::max(0, static_cast<int>((tsv.y - ry) / die.bin_h));
            int r_max = std::min(die.bin_rows - 1,
                                 static_cast<int>((tsv.y + ry) / die.bin_h));

            for (int r = r_min; r <= r_max; ++r) {
                for (int c = c_min; c <= c_max; ++c) {
                    Bin& b = die.bins[static_cast<size_t>(r * die.bin_cols + c)];
                    b.density += A_ratio
                        * bell_func(tsv.x - b.cx, rx)
                        * bell_func(tsv.y - b.cy, ry);
                }
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
    std::vector<double>& gx, std::vector<double>& gy,
    std::vector<double>* gx_tsv, std::vector<double>* gy_tsv) const
{
    int n = static_cast<int>(modules_.size());
    std::fill(gx.begin(), gx.end(), 0.0);
    std::fill(gy.begin(), gy.end(), 0.0);
    if (gx_tsv) std::fill(gx_tsv->begin(), gx_tsv->end(), 0.0);
    if (gy_tsv) std::fill(gy_tsv->begin(), gy_tsv->end(), 0.0);

    for (int i = 0; i < n; ++i) {
        const Module& m = modules_[i];
        if (m.is_terminal) continue;

        const Die& die = dies_[static_cast<size_t>(m.tier_id)];

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
                const Bin& b    = die.bins[static_cast<size_t>(r * die.bin_cols + c)];
                double overflow = b.density - b.target_density;
                if (overflow <= 0.0) continue;

                double dx_m   = m.x - b.cx;
                double dy_m   = m.y - b.cy;
                double coeff  = 2.0 * overflow * A_ratio;
                gx[static_cast<size_t>(i)] += coeff * bell_grad(dx_m, rx) * bell_func(dy_m, ry);
                gy[static_cast<size_t>(i)] += coeff * bell_func(dx_m, rx) * bell_grad(dy_m, ry);
            }
        }
    }

    // Phase 2：TSV 只從 tier_below density overflow 收到斥力
    if (analytical_tsv_active_ && gx_tsv && gy_tsv) {
        const double tsv_w    = cfg_.analytical_tsv_width;
        const double tsv_h    = cfg_.analytical_tsv_height;
        const double tsv_area = tsv_w * tsv_h;

        for (int i = 0; i < static_cast<int>(tsvs_.size()); ++i) {
            const TSV& tsv  = tsvs_[static_cast<size_t>(i)];
            const Die& die  = dies_[static_cast<size_t>(tsv.tier_below())];

            double rx = tsv_w * 0.5 + std::max(smooth_sigma_, die.bin_w);
            double ry = tsv_h * 0.5 + std::max(smooth_sigma_, die.bin_h);
            double A_ratio = tsv_area * (9.0 / 16.0) / (rx * ry);

            int c_min = std::max(0, static_cast<int>((tsv.x - rx) / die.bin_w));
            int c_max = std::min(die.bin_cols - 1,
                                 static_cast<int>((tsv.x + rx) / die.bin_w));
            int r_min = std::max(0, static_cast<int>((tsv.y - ry) / die.bin_h));
            int r_max = std::min(die.bin_rows - 1,
                                 static_cast<int>((tsv.y + ry) / die.bin_h));

            for (int r = r_min; r <= r_max; ++r) {
                for (int c = c_min; c <= c_max; ++c) {
                    const Bin& b    = die.bins[static_cast<size_t>(r * die.bin_cols + c)];
                    double overflow = b.density - b.target_density;
                    if (overflow <= 0.0) continue;

                    double dx = tsv.x - b.cx;
                    double dy = tsv.y - b.cy;
                    double coeff = 2.0 * overflow * A_ratio;
                    (*gx_tsv)[static_cast<size_t>(i)] += coeff * bell_grad(dx, rx) * bell_func(dy, ry);
                    (*gy_tsv)[static_cast<size_t>(i)] += coeff * bell_func(dx, rx) * bell_grad(dy, ry);
                }
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

    for (int ni = 0; ni < static_cast<int>(nets_.size()); ++ni) {
        const Net& net = nets_[static_cast<size_t>(ni)];
        if (net.pins.size() < 2) continue;

        // 取此 net 的 TSV 索引（索引已在 build_tsvs() 後建立）
        const std::vector<int>* tsv_idx_ptr = nullptr;
        std::vector<int> fallback;
        if (!net_to_tsvs_.empty()) {
            tsv_idx_ptr = &net_to_tsvs_[static_cast<size_t>(ni)];
        } else {
            for (int k = 0; k < static_cast<int>(tsvs_.size()); ++k)
                if (tsvs_[static_cast<size_t>(k)].net_id == net.id)
                    fallback.push_back(k);
            tsv_idx_ptr = &fallback;
        }

        const int t_lo = net.is_cross_tier ? net.min_tier : 0;
        const int t_hi = net.is_cross_tier ? net.max_tier : (num_tiers - 1);

        for (int t = t_lo; t <= t_hi; ++t) {
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

            for (int ti : *tsv_idx_ptr) {
                const TSV& tsv = tsvs_[static_cast<size_t>(ti)];
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
