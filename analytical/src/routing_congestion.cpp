// 3D IC Analytical Floorplanner - Routability-Driven Congestion Penalty (可微)
//
// 懲罰能量：P_route = Σ_e (U(e) - C)²
//
// 平滑指示函數（數值穩定型）：
//   f(u, x, v) = σ(t),  t = ε*(u-x)*(v-x)
//   σ(t) = 1/(1+exp(t))
//   若 t >= 0 改寫為 exp(-t)/(1+exp(-t)) 避免 exp overflow
//
// Usage_H / Usage_V 對 module 座標的解析梯度以 chain rule 推導並逐 edge 累加。

#include "routing_congestion.h"

#include <algorithm>
#include <cmath>

// ============================================================
// 數值穩定 sigmoid：σ(t) = 1/(1+exp(t))
// ============================================================
static inline double sigmoid_stable(double t)
{
    if (t >= 0.0)
        return std::exp(-t) / (1.0 + std::exp(-t));
    return 1.0 / (1.0 + std::exp(t));
}

// σ'(t) = -σ(t)*(1-σ(t))
static inline double sigmoid_deriv(double sig_val)
{
    return -sig_val * (1.0 - sig_val);
}

// ============================================================
// f(u, x_eval, v) = σ(ε*(u-x_eval)*(v-x_eval))
// 回傳 f 值與 ∂f/∂u、∂f/∂v（x_eval 是常數邊座標）
// ============================================================
static inline void eval_f(double u, double x_eval, double v, double eps,
                           double& f, double& df_du, double& df_dv)
{
    const double t   = eps * (u - x_eval) * (v - x_eval);
    const double sg  = sigmoid_stable(t);
    const double sgd = sigmoid_deriv(sg);
    f     = sg;
    // ∂t/∂u = ε*(v - x_eval)
    df_du = sgd * eps * (v - x_eval);
    // ∂t/∂v = ε*(u - x_eval)
    df_dv = sgd * eps * (u - x_eval);
}

// ============================================================
// cover(u, xe1, xe2, v, eps)：
//   cover = 1 - (1-f1)*(1-f2)
//   f1 = f(u, xe1, v),  f2 = f(u, xe2, v)
// 回傳 cover 與 ∂cover/∂u、∂cover/∂v
// ============================================================
static inline void eval_cover(double u, double xe1, double xe2, double v, double eps,
                               double& cov, double& dcov_du, double& dcov_dv)
{
    double f1, df1_du, df1_dv;
    double f2, df2_du, df2_dv;
    eval_f(u, xe1, v, eps, f1, df1_du, df1_dv);
    eval_f(u, xe2, v, eps, f2, df2_du, df2_dv);

    const double A = 1.0 - f1;
    const double B = 1.0 - f2;
    cov      = 1.0 - A * B;
    // ∂cov/∂u = df1_du*B + A*df2_du
    dcov_du  = df1_du * B + A * df2_du;
    // ∂cov/∂v = df1_dv*B + A*df2_dv
    dcov_dv  = df1_dv * B + A * df2_dv;
}

// ============================================================
// 對一對 2-pin (p1, p2) 在同一 tier 的 die 上累加壅塞梯度
// 回傳純梯度 ∂P_route/∂x，不乘 alpha（alpha 在外部 merge 步驟施加）
// ============================================================
static void accumulate_pair(
    double x1, double y1, double x2, double y2,
    int ia, int ib, bool ma_move, bool mb_move,
    const Die& die,
    double eps, double cap,
    std::vector<double>& gx, std::vector<double>& gy)
{
    const double bw  = die.bin_w;
    const double bh  = die.bin_h;
    const int    R   = die.bin_rows;
    const int    C   = die.bin_cols;

    // col_span / row_span：可微連續近似（避免 floor 斷點）
    constexpr double EPS_FLOOR = 1e-6;
    const double col_span = std::max(EPS_FLOOR, std::fabs(x1 - x2) / bw + 1.0);
    const double row_span = std::max(EPS_FLOOR, std::fabs(y1 - y2) / bh + 1.0);

    // ---- Horizontal edges：e_H(hr, hc) ----
    // hr ∈ [0,R]，y_e = hr*bh；hc ∈ [0,C-1]，x ∈ [hc*bw,(hc+1)*bw]
    {
        const int hr_lo = std::max(0, static_cast<int>(std::floor(std::min(y1,y2)/bh)));
        const int hr_hi = std::min(R, static_cast<int>(std::ceil (std::max(y1,y2)/bh)));
        const int hc_lo = std::max(0,   static_cast<int>(std::floor(std::min(x1,x2)/bw)) - 1);
        const int hc_hi = std::min(C-1, static_cast<int>(std::ceil (std::max(x1,x2)/bw)));

        for (int hr = hr_lo; hr <= hr_hi; ++hr) {
            const double ye = hr * bh;
            // Vy = σ(ε*(y1-ye)*(y2-ye))
            double Vy, dVy_dy1, dVy_dy2;
            eval_f(y1, ye, y2, eps, Vy, dVy_dy1, dVy_dy2);

            for (int hc = hc_lo; hc <= hc_hi; ++hc) {
                const double xe1 = hc * bw;
                const double xe2 = (hc + 1) * bw;

                double cov, dcov_dx1, dcov_dx2;
                eval_cover(x1, xe1, xe2, x2, eps, cov, dcov_dx1, dcov_dx2);

                const double U    = Vy * cov / col_span;
                const double dP_dU = 2.0 * (U - cap);

                // ∂col_span/∂x1 = sign(x1-x2)/bw；∂col_span/∂x2 = -sign(x1-x2)/bw
                const double sgn_dx  = (x1 >= x2) ? 1.0 : -1.0;
                const double d_cs_x1 =  sgn_dx / bw;
                const double d_cs_x2 = -sgn_dx / bw;

                const double inv_cs  = 1.0 / col_span;
                const double inv_cs2 = inv_cs * inv_cs;

                // ∂U/∂x1 = Vy*(dcov_dx1*col_span - cov*d_cs_x1) / col_span²
                const double dU_dx1 = Vy * (dcov_dx1 * col_span - cov * d_cs_x1) * inv_cs2;
                const double dU_dx2 = Vy * (dcov_dx2 * col_span - cov * d_cs_x2) * inv_cs2;
                // ∂U/∂y1 = dVy_dy1*cov/col_span
                const double dU_dy1 = dVy_dy1 * cov * inv_cs;
                const double dU_dy2 = dVy_dy2 * cov * inv_cs;

                if (ma_move) {
                    gx[ia] += dP_dU * dU_dx1;
                    gy[ia] += dP_dU * dU_dy1;
                }
                if (mb_move) {
                    gx[ib] += dP_dU * dU_dx2;
                    gy[ib] += dP_dU * dU_dy2;
                }
            }
        }
    }

    // ---- Vertical edges：e_V(vc, vr) ----
    // vc ∈ [0,C]，x_e = vc*bw；vr ∈ [0,R-1]，y ∈ [vr*bh,(vr+1)*bh]
    {
        const int vc_lo = std::max(0, static_cast<int>(std::floor(std::min(x1,x2)/bw)));
        const int vc_hi = std::min(C, static_cast<int>(std::ceil (std::max(x1,x2)/bw)));
        const int vr_lo = std::max(0,   static_cast<int>(std::floor(std::min(y1,y2)/bh)) - 1);
        const int vr_hi = std::min(R-1, static_cast<int>(std::ceil (std::max(y1,y2)/bh)));

        for (int vc = vc_lo; vc <= vc_hi; ++vc) {
            const double xe = vc * bw;
            // Vx = σ(ε*(x1-xe)*(x2-xe))
            double Vx, dVx_dx1, dVx_dx2;
            eval_f(x1, xe, x2, eps, Vx, dVx_dx1, dVx_dx2);

            for (int vr = vr_lo; vr <= vr_hi; ++vr) {
                const double ye1 = vr * bh;
                const double ye2 = (vr + 1) * bh;

                double cov, dcov_dy1, dcov_dy2;
                eval_cover(y1, ye1, ye2, y2, eps, cov, dcov_dy1, dcov_dy2);

                const double U     = Vx * cov / row_span;
                const double dP_dU = 2.0 * (U - cap);

                const double sgn_dy  = (y1 >= y2) ? 1.0 : -1.0;
                const double d_rs_y1 =  sgn_dy / bh;
                const double d_rs_y2 = -sgn_dy / bh;

                const double inv_rs  = 1.0 / row_span;
                const double inv_rs2 = inv_rs * inv_rs;

                const double dU_dy1 = Vx * (dcov_dy1 * row_span - cov * d_rs_y1) * inv_rs2;
                const double dU_dy2 = Vx * (dcov_dy2 * row_span - cov * d_rs_y2) * inv_rs2;
                const double dU_dx1 = dVx_dx1 * cov * inv_rs;
                const double dU_dx2 = dVx_dx2 * cov * inv_rs;

                if (ma_move) {
                    gx[ia] += dP_dU * dU_dx1;
                    gy[ia] += dP_dU * dU_dy1;
                }
                if (mb_move) {
                    gx[ib] += dP_dU * dU_dx2;
                    gy[ib] += dP_dU * dU_dy2;
                }
            }
        }
    }
}

// ============================================================
// 公開介面
// ============================================================
void calculate_routing_congestion_gradient(
    const PlacementEngine& engine,
    std::vector<double>&   gx,
    std::vector<double>&   gy)
{
    const PlacementConfig& cfg = engine.config();
    if (cfg.routing_congestion_alpha == 0.0) return;

    // alpha 不在此處施加：gx_rc 儲存純梯度 ∂P_route/∂x，
    // alpha 與 RMS 縮放統一在 floorplanner.cpp 的 merge 步驟處理
    const double eps = cfg.routing_sigmoid_eps;
    const double cap = cfg.routing_capacity_C;

    const auto& modules   = engine.modules();
    const auto& nets      = engine.nets();
    const auto& dies      = engine.dies();
    const int   num_tiers = static_cast<int>(dies.size());

    for (const Net& net : nets) {
        if (net.pins.size() < 2) continue;

        for (int tk = 0; tk < num_tiers; ++tk) {
            // 蒐集屬於 tier tk 的 pin（terminal 歸 tier 0）
            std::vector<int> tier_pins;
            tier_pins.reserve(net.pins.size());
            for (int pid : net.pins) {
                const Module& m = modules[pid];
                const int m_tier = m.is_terminal ? 0 : m.tier_id;
                if (m_tier == tk) tier_pins.push_back(pid);
            }
            if (static_cast<int>(tier_pins.size()) < 2) continue;

            const Die& die = dies[tk];
            const int  np  = static_cast<int>(tier_pins.size());

            // 所有無序對 (a, b)
            for (int ai = 0; ai < np; ++ai) {
                for (int bi = ai + 1; bi < np; ++bi) {
                    const int ia = tier_pins[ai];
                    const int ib = tier_pins[bi];
                    const Module& ma = modules[ia];
                    const Module& mb = modules[ib];

                    const bool ma_move = !ma.is_terminal && !ma.is_fixed;
                    const bool mb_move = !mb.is_terminal && !mb.is_fixed;
                    if (!ma_move && !mb_move) continue;

                    accumulate_pair(
                        ma.x, ma.y, mb.x, mb.y,
                        ia, ib, ma_move, mb_move,
                        die, eps, cap,
                        gx, gy);
                }
            }
        }
    }
}
