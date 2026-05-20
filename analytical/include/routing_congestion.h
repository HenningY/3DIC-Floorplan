// 3D IC Analytical Floorplanner - Routability-Driven Congestion Penalty
//
// 懲罰能量（對 module 位置完全可微）：
//   P_route = Σ_e (U(e) - C)²
//
// 平滑指示函數：f(u, x, v) = σ(ε*(u-x)*(v-x)) = 1/(1+exp(ε*(u-x)*(v-x)))
//   x 在 [min(u,v), max(u,v)] 內時乘積 < 0 → σ > 0.5（近似「在區間內」）
//
// Usage_H(e) = Vy(y1,y_e,y2) * cover_x(x1,x_e1,x_e2,x2) / col_span
// Usage_V(e) = Vx(x1,x_e,x2) * cover_y(y1,y_e1,y_e2,y2) / row_span
//
// routing_congestion_alpha = 0 時立即 return，不影響既有行為。
#pragma once

#include "floorplanner.h"
#include <vector>

// 對 gx[i]/gy[i]（與 modules_[i] 對應，長度 = modules_.size()）累加壅塞梯度。
// alpha=0 時函式立即返回。
void calculate_routing_congestion_gradient(
    const PlacementEngine& engine,
    std::vector<double>&   gx,
    std::vector<double>&   gy);
