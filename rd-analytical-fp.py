#!/usr/bin/env python3
"""
Analytical floorplanner: Stable-LSE wirelength + Electrostatic repulsion spreading.

解析梯度 (jac=True) 傳給 L-BFGS-B，速度快且正確收斂。
座標向量格式：z = [x0, x1, ..., xN-1,  y0, y1, ..., yN-1]
"""

import argparse
import time
import numpy as np
from scipy.optimize import minimize


class AnalyticalGD:
    def __init__(self, modules, nets, die_width, die_height,
                 lam_wl=1.0, lam_rep=5.0, maxiter=800):
        self.modules = modules        # {name: {'w': w, 'h': h}}
        self.mod_names = list(modules.keys())
        self.mod_to_idx = {name: i for i, name in enumerate(self.mod_names)}
        self.nets = nets              # [[node, ...], ...]
        self.W = die_width
        self.H = die_height
        self.maxiter = maxiter
        # 使用者指定的相對權重（正規化後才會實際使用）
        self._lam_wl_user  = lam_wl
        self._lam_rep_user = lam_rep

        N = len(self.mod_names)
        blocks_list = [modules[n] for n in self.mod_names]

        # 模組尺寸陣列
        self.mod_w = np.array([b['w'] for b in blocks_list])
        self.mod_h = np.array([b['h'] for b in blocks_list])

        # LSE 平滑因子：die 較長邊的 5%
        self.gamma = max(die_width, die_height) * 0.05

        # 靜電斥力電荷（正規化面積）
        areas = self.mod_w * self.mod_h
        Q = areas / np.mean(areas)
        self.QQ = Q[:, None] * Q[None, :]
        np.fill_diagonal(self.QQ, 0.0)

        # 平均尺寸作為 epsilon 防止除以零
        avg_size = np.mean(np.sqrt(areas))
        self.eps = avg_size ** 2

        # 內部有效權重（由 solve() 做正規化後設定）
        self.lam_wl  = lam_wl
        self.lam_rep = lam_rep

        # 預先建立 net index list（跳過不在模組表的 node）
        self._net_indices = []
        for net in self.nets:
            idx = [self.mod_to_idx[m] for m in net if m in self.mod_to_idx]
            if len(idx) >= 2:
                self._net_indices.append(np.array(idx, dtype=int))

    def objective_and_gradient(self, z):
        """
        同時回傳目標函數值與解析梯度，供 scipy jac=True 使用。
        z = [x0..xN-1, y0..yN-1]
        """
        N = len(self.mod_names)
        x = z[:N]
        y = z[N:]
        g = self.gamma

        grad_x = np.zeros(N)
        grad_y = np.zeros(N)
        E_wl = 0.0

        # ── 1. Stable-LSE wirelength + 解析梯度 ──────────────────────────────
        for indices in self._net_indices:
            nx = x[indices]
            ny = y[indices]

            # X 方向
            px = nx / g
            mx_p, mx_n = np.max(px), np.max(-px)
            ep = np.exp(px - mx_p);  sp = ep.sum()
            en = np.exp(-px - mx_n); sn = en.sum()
            lse_x = g * (mx_p + np.log(sp) + mx_n + np.log(sn))
            # d(lse_x)/d(x_k) = ep[k]/sp - en[k]/sn
            glx = ep / sp - en / sn

            # Y 方向
            py = ny / g
            my_p, my_n = np.max(py), np.max(-py)
            epy = np.exp(py - my_p);  spy = epy.sum()
            eny = np.exp(-py - my_n); sny = eny.sum()
            lse_y = g * (my_p + np.log(spy) + my_n + np.log(sny))
            gly = epy / spy - eny / sny

            E_wl += lse_x + lse_y
            # 累加梯度到全域陣列
            np.add.at(grad_x, indices, glx)
            np.add.at(grad_y, indices, gly)

        # ── 2. 靜電斥力 + 解析梯度 ──────────────────────────────────────────
        dx = x[:, None] - x[None, :]  # (N, N)
        dy = y[:, None] - y[None, :]
        dist_sq = dx ** 2 + dy ** 2 + self.eps
        inv_dist = 1.0 / np.sqrt(dist_sq)

        E_rep = 0.5 * np.sum(self.QQ * inv_dist)

        inv_dist3 = inv_dist ** 3
        Fx = self.QQ * inv_dist3 * dx
        Fy = self.QQ * inv_dist3 * dy
        # 斥力梯度（負號：斥力方向與距離向量相反）
        grad_rep_x = -np.sum(Fx, axis=1)
        grad_rep_y = -np.sum(Fy, axis=1)

        # ── 合併（邊界由 L-BFGS-B bounds 處理，不需重複懲罰）────────────────
        E  = self.lam_wl * E_wl  + self.lam_rep * E_rep
        gx = self.lam_wl * grad_x + self.lam_rep * grad_rep_x
        gy = self.lam_wl * grad_y + self.lam_rep * grad_rep_y

        return E, np.concatenate([gx, gy])

    def solve(self, seed=42):
        N = len(self.mod_names)

        # 隨機初始化：在 die 範圍內均勻散開，避免梯度對稱抵消
        # 固定 seed 保證重現性，可透過 --seed 改變初始分佈
        np.random.seed(seed)
        init_x = np.random.uniform(0, self.W * 0.8, N)
        init_y = np.random.uniform(0, self.H * 0.8, N)
        init_x = np.clip(init_x, 0, np.maximum(0, self.W - self.mod_w))
        init_y = np.clip(init_y, 0, np.maximum(0, self.H - self.mod_h))
        z0 = np.concatenate([init_x, init_y])

        # ── 自動正規化各項 energy ────────────────────────────────────────────
        # 在 z0 量測各項 raw energy，讓使用者的 lam 真正代表「相對重要性」
        # 例如 lam_wl=1, lam_rep=1 → 兩項貢獻相等；lam_wl=3 → 線長貢獻佔 3/4
        self.lam_wl, self.lam_rep = 1.0, 0.0
        E_wl_init, _ = self.objective_and_gradient(z0)

        self.lam_wl, self.lam_rep = 0.0, 1.0
        E_rep_init, _ = self.objective_and_gradient(z0)

        scale_wl  = max(E_wl_init,  1e-10)
        scale_rep = max(E_rep_init, 1e-10)

        # 有效權重 = 使用者設定值 / 初始 energy 規模
        self.lam_wl  = self._lam_wl_user  / scale_wl
        self.lam_rep = self._lam_rep_user / scale_rep

        print(f"\nGD Stage  seed={seed}, γ={self.gamma:.1f}, maxiter={self.maxiter}")
        print(f"  初始 energy: E_wl={E_wl_init:.3e},  E_rep={E_rep_init:.3e}")
        print(f"  使用者比重:  lam_wl={self._lam_wl_user:.3g}  lam_rep={self._lam_rep_user:.3g}")
        print(f"  實際貢獻比: wl:{self._lam_wl_user:.3g}份  rep:{self._lam_rep_user:.3g}份  "
              f"(若相等則各佔 50%)")

        bounds = ([(0.0, max(0.0, self.W - self.mod_w[i])) for i in range(N)] +
                  [(0.0, max(0.0, self.H - self.mod_h[i])) for i in range(N)])

        res = minimize(
            self.objective_and_gradient,
            z0,
            method='L-BFGS-B',
            jac=True,
            bounds=bounds,
            options={'maxiter': self.maxiter, 'disp': False}
        )
        if not res.success:
            print(f"[Warning] 未完全收斂: {res.message}")

        # 診斷：最終各項 energy 實際貢獻
        self.lam_wl, self.lam_rep = 1.0, 0.0
        E_wl_f, _ = self.objective_and_gradient(res.x)
        self.lam_wl, self.lam_rep = 0.0, 1.0
        E_rep_f, _ = self.objective_and_gradient(res.x)
        # 還原有效權重（供呼叫端繼續使用）
        self.lam_wl  = self._lam_wl_user  / scale_wl
        self.lam_rep = self._lam_rep_user / scale_rep

        wl_contrib  = self._lam_wl_user  / scale_wl  * E_wl_f
        rep_contrib = self._lam_rep_user / scale_rep * E_rep_f
        total = wl_contrib + rep_contrib + 1e-30
        print(f"  最終 energy 貢獻: wl={wl_contrib:.3e} ({wl_contrib/total*100:.1f}%),  "
              f"rep={rep_contrib:.3e} ({rep_contrib/total*100:.1f}%)")

        final_x = res.x[:N]
        final_y = res.x[N:]
        return {name: (final_x[i], final_y[i]) for i, name in enumerate(self.mod_names)}


# ── 輸入解析 ──────────────────────────────────────────────────────────────────

def parse_block_file(block_file):
    """解析 .block 檔案，回傳 modules, terminals, die_w, die_h"""
    modules = {}
    terminals = {}
    die_w, die_h = 10000.0, 10000.0

    with open(block_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            if parts[0] == 'Outline:':
                die_w, die_h = float(parts[1]), float(parts[2])
            elif parts[0] in ('NumBlocks:', 'NumTerminals:'):
                continue
            elif len(parts) >= 4 and parts[1] == 'terminal':
                terminals[parts[0]] = {'x': float(parts[2]), 'y': float(parts[3])}
            elif len(parts) >= 3:
                try:
                    modules[parts[0]] = {'w': float(parts[1]), 'h': float(parts[2])}
                except ValueError:
                    pass

    return modules, terminals, die_w, die_h


def parse_net_file(net_file):
    """解析 .nets 檔案，回傳 list of nets"""
    nets = []
    current_net = []

    with open(net_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('NumNets:'):
                continue
            if line.startswith('NetDegree:'):
                if current_net:
                    nets.append(current_net)
                current_net = []
            else:
                current_net.append(line)

    if current_net:
        nets.append(current_net)

    return nets


# ── 計算 HPWL ────────────────────────────────────────────────────────────────

def compute_hpwl(pos, modules, terminals, nets):
    """HPWL：模組以中心點計算，terminal 以固定座標計算"""
    hpwl = 0.0
    for net in nets:
        xs, ys = [], []
        for node in net:
            if node in pos:
                m = modules[node]
                xs.append(pos[node][0] + m['w'] / 2)
                ys.append(pos[node][1] + m['h'] / 2)
            elif node in terminals:
                xs.append(terminals[node]['x'])
                ys.append(terminals[node]['y'])
        if len(xs) >= 2:
            hpwl += (max(xs) - min(xs)) + (max(ys) - min(ys))
    return hpwl


# ── 主程式 ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Analytical Floorplanner (Stable-LSE wirelength + Electrostatic repulsion)'
    )
    parser.add_argument('--block', '-b', required=True, help='Input .block file')
    parser.add_argument('--net',   '-n', required=True, help='Input .nets file')
    parser.add_argument('--output', '-o', default='output.out', help='Output .out file')
    parser.add_argument('--alpha',   type=float, default=0.5,
                        help='Cost weight: cost = alpha*area + (1-alpha)*HPWL (default: 0.5)')
    parser.add_argument('--lam-wl',  type=float, default=1.0,
                        help='線長引力相對權重，值越大線長越短 (default: 1.0)')
    parser.add_argument('--lam-rep', type=float, default=1.0,
                        help='靜電斥力相對權重，值越大模組分佈越均勻 (default: 1.0)')
    parser.add_argument('--maxiter', type=int,   default=500,
                        help='Max L-BFGS-B iterations (default: 500)')
    parser.add_argument('--seed',      type=int,   default=42,
                        help='Random seed for initial placement (default: 42)')
    args = parser.parse_args()

    t0 = time.time()

    modules, terminals, die_w, die_h = parse_block_file(args.block)
    nets = parse_net_file(args.net)

    print(f"載入 {len(modules)} 個模組、{len(terminals)} 個 terminal、{len(nets)} 條 net")
    print(f"Die outline: {die_w} x {die_h}")

    gd = AnalyticalGD(
        modules, nets, die_w, die_h,
        lam_wl=args.lam_wl,
        lam_rep=args.lam_rep,
        maxiter=args.maxiter
    )
    pos = gd.solve(seed=args.seed)

    # 正規化：整體平移使最小 (x, y) 對齊至 (0, 0)
    min_x = min(p[0] for p in pos.values())
    min_y = min(p[1] for p in pos.values())
    pos = {name: (p[0] - min_x, p[1] - min_y) for name, p in pos.items()}

    # 計算指標
    hpwl = compute_hpwl(pos, modules, terminals, nets)
    all_x2 = [pos[n][0] + modules[n]['w'] for n in modules]
    all_y2 = [pos[n][1] + modules[n]['h'] for n in modules]
    chip_w = max(all_x2)
    chip_h = max(all_y2)
    area = chip_w * chip_h
    cost = (1.0 - args.alpha) * hpwl + args.alpha * area
    runtime = time.time() - t0

    # 輸出 .out 檔（格式對齊參考輸出）
    with open(args.output, 'w') as f:
        f.write(f"{cost:.6f}\n")
        f.write(f"{hpwl:.6f}\n")
        f.write(f"{area:.6f}\n")
        f.write(f"{chip_w:.0f} {chip_h:.0f}\n")
        f.write(f"{runtime:.6f}\n")
        for name in gd.mod_names:
            m = modules[name]
            x1, y1 = pos[name]
            x2, y2 = x1 + m['w'], y1 + m['h']
            f.write(f"{name} {x1:.0f} {y1:.0f} {x2:.0f} {y2:.0f}\n")

    print(f"\n完成！HPWL={hpwl:.0f}, Area={area:.0f}, Cost={cost:.0f}")
    print(f"Chip size: {chip_w:.0f} x {chip_h:.0f}")
    print(f"Runtime: {runtime:.3f} s")
    print(f"Output 已寫入: {args.output}")


if __name__ == "__main__":
    main()
