#!/usr/bin/env python3
"""
Analytical floorplanner using Electrostatic Repulsive Force Model + Netlist Attractive Force.

此版本實作了你要求的「電場斥力模型」(Electrostatic Repulsion):
- 將每個 module 視為帶電粒子（電荷量與其面積成正比）。
- 模組之間具有靜電斥力 (Coulomb-like force)，使模組向外散開。
- Netlist 則產生類似彈簧的拉力 (Quadratic Wirelength / Laplacian)，將有連接的模組拉近。
- 利用 Scipy 的 L-BFGS-B 同時將所有坐標做 O(N^2) 分析梯度的最佳化。
- 運算速度極快，並且最後會平移對齊至 (0,0) 並輸出 .out 格式。
"""

import sys
import time
import math
import numpy as np
from scipy.optimize import minimize

class Block:
    def __init__(self, name, w, h, die_id=0):
        self.name = name
        self.w = float(w)
        self.h = float(h)
        self.area = self.w * self.h
        self.die_id = die_id

class Terminal:
    def __init__(self, name, x, y):
        self.name = name
        self.x = float(x)
        self.y = float(y)

def parse_files(block_file, net_file):
    blocks = {}
    terminals = {}
    die_outlines = {}
    
    with open(block_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line: continue
            parts = line.split()
            
            # 解析 Outline (單 Die) 或是 NumDie 下的多個 Outline
            if parts[0] == "Outline:":
                # 在舊版 2D 裡通常是 Outline: W H
                if len(parts) >= 3:
                    die_outlines[0] = (float(parts[1]), float(parts[2]))
            elif parts[0] == "NumDie:":
                pass # 我們可以單純等待下一行 Outline 出現
                
            if parts[0] in ("NumBlocks:", "NumTerminals:"):
                continue
            
            if len(parts) >= 4 and parts[1] == "terminal":
                terminals[parts[0]] = Terminal(parts[0], parts[2], parts[3])
            elif len(parts) >= 3 and parts[0] not in ("Outline:", "NumDie:"):
                try:
                    w, h = float(parts[1]), float(parts[2])
                    die_id = int(parts[3]) if len(parts) >= 4 else 0
                    blocks[parts[0]] = Block(parts[0], w, h, die_id)
                except ValueError:
                    pass
                
    nets = []
    with open(net_file, 'r') as f:
        current_net = []
        for line in f:
            line = line.strip()
            if not line: continue
            if line.startswith("NumNets:"):
                continue
            if line.startswith("NetDegree:"):
                if current_net:
                    nets.append(current_net)
                    current_net = []
                continue
            current_net.append(line)
        if current_net:
            nets.append(current_net)
            
    return blocks, terminals, nets, die_outlines

def optimize_analytical_placement(blocks, terminals, nets, die_outlines, lam_wl=1.0, lam_rep=1e6, lam_bound=1e8):
    N = len(blocks)
    if N == 0:
        return {}
        
    block_names = list(blocks.keys())
    name2idx = {name: i for i, name in enumerate(block_names)}
    blocks_list = [blocks[name] for name in block_names]
    
    # 解析 Die 邊界限制
    # 如果只有給一個 Die 的 outline，預設所有 Die 都用同一個 outline 大小
    max_w = max_h = 10000.0
    if len(die_outlines) > 0:
        max_w, max_h = die_outlines.get(0, list(die_outlines.values())[0])

    # 建立陣列存放每個 module 的大小與對應的邊界
    mod_w = np.array([b.w for b in blocks_list])
    mod_h = np.array([b.h for b in blocks_list])
    die_bound_w = np.array([die_outlines.get(b.die_id, (max_w, max_h))[0] for b in blocks_list])
    die_bound_h = np.array([die_outlines.get(b.die_id, (max_w, max_h))[1] for b in blocks_list])
    
    # 1. 建立引力的 Laplacian 矩陣 (Attractive Force)
    L = np.zeros((N, N))
    Bx = np.zeros(N)
    By = np.zeros(N)
    
    for net in nets:
        # Clique 模型
        for i in range(len(net)):
            for j in range(i+1, len(net)):
                n1, n2 = net[i], net[j]
                is_b1 = n1 in name2idx
                is_b2 = n2 in name2idx
                
                if is_b1 and is_b2:
                    idx1, idx2 = name2idx[n1], name2idx[n2]
                    L[idx1, idx1] += 1.0
                    L[idx2, idx2] += 1.0
                    L[idx1, idx2] -= 1.0
                    L[idx2, idx1] -= 1.0
                elif is_b1 and not is_b2:
                    if n2 in terminals:
                        idx1 = name2idx[n1]
                        tx, ty = terminals[n2].x, terminals[n2].y
                        L[idx1, idx1] += 1.0
                        Bx[idx1] -= 2.0 * tx
                        By[idx1] -= 2.0 * ty
                elif not is_b1 and is_b2:
                    if n1 in terminals:
                        idx2 = name2idx[n2]
                        tx, ty = terminals[n1].x, terminals[n1].y
                        L[idx2, idx2] += 1.0
                        Bx[idx2] -= 2.0 * tx
                        By[idx2] -= 2.0 * ty

    # 2. 建立靜電場斥力所需的電荷矩陣 (Electrostatic Repulsive Force)
    # 把面積當作電荷 (Charge)
    Q = np.array([b.area for b in blocks_list])
    Q_mean = np.mean(Q) if np.mean(Q) > 0 else 1.0
    # 正規化電荷，避免數值爆炸
    Q_norm = Q / Q_mean
    QQ = Q_norm[:, None] * Q_norm[None, :]
    np.fill_diagonal(QQ, 0.0) # 自己對自己沒有斥力
    
    # 確保只有同一個 Die 內的 module 才會互相排斥
    die_ids = np.array([b.die_id for b in blocks_list])
    same_die = (die_ids[:, None] == die_ids[None, :])
    QQ = QQ * same_die
    
    # eps 防止除以零，並且用平均尺寸平滑化斥力勢能
    avg_W = np.mean([math.sqrt(b.area) for b in blocks_list]) if N > 0 else 100.0
    eps = (avg_W)**2
    
    # 3. 結合引力與斥力的目標函數與梯度 (Objective and Analytical Gradient)
    def objective_and_gradient(z):
        x = z[:N]
        y = z[N:]
        
        # --- Wirelength 彈簧引力 ---
        Lx = L @ x
        Ly = L @ y
        E_wl = np.dot(x, Lx) + np.dot(y, Ly) + np.dot(Bx, x) + np.dot(By, y)
        grad_wl_x = 2.0 * Lx + Bx
        grad_wl_y = 2.0 * Ly + By
        
        # --- Electrostatic 靜電斥力 ---
        dx = x[:, None] - x[None, :]
        dy = y[:, None] - y[None, :]
        dist_sq = dx**2 + dy**2 + eps
        
        inv_dist = 1.0 / np.sqrt(dist_sq)
        
        # 電勢能 Energy = sum (Q1 * Q2 / dist)
        E_rep = 0.5 * np.sum(QQ * inv_dist)
        
        # 斥力梯度 Gradient = - Q1 * Q2 * dx / dist^3
        inv_dist3 = inv_dist ** 3
        Fx = QQ * inv_dist3 * dx
        Fy = QQ * inv_dist3 * dy
        
        # 對 j 軸加總，得到每個 i 受到的總斥力梯度
        grad_rep_x = -np.sum(Fx, axis=1)
        grad_rep_y = -np.sum(Fy, axis=1)
        
        # --- 合併總能量 ---
        
        # --- Bounding Box 邊界斥力 (Boundary Penalty Force) ---
        # 如果 module 跑出 die outline，就給予極大的二次方懲罰將其拉回
        # max(0, -x) 表示小於 0 時的違反量，max(0, x + w - W) 表示大於邊界時的違反量
        viol_x_left = np.maximum(0.0, -x)
        viol_x_right = np.maximum(0.0, x + mod_w - die_bound_w)
        viol_y_bottom = np.maximum(0.0, -y)
        viol_y_top = np.maximum(0.0, y + mod_h - die_bound_h)
        
        E_bound = np.sum(viol_x_left**2 + viol_x_right**2 + viol_y_bottom**2 + viol_y_top**2)
        
        grad_bound_x = 2.0 * (-viol_x_left + viol_x_right)
        grad_bound_y = 2.0 * (-viol_y_bottom + viol_y_top)
        
        E = lam_wl * E_wl + lam_rep * E_rep + lam_bound * E_bound
        grad_x = lam_wl * grad_wl_x + lam_rep * grad_rep_x + lam_bound * grad_bound_x
        grad_y = lam_wl * grad_wl_y + lam_rep * grad_rep_y + lam_bound * grad_bound_y
        
        grad = np.concatenate([grad_x, grad_y])
        return E, grad

    # 初始化位置：在 Outline 內部均勻撒點，加入亂數散射
    np.random.seed(42)
    init_x = np.random.uniform(0, max_w * 0.8, N)
    init_y = np.random.uniform(0, max_h * 0.8, N)
    
    z0 = np.concatenate([init_x, init_y])
    
    # 啟動 Scipy 優化，由於我們手寫了解析梯度 (jac=True)，速度會非常快
    res = minimize(
        objective_and_gradient, 
        z0, 
        method='L-BFGS-B', 
        jac=True, 
        options={'maxiter': 500, 'disp': True}
    )
    
    final_x = res.x[:N]
    final_y = res.x[N:]
    
    pos = {}
    for i, name in enumerate(block_names):
        pos[name] = (final_x[i], final_y[i])
        
    return pos

def compute_hpwl(pos, blocks, terminals, nets):
    hpwl = 0.0
    for net in nets:
        xs, ys = [], []
        for node in net:
            if node in pos:
                b = blocks[node]
                # HPWL 測量對準模組中心
                xs.append(pos[node][0] + b.w/2)
                ys.append(pos[node][1] + b.h/2)
            elif node in terminals:
                xs.append(terminals[node].x)
                ys.append(terminals[node].y)
        if xs:
            hpwl += (max(xs) - min(xs)) + (max(ys) - min(ys))
    return hpwl

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 analytical_floorplan.py [alpha] <block_file> <net_file> [output_file]")
        return
        
    args = sys.argv[1:]
    try:
        alpha = float(args[0])
        args = args[1:]
    except ValueError:
        alpha = 0.5
        
    if len(args) < 2:
        print("Usage: python3 analytical_floorplan.py [alpha] <block_file> <net_file> [output_file]")
        return
        
    block_file = args[0]
    net_file = args[1]
    out_file = args[2] if len(args) > 2 else "output.out"
    
    # 這些權重可以自由調整。引力保持為 1.0
    # 斥力權重 lam_rep 調大會讓 module 散得更開，調小則會偏向縮成一團
    lam_wl = 1.0
    lam_rep = 5e10
    lam_bound = 1e9
    
    t0 = time.time()
    blocks, terminals, nets, die_outlines = parse_files(block_file, net_file)
    
    # 執行電場模型最佳化
    pos = optimize_analytical_placement(blocks, terminals, nets, die_outlines, lam_wl, lam_rep, lam_bound)
    
    # 標準化輸出位置，確保最左下角切齊 (0,0)
    die_groups = {}
    for name, b in blocks.items():
        die_groups.setdefault(b.die_id, []).append(name)
        
    final_coords = {}
    for die_id, group in die_groups.items():
        min_x = min([pos[n][0] for n in group])
        min_y = min([pos[n][1] for n in group])
        for n in group:
            final_coords[n] = (pos[n][0] - min_x, pos[n][1] - min_y)
            
    hpwl = compute_hpwl(final_coords, blocks, terminals, nets)
    
    # 計算包圍面積 Area
    all_x2 = [final_coords[n][0] + blocks[n].w for n in blocks]
    all_y2 = [final_coords[n][1] + blocks[n].h for n in blocks]
    
    max_w = max(all_x2) if all_x2 else 0
    max_h = max(all_y2) if all_y2 else 0
    area = max_w * max_h
    
    cost = (1.0 - alpha) * hpwl + alpha * area
    runtime = time.time() - t0
    
    with open(out_file, 'w') as f:
        f.write(f"{cost:.0f}\n")
        f.write(f"{hpwl:.0f}\n")
        f.write(f"{area:.0f}\n")
        f.write(f"{max_w:.0f} {max_h:.0f}\n")
        f.write(f"{runtime:.3f}\n")
        for name in blocks:
            b = blocks[name]
            x1, y1 = final_coords[name]
            x2, y2 = x1 + b.w, y1 + b.h
            f.write(f"{name} {b.die_id} {x1:.0f} {y1:.0f} {x2:.0f} {y2:.0f}\n")
            
    print(f"[Analytical] Electrostatic model optimization done. Wrote to {out_file} in {runtime:.3f} s.")

if __name__ == "__main__":
    main()
