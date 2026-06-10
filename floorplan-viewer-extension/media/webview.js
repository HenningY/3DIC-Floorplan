// @ts-check
/* global acquireVsCodeApi */
const vscode = acquireVsCodeApi();
const logEl = document.getElementById('log');

function log(msg) {
  logEl.textContent += msg + '\n';
  logEl.scrollTop = logEl.scrollHeight;
}

// 整數顯示為整數；小數最多三位（四捨五入，去掉尾端多餘 0）
function formatDecimalMax3(n) {
  return String(parseFloat(n.toFixed(3)));
}

// ── 狀態 ──
var sceneData = null;
var currentMode = 'overview'; // 'overview' | 數字 tier index
var moduleOverrides = {};     // { [name]: { xll, yll, xur, yur } } 暫存位置
var selectedMod = null;       // 目前選取的 module name
var currentTransform = null;  // 最新一次 makeTransform 的結果（用於座標轉換）
var dragState = null;         // 拖曳狀態
var panState = null;          // 平移 floorplan 狀態
var viewZoom = 1;             // 1 = 100% fit
var viewPanX = 0;
var viewPanY = 0;
var constraintData = [];      // 已解析的 constraint 陣列
var constraintFileLoaded = false; // 是否已載入 constraint 檔

var TIER_FILLS   = ['#4a90d9','#5cb85c','#f0ad4e','#d9534f','#9b59b6','#1abc9c'];
var TIER_STROKES = ['#2c6fa8','#3d8b3d','#c8892a','#b33030','#7d3f9c','#148a74'];

// ── Result / Info 區塊更新 ──
function updateInfoSection() {
  var infoEl = document.getElementById('rpInfoRows');
  if (!infoEl) { return; }
  if (!sceneData) { infoEl.innerHTML = ''; return; }
  var html = '';
  if (currentMode === 'overview') {
    for (var t = 0; t < sceneData.outlines.length; t++) {
      var cnt = sceneData.modules.filter(function(m) { return m.tier === t; }).length;
      html += '<div class="rp-stat-row">' +
        '<span class="rp-stat-label" style="color:' + TIER_FILLS[t % TIER_FILLS.length] + '">Tier ' + t + '</span>' +
        '<span class="rp-stat-value">' + cnt + ' modules</span>' +
        '</div>';
    }
  } else {
    var tier = currentMode;
    var modCnt = sceneData.modules.filter(function(m) { return m.tier === tier; }).length;
    var tsvCnt = sceneData.tsvs.filter(function(ts) { return ts.tierBelow === tier; }).length;
    html += '<div class="rp-stat-row">' +
      '<span class="rp-stat-label" style="color:' + TIER_FILLS[tier % TIER_FILLS.length] + '">Tier ' + tier + '</span>' +
      '<span class="rp-stat-value">' + modCnt + ' modules, ' + tsvCnt + ' TSVs</span>' +
      '</div>';
  }
  infoEl.innerHTML = html;
}

// ── Constraint 清除按鈕顯示控制 ──
function updateClearConstraintBtn() {
  var val = document.getElementById('constraintPath').value;
  document.getElementById('clearConstraint').style.display = val ? 'inline-block' : 'none';
}

function chk(id) { return document.getElementById(id).checked; }

function escXml(s) {
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

// 取得 module 的有效位置（若有 override 則用 override）
function getEffMod(m) {
  var ov = moduleOverrides[m.name];
  if (!ov) return m;
  return { name: m.name, tier: m.tier, xll: ov.xll, yll: ov.yll, xur: ov.xur, yur: ov.yur };
}

// Constraint 查詢
function isFixed(name) {
  return constraintData.some(function(c) { return c.type === 'FIXED' && c.name === name; });
}
// 回傳 module 所屬的 REPULSE 群組編號（1-based），0 表示不屬於任何群組
function repulseGroupIdx(name) {
  var groups = constraintData.filter(function(c) { return c.type === 'REPULSE'; });
  for (var gi = 0; gi < groups.length; gi++) {
    if (groups[gi].names && groups[gi].names.indexOf(name) >= 0) { return gi + 1; }
  }
  return 0;
}

function hasMultiRepulse() {
  return constraintData.filter(function(c) { return c.type === 'REPULSE'; }).length > 1;
}

// REPULSE 群組藍色角標（module / terminal 共用）
function appendRepulseBadge(parts, x, y, maxSize, rgi) {
  if (!rgi) { return; }
  var bs = Math.min(maxSize, 12);
  var rlabel = hasMultiRepulse() ? 'R' + rgi : 'R';
  parts.push('<rect x="' + x + '" y="' + y + '" width="' + bs + '" height="' + bs + '"' +
    ' fill="#2288bb" rx="2" style="pointer-events:none"/>');
  parts.push('<text x="' + (x + bs / 2) + '" y="' + (y + bs / 2) + '"' +
    ' text-anchor="middle" dominant-baseline="middle"' +
    ' font-size="' + Math.max(6, bs * 0.65) + '" font-weight="bold" fill="#fff"' +
    ' style="pointer-events:none">' + rlabel + '</text>');
}

function ensureCanvasHoverTag() {
  var container = document.getElementById('canvas-container');
  var tag = document.getElementById('canvas-hover-tag');
  if (!tag) {
    tag = document.createElement('div');
    tag.id = 'canvas-hover-tag';
    tag.className = 'canvas-hover-tag';
    container.appendChild(tag);
  }
  return tag;
}

function addHoverTagListeners(container) {
  var tag = ensureCanvasHoverTag();
  tag.style.display = 'none';
  container.querySelectorAll('.hover-tag-target').forEach(function(el) {
    el.addEventListener('mouseenter', function() {
      tag.textContent = el.dataset.label || '';
      tag.style.display = 'block';
    });
    el.addEventListener('mousemove', function(e) {
      var bounds = container.getBoundingClientRect();
      tag.style.left = (e.clientX - bounds.left + 8) + 'px';
      tag.style.top = (e.clientY - bounds.top - 22) + 'px';
    });
    el.addEventListener('mouseleave', function() {
      tag.style.display = 'none';
    });
  });
}

function populatePresets(presets) {
  var sel = document.getElementById('presetSelect');
  while (sel.options.length > 1) {
    sel.remove(1);
  }
  (presets || []).forEach(function(p) {
    var o = document.createElement('option');
    o.value = JSON.stringify({ b: p.block, n: p.nets, o: p.output, c: p.constraint || '' });
    o.textContent = p.label;
    sel.appendChild(o);
  });
}

function syncPresetSelectFromPaths() {
  var sel = document.getElementById('presetSelect');
  var b = document.getElementById('blockPath').value;
  var n = document.getElementById('netsPath').value;
  var o = document.getElementById('outPath').value;
  var match = '';
  for (var i = 1; i < sel.options.length; i++) {
    try {
      var j = JSON.parse(sel.options[i].value);
      if (j.b === b && j.n === n && j.o === o) {
        match = sel.options[i].value;
        break;
      }
    } catch (e2) {}
  }
  sel.value = match || '';
}

document.getElementById('presetSelect').onchange = function() {
  var sel = document.getElementById('presetSelect');
  if (!sel.value) { return; }
  try {
    var j = JSON.parse(sel.value);
    document.getElementById('blockPath').value      = j.b;
    document.getElementById('netsPath').value       = j.n;
    document.getElementById('outPath').value        = j.o;
    document.getElementById('constraintPath').value = j.c || '';
    updateClearConstraintBtn();
    // 立即同步 constraint 清單
    if (j.c) {
      vscode.postMessage({ type: 'loadConstraint', path: j.c });
    } else {
      constraintFileLoaded = false;
      constraintData = [];
      buildConstraintList();
      render();
    }
  } catch (e3) {}
};

// ── 浮動面板開關 ──
function togglePanel(id) {
  var target = document.getElementById(id);
  var wasOpen = target.classList.contains('open');
  // 關閉所有
  document.querySelectorAll('.float-panel').forEach(function(p) { p.classList.remove('open'); });
  if (!wasOpen) { target.classList.add('open'); }
}
document.addEventListener('click', function(e) {
  if (!e.target.closest('.canvas-toolbar-left') && !e.target.closest('.canvas-toolbar-right')) {
    document.querySelectorAll('.float-panel').forEach(function(p) { p.classList.remove('open'); });
  }
});
document.getElementById('btnTier').onclick = function(e) { e.stopPropagation(); togglePanel('panelTier'); };
document.getElementById('btnView').onclick  = function(e) { e.stopPropagation(); togglePanel('panelView'); };

// ── Tier 面板：動態建立選項 ──
function buildTierPanel(numDie) {
  var panel = document.getElementById('panelTier');
  // 清除舊的 tier 選項（保留 overview）
  panel.querySelectorAll('.fp-option[data-mode]').forEach(function(el) {
    if (el.dataset.mode !== 'overview') { el.remove(); }
  });
  panel.querySelector('hr.fp-sep') && panel.querySelector('hr.fp-sep').remove();

  var sep = document.createElement('hr');
  sep.className = 'fp-sep';
  panel.appendChild(sep);

  for (var t = 0; t < numDie; t++) {
    (function(tier) {
      var opt = document.createElement('div');
      opt.className = 'fp-option' + (currentMode === tier ? ' active' : '');
      opt.dataset.mode = String(tier);
      var dot = '<span class="tier-dot" style="background:' + TIER_FILLS[tier % TIER_FILLS.length] + '"></span>';
      opt.innerHTML = dot + 'Tier ' + tier;
      opt.onclick = function() {
        currentMode = tier;
        updateTierBtn();
        updateTierPanelActive();
        document.getElementById('panelTier').classList.remove('open');
        render();
      };
      panel.appendChild(opt);
    })(t);
  }

  // Overview option click
  document.getElementById('optOverview').onclick = function() {
    currentMode = 'overview';
    updateTierBtn();
    updateTierPanelActive();
    document.getElementById('panelTier').classList.remove('open');
    render();
  };
}

function updateTierBtn() {
  var btn = document.getElementById('btnTier');
  btn.textContent = (currentMode === 'overview') ? 'Overview ▾' : 'Tier ' + currentMode + ' ▾';
}

function updateTierPanelActive() {
  document.querySelectorAll('#panelTier .fp-option').forEach(function(el) {
    var mode = el.dataset.mode === 'overview' ? 'overview' : parseInt(el.dataset.mode);
    el.classList.toggle('active', currentMode === mode);
  });
}

// ── Net 連線輔助 ──

// 回傳 module 的中心座標與所在 tier（帶 override）
function modCenter(name) {
  var m = sceneData.modules.find(function(mod) { return mod.name === name; });
  if (!m) { return null; }
  var em = getEffMod(m);
  return { x: (em.xll + em.xur) / 2, y: (em.yll + em.yur) / 2, tier: m.tier };
}

// TSV netName 格式為 "net{i}"（對應 sceneData.nets[i]）
// 回傳 net index，解析失敗則回傳 -1
function tsvNetIndex(tsv) {
  var m = tsv.netName.match(/^net(\d+)$/i);
  return m ? parseInt(m[1], 10) : -1;
}

// 回傳 terminal 的座標（無 tier，出現在所有 tier view）
function termPos(name) {
  if (!sceneData.terminals) { return null; }
  var t = sceneData.terminals.find(function(tp) { return tp.name === name; });
  return t ? { x: t.x, y: t.y } : null;
}

// 建立：net index → true，代表該 net 有 TSV（跨層 net）
function buildCrossTierSet() {
  var set = {};
  if (!sceneData.tsvs) { return set; }
  sceneData.tsvs.forEach(function(tsv) {
    var idx = tsvNetIndex(tsv);
    if (idx >= 0) { set[idx] = true; }
  });
  return set;
}

// 共用：依 allPoints 數量畫直線或 star topology
function drawNetLines(parts, allPoints, stroke, dash, tx, ty) {
  if (allPoints.length < 2) { return; }
  if (allPoints.length === 2) {
    parts.push('<line x1="' + tx(allPoints[0].x) + '" y1="' + ty(allPoints[0].y) + '"' +
      ' x2="' + tx(allPoints[1].x) + '" y2="' + ty(allPoints[1].y) + '"' +
      ' stroke="' + stroke + '" stroke-width="1"' + (dash ? ' stroke-dasharray="4 3"' : '') +
      ' opacity="0.45" style="pointer-events:none"/>');
  } else {
    var cx = 0, cy = 0;
    allPoints.forEach(function(p) { cx += p.x; cy += p.y; });
    cx /= allPoints.length;
    cy /= allPoints.length;
    allPoints.forEach(function(p) {
      parts.push('<line x1="' + tx(p.x) + '" y1="' + ty(p.y) + '"' +
        ' x2="' + tx(cx) + '" y2="' + ty(cy) + '"' +
        ' stroke="' + stroke + '" stroke-width="1"' + (dash ? ' stroke-dasharray="4 3"' : '') +
        ' opacity="0.45" style="pointer-events:none"/>');
    });
    parts.push('<circle cx="' + tx(cx) + '" cy="' + ty(cy) + '" r="2.5"' +
      ' fill="' + stroke + '" opacity="0.65" style="pointer-events:none"/>');
  }
}

// ── 層內連線（intra-tier net：無 TSV 的 net）──
// 包含 terminal 作為連線點；只在有 module 的 tier 上畫（避免在不含 module 的 tier 重複）
function renderNetsIntra(parts, tier, tx, ty, scale) {
  if (!sceneData.nets || sceneData.nets.length === 0) { return; }
  var fill = TIER_FILLS[tier % TIER_FILLS.length];
  var crossSet = buildCrossTierSet();

  sceneData.nets.forEach(function(net, idx) {
    if (crossSet[idx]) { return; }  // 跨層 net 由 renderNetsInter 處理
    // 若有選取的 module，只顯示含該 module 的 net
    if (selectedMod && net.pins.indexOf(selectedMod) < 0) { return; }

    var modPts = [], termPts = [];
    net.pins.forEach(function(pin) {
      var c = modCenter(pin);
      if (c && c.tier === tier) { modPts.push({ x: c.x, y: c.y }); }
      else if (!c) {
        var tp = termPos(pin);
        if (tp) { termPts.push(tp); }
      }
    });

    // 至少要有一個本 tier 的 module，terminal 才參與（避免同一條 net 在多個 tier 重複畫）
    if (modPts.length === 0) { return; }

    drawNetLines(parts, modPts.concat(termPts), fill, false, tx, ty);
  });
}

// ── 跨層連線（inter-tier net：有 TSV 的 net）──
// 在本 tier 上收集：module 中心 + TSV 虛擬點 + terminal 位置，一起連線
function renderNetsInter(parts, tier, tx, ty, scale) {
  if (!sceneData.nets || sceneData.nets.length === 0) { return; }
  if (!sceneData.tsvs || sceneData.tsvs.length === 0) { return; }

  // 建立：net index → 本 tier 上的 TSV 座標列表
  var netTsvMap = {};
  sceneData.tsvs.forEach(function(tsv) {
    if (tsv.tierBelow !== tier && tsv.tierAbove !== tier) { return; }
    var idx = tsvNetIndex(tsv);
    if (idx < 0 || idx >= sceneData.nets.length) { return; }
    if (!netTsvMap[idx]) { netTsvMap[idx] = []; }
    netTsvMap[idx].push({ x: tsv.x, y: tsv.y });
  });

  sceneData.nets.forEach(function(net, idx) {
    var tsvPoints = netTsvMap[idx];
    if (!tsvPoints || tsvPoints.length === 0) { return; }
    // 若有選取的 module，只顯示含該 module 的 net
    if (selectedMod && net.pins.indexOf(selectedMod) < 0) { return; }

    var modPts = [], termPts = [];
    net.pins.forEach(function(pin) {
      var c = modCenter(pin);
      if (c && c.tier === tier) { modPts.push({ x: c.x, y: c.y }); }
      else if (!c) {
        var tp = termPos(pin);
        if (tp) { termPts.push(tp); }
      }
    });

    // 本 tier 的所有連接點：module 中心 + TSV 虛擬點 + terminal
    var allPoints = modPts.concat(tsvPoints).concat(termPts);

    if (allPoints.length < 2) {
      // 只有孤立 TSV 虛擬點（無 module 也無 terminal）→ 空心圓標示
      tsvPoints.forEach(function(p) {
        parts.push('<circle cx="' + tx(p.x) + '" cy="' + ty(p.y) + '" r="2"' +
          ' fill="none" stroke="#ffcc44" stroke-width="1" opacity="0.4"' +
          ' style="pointer-events:none"/>');
      });
      return;
    }

    drawNetLines(parts, allPoints, '#ffcc44', true, tx, ty);
  });
}

// Die outline + grid + axis ticks（外框細實線；格線為淡虛線）
// 根據邊長自動選擇刻度步距
function calcGridStep(size) {
  if (size < 20)  { return 2;  }
  if (size < 50)  { return 10; }
  if (size < 100) { return 20; }
  if (size < 150) { return 30; }
  if (size < 500) { return 50; }
  if (size < 1000) { return 100; }
  if (size < 2000) { return 200; }
  if (size < 5000) { return 500; }
  return 1000;
}
function appendDieWithGridAxes(parts, tx, ty, W, H, scale) {
  var x0 = tx(0), yTop = ty(H), wPx = W * scale, hPx = H * scale;
  parts.push('<rect x="' + x0 + '" y="' + yTop + '" width="' + wPx + '" height="' + hPx + '" fill="#252830"/>');
  var stepX = calcGridStep(W);
  var stepY = calcGridStep(H);
  var g;
  for (g = stepX; g < W; g += stepX) {
    parts.push('<line x1="' + tx(g) + '" y1="' + ty(0) + '" x2="' + tx(g) + '" y2="' + ty(H) + '" stroke="#3a4558" stroke-width="0.5" stroke-dasharray="3 4" opacity="0.5"/>');
  }
  for (g = stepY; g < H; g += stepY) {
    parts.push('<line x1="' + tx(0) + '" y1="' + ty(g) + '" x2="' + tx(W) + '" y2="' + ty(g) + '" stroke="#3a4558" stroke-width="0.5" stroke-dasharray="3 4" opacity="0.5"/>');
  }
  parts.push('<rect x="' + x0 + '" y="' + yTop + '" width="' + wPx + '" height="' + hPx + '" fill="none" stroke="#6b7d92" stroke-width="1"/>');
  var fs = Math.max(6, Math.min(10, scale * 0.14));
  var tick = Math.max(3, scale * 0.1);
  for (g = 0; g <= W; g += stepX) {
    parts.push('<line x1="' + tx(g) + '" y1="' + ty(0) + '" x2="' + tx(g) + '" y2="' + (ty(0) + tick) + '" stroke="#8b9db0" stroke-width="1"/>');
    parts.push('<text x="' + tx(g) + '" y="' + (ty(0) + tick + fs + 2) + '" text-anchor="middle" font-size="' + fs + '" fill="#9aacbf">' + g + '</text>');
  }
  for (g = 0; g <= H; g += stepY) {
    parts.push('<line x1="' + (tx(0) - tick) + '" y1="' + ty(g) + '" x2="' + tx(0) + '" y2="' + ty(g) + '" stroke="#8b9db0" stroke-width="1"/>');
    parts.push('<text x="' + (tx(0) - tick - 4) + '" y="' + ty(g) + '" text-anchor="end" dominant-baseline="middle" font-size="' + fs + '" fill="#9aacbf">' + g + '</text>');
  }
}

// ── 共用繪製工具：計算置中的座標轉換（預留四周給座標軸標示）──
function resetViewTransform() {
  viewZoom = 1;
  viewPanX = 0;
  viewPanY = 0;
}

function makeTransform(outline, ox, oy, pw, ph) {
  var W = outline.width, H = outline.height;
  var PAD_L = 35, PAD_R = 25, PAD_T = 20, PAD_B = 30;
  var availW = pw - PAD_L - PAD_R;
  var availH = ph - PAD_T - PAD_B;
  var baseScale = Math.min(availW / W, availH / H);
  var baseDiePxW = W * baseScale, baseDiePxH = H * baseScale;
  var baseStartX = ox + PAD_L + (availW - baseDiePxW) / 2;
  var baseStartY = oy + PAD_T + (availH - baseDiePxH) / 2;
  var centerX = baseStartX + baseDiePxW / 2;
  var centerY = baseStartY + baseDiePxH / 2;

  var scale = baseScale * viewZoom;
  var diePxW = W * scale, diePxH = H * scale;
  var startX = centerX - diePxW / 2 + viewPanX;
  var startY = centerY - diePxH / 2 + viewPanY;

  currentTransform = { startX: startX, startY: startY, scale: scale, W: W, H: H, baseScale: baseScale };
  return {
    scale: scale, W: W, H: H,
    tx: function(x) { return startX + x * scale; },
    ty: function(y) { return startY + (H - y) * scale; },
    diePxW: diePxW, diePxH: diePxH
  };
}

// ── 單層渲染（個別 Tier 全螢幕） ──
function renderTierInto(parts, tier, ox, oy, pw, ph) {
  var outline = sceneData.outlines[Math.min(tier, sceneData.outlines.length - 1)];
  var tr = makeTransform(outline, ox, oy, pw, ph);
  var tx = tr.tx, ty = tr.ty, scale = tr.scale;

  appendDieWithGridAxes(parts, tx, ty, tr.W, tr.H, tr.scale);

  // Modules（只顯示此層，支援 override 位置與選取高亮）
  if (chk('chkModules')) {
    var fill   = TIER_FILLS[tier % TIER_FILLS.length];
    var stroke = TIER_STROKES[tier % TIER_STROKES.length];
    var showML = chk('chkModuleLabels');
    var mods = sceneData.modules.filter(function(m) { return m.tier === tier; });
    for (var mi = 0; mi < mods.length; mi++) {
      var m = mods[mi];
      var em = getEffMod(m);
      var rx = tx(em.xll), ry = ty(em.yur);
      var rw = Math.max((em.xur - em.xll) * scale, 0.8);
      var rh = Math.max((em.yur - em.yll) * scale, 0.8);
      var isSel = selectedMod === m.name;
      var fixed = isFixed(m.name);
      var rgi = repulseGroupIdx(m.name);
      parts.push('<rect class="module-rect" data-modname="' + escXml(m.name) + '"' +
        ' x="' + rx + '" y="' + ry + '" width="' + rw + '" height="' + rh + '"' +
        ' fill="' + fill + '" fill-opacity="' + (isSel ? '0.6' : '0.35') + '"' +
        ' stroke="' + (isSel ? '#ffffff' : (fixed ? '#e08020' : stroke)) + '" stroke-width="' + (isSel ? '2' : (fixed ? '1.5' : '0.8')) + '"/>');
      if (fixed) {
        parts.push('<rect x="' + rx + '" y="' + ry + '" width="' + rw + '" height="' + rh + '" fill="url(#hatch-fixed)" style="pointer-events:none"/>');
      }
      appendRepulseBadge(parts, rx, ry, Math.min(14, rw * 0.4, rh * 0.4), rgi);
      if (showML && rw > 14 && rh > 8) {
        var cx = rx + rw/2, cy = ry + rh/2;
        var fs = Math.min(9, rh * 0.45, rw * 0.18);
        if (fs > 3.5) {
          parts.push('<text x="' + cx + '" y="' + cy + '" text-anchor="middle" dominant-baseline="middle" font-size="' + fs + '" fill="#fff" opacity="0.85" style="pointer-events:none">' + escXml(m.name) + '</text>');
        }
      }
    }
  }

  // Net 連線（在 modules 之上、TSV 之下渲染）
  if (chk('chkNetsIntra')) { renderNetsIntra(parts, tier, tx, ty, scale); }
  if (chk('chkNetsInter')) { renderNetsInter(parts, tier, tx, ty, scale); }

  // TSVs：只顯示在下層（tierBelow === tier）
  if (chk('chkTsvs')) {
    var showTL = chk('chkTsvLabels');
    var tsvUserSz = parseFloat(document.getElementById('tsvSize').value) || 3;
    var tsvHalf = Math.max(0.1, tsvUserSz * scale / 2);
    var tsvList = sceneData.tsvs.filter(function(t) { return t.tierBelow === tier; });
    for (var ti = 0; ti < tsvList.length; ti++) {
      var t = tsvList[ti];
      var px = tx(t.x), py = ty(t.y);
      var tsvX = px - tsvHalf, tsvY = py - tsvHalf, tsvSz = tsvHalf * 2;
      parts.push('<rect class="hover-tag-target" data-label="' + escXml(t.netName) + '"' +
        ' x="' + tsvX + '" y="' + tsvY + '" width="' + tsvSz + '" height="' + tsvSz + '"' +
        ' fill="#ff5566" opacity="0.5" stroke="#ffaabb" stroke-width="0.7"/>');
      if (showTL) {
        parts.push('<text x="' + (px + tsvHalf + 2) + '" y="' + py + '" font-size="6" fill="#ff8899" dominant-baseline="middle" opacity="0.7">' + escXml(t.netName) + '</text>');
      }
    }
  }

  // Terminals
  if (chk('chkTerminals')) {
    var showTmL = chk('chkTerminalLabels');
    var S = Math.max(4, scale * 0.8);
    for (var ti2 = 0; ti2 < sceneData.terminals.length; ti2++) {
      var tp = sceneData.terminals[ti2];
      var px = tx(tp.x), py = ty(tp.y);
      var termX = px - S / 2, termY = py - S / 2;
      var termRgi = repulseGroupIdx(tp.name);
      parts.push('<rect class="hover-tag-target" data-label="' + escXml(tp.name) + '"' +
        ' x="' + termX + '" y="' + termY + '" width="' + S + '" height="' + S + '"' +
        ' fill="#44cc88" opacity="0.9"/>');
      appendRepulseBadge(parts, termX, termY, S, termRgi);
      if (showTmL) {
        parts.push('<text x="' + (px+S/2+2) + '" y="' + py + '" font-size="6" fill="#66ddaa" dominant-baseline="middle" opacity="0.7">' + escXml(tp.name) + '</text>');
      }
    }
  }
}

// ── Overview 渲染：所有 tier 疊合在同一張圖 ──
function renderOverlaid(parts, cw, ch) {
  var maxW = 0, maxH = 0;
  for (var t = 0; t < sceneData.outlines.length; t++) {
    maxW = Math.max(maxW, sceneData.outlines[t].width);
    maxH = Math.max(maxH, sceneData.outlines[t].height);
  }
  var fakeOutline = { width: maxW, height: maxH };
  var tr = makeTransform(fakeOutline, 0, 0, cw, ch);
  var tx = tr.tx, ty = tr.ty, scale = tr.scale;

  appendDieWithGridAxes(parts, tx, ty, maxW, maxH, tr.scale);

  if (chk('chkModules')) {
    var showML = chk('chkModuleLabels');
    for (var t = 0; t < sceneData.outlines.length; t++) {
      var fill   = TIER_FILLS[t % TIER_FILLS.length];
      var stroke = TIER_STROKES[t % TIER_STROKES.length];
      var mods = sceneData.modules.filter(function(m) { return m.tier === t; });
      for (var mi = 0; mi < mods.length; mi++) {
        var m = mods[mi];
        var em = getEffMod(m);
        var rx = tx(em.xll), ry = ty(em.yur);
        var rw = Math.max((em.xur - em.xll) * scale, 0.8);
        var rh = Math.max((em.yur - em.yll) * scale, 0.8);
        var isSel = selectedMod === m.name;
        var fixed = isFixed(m.name);
        var rgi = repulseGroupIdx(m.name);
        parts.push('<rect class="module-rect" data-modname="' + escXml(m.name) + '"' +
          ' x="' + rx + '" y="' + ry + '" width="' + rw + '" height="' + rh + '"' +
          ' fill="' + fill + '" fill-opacity="' + (isSel ? '0.6' : '0.28') + '"' +
          ' stroke="' + (isSel ? '#ffffff' : (fixed ? '#e08020' : stroke)) + '" stroke-width="' + (isSel ? '2' : (fixed ? '1.5' : '0.7')) + '"/>');
        if (fixed) {
          parts.push('<rect x="' + rx + '" y="' + ry + '" width="' + rw + '" height="' + rh + '" fill="url(#hatch-fixed)" style="pointer-events:none"/>');
        }
        appendRepulseBadge(parts, rx, ry, Math.min(14, rw * 0.4, rh * 0.4), rgi);
        if (showML && rw > 14 && rh > 8) {
          var cx = rx + rw/2, cy = ry + rh/2;
          var fs = Math.min(9, rh * 0.45, rw * 0.18);
          if (fs > 3.5) {
            parts.push('<text x="' + cx + '" y="' + cy + '" text-anchor="middle" dominant-baseline="middle" font-size="' + fs + '" fill="#fff" opacity="0.8" style="pointer-events:none">' + escXml(m.name) + '</text>');
          }
        }
      }
    }
  }

  // Net 連線 overview：所有 tier 一起畫
  if (chk('chkNetsIntra')) {
    for (var nt = 0; nt < sceneData.outlines.length; nt++) {
      renderNetsIntra(parts, nt, tx, ty, scale);
    }
  }
  if (chk('chkNetsInter')) {
    for (var nt2 = 0; nt2 < sceneData.outlines.length; nt2++) {
      renderNetsInter(parts, nt2, tx, ty, scale);
    }
  }

  // TSVs（顯示在下層位置，即 tierBelow 所在層）
  if (chk('chkTsvs')) {
    var showTL = chk('chkTsvLabels');
    var tsvUserSz = parseFloat(document.getElementById('tsvSize').value) || 3;
    var tsvHalf = Math.max(0.1, tsvUserSz * scale / 2);
    for (var ti = 0; ti < sceneData.tsvs.length; ti++) {
      var t = sceneData.tsvs[ti];
      var px = tx(t.x), py = ty(t.y);
      var tsvX = px - tsvHalf, tsvY = py - tsvHalf, tsvSz = tsvHalf * 2;
      parts.push('<rect class="hover-tag-target" data-label="' + escXml(t.netName) + '"' +
        ' x="' + tsvX + '" y="' + tsvY + '" width="' + tsvSz + '" height="' + tsvSz + '"' +
        ' fill="#ff5566" opacity="0.5" stroke="#ffaabb" stroke-width="0.7"/>');
      if (showTL) {
        parts.push('<text x="' + (px + tsvHalf + 2) + '" y="' + py + '" font-size="6" fill="#ff8899" dominant-baseline="middle" opacity="0.7">' + escXml(t.netName) + '</text>');
      }
    }
  }

  // Terminals
  if (chk('chkTerminals')) {
    var showTmL = chk('chkTerminalLabels');
    var S = Math.max(4, scale * 0.8);
    for (var ti2 = 0; ti2 < sceneData.terminals.length; ti2++) {
      var tp = sceneData.terminals[ti2];
      var px = tx(tp.x), py = ty(tp.y);
      var termX = px - S / 2, termY = py - S / 2;
      var termRgi = repulseGroupIdx(tp.name);
      parts.push('<rect class="hover-tag-target" data-label="' + escXml(tp.name) + '"' +
        ' x="' + termX + '" y="' + termY + '" width="' + S + '" height="' + S + '"' +
        ' fill="#44cc88" opacity="0.9"/>');
      appendRepulseBadge(parts, termX, termY, S, termRgi);
      if (showTmL) {
        parts.push('<text x="' + (px+S/2+2) + '" y="' + py + '" font-size="6" fill="#66ddaa" dominant-baseline="middle" opacity="0.7">' + escXml(tp.name) + '</text>');
      }
    }
  }

  // 圖例（右上角）
  var numDie = sceneData.outlines.length;
  for (var lt = 0; lt < numDie; lt++) {
    var lx = cw - 8, ly = 14 + lt * 18;
    parts.push('<rect x="' + (lx-52) + '" y="' + (ly-9) + '" width="11" height="11" fill="' + TIER_FILLS[lt%TIER_FILLS.length] + '" fill-opacity="0.6" rx="2"/>');
    parts.push('<text x="' + (lx-37) + '" y="' + ly + '" font-size="11" fill="' + TIER_FILLS[lt%TIER_FILLS.length] + '">Tier ' + lt + '</text>');
  }
}

// ── 主渲染函數 ──
function render() {
  var container = document.getElementById('canvas-container');
  if (!sceneData) {
    container.innerHTML = '<div class="placeholder">Please press "Reload 2D" to load the layout</div>';
    return;
  }
  var cw = Math.max(container.clientWidth  - 2, 200);
  var ch = Math.max(container.clientHeight - 2, 200);
  var parts = [];
  parts.push('<defs><pattern id="hatch-fixed" patternUnits="userSpaceOnUse" width="7" height="7"><line x1="0" y1="7" x2="7" y2="0" stroke="#ffffff" stroke-width="0.8" opacity="0.28"/></pattern></defs>');
  parts.push('<rect x="0" y="0" width="' + cw + '" height="' + ch + '" fill="var(--vscode-editor-background)"/>');

  if (currentMode === 'overview') {
    renderOverlaid(parts, cw, ch);
  } else {
    renderTierInto(parts, currentMode, 0, 0, cw, ch);
  }

  container.innerHTML = '<svg width="' + cw + '" height="' + ch + '" xmlns="http://www.w3.org/2000/svg">' + parts.join('') + '</svg>';
  container.classList.add('can-pan');
  container.classList.remove('is-panning');
  addModuleListeners(container);
  addHoverTagListeners(container);
  updateInfoSection();
}

// ── 檔案操作事件 ──
function currentInputPaths() {
  return {
    blockPath:      document.getElementById('blockPath').value.trim(),
    netsPath:       document.getElementById('netsPath').value.trim(),
    constraintPath: document.getElementById('constraintPath').value.trim(),
  };
}

document.getElementById('pickBlock').onclick = function() {
  vscode.postMessage(Object.assign({ type: 'pickBlock' }, currentInputPaths()));
};
document.getElementById('pickNets').onclick = function() {
  vscode.postMessage(Object.assign({ type: 'pickNets' }, currentInputPaths()));
};
document.getElementById('runFloorplan').onclick = function() {
  vscode.postMessage({
    type:       'run',
    block:      document.getElementById('blockPath').value,
    nets:       document.getElementById('netsPath').value,
    output:     document.getElementById('outPath').value,
    constraint: document.getElementById('constraintPath').value.trim(),
  });
};
document.getElementById('reload2d').onclick = function() {
  vscode.postMessage({
    type: 'reload2d',
    blockPath:      document.getElementById('blockPath').value,
    netsPath:       document.getElementById('netsPath').value.trim(),
    outputPath:     document.getElementById('outPath').value.trim(),
    constraintPath: document.getElementById('constraintPath').value.trim(),
  });
};

document.getElementById('btnZoomIn').onclick = function() {
  viewZoom = Math.min(20, viewZoom * 1.25);
  render();
};
document.getElementById('btnZoomOut').onclick = function() {
  viewZoom = Math.max(0.2, viewZoom / 1.25);
  render();
};
document.getElementById('btnZoomReset').onclick = function() {
  resetViewTransform();
  render();
};

// ── Module 互動：加入事件監聽（每次 render 後呼叫）──
function addModuleListeners(container) {
  var svgEl = container.querySelector('svg');
  if (svgEl) {
    svgEl.addEventListener('mousedown', function(e) {
      if (e.target.classList.contains('module-rect')) { return; }
      if (e.button !== 0) { return; }
      panState = {
        startClientX: e.clientX,
        startClientY: e.clientY,
        origPanX: viewPanX,
        origPanY: viewPanY,
        moved: false,
      };
      container.classList.add('is-panning');
      document.addEventListener('mousemove', onPanMove);
      document.addEventListener('mouseup', onPanEnd);
    });
  }
  container.querySelectorAll('.module-rect').forEach(function(el) {
    el.addEventListener('mousedown', function(e) {
      e.stopPropagation(); e.preventDefault();
      var name = el.dataset.modname;
      selectedMod = name;
      updateRightPanel();
      if (!currentTransform) { render(); return; }
      var bounds = container.getBoundingClientRect();
      var orig = sceneData.modules.find(function(m) { return m.name === name; });
      var em = getEffMod(orig);
      dragState = {
        name: name,
        startSvgX: e.clientX - bounds.left,
        startSvgY: e.clientY - bounds.top,
        origXll: em.xll, origYll: em.yll,
        w: em.xur - em.xll, h: em.yur - em.yll
      };
      document.addEventListener('mousemove', onDragMove);
      document.addEventListener('mouseup',   onDragEnd);
      render();
    });
  });
}

function onPanMove(e) {
  if (!panState) { return; }
  var dx = e.clientX - panState.startClientX;
  var dy = e.clientY - panState.startClientY;
  if (Math.abs(dx) > 2 || Math.abs(dy) > 2) { panState.moved = true; }
  viewPanX = panState.origPanX + dx;
  viewPanY = panState.origPanY + dy;
  render();
}

function onPanEnd() {
  var container = document.getElementById('canvas-container');
  container.classList.remove('is-panning');
  document.removeEventListener('mousemove', onPanMove);
  document.removeEventListener('mouseup', onPanEnd);
  if (panState && !panState.moved && selectedMod !== null) {
    selectedMod = null;
    updateRightPanel();
    render();
  }
  panState = null;
}

function onDragMove(e) {
  if (!dragState || !currentTransform) return;
  var container = document.getElementById('canvas-container');
  var bounds = container.getBoundingClientRect();
  var dx = (e.clientX - bounds.left - dragState.startSvgX) / currentTransform.scale;
  var dy = -((e.clientY - bounds.top)  - dragState.startSvgY) / currentTransform.scale;
  var newXll = dragState.origXll + dx;
  var newYll = dragState.origYll + dy;
  var orig = sceneData.modules.find(function(m) { return m.name === dragState.name; });
  if (orig) {
    var outline = sceneData.outlines[Math.min(orig.tier, sceneData.outlines.length - 1)];
    newXll = Math.max(0, Math.min(outline.width  - dragState.w, newXll));
    newYll = Math.max(0, Math.min(outline.height - dragState.h, newYll));
  }
  moduleOverrides[dragState.name] = { xll: newXll, yll: newYll, xur: newXll + dragState.w, yur: newYll + dragState.h };
  render();
  updateRightPanel();
}

function onDragEnd() {
  dragState = null;
  document.removeEventListener('mousemove', onDragMove);
  document.removeEventListener('mouseup',   onDragEnd);
}

// ── 右側面板更新（Selected 區域固定顯示，無選取時留空）──
function updateRightPanel() {
  if (!selectedMod || !sceneData) {
    document.getElementById('rpModName').textContent = '—';
    document.getElementById('rpSize').textContent = '';
    document.getElementById('rpInputX').value = '';
    document.getElementById('rpInputY').value = '';
    document.getElementById('rpLlX').textContent = '—';
    document.getElementById('rpLlY').textContent = '—';
    document.getElementById('rpUrX').textContent = '—';
    document.getElementById('rpUrY').textContent = '—';
    return;
  }
  var orig = sceneData.modules.find(function(m) { return m.name === selectedMod; });
  if (!orig) return;
  var em = getEffMod(orig);
  document.getElementById('rpModName').textContent = selectedMod;
  var w = (em.xur - em.xll).toFixed(1);
  var h = (em.yur - em.yll).toFixed(1);
  document.getElementById('rpSize').textContent = 'Width × Height: ' + w + ' × ' + h;
  document.getElementById('rpInputX').value = em.xll.toFixed(2);
  document.getElementById('rpInputY').value = em.yll.toFixed(2);
  document.getElementById('rpLlX').textContent = em.xll.toFixed(2);
  document.getElementById('rpLlY').textContent = em.yll.toFixed(2);
  document.getElementById('rpUrX').textContent = em.xur.toFixed(2);
  document.getElementById('rpUrY').textContent = em.yur.toFixed(2);
}

// ── 右側按鈕 ──
document.getElementById('btnRotate').onclick = function() {
  if (!selectedMod) return;
  var orig = sceneData.modules.find(function(m) { return m.name === selectedMod; });
  if (!orig) return;
  var em = getEffMod(orig);
  var cx = (em.xll + em.xur) / 2, cy = (em.yll + em.yur) / 2;
  var oldW = em.xur - em.xll, oldH = em.yur - em.yll;
  moduleOverrides[selectedMod] = { xll: cx - oldH/2, yll: cy - oldW/2, xur: cx + oldH/2, yur: cy + oldW/2 };
  render(); updateRightPanel();
};

document.getElementById('btnSetPos').onclick = function() {
  if (!selectedMod) return;
  var x = parseFloat(document.getElementById('rpInputX').value);
  var y = parseFloat(document.getElementById('rpInputY').value);
  if (isNaN(x) || isNaN(y)) return;
  var orig = sceneData.modules.find(function(m) { return m.name === selectedMod; });
  if (!orig) return;
  var em = getEffMod(orig);
  var w = em.xur - em.xll, h = em.yur - em.yll;
  moduleOverrides[selectedMod] = { xll: x, yll: y, xur: x + w, yur: y + h };
  render(); updateRightPanel();
};

document.getElementById('btnResetPos').onclick = function() {
  if (!selectedMod) return;
  delete moduleOverrides[selectedMod];
  render(); updateRightPanel();
};

document.getElementById('btnResetAll').onclick = function() {
  moduleOverrides = {};
  render(); updateRightPanel();
};

document.getElementById('btnApplyFixed').onclick = function() {
  if (!selectedMod || !sceneData) { return; }
  var orig = sceneData.modules.find(function(m) { return m.name === selectedMod; });
  if (!orig) { return; }
  var em = getEffMod(orig);

  // 驗證尺寸是否符合原始 width×height 或旋轉後的 height×width
  var origW = orig.xur - orig.xll;
  var origH = orig.yur - orig.yll;
  var curW  = em.xur - em.xll;
  var curH  = em.yur - em.yll;
  var EPS   = 0.01;
  var matchNormal  = Math.abs(curW - origW) <= EPS && Math.abs(curH - origH) <= EPS;
  var matchRotated = Math.abs(curW - origH) <= EPS && Math.abs(curH - origW) <= EPS;
  if (!matchNormal && !matchRotated) {
    log('[WARN] Fixed "' + selectedMod + '": dimensions (' + curW.toFixed(2) + ' × ' + curH.toFixed(2) +
      ') do not match original size (' + origW.toFixed(2) + ' × ' + origH.toFixed(2) +
      ') or its rotated form (' + origH.toFixed(2) + ' × ' + origW.toFixed(2) +
      '). Please use ✓ Set with correct coordinates first.');
    return;
  }

  // 檢查 constraint 中是否已存在同名 FIXED
  var alreadyFixed = constraintData.some(function(c) {
    return c.type === 'FIXED' && c.name === selectedMod;
  });
  if (alreadyFixed) {
    log('[WARN] Fixed "' + selectedMod + '": already has a FIXED constraint. Remove it first before re-applying.');
    return;
  }

  var line = 'FIXED ' + selectedMod +
    ' ' + em.xll.toFixed(4) +
    ' ' + em.yll.toFixed(4) +
    ' ' + em.xur.toFixed(4) +
    ' ' + em.yur.toFixed(4);
  vscode.postMessage({ type: 'addConstraint', line: line });
};

// ── Constraint 驗證：回傳 warning 字串陣列，無問題則空陣列 ──
// excludeIdx: 當更新既有行時，排除自身 index 避免誤判重複
function validateFixedConstraint(name, llx, lly, urx, ury, excludeIdx) {
  var warnings = [];
  if (!name) {
    warnings.push('module name is empty.');
    return warnings;
  }
  // 座標欄位不能為空且必須是數字
  if (isNaN(llx)) { warnings.push('llx is empty or not a number.'); }
  if (isNaN(lly)) { warnings.push('lly is empty or not a number.'); }
  if (isNaN(urx)) { warnings.push('urx is empty or not a number.'); }
  if (isNaN(ury)) { warnings.push('ury is empty or not a number.'); }
  if (warnings.length > 0) { return warnings; }
  // 在已讀取資料時確認 module 存在
  if (sceneData) {
    var found = sceneData.modules.some(function(m) { return m.name === name; });
    if (!found) {
      warnings.push('module "' + name + '" not found in current floorplan data.');
      return warnings;
    }
  }
  // 重複 FIXED 檢查
  var dup = constraintData.some(function(c, ci) {
    return c.type === 'FIXED' && c.name === name &&
           (excludeIdx === undefined || ci !== excludeIdx);
  });
  if (dup) {
    warnings.push('module "' + name + '" already has a FIXED constraint.');
  }
  // 尺寸與 sceneData 比對（只在有讀取資料時才檢查）
  if (sceneData) {
    var orig = sceneData.modules.find(function(m) { return m.name === name; });
    if (orig) {
      var origW = orig.xur - orig.xll, origH = orig.yur - orig.yll;
      var curW  = urx - llx,           curH  = ury - lly;
      var EPS   = 0.01;
      // 允許旋轉：(curW≈origW && curH≈origH) 或 (curW≈origH && curH≈origW)
      var matchNormal  = Math.abs(curW - origW) <= EPS && Math.abs(curH - origH) <= EPS;
      var matchRotated = Math.abs(curW - origH) <= EPS && Math.abs(curH - origW) <= EPS;
      if (!matchNormal && !matchRotated) {
        warnings.push('dimensions (' + curW.toFixed(2) + ' × ' + curH.toFixed(2) +
          ') do not match module "' + name + '" original size (' +
          origW.toFixed(2) + ' × ' + origH.toFixed(2) + ') or its rotated form (' +
          origH.toFixed(2) + ' × ' + origW.toFixed(2) + ').');
      }
    }
  }
  return warnings;
}

// REPULSE constraint 驗證
function validateRepulseConstraint(strengthStr, namesStr) {
  var warnings = [];
  if (!strengthStr) {
    warnings.push('strength is empty.');
  } else if (isNaN(parseFloat(strengthStr))) {
    warnings.push('strength "' + strengthStr + '" is not a number.');
  }
  var names = namesStr.trim().split(/\s+/).filter(function(n) { return n.length > 0; });
  if (names.length < 2) {
    warnings.push('at least 2 names (module or terminal) are required (got ' + names.length + ').');
    return warnings;
  }
  // 確認每個 name 對應 module 或 terminal
  if (sceneData) {
    names.forEach(function(n) {
      var isMod = sceneData.modules.some(function(m) { return m.name === n; });
      var isTerm = sceneData.terminals && sceneData.terminals.some(function(t) { return t.name === n; });
      if (!isMod && !isTerm) {
        warnings.push('"' + n + '" not found as module or terminal in current floorplan data.');
      }
    });
  }
  return warnings;
}

function ensurePendingLabel(list) {
  var label = list.querySelector('.cst-pending-label');
  if (!label) {
    label = document.createElement('div');
    label.className = 'cst-pending-label';
    label.textContent = 'Pending';
    var addRow = list.querySelector('.cst-add-row');
    if (addRow) {
      list.insertBefore(label, addRow.nextSibling);
    } else {
      list.appendChild(label);
    }
  }
  return label;
}

function updatePendingLabelVisibility(list) {
  var label = list.querySelector('.cst-pending-label');
  if (!label) { return; }
  label.style.display = list.querySelector('.cst-row-pending') ? 'block' : 'none';
}

function updateFileLabelVisibility(list) {
  var label = list.querySelector('.cst-file-label');
  if (!label) { return; }
  label.style.display = list.querySelector('.cst-row-file') ? 'block' : 'none';
}

// ── 新增待確認列（不立即寫檔，按 ✓ 才寫） ──
function appendPendingRow(type, list) {
  var row = document.createElement('div');
  row.className = 'cst-row cst-row-pending';

  var badge = document.createElement('span');
  badge.className = 'cst-badge cst-badge-' + type.toLowerCase();
  badge.textContent = type === 'FIXED' ? 'F' : 'R';

  var fields = document.createElement('div');
  fields.className = 'cst-fields';

  if (type === 'FIXED') {
    ['name','llx','lly','urx','ury'].forEach(function(key) {
      var inp = document.createElement('input');
      inp.type = 'text';
      inp.value = '';
      inp.placeholder = key;
      inp.title = key;
      inp.className = 'cst-inp ' + (key === 'name' ? 'cst-inp-name' : 'cst-inp-coord');
      inp.dataset.key = key;
      fields.appendChild(inp);
    });
  } else {
    var sInp = document.createElement('input');
    sInp.type = 'text'; sInp.value = '1.0'; sInp.placeholder = 'strength';
    sInp.title = 'strength'; sInp.dataset.key = 'strength';
    sInp.className = 'cst-inp cst-inp-strength';

    var nInp = document.createElement('input');
    nInp.type = 'text'; nInp.value = ''; nInp.placeholder = 'name1 name2 …';
    nInp.title = 'module or terminal names (space-separated)'; nInp.dataset.key = 'names';
    nInp.className = 'cst-inp cst-inp-names';

    fields.appendChild(sInp);
    fields.appendChild(nInp);
  }

  var confirmBtn = document.createElement('button');
  confirmBtn.className = 'cst-confirm'; confirmBtn.textContent = '＋';
  confirmBtn.title = 'Add (writes file)';
  confirmBtn.onclick = function() {
    function v(key) {
      var el = fields.querySelector('[data-key="' + key + '"]');
      return el ? el.value.trim() : '';
    }
      var line;
    if (type === 'FIXED') {
      var name = v('name');
      var llx = parseFloat(v('llx')), lly = parseFloat(v('lly'));
      var urx = parseFloat(v('urx')), ury = parseFloat(v('ury'));
      var warns = validateFixedConstraint(name, llx, lly, urx, ury, undefined);
      if (warns.length > 0) {
        warns.forEach(function(w) { log('[WARN] FIXED "' + name + '": ' + w); });
        return;
      }
      line = 'FIXED ' + name + ' ' + v('llx') + ' ' + v('lly') + ' ' + v('urx') + ' ' + v('ury');
    } else {
      var rWarns = validateRepulseConstraint(v('strength'), v('names'));
      if (rWarns.length > 0) {
        rWarns.forEach(function(w) { log('[WARN] REPULSE: ' + w); });
        return;
      }
      line = 'REPULSE ' + v('strength') + ' ' + v('names');
    }
    vscode.postMessage({ type: 'addConstraint', line: line });
    row.remove();
    updatePendingLabelVisibility(list);
  };

  var delBtn = document.createElement('button');
  delBtn.className = 'cst-del'; delBtn.textContent = '✕';
  delBtn.title = 'Cancel';
  delBtn.onclick = function() {
    row.remove();
    updatePendingLabelVisibility(list);
  };

  row.appendChild(badge); row.appendChild(fields);
  row.appendChild(confirmBtn); row.appendChild(delBtn);

  var pendingLabel = ensurePendingLabel(list);
  list.insertBefore(row, pendingLabel.nextSibling);
  updatePendingLabelVisibility(list);
  // 聚焦到第一個輸入欄
  var firstInp = fields.querySelector('input');
  if (firstInp) { firstInp.focus(); }
}

// ── Constraint 清單渲染（可編輯）──
function buildConstraintList() {
  var sec  = document.getElementById('rpConstraintSection');
  var list = document.getElementById('constraintList');
  if (!constraintFileLoaded) { sec.style.display = 'none'; return; }
  sec.style.display = 'block';
  list.innerHTML = '';

  // 新增列固定在最上方
  var addRow = document.createElement('div');
  addRow.className = 'cst-add-row';

  var typeSel = document.createElement('select');
  typeSel.className = 'cst-add-type';
  ['FIXED', 'REPULSE'].forEach(function(t) {
    var opt = document.createElement('option');
    opt.value = t; opt.textContent = t;
    typeSel.appendChild(opt);
  });

  var addBtn = document.createElement('button');
  addBtn.className = 'cst-add-btn'; addBtn.textContent = 'Add Constraint';
  addBtn.onclick = function() {
    appendPendingRow(typeSel.value, list);
  };

  addRow.appendChild(typeSel); addRow.appendChild(addBtn);
  list.appendChild(addRow);

  var pendingLabel = document.createElement('div');
  pendingLabel.className = 'cst-pending-label';
  pendingLabel.textContent = 'Pending';
  list.appendChild(pendingLabel);

  var fileLabel = document.createElement('div');
  fileLabel.className = 'cst-file-label';
  fileLabel.textContent = '.constraint file';
  list.appendChild(fileLabel);

  constraintData.forEach(function(c, i) {
    var row = document.createElement('div');
    row.className = 'cst-row cst-row-file';

    var badge = document.createElement('span');
    badge.className = 'cst-badge cst-badge-' + c.type.toLowerCase();
    badge.textContent = c.type === 'FIXED' ? 'F' : 'R';

    var fields = document.createElement('div');
    fields.className = 'cst-fields';

    if (c.type === 'FIXED') {
      ['name','llx','lly','urx','ury'].forEach(function(key) {
        var inp = document.createElement('input');
        inp.type = 'text';
        inp.value = c[key];
        inp.placeholder = key;
        inp.title = key;
        inp.className = 'cst-inp ' + (key === 'name' ? 'cst-inp-name' : 'cst-inp-coord');
        inp.dataset.key = key;
        fields.appendChild(inp);
      });
    } else {
      var strengthInp = document.createElement('input');
      strengthInp.type = 'text';
      strengthInp.value = c.strength; strengthInp.placeholder = 'strength';
      strengthInp.title = 'strength'; strengthInp.dataset.key = 'strength';
      strengthInp.className = 'cst-inp cst-inp-strength';

      var namesInp = document.createElement('input');
      namesInp.type = 'text';
      namesInp.value = c.names.join(' '); namesInp.placeholder = 'name1 name2 …';
      namesInp.title = 'module or terminal names (space-separated)'; namesInp.dataset.key = 'names';
      namesInp.className = 'cst-inp cst-inp-names';

      fields.appendChild(strengthInp);
      fields.appendChild(namesInp);
    }

    var confirm = document.createElement('button');
    confirm.className = 'cst-confirm'; confirm.textContent = '✓';
    confirm.title = 'Save (writes file)';
    confirm.onclick = (function(idx, type, fieldsEl) {
      return function() {
        var line;
        function v(key) {
          var el = fieldsEl.querySelector('[data-key="' + key + '"]');
          return el ? el.value.trim() : '';
        }
        if (type === 'FIXED') {
          var name = v('name');
          var llx = parseFloat(v('llx')), lly = parseFloat(v('lly'));
          var urx = parseFloat(v('urx')), ury = parseFloat(v('ury'));
          // 驗證：排除自身 index（避免誤判自己與自己重複）
          var warns = validateFixedConstraint(name, llx, lly, urx, ury, idx);
          if (warns.length > 0) {
            warns.forEach(function(w) { log('[WARN] FIXED "' + name + '": ' + w); });
            return;
          }
          line = 'FIXED ' + name + ' ' + v('llx') + ' ' + v('lly') + ' ' + v('urx') + ' ' + v('ury');
        } else {
          var rWarns = validateRepulseConstraint(v('strength'), v('names'));
          if (rWarns.length > 0) {
            rWarns.forEach(function(w) { log('[WARN] REPULSE: ' + w); });
            return;
          }
          line = 'REPULSE ' + v('strength') + ' ' + v('names');
        }
        vscode.postMessage({ type: 'updateConstraint', index: idx, line: line });
      };
    })(i, c.type, fields);

    var del = document.createElement('button');
    del.className = 'cst-del'; del.textContent = '✕';
    del.title = 'Delete (writes file)';
    del.onclick = (function(idx) {
      return function() { vscode.postMessage({ type: 'deleteConstraint', index: idx }); };
    })(i);

    row.appendChild(badge); row.appendChild(fields);
    row.appendChild(confirm); row.appendChild(del);
    list.appendChild(row);
  });
  updateFileLabelVisibility(list);
}

document.getElementById('pickConstraint').onclick = function() {
  vscode.postMessage(Object.assign({ type: 'pickConstraint' }, currentInputPaths()));
};

document.getElementById('clearConstraint').onclick = function() {
  document.getElementById('constraintPath').value = '';
  updateClearConstraintBtn();
  vscode.postMessage({ type: 'clearConstraint' });
};

// View 面板 checkbox / input 變更 → 重新渲染
['chkModules','chkTsvs','chkTerminals',
 'chkNetsIntra','chkNetsInter',
 'chkModuleLabels','chkTsvLabels','chkTerminalLabels'].forEach(function(id) {
  document.getElementById(id).onchange = render;
});
document.getElementById('tsvSize').oninput = render;

// ── Canvas resize → 重新渲染 ──
new ResizeObserver(function() { render(); }).observe(document.getElementById('canvas-container'));

// ── 可拖曳分隔線 ──
function makeResizer(resizerId, getTarget, getSide) {
  var resizer = document.getElementById(resizerId);
  var startX, startW;
  resizer.addEventListener('mousedown', function(e) {
    startX = e.clientX;
    startW = getTarget().getBoundingClientRect().width;
    resizer.classList.add('dragging');
    document.body.style.cursor = 'col-resize';
    document.body.style.userSelect = 'none';
    function onMove(e) {
      var dx = e.clientX - startX;
      var newW = Math.max(120, Math.min(600, startW + getSide() * dx));
      getTarget().style.flex = '0 0 ' + newW + 'px';
    }
    function onUp() {
      resizer.classList.remove('dragging');
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
      document.removeEventListener('mousemove', onMove);
      document.removeEventListener('mouseup', onUp);
      render();
    }
    document.addEventListener('mousemove', onMove);
    document.addEventListener('mouseup', onUp);
    e.preventDefault();
  });
}
makeResizer('resizerL', function() { return document.getElementById('leftPanel'); }, function() { return 1; });
makeResizer('resizerR', function() { return document.getElementById('rightPanel'); }, function() { return -1; });

// ── 接收 extension host 訊息 ──
window.addEventListener('message', function(e) {
  var msg = e.data;
  if (msg.type === 'setPaths') {
    if (msg.block)      document.getElementById('blockPath').value      = msg.block;
    if (msg.nets)       document.getElementById('netsPath').value       = msg.nets;
    if (msg.output)     document.getElementById('outPath').value        = msg.output;
    if (msg.constraint !== undefined) document.getElementById('constraintPath').value = msg.constraint;
    updateClearConstraintBtn();
    syncPresetSelectFromPaths();
  }
  if (msg.type === 'constraintData') {
    constraintData = msg.constraints || [];
    // msg.reset === true 表示使用者主動清除，不顯示 constraint 區塊
    constraintFileLoaded = !msg.reset;
    buildConstraintList();
    render();
  }
  if (msg.type === 'presets') {
    populatePresets(msg.presets);
    syncPresetSelectFromPaths();
  }
  if (msg.type === 'log') { log(msg.message); }
  if (msg.type === 'sceneData') {
    sceneData = msg.payload;
    currentMode = 'overview';
    resetViewTransform();
    buildTierPanel(sceneData.outlines.length);
    updateTierBtn();
    render(); // render() 內已呼叫 updateInfoSection()
    log('[OK] ' + sceneData.modules.length + ' modules, ' +
        sceneData.tsvs.length + ' TSVs, ' +
        sceneData.terminals.length + ' terminals');
    var resSec = document.getElementById('rpResultSection');
    resSec.style.display = 'block'; // 有讀檔就顯示 Result/Info 區塊
    var hpwlBlock = document.getElementById('rpHpwlBlock');
    if (msg.payload.hpwl) {
      var hpwlNum = parseFloat(msg.payload.hpwl);
      document.getElementById('rpHpwl').textContent = isNaN(hpwlNum) ? msg.payload.hpwl : hpwlNum.toFixed(3);
      var dieParts = msg.payload.die.trim().split(/\s+/);
      var dieW = parseFloat(dieParts[0]);
      var dieH = parseFloat(dieParts[1]);
      document.getElementById('rpDie').textContent =
        (isNaN(dieW) || isNaN(dieH))
          ? msg.payload.die.replace(/\s+/, ' × ')
          : formatDecimalMax3(dieW) + ' × ' + formatDecimalMax3(dieH);
      document.getElementById('rpTime').textContent = msg.payload.time;
      hpwlBlock.style.display = 'block';
    } else {
      hpwlBlock.style.display = 'none';
    }
  }
});

vscode.postMessage({ type: 'ready' });
