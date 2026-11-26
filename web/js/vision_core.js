// VisionInterface.html 用的計算（不碰畫面，node 也能跑測試）：座標換算、頻率、統計、訊息解析、CSV。
(function (root) {
    'use strict';

    // =======================================================================
    // 設定：要跟這些檔案對齊，改了那邊要回來改這裡
    // =======================================================================
    var CONFIG = {
        labels: ['b1', 'b2', 'b3', 'b4'],
        // bottle_localization/config/bottle_dims.yaml（m）
        dims: {
            b1: { diameter: 0.08290, height: 0.09230 },
            b2: { diameter: 0.04943, height: 0.07803 },
            b3: { diameter: 0.04485, height: 0.08988 },
            b4: { diameter: 0.03785, height: 0.07924 }
        },
        // 桌面座標（同 sp3_task_manager：task_manager.yaml 的 frames、table，grasp_plan.meas_to_table）
        //   原點 = 桌緣（靠機器人那一邊）上、正對 base_link 左右中線的點，高度 = 桌面；[右, 前, 上] cm
        //   右 = −y×100、前 = x×100 − 桌緣離 base_link、上 = z×100 − 桌面在 base_link 的高度
        table: {
            edge_from_base_link_cm: 13.4,   // 2026-11-25 實量：右肩垂線到桌緣 14.8 − 肩在 base_link 後 1.375（task_manager.yaml 當時仍是 12.0）
            top_z_cm: -39.0,                // 120.2（frames.height_offset_cm）− 81.2（table.top_cm）= 39.0 cm 在 base_link 下方
            depth_cm: 75.0,                 // table.depth_cm
            width_cm: 150.0,                // table.width_cm
            center_right_cm: 0.0            // table.center_right_cm
        },
        // 頭部刻度條：/head_tf/status 沒帶範圍時（舊版節點）用這組，同 head_tf/config/neck_head.yaml
        head: {
            reference_ticks: { '28': 2047, '29': 1708, '30': 2024 },
            allowed_ticks: { '28': [1877, 2218], '29': [1500, 1820], '30': [2014, 2034] },
            margin_ticks: 6
        },
        deg_per_tick: 360 / 4096
    };

    function baseToTable(x, y, z, table) {
        table = table || CONFIG.table;
        return {
            right: -y * 100.0,
            front: x * 100.0 - table.edge_from_base_link_cm,
            up: z * 100.0 - table.top_z_cm
        };
    }

    // 最近 windowS 秒內收到幾筆 -> Hz
    function RateMeter(windowS) {
        this.window = (windowS || 2.0) * 1000;
        this.times = [];
    }
    RateMeter.prototype.add = function (nowMs) { this.times.push(nowMs); this.trim(nowMs); };
    RateMeter.prototype.trim = function (nowMs) {
        while (this.times.length && this.times[0] < nowMs - this.window) this.times.shift();
    };
    RateMeter.prototype.hz = function (nowMs) {
        this.trim(nowMs);
        return this.times.length / (this.window / 1000);
    };
    RateMeter.prototype.last = function () { return this.times.length ? this.times[this.times.length - 1] : null; };

    function mean(a) {
        if (!a.length) return NaN;
        var s = 0;
        for (var i = 0; i < a.length; i++) s += a[i];
        return s / a.length;
    }

    function std(a) {      // 母體標準差（同 bottle_viewer 的 statistics.pstdev）
        if (!a.length) return NaN;
        var m = mean(a), s = 0;
        for (var i = 0; i < a.length; i++) s += (a[i] - m) * (a[i] - m);
        return Math.sqrt(s / a.length);
    }

    // vision_msgs/Detection3DArray（rosbridge 轉成的 JSON）-> [{label, score, x, y, z}]（m，base_link）
    function parseDetections3D(msg) {
        var out = [];
        (msg.detections || []).forEach(function (d) {
            var r = (d.results && d.results[0]) || {};
            var h = r.hypothesis || {};
            var p = (d.bbox && d.bbox.center && d.bbox.center.position) ||
                    (r.pose && r.pose.pose && r.pose.pose.position);
            if (!p) return;
            out.push({ label: h.class_id || d.id || '?', score: h.score != null ? h.score : NaN,
                       x: p.x, y: p.y, z: p.z });
        });
        return out;
    }

    // /med_detect/bottles 的 JSON 字串 -> {label: 信心值}
    function parseYolo(text) {
        var out = {};
        var j = JSON.parse(text);
        (j.bottles || []).forEach(function (b) { out[b.label] = b.confidence; });
        return out;
    }

    // 每個類別最近 windowS 秒的位置，算抖動
    function Track(windowS) {
        this.window = (windowS || 3.0) * 1000;
        this.samples = {};      // label -> [{t, x, y, z}]
    }
    Track.prototype.add = function (nowMs, det) {
        var a = this.samples[det.label] || (this.samples[det.label] = []);
        a.push({ t: nowMs, x: det.x, y: det.y, z: det.z });
        while (a.length && a[0].t < nowMs - this.window) a.shift();
    };
    // 回 {n, horizontal_mm, z_mm}；水平抖動 = sqrt(std(x)² + std(y)²)
    Track.prototype.jitter = function (label, nowMs) {
        var a = (this.samples[label] || []).filter(function (s) { return s.t >= nowMs - this.window; }, this);
        if (a.length < 2) return { n: a.length, horizontal_mm: NaN, z_mm: NaN };
        var sx = std(a.map(function (s) { return s.x; })), sy = std(a.map(function (s) { return s.y; }));
        return { n: a.length, horizontal_mm: Math.sqrt(sx * sx + sy * sy) * 1000,
                 z_mm: std(a.map(function (s) { return s.z; })) * 1000 };
    };

    // 錄製的樣本 [{t, label, score, x, y, z, ticks}] -> 每個類別的摘要
    function summarize(samples, table) {
        var by = {};
        samples.forEach(function (s) { (by[s.label] = by[s.label] || []).push(s); });
        var out = [];
        Object.keys(by).sort().forEach(function (label) {
            var a = by[label];
            var xs = a.map(function (s) { return s.x; }), ys = a.map(function (s) { return s.y; }),
                zs = a.map(function (s) { return s.z; });
            var m = { x: mean(xs), y: mean(ys), z: mean(zs) };
            out.push({ label: label, n: a.length, mean: m, table: baseToTable(m.x, m.y, m.z, table),
                       std_mm: { x: std(xs) * 1000, y: std(ys) * 1000, z: std(zs) * 1000 } });
        });
        return out;
    }

    function f(v, d) { return (v == null || isNaN(v)) ? '' : Number(v).toFixed(d); }

    function samplesCsv(samples, table) {
        var rows = ['time_s,label,score,x_m,y_m,z_m,right_cm,front_cm,up_cm,tick28,tick29,tick30'];
        var t0 = samples.length ? samples[0].t : 0;
        samples.forEach(function (s) {
            var tb = baseToTable(s.x, s.y, s.z, table), k = s.ticks || {};
            rows.push([f((s.t - t0) / 1000, 3), s.label, f(s.score, 3), f(s.x, 4), f(s.y, 4), f(s.z, 4),
                       f(tb.right, 1), f(tb.front, 1), f(tb.up, 1), f(k['28'], 0), f(k['29'], 0), f(k['30'], 0)].join(','));
        });
        return rows.join('\n') + '\n';
    }

    function summaryCsv(summary) {
        var rows = ['label,n,x_m,y_m,z_m,right_cm,front_cm,up_cm,std_x_mm,std_y_mm,std_z_mm'];
        summary.forEach(function (s) {
            rows.push([s.label, s.n, f(s.mean.x, 4), f(s.mean.y, 4), f(s.mean.z, 4), f(s.table.right, 1),
                       f(s.table.front, 1), f(s.table.up, 1), f(s.std_mm.x, 1), f(s.std_mm.y, 1), f(s.std_mm.z, 1)].join(','));
        });
        return rows.join('\n') + '\n';
    }

    // 頭部刻度：離基準幾度、在不在範圍內
    function headTick(tick, ref, range, margin) {
        var deg = (tick - ref) * CONFIG.deg_per_tick;
        var state = (tick >= range[0] && tick <= range[1]) ? 'ok'
            : (tick >= range[0] - margin && tick <= range[1] + margin) ? 'margin' : 'out';
        return { deg: deg, state: state };
    }

    var api = {
        CONFIG: CONFIG, baseToTable: baseToTable, RateMeter: RateMeter, mean: mean, std: std,
        parseDetections3D: parseDetections3D, parseYolo: parseYolo, Track: Track, summarize: summarize,
        samplesCsv: samplesCsv, summaryCsv: summaryCsv, headTick: headTick
    };
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.VisionCore = api;
})(this);
