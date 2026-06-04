#!/usr/bin/env python3
"""
make_figures.py — 从 diag_monitor --log 的飞行 CSV 生成阶段性报告用图。
Generate stage-report figures from diag_monitor ``--log`` flight CSVs.

依赖 / deps: numpy, matplotlib  (Windows 上用 ``py``: 已确认可用)

用法 / usage
------------
单次 run  / single run:
    py report/make_figures.py report/data/flight_cross5_line.csv --out report/figures

多次对比 / multi-run compare (>= 2 个 CSV):
    py report/make_figures.py report/data/flight_cross5_line.csv \
                              report/data/flight_star5_line.csv \
                              --out report/figures --labels cross5-line,star5-line

产出 / outputs (PNG @150dpi → --out 目录):
    <stem>_panels.png    六联面板: pos_err / formation_err / min_spacing /
                          altitude z / max_solve_ms / horizontal speed
    <stem>_traj.png      俯视 XY 轨迹 (仅当 CSV 含 d{i}_x / d{i}_y)
    compare_metrics.png  关键指标柱状对比 (>= 2 个 CSV 时)
    compare_spacing.png  最小间距时间序列叠加 (>= 2 个 CSV 时)
    metrics_table.md     每个 run 指标汇总(Markdown 表, 可直接贴进报告)

设计说明 / notes
- 图内文字一律英文，避免 matplotlib 缺中文字体导致方框；中文说明放报告图注。
- d_safe=1.5 m / warn=1.8 m 与 diag_monitor.py 常量一致。
- 纯 numpy+matplotlib，不依赖 pandas，便于在最小环境运行。
"""
import argparse
import csv
import math
import os

import numpy as np

import matplotlib
matplotlib.use('Agg')          # headless: 仅存文件，不弹窗
import matplotlib.pyplot as plt  # noqa: E402

D_SAFE = 1.5     # m, 碰撞软约束距离 (= diag_monitor SPACING_CRIT)
D_WARN = 1.8     # m, 预警间距     (= diag_monitor SPACING_WARN)
READY_ERR = 0.5  # m, "进编队" 判定阈值 (= leader ready_pos_err)


# --------------------------------------------------------------------- io
def _to_float(s):
    try:
        v = float(s)
        return v if math.isfinite(v) else math.nan
    except (TypeError, ValueError):
        return math.nan


def load_csv(path):
    """读 CSV → (cols dict[str->np.ndarray], ndr int, has_xy bool)。"""
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f'空日志 / empty CSV: {path}')
    names = list(rows[0].keys())
    cols = {n: np.array([_to_float(r.get(n)) for r in rows]) for n in names}
    ndr = sum(1 for n in names if n.endswith('_poserr'))
    has_xy = ('d0_x' in cols) and ('d0_y' in cols)
    return cols, ndr, has_xy


def get(cols, name, n):
    """取列, 缺失则返回全 nan 数组(长度 n)。"""
    return cols.get(name, np.full(n, math.nan))


# --------------------------------------------------------------- summary
def summarize(path):
    """返回单次 run 的指标 dict, 供表格与对比图使用。"""
    cols, ndr, has_xy = load_csv(path)
    t = cols['t']
    n = len(t)
    dur = float(t[-1] - t[0]) if n > 1 else 0.0

    per_drone = []
    for i in range(ndr):
        pe = get(cols, f'd{i}_poserr', n)
        ze = np.abs(get(cols, f'd{i}_zerr', n))
        per_drone.append({
            'poserr_max': float(np.nanmax(pe)) if np.any(np.isfinite(pe)) else math.nan,
            'poserr_mean': float(np.nanmean(pe)) if np.any(np.isfinite(pe)) else math.nan,
            'zerr_max': float(np.nanmax(ze)) if np.any(np.isfinite(ze)) else math.nan,
        })

    poserr_overall = max((d['poserr_max'] for d in per_drone
                          if math.isfinite(d['poserr_max'])), default=math.nan)

    sp = get(cols, 'min_spacing', n)
    if np.any(np.isfinite(sp)):
        idx = int(np.nanargmin(sp))
        min_spacing, min_spacing_t = float(sp[idx]), float(t[idx])
    else:
        min_spacing, min_spacing_t = math.nan, math.nan

    ferr = get(cols, 'formation_max_err', n)
    form_max = float(np.nanmax(ferr)) if np.any(np.isfinite(ferr)) else math.nan

    ms = get(cols, 'max_solve_ms', n)
    solve_max = float(np.nanmax(ms)) if np.any(np.isfinite(ms)) else math.nan
    solve_mean = float(np.nanmean(ms)) if np.any(np.isfinite(ms)) else math.nan

    viol = get(cols, 'safety_violations', n)
    viol_last = int(np.nanmax(viol)) if np.any(np.isfinite(viol)) else 0
    fb = get(cols, 'total_fallbacks', n)
    fb_last = int(np.nanmax(fb)) if np.any(np.isfinite(fb)) else 0

    # 编队成型时间: 第一次全员 poserr < READY_ERR
    settle = math.nan
    for k in range(n):
        errs = [get(cols, f'd{i}_poserr', n)[k] for i in range(ndr)]
        if errs and all(math.isfinite(e) and e < READY_ERR for e in errs):
            settle = float(t[k])
            break

    return {
        'path': path, 'stem': os.path.splitext(os.path.basename(path))[0],
        'ndr': ndr, 'dur': dur, 'has_xy': has_xy,
        'per_drone': per_drone, 'poserr_overall': poserr_overall,
        'min_spacing': min_spacing, 'min_spacing_t': min_spacing_t,
        'form_max': form_max, 'solve_max': solve_max, 'solve_mean': solve_mean,
        'violations': viol_last, 'fallbacks': fb_last, 'settle': settle,
        'cols': cols,
    }


def _fmt(v, p=2):
    return '—' if (v is None or (isinstance(v, float) and not math.isfinite(v))) else f'{v:.{p}f}'


# ----------------------------------------------------------- single run
def fig_panels(s, out_dir):
    cols, n, ndr = s['cols'], len(s['cols']['t']), s['ndr']
    t = cols['t']
    fig, ax = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle(f"{s['stem']}   ({ndr} UAV, {s['dur']:.0f}s)", fontsize=13)

    # pos_err per drone
    for i in range(ndr):
        ax[0, 0].plot(t, get(cols, f'd{i}_poserr', n), label=f'd{i}')
    ax[0, 0].axhline(READY_ERR, ls='--', c='gray', lw=1, label=f'ready {READY_ERR}m')
    ax[0, 0].set_title('Per-UAV tracking error (pos_err)')
    ax[0, 0].set_ylabel('pos_err [m]'); ax[0, 0].legend(fontsize=8, ncol=2)

    # formation max err
    ax[0, 1].plot(t, get(cols, 'formation_max_err', n), c='tab:purple')
    ax[0, 1].set_title('Formation max error (vs leader+offset)')
    ax[0, 1].set_ylabel('max err [m]')

    # min spacing + safety lines
    ax[1, 0].plot(t, get(cols, 'min_spacing', n), c='tab:red')
    ax[1, 0].axhline(D_WARN, ls='--', c='orange', lw=1, label=f'warn {D_WARN}m')
    ax[1, 0].axhline(D_SAFE, ls='--', c='red', lw=1, label=f'd_safe {D_SAFE}m')
    ax[1, 0].set_title('Minimum inter-UAV spacing')
    ax[1, 0].set_ylabel('min spacing [m]'); ax[1, 0].legend(fontsize=8)

    # altitude z
    for i in range(ndr):
        ax[1, 1].plot(t, get(cols, f'd{i}_z', n), label=f'd{i}')
    ax[1, 1].set_title('Altitude z (NED, more negative = higher)')
    ax[1, 1].set_ylabel('z [m]'); ax[1, 1].legend(fontsize=8, ncol=2)

    # solve time
    ax[2, 0].plot(t, get(cols, 'max_solve_ms', n), c='tab:green')
    ax[2, 0].set_title('MPC solve time (max over swarm)')
    ax[2, 0].set_ylabel('solve [ms]'); ax[2, 0].set_xlabel('t [s]')

    # horizontal speed
    for i in range(ndr):
        ax[2, 1].plot(t, get(cols, f'd{i}_velxy', n), label=f'd{i}')
    ax[2, 1].set_title('Horizontal speed')
    ax[2, 1].set_ylabel('|v_xy| [m/s]'); ax[2, 1].set_xlabel('t [s]')
    ax[2, 1].legend(fontsize=8, ncol=2)

    for a in ax.flat:
        a.grid(True, alpha=0.3)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    p = os.path.join(out_dir, f"{s['stem']}_panels.png")
    fig.savefig(p, dpi=150); plt.close(fig)
    return p


def fig_traj(s, out_dir):
    """俯视 XY 轨迹: 横轴 East(y), 纵轴 North(x)。仅当含 x,y。"""
    if not s['has_xy']:
        return None
    cols, n, ndr = s['cols'], len(s['cols']['t']), s['ndr']
    fig, axx = plt.subplots(figsize=(8, 8))
    for i in range(ndr):
        x = get(cols, f'd{i}_x', n)   # North
        y = get(cols, f'd{i}_y', n)   # East
        line, = axx.plot(y, x, lw=1.2, label=f'd{i}')
        m = np.isfinite(x) & np.isfinite(y)
        if np.any(m):
            xi, yi = x[m], y[m]
            axx.plot(yi[0], xi[0], 'o', color=line.get_color(), ms=6)   # start
            axx.plot(yi[-1], xi[-1], 's', color=line.get_color(), ms=6)  # end
    lx = get(cols, 'leader_x', n)
    ly = get(cols, 'leader_y', n)
    if np.any(np.isfinite(lx)) and np.any(np.isfinite(ly)):
        axx.plot(ly, lx, 'k--', lw=1.5, label='leader')
    axx.set_title(f"{s['stem']} — top-down trajectory  (o=start, s=end)")
    axx.set_xlabel('East  y [m]'); axx.set_ylabel('North  x [m]')
    axx.axis('equal'); axx.grid(True, alpha=0.3); axx.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    p = os.path.join(out_dir, f"{s['stem']}_traj.png")
    fig.savefig(p, dpi=150); plt.close(fig)
    return p


# ------------------------------------------------------------- compare
def fig_compare_metrics(summaries, labels, out_dir):
    metrics = [
        ('poserr_overall', 'max pos_err [m]', False),
        ('min_spacing', 'min spacing [m]', False),
        ('solve_max', 'max solve [ms]', False),
        ('settle', 'settle time [s]', False),
    ]
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    xs = np.arange(len(summaries))
    for a, (key, title, _) in zip(ax.flat, metrics):
        vals = [s[key] if math.isfinite(s[key]) else 0.0 for s in summaries]
        bars = a.bar(xs, vals, color='tab:blue', alpha=0.8)
        if key == 'min_spacing':
            a.axhline(D_SAFE, ls='--', c='red', lw=1)
            a.axhline(D_WARN, ls='--', c='orange', lw=1)
        a.set_title(title)
        a.set_xticks(xs); a.set_xticklabels(labels, rotation=20, ha='right', fontsize=8)
        a.grid(True, axis='y', alpha=0.3)
        for b, v in zip(bars, vals):
            a.annotate(f'{v:.2f}', (b.get_x() + b.get_width() / 2, v),
                       ha='center', va='bottom', fontsize=8)
    fig.suptitle('Cross-scenario stability comparison', fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p = os.path.join(out_dir, 'compare_metrics.png')
    fig.savefig(p, dpi=150); plt.close(fig)
    return p


def fig_compare_spacing(summaries, labels, out_dir):
    fig, axx = plt.subplots(figsize=(11, 5))
    for s, lab in zip(summaries, labels):
        cols = s['cols']; n = len(cols['t'])
        axx.plot(cols['t'], get(cols, 'min_spacing', n), lw=1.3, label=lab)
    axx.axhline(D_SAFE, ls='--', c='red', lw=1, label=f'd_safe {D_SAFE}m')
    axx.axhline(D_WARN, ls='--', c='orange', lw=1, label=f'warn {D_WARN}m')
    axx.set_title('Minimum inter-UAV spacing — all runs')
    axx.set_xlabel('t [s]'); axx.set_ylabel('min spacing [m]')
    axx.grid(True, alpha=0.3); axx.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    p = os.path.join(out_dir, 'compare_spacing.png')
    fig.savefig(p, dpi=150); plt.close(fig)
    return p


def write_metrics_table(summaries, labels, out_dir):
    lines = [
        '# Run metrics summary / 各 run 指标汇总',
        '',
        '| Run | UAV | dur[s] | max pos_err[m] | settle[s] | min spacing[m] '
        '| form_max[m] | solve max/mean[ms] | violations | fallbacks |',
        '|---|---|---|---|---|---|---|---|---|---|',
    ]
    for s, lab in zip(summaries, labels):
        lines.append(
            f"| {lab} | {s['ndr']} | {_fmt(s['dur'],0)} | {_fmt(s['poserr_overall'])} "
            f"| {_fmt(s['settle'],0)} | {_fmt(s['min_spacing'])} | {_fmt(s['form_max'])} "
            f"| {_fmt(s['solve_max'])}/{_fmt(s['solve_mean'])} "
            f"| {s['violations']} | {s['fallbacks']} |"
        )
    p = os.path.join(out_dir, 'metrics_table.md')
    with open(p, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    return p


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description='生成阶段性报告图 / make stage-report figures')
    ap.add_argument('csv', nargs='+', help='一个或多个 flight CSV')
    ap.add_argument('--out', default='report/figures', help='图片输出目录')
    ap.add_argument('--labels', default=None,
                    help='对比图标签, 逗号分隔; 缺省用文件名')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    summaries = [summarize(p) for p in args.csv]
    labels = (args.labels.split(',') if args.labels
              else [s['stem'] for s in summaries])
    if len(labels) != len(summaries):
        ap.error('--labels 数量与 CSV 数量不一致')

    made = []
    for s in summaries:
        made.append(fig_panels(s, args.out))
        tp = fig_traj(s, args.out)
        if tp:
            made.append(tp)
        else:
            print(f"  [note] {s['stem']}: 无 d*_x/d*_y 列, 跳过轨迹图 "
                  f"(用更新后的 diag_monitor.py 重跑即可获得)")

    if len(summaries) >= 2:
        made.append(fig_compare_metrics(summaries, labels, args.out))
        made.append(fig_compare_spacing(summaries, labels, args.out))
    made.append(write_metrics_table(summaries, labels, args.out))

    print('\n== 已生成 / generated ==')
    for p in made:
        print('  ', p)
    print('\n== 指标速览 / quick metrics ==')
    for s, lab in zip(summaries, labels):
        print(f"  {lab}: pos_err_max={_fmt(s['poserr_overall'])}m  "
              f"min_spacing={_fmt(s['min_spacing'])}m  "
              f"settle={_fmt(s['settle'],0)}s  "
              f"solve_max={_fmt(s['solve_max'])}ms  "
              f"violations={s['violations']}  fallbacks={s['fallbacks']}")


if __name__ == '__main__':
    main()
