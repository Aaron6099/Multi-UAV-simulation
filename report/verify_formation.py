#!/usr/bin/env python3
"""
verify_formation.py — pair2 / trio3 编队 MPC 独立验证（无 ROS2/PX4）

每架无人机各自运行独立的 acados OCP，目标 = leader_pos + 队形偏移。
验证多机同时跟踪各自目标时的队形精度、间距安全与求解速度。

OCP 参数与 mpc_node.py / scenarios.yaml defaults 一致：
  N=20, dt=0.05, q_pos=4, q_vel=2, r_acc=0.1
注：实机 mpc_horizon=30；N=20 与现有 verify_mpc_step.py 一致，定性结论不变。

运行（Ubuntu, acados 已安装）：
    python3 report/verify_formation.py
输出：
    report/figures/verify_pair2_hover.png
    report/figures/verify_pair2_line.png
    report/figures/verify_trio3_circle.png
"""
import os, sys, time, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ACADOS_SRC = os.path.expanduser('~/acados')
os.environ.setdefault('ACADOS_SOURCE_DIR', ACADOS_SRC)
sys.path.insert(0, os.path.join(ACADOS_SRC, 'interfaces', 'acados_template'))
from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
import casadi as ca

# ── 参数（scenarios.yaml defaults）────────────────────────────────────────────
N          = 20;    DT     = 0.05          # horizon / 50 Hz
Q_POS=4.0; Q_VEL=2.0; R_ACC=0.1; Q_TERM=2.0
MAX_SPD=3.0;  MAX_ACC=4.0;  D_SAFE=1.5    # m/s, m/s², m
TARGET_ALT = -5.0                          # NED (离地 5 m)

OUTDIR    = os.path.join(os.path.dirname(__file__), 'figures')
BUILD_DIR = '/tmp/acados_verify_formation'
os.makedirs(OUTDIR,    exist_ok=True)
os.makedirs(BUILD_DIR, exist_ok=True)
COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']

# ── 队形偏移（scenarios.yaml formations.*.birth，z 叠 target_alt）─────────────
FORMATIONS = {
    'pair2': {
        'offsets': np.array([[  0,      0,      TARGET_ALT],
                             [ -3,      0,      TARGET_ALT]]),
        'labels':  ['d0', 'd1'],
    },
    'trio3': {
        'offsets': np.array([[ 3,     0,      TARGET_ALT],
                             [-1.5,   2.598,  TARGET_ALT],
                             [-1.5,  -2.598,  TARGET_ALT]]),
        'labels':  ['d0', 'd1', 'd2'],
    },
}

# ── 仿真场景 ──────────────────────────────────────────────────────────────────
SCENARIOS = [
    # (formation, mode, T_SIM_s, fname, title)
    ('pair2', 'hover',  40.0, 'verify_pair2_hover.png',
     'pair2 Hover — MPC Formation Verification (standalone acados)'),
    ('pair2', 'line',   70.0, 'verify_pair2_line.png',
     'pair2 Line v=0.5 m/s, d=20 m — MPC Formation Verification'),
    ('trio3', 'circle', 80.0, 'verify_trio3_circle.png',
     'trio3 Circle R=10 m, v=1.5 m/s — MPC Formation Verification'),
]

MAX_DRONES = 3  # trio3 最多 3 机


# ── OCP solver 构建 ───────────────────────────────────────────────────────────
def build_solver():
    """构建双积分 OCP solver（第一次编译 C 代码；后续调用复用缓存）。"""
    ocp = AcadosOcp()
    m = AcadosModel(); m.name = 'di_form'
    x = ca.MX.sym('x', 6); u = ca.MX.sym('u', 3)
    m.x = x; m.u = u
    m.f_expl_expr = ca.vertcat(x[3], x[4], x[5], u[0], u[1], u[2])
    ocp.model = m

    Q_d = np.array([Q_POS, Q_POS, Q_POS, Q_VEL, Q_VEL, Q_VEL])
    ocp.cost.cost_type   = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.Vx   = np.vstack([np.eye(6),    np.zeros((3, 6))])
    ocp.cost.Vu   = np.vstack([np.zeros((6, 3)), np.eye(3)])
    ocp.cost.Vx_e = np.eye(6)
    ocp.cost.W    = np.diag(np.concatenate([Q_d, np.full(3, R_ACC)]))
    ocp.cost.W_e  = np.diag(Q_d * Q_TERM)
    ocp.cost.yref   = np.zeros(9)
    ocp.cost.yref_e = np.zeros(6)

    ocp.constraints.lbu    = np.full(3, -MAX_ACC)
    ocp.constraints.ubu    = np.full(3,  MAX_ACC)
    ocp.constraints.idxbu  = np.arange(3)
    ocp.constraints.x0     = np.zeros(6)

    ocp.solver_options.N_horizon       = N
    ocp.solver_options.tf              = N * DT
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver       = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx  = 'GAUSS_NEWTON'
    ocp.solver_options.print_level     = 0
    ocp.code_export_directory          = BUILD_DIR

    return AcadosOcpSolver(ocp, json_file=os.path.join(BUILD_DIR, 'ocp.json'))


# ── leader 轨迹 ───────────────────────────────────────────────────────────────
def make_leader(mode, T):
    t = np.arange(T) * DT
    traj = np.zeros((T, 3))
    if mode == 'hover':
        traj[:] = [0, 0, TARGET_ALT]
    elif mode == 'line':
        spd = 0.5  # scenarios.yaml S2_pair2_line speed
        for i, ti in enumerate(t):
            if   ti < 10.0:               traj[i] = [0,               0, TARGET_ALT]
            elif ti < 10.0 + 20.0 / spd:  traj[i] = [(ti-10.0)*spd,  0, TARGET_ALT]
            else:                          traj[i] = [20.0,            0, TARGET_ALT]
    elif mode == 'circle':
        R, spd = 10.0, 1.5  # scenarios.yaml S3_trio3_circle
        omega = spd / R
        for i, ti in enumerate(t):
            if ti < 10.0:
                traj[i] = [R, 0, TARGET_ALT]
            else:
                th = omega * (ti - 10.0)
                traj[i] = [R * math.cos(th), R * math.sin(th), TARGET_ALT]
    return traj


# ── 仿真主循环 ────────────────────────────────────────────────────────────────
def run_formation(solvers, formation_name, mode, T_SIM_s):
    """
    每步依次为每架无人机独立求解 OCP（各自独立热启）。
    目标 = leader_pos + 队形偏移；无机间耦合代价（与实机主要差异）。
    """
    form    = FORMATIONS[formation_name]
    offsets = form['offsets']
    labels  = form['labels']
    n       = len(offsets)
    T       = int(T_SIM_s / DT)
    leader  = make_leader(mode, T)

    # 初始状态：各机已在首步队形目标位，速度=0
    states = [np.array([*(leader[0] + off), 0.0, 0.0, 0.0]) for off in offsets]

    per = {lb: dict(t=[], px=[], py=[], pz=[], poserr=[], solve_ms=[])
           for lb in labels}
    form_err, min_sp, t_log = [], [], []

    for k in range(T):
        lp      = leader[k]
        targets = [lp + off for off in offsets]
        new_st  = []

        for i, (lb, st, tgt) in enumerate(zip(labels, states, targets)):
            slv = solvers[i]
            yr  = np.concatenate([tgt, np.zeros(3), np.zeros(3)])
            yre = np.concatenate([tgt, np.zeros(3)])
            slv.set(0, 'lbx', st); slv.set(0, 'ubx', st)
            for j in range(N): slv.set(j, 'yref', yr)
            slv.set(N, 'yref', yre)

            t0 = time.perf_counter()
            slv.solve()
            ms = (time.perf_counter() - t0) * 1e3

            u = np.clip(slv.get(0, 'u'), -MAX_ACC, MAX_ACC)
            v_new = st[3:6] + u * DT
            spd = np.linalg.norm(v_new[:2])
            if spd > MAX_SPD: v_new[:2] *= MAX_SPD / spd
            p_new = st[:3] + st[3:6] * DT + 0.5 * u * DT**2
            ns = np.concatenate([p_new, v_new])
            new_st.append(ns)

            per[lb]['t'].append(k * DT); per[lb]['px'].append(ns[0])
            per[lb]['py'].append(ns[1]);  per[lb]['pz'].append(ns[2])
            per[lb]['poserr'].append(float(np.linalg.norm(ns[:3] - tgt)))
            per[lb]['solve_ms'].append(ms)

        states = new_st
        t_log.append(k * DT)
        form_err.append(max(float(np.linalg.norm(states[i][:3] - targets[i])) for i in range(n)))
        spacings = [float(np.linalg.norm(states[i][:3] - states[j][:3]))
                    for i in range(n) for j in range(i+1, n)]
        min_sp.append(min(spacings))

    for lb in labels:
        for key in per[lb]: per[lb][key] = np.array(per[lb][key])
    return dict(per=per, labels=labels,
                t=np.array(t_log), leader=leader,
                formation_max_err=np.array(form_err),
                min_spacing=np.array(min_sp))


# ── 绘图 ──────────────────────────────────────────────────────────────────────
def plot_formation(logs, title, fname):
    per = logs['per']; labels = logs['labels']; t = logs['t']
    fig, axes = plt.subplots(3, 2, figsize=(13, 11))
    fig.suptitle(title, fontsize=12, fontweight='bold')

    # 1. XY 轨迹
    ax = axes[0, 0]
    for i, lb in enumerate(labels):
        ax.plot(per[lb]['px'], per[lb]['py'], color=COLORS[i], lw=1.0, label=lb)
        ax.scatter(per[lb]['px'][0],  per[lb]['py'][0],  color=COLORS[i], marker='o', s=45, zorder=5)
        ax.scatter(per[lb]['px'][-1], per[lb]['py'][-1], color=COLORS[i], marker='s', s=45, zorder=5)
    ax.plot(logs['leader'][:, 0], logs['leader'][:, 1], 'k--', lw=0.8, alpha=0.45, label='leader ref')
    ax.set_xlabel('X [m] NED'); ax.set_ylabel('Y [m] NED')
    ax.set_title('XY Trajectory  (○start  ■end)')
    ax.legend(fontsize=7); ax.set_aspect('equal', 'datalim'); ax.grid(True, alpha=0.3)

    # 2. 高度
    ax = axes[0, 1]
    for i, lb in enumerate(labels):
        ax.plot(t, -per[lb]['pz'], color=COLORS[i], lw=1.0, label=lb)
    ax.axhline(-TARGET_ALT, color='k', ls='--', lw=0.8, label=f'target {-TARGET_ALT:.0f} m')
    ax.set_xlabel('t [s]'); ax.set_ylabel('Altitude [m]')
    ax.set_title('Altitude (NED z → +up)'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # 3. 队形最大误差
    ax = axes[1, 0]; fm = logs['formation_max_err']; half = len(t) // 2
    ax.plot(t, fm, color='tab:red', lw=1.2)
    ax.fill_between(t, 0, fm, alpha=0.12, color='tab:red')
    mu = np.mean(fm[half:])
    ax.axhline(mu,  color='tab:red', ls='--', lw=0.8, label=f'mean(2nd half) {mu:.2f} m')
    ax.axhline(0.5, color='gray',    ls=':',  lw=0.8, label='0.5 m')
    ax.set_xlabel('t [s]'); ax.set_ylabel('Error [m]')
    ax.set_title('Formation Max Error'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # 4. 最小间距
    ax = axes[1, 1]; ms = logs['min_spacing']
    ax.plot(t, ms, color='tab:green', lw=1.2)
    ax.fill_between(t, 0, ms, alpha=0.08, color='tab:green')
    ax.axhline(D_SAFE, color='red', ls='--', lw=0.9, label=f'd_safe = {D_SAFE} m')
    ax.set_xlabel('t [s]'); ax.set_ylabel('Spacing [m]')
    ax.set_title('Minimum Inter-drone Spacing'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # 5. 逐机跟踪误差
    ax = axes[2, 0]
    for i, lb in enumerate(labels):
        ax.plot(t, per[lb]['poserr'], color=COLORS[i], lw=1.0, label=lb)
    ax.axhline(0.5, color='gray', ls='--', lw=0.8, label='0.5 m')
    ax.set_xlabel('t [s]'); ax.set_ylabel('pos_err [m]')
    ax.set_title('Per-drone Position Error'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # 6. 求解时间
    ax = axes[2, 1]
    for i, lb in enumerate(labels):
        ax.plot(t, per[lb]['solve_ms'], color=COLORS[i], lw=0.8, alpha=0.8, label=lb)
    ax.axhline(12.0, color='red', ls='--', lw=0.8, label='12 ms budget')
    ax.set_xlabel('t [s]'); ax.set_ylabel('ms')
    ax.set_title('acados Solve Time per drone'); ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = os.path.join(OUTDIR, fname)
    fig.savefig(out, dpi=150); plt.close(fig)
    return out


# ── 判定 ──────────────────────────────────────────────────────────────────────
def print_verdict(logs, label):
    t = logs['t']; half = len(t) // 2
    fm = logs['formation_max_err']; ms = logs['min_spacing']
    all_solve = np.concatenate([logs['per'][lb]['solve_ms'] for lb in logs['labels']])
    print(f"  [{label}]")
    print(f"    form_err  2nd-half mean={np.mean(fm[half:]):.3f} m  max={np.max(fm[half:]):.3f} m")
    print(f"    min_spacing  min={np.min(ms):.3f} m  (d_safe={D_SAFE} m)")
    print(f"    solve  mean={np.mean(all_solve):.3f} ms  max={np.max(all_solve):.3f} ms")
    ok_form  = np.mean(fm[half:]) < 0.5
    ok_space = np.min(ms) >= D_SAFE
    ok_solve = np.max(all_solve) < 12.0
    if not ok_form:  print(f"    ⚠️  form_err {np.mean(fm[half:]):.3f} m ≥ 0.5 m")
    if not ok_space: print(f"    ⚠️  min_spacing {np.min(ms):.3f} m < d_safe {D_SAFE} m")
    if not ok_solve: print(f"    ⚠️  solve max {np.max(all_solve):.2f} ms ≥ 12 ms")
    ok = ok_form and ok_space and ok_solve
    print(f"    VERDICT: {'PASS ✅' if ok else 'REVIEW ⚠️'}")
    return ok


# ── main ─────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print(f"Building {MAX_DRONES} independent acados solvers (C code compiled once)...")
    solvers = []
    for i in range(MAX_DRONES):
        solvers.append(build_solver())
        print(f"  solver {i+1}/{MAX_DRONES} ready")

    results = []
    for formation, mode, tsim, fname, title in SCENARIOS:
        print(f"\n{'='*60}")
        print(f"  {formation} · {mode}  ({tsim}s, {int(tsim/DT)} steps)")
        logs = run_formation(solvers, formation, mode, tsim)
        out  = plot_formation(logs, title, fname)
        ok   = print_verdict(logs, f"{formation}-{mode}")
        results.append((f"{formation}-{mode}", ok, out))

    print(f"\n{'='*60}")
    print("SUMMARY")
    for name, ok, out in results:
        print(f"  {name:30s}  {'PASS ✅' if ok else 'REVIEW ⚠️'}  → {os.path.basename(out)}")
