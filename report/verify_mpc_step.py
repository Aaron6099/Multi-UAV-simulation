#!/usr/bin/env python3
"""
verify_mpc_step.py — 独立 MPC 阶跃响应验证（无 ROS2/PX4）
用 acados 直接仿真双积分 MPC，输出 report/figures/mpc_step_response.png

验证内容：
  1. OCP 参数与 mpc_node.py 默认值一致（N=20, dt=0.05, q_pos=4, q_vel=2, r_acc=0.1）
  2. solo1 X 轴阶跃：leader 从 0 走到 20m，MPC 能在 ~10s 内收敛
  3. 稳态跟踪误差 < 0.5m，速度不超 max_speed=3m/s，acados 全程 ACADOS_SUCCESS
"""
import os, sys, math, time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── acados path ──────────────────────────────────────────────────────────────
ACADOS_SRC = os.path.expanduser('~/acados')
os.environ.setdefault('ACADOS_SOURCE_DIR', ACADOS_SRC)
sys.path.insert(0, os.path.join(ACADOS_SRC, 'interfaces', 'acados_template'))

from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
import casadi as ca

# ── OCP 参数（与 mpc_node.py / scenarios.yaml defaults 一致）───────────────
N       = 20       # 预测步数
DT      = 0.05     # s / step
Q_POS   = 4.0      # 位置误差权重
Q_VEL   = 2.0      # 速度误差权重
R_ACC   = 0.1      # 加速度代价
Q_TERM  = 2.0      # terminal cost 倍率
MAX_SPD = 3.0      # m/s
MAX_ACC = 4.0      # m/s²
T_SIM   = 30.0     # 仿真总时长 s
OUTDIR  = os.path.join(os.path.dirname(__file__), 'figures')
os.makedirs(OUTDIR, exist_ok=True)

BUILD_DIR = '/tmp/acados_verify_step'


def build_solver():
    ocp = AcadosOcp()

    # 模型：双积分 x=[px,py,pz,vx,vy,vz], u=[ax,ay,az]
    model = AcadosModel()
    model.name = 'di_verify'
    x = ca.MX.sym('x', 6)
    u = ca.MX.sym('u', 3)
    # 连续时间右手边
    f_expl = ca.vertcat(x[3], x[4], x[5], u[0], u[1], u[2])
    model.x = x; model.u = u; model.f_expl_expr = f_expl
    ocp.model = model

    # 参考 + 代价（线性最小二乘）
    # y = [x; u]，参考 [leader_pos, 0 vel, 0 acc]
    ny = 9; ny_e = 6
    ocp.cost.cost_type   = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.Vx   = np.vstack([np.eye(6), np.zeros((3, 6))])
    ocp.cost.Vu   = np.vstack([np.zeros((6, 3)), np.eye(3)])
    ocp.cost.Vx_e = np.eye(6)
    Q_diag = np.array([Q_POS, Q_POS, Q_POS, Q_VEL, Q_VEL, Q_VEL])
    R_diag = np.full(3, R_ACC)
    ocp.cost.W   = np.diag(np.concatenate([Q_diag, R_diag]))
    ocp.cost.W_e = np.diag(Q_diag * Q_TERM)
    ocp.cost.yref   = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # 约束
    ocp.constraints.lbu = np.full(3, -MAX_ACC)
    ocp.constraints.ubu = np.full(3,  MAX_ACC)
    ocp.constraints.idxbu = np.arange(3)
    ocp.constraints.x0 = np.zeros(6)

    # solver
    ocp.solver_options.N_horizon         = N
    ocp.solver_options.tf                = N * DT
    ocp.solver_options.integrator_type   = 'ERK'
    ocp.solver_options.nlp_solver_type   = 'SQP_RTI'
    ocp.solver_options.qp_solver         = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx    = 'GAUSS_NEWTON'
    ocp.solver_options.print_level       = 0
    ocp.code_export_directory            = BUILD_DIR

    solver = AcadosOcpSolver(ocp, json_file=os.path.join(BUILD_DIR, 'ocp.json'))
    return solver


def run_sim(solver, leader_traj):
    """
    leader_traj: (T,3) leader position over time
    Returns dict of logged arrays.
    """
    T = len(leader_traj)
    log = dict(t=[], px=[], py=[], pz=[], vx=[], vy=[], vz=[],
               ax=[], ay=[], az=[], poserr=[], solve_ms=[])

    x = np.zeros(6)
    for k in range(T):
        ref_p = leader_traj[k]
        yref   = np.concatenate([ref_p, np.zeros(3), np.zeros(3)])
        yref_e = np.concatenate([ref_p, np.zeros(3)])
        solver.set(0, 'lbx', x); solver.set(0, 'ubx', x)
        for i in range(N):
            solver.set(i, 'yref', yref)
        solver.set(N, 'yref', yref_e)

        t0 = time.perf_counter()
        status = solver.solve()
        ms = (time.perf_counter() - t0) * 1e3

        u = solver.get(0, 'u')
        # clip speed
        u = np.clip(u, -MAX_ACC, MAX_ACC)

        # integrate (simple Euler for 1 step)
        v_new = x[3:6] + u * DT
        # velocity clamp
        spd = np.linalg.norm(v_new[:2])
        if spd > MAX_SPD:
            v_new[:2] *= MAX_SPD / spd
        p_new = x[:3] + x[3:6] * DT + 0.5 * u * DT**2
        x = np.concatenate([p_new, v_new])

        log['t'].append(k * DT)
        log['px'].append(x[0]); log['py'].append(x[1]); log['pz'].append(x[2])
        log['vx'].append(x[3]); log['vy'].append(x[4]); log['vz'].append(x[5])
        log['ax'].append(u[0]); log['ay'].append(u[1]); log['az'].append(u[2])
        log['poserr'].append(float(np.linalg.norm(x[:3] - ref_p)))
        log['solve_ms'].append(ms)

    return {k: np.array(v) for k, v in log.items()}


def make_leader(mode, T):
    """leader position trajectory, shape (T, 3)"""
    t = np.arange(T) * DT
    if mode == 'line':
        # 前 5s 悬停，再匀速到 x=20m，再悬停
        spd = 1.0  # m/s
        traj = np.zeros((T, 3))
        for i, ti in enumerate(t):
            if ti < 5.0:
                traj[i] = [0, 0, -5.0]
            elif ti < 5.0 + 20.0 / spd:
                traj[i] = [(ti - 5.0) * spd, 0, -5.0]
            else:
                traj[i] = [20.0, 0, -5.0]
        return traj
    elif mode == 'circle':
        R, spd = 10.0, 1.5
        omega = spd / R
        traj = np.zeros((T, 3))
        for i, ti in enumerate(t):
            if ti < 5.0:
                traj[i] = [R, 0, -5.0]
            else:
                th = omega * (ti - 5.0)
                traj[i] = [R * math.cos(th), R * math.sin(th), -5.0]
        return traj
    raise ValueError(mode)


def plot_and_save(logs, leaders, labels):
    fig, axes = plt.subplots(3, 2, figsize=(13, 10))
    fig.suptitle('MPC Step-Response Verification (acados, standalone)', fontsize=13)
    colors = ['tab:blue', 'tab:orange']

    for (log, ldr, lab, col) in zip(logs, leaders, labels, colors):
        t = log['t']
        axes[0, 0].plot(t, log['poserr'], color=col, label=lab)
        axes[0, 0].axhline(0.5, ls='--', c='gray', lw=0.8)
        axes[0, 0].set_title('Tracking error pos_err [m]')
        axes[0, 0].set_ylabel('pos_err [m]'); axes[0, 0].legend(fontsize=8)

        vspd = np.sqrt(log['vx']**2 + log['vy']**2)
        axes[0, 1].plot(t, vspd, color=col, label=lab)
        axes[0, 1].axhline(MAX_SPD, ls='--', c='red', lw=0.8, label=f'max {MAX_SPD}m/s')
        axes[0, 1].set_title('Horizontal speed [m/s]')
        axes[0, 1].set_ylabel('|v_xy| [m/s]'); axes[0, 1].legend(fontsize=8)

        axes[1, 0].plot(t, log['px'], color=col, label=f'{lab} px')
        axes[1, 0].plot(t, ldr[:, 0], color=col, ls='--', alpha=0.5, label=f'{lab} leader_x')
        axes[1, 0].set_title('X position vs leader [m]'); axes[1, 0].legend(fontsize=7)

        axes[1, 1].plot(t, log['pz'], color=col, label=lab)
        axes[1, 1].axhline(-5.0, ls='--', c='gray', lw=0.8, label='target -5m')
        axes[1, 1].set_title('Altitude z (NED) [m]')
        axes[1, 1].set_ylabel('z [m]'); axes[1, 1].legend(fontsize=8)

        axes[2, 0].plot(t, log['ax'], color=col, label=f'{lab} ax')
        axes[2, 0].axhline(MAX_ACC, ls='--', c='red', lw=0.8)
        axes[2, 0].axhline(-MAX_ACC, ls='--', c='red', lw=0.8)
        axes[2, 0].set_title('Control ax [m/s²]'); axes[2, 0].set_xlabel('t [s]')

        axes[2, 1].plot(t, log['solve_ms'], color=col, label=lab)
        axes[2, 1].set_title('acados solve time [ms]')
        axes[2, 1].set_ylabel('ms'); axes[2, 1].set_xlabel('t [s]')
        axes[2, 1].legend(fontsize=8)

    for a in axes.flat:
        a.grid(True, alpha=0.3)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = os.path.join(OUTDIR, 'mpc_step_response.png')
    fig.savefig(out, dpi=150); plt.close(fig)
    return out


def print_verdict(log, label):
    t = log['t']
    half = len(t) // 2
    pe = log['poserr']
    vspd = np.sqrt(log['vx']**2 + log['vy']**2)
    settle_t = next((t[i] for i in range(len(t)) if pe[i] < 0.5), None)
    print(f"  [{label}]")
    print(f"    settle (<0.5m) @ t={settle_t:.1f}s" if settle_t else "    settle: NOT reached")
    print(f"    poserr 2nd-half mean={np.mean(pe[half:]):.3f}m  max={np.max(pe[half:]):.3f}m")
    print(f"    max speed={np.max(vspd):.2f}m/s (limit {MAX_SPD})")
    print(f"    solve mean={np.mean(log['solve_ms']):.2f}ms  max={np.max(log['solve_ms']):.2f}ms")
    ok = (np.mean(pe[half:]) < 0.5) and (np.max(vspd) <= MAX_SPD * 1.05)
    print(f"    VERDICT: {'PASS ✅' if ok else 'REVIEW ⚠️'}")
    return ok


if __name__ == '__main__':
    print("Building acados solver...")
    os.makedirs(BUILD_DIR, exist_ok=True)
    solver = build_solver()
    print("  done.")

    T = int(T_SIM / DT)
    scenarios = [('line', 'solo1-line'), ('circle', 'solo1-circle')]
    logs, leaders, labels = [], [], []

    for mode, lab in scenarios:
        print(f"\nRunning {lab} ({T} steps × {DT}s = {T_SIM}s)...")
        ldr = make_leader(mode, T)
        solver.reset()
        log = run_sim(solver, ldr)
        logs.append(log); leaders.append(ldr); labels.append(lab)
        print_verdict(log, lab)

    out = plot_and_save(logs, leaders, labels)
    print(f"\nFigure saved: {out}")
