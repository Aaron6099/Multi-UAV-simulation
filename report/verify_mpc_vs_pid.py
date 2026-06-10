#!/usr/bin/env python3
"""
verify_mpc_vs_pid.py — MPC vs PID 阶跃响应对比图
  - PID: 用报告 §5 记载的串级增益，纯 Python 仿真（线性化，Vz 阶跃场景）
  - MPC: 读取 T2 已生成的 mpc_step_response 数据（重跑 solo1-line），对比跟踪误差/速度
输出: report/figures/mpc_vs_pid_compare.png
"""
import os, sys, math, time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ACADOS_SRC = os.path.expanduser('~/acados')
os.environ.setdefault('ACADOS_SOURCE_DIR', ACADOS_SRC)
sys.path.insert(0, os.path.join(ACADOS_SRC, 'interfaces', 'acados_template'))

from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
import casadi as ca

OUTDIR = os.path.join(os.path.dirname(__file__), 'figures')
os.makedirs(OUTDIR, exist_ok=True)
BUILD_DIR = '/tmp/acados_verify_step'

# ── MPC 参数（与 mpc_node.py 一致）─────────────────────────────────────────
N_MPC   = 20;  DT = 0.05
Q_POS=4.0; Q_VEL=2.0; R_ACC=0.1; Q_TERM=2.0
MAX_ACC=4.0; MAX_SPD=3.0

# ── PID 增益（来自报告 §5 表格）──────────────────────────────────────────────
# 串级：速度→倾角→推力，简化为 1D 纵向 (x 轴)
# Forward velocity PD: P=10, D=0.1, N=20
# Vel→angle PID: P=3, I=0.2
# 等效离散化: 用 Forward-Euler, Ts=0.05s
class PID1D:
    def __init__(self, P, I, D, N, sat=None, Ts=DT):
        self.P=P; self.I=I; self.D=D; self.N=N; self.sat=sat; self.Ts=Ts
        self._int=0.0; self._prev=0.0; self._filt=0.0
    def step(self, err):
        # Derivative with filter: D*N/(1+N*Ts) approx
        alpha = self.N * self.Ts
        deriv = (self.D * self.N * (err - self._prev)) / (1 + alpha)
        self._filt = self._filt * (1 - alpha) + deriv * alpha
        self._int += err * self.Ts
        out = self.P * err + self.I * self._int + self._filt
        if self.sat:
            out = max(-self.sat, min(self.sat, out))
        self._prev = err
        return out

def simulate_pid_1d(leader_x, dt=DT):
    """
    1D 串级 PID 仿真: leader_x position reference → position tracking
    内环: 速度→加速度指令  外环: 位置→速度指令
    """
    # 外环位置 PD (Forward position: P=1, D=0.741, N=1.14)
    pos_ctrl = PID1D(P=1, I=0, D=0.741, N=1.14)
    # 内环速度 PD (Forward velocity: P=10, D=0.1, N=20) + 角度环 (P=3, I=0.2)
    vel_ctrl = PID1D(P=10, I=0, D=0.1, N=20)
    ang_ctrl = PID1D(P=3, I=0.2, D=0, N=100)

    px = 0.0; vx = 0.0
    t_log=[]; px_log=[]; vx_log=[]; ax_log=[]; err_log=[]

    for k, lx in enumerate(leader_x):
        # 外环 → 速度指令
        v_ref = pos_ctrl.step(lx - px)
        v_ref = max(-MAX_SPD, min(MAX_SPD, v_ref))
        # 内环 → 加速度指令 (经倾角环等效)
        ax = vel_ctrl.step(v_ref - vx)
        ax = ang_ctrl.step(ax)     # vel→angle→accel 等效链
        ax = max(-MAX_ACC, min(MAX_ACC, ax))
        # 积分（简单质点模型，无旋翼动力学）
        vx = vx + ax * dt
        vx = max(-MAX_SPD, min(MAX_SPD, vx))
        px = px + vx * dt
        t_log.append(k * dt)
        px_log.append(px); vx_log.append(vx); ax_log.append(ax)
        err_log.append(abs(px - lx))

    return dict(t=np.array(t_log), px=np.array(px_log), vx=np.array(vx_log),
                ax=np.array(ax_log), poserr=np.array(err_log))

def get_mpc_1d(leader_x):
    """Run MPC solo1-line in 1D (just x axis, z fixed at -5)"""
    ocp = AcadosOcp()
    model = AcadosModel(); model.name = 'di_cmp'
    x = ca.MX.sym('x', 6); u = ca.MX.sym('u', 3)
    model.x=x; model.u=u
    model.f_expl_expr = ca.vertcat(x[3],x[4],x[5],u[0],u[1],u[2])
    ocp.model = model
    ocp.cost.cost_type='LINEAR_LS'; ocp.cost.cost_type_e='LINEAR_LS'
    ocp.cost.Vx   = np.vstack([np.eye(6), np.zeros((3,6))])
    ocp.cost.Vu   = np.vstack([np.zeros((6,3)), np.eye(3)])
    ocp.cost.Vx_e = np.eye(6)
    Q_d = np.array([Q_POS,Q_POS,Q_POS,Q_VEL,Q_VEL,Q_VEL])
    ocp.cost.W   = np.diag(np.concatenate([Q_d, np.full(3, R_ACC)]))
    ocp.cost.W_e = np.diag(Q_d * Q_TERM)
    ocp.cost.yref   = np.zeros(9); ocp.cost.yref_e = np.zeros(6)
    ocp.constraints.lbu=np.full(3,-MAX_ACC); ocp.constraints.ubu=np.full(3,MAX_ACC)
    ocp.constraints.idxbu=np.arange(3); ocp.constraints.x0=np.zeros(6)
    ocp.solver_options.N_horizon=N_MPC; ocp.solver_options.tf=N_MPC*DT
    ocp.solver_options.integrator_type='ERK'
    ocp.solver_options.nlp_solver_type='SQP_RTI'
    ocp.solver_options.qp_solver='PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx='GAUSS_NEWTON'
    ocp.solver_options.print_level=0
    ocp.code_export_directory = '/tmp/acados_cmp'

    solver = AcadosOcpSolver(ocp, json_file='/tmp/acados_cmp/ocp.json')
    xs = np.zeros(6); xs[2]=-5.0  # start at z=-5

    t_l=[]; px_l=[]; vx_l=[]; ax_l=[]; err_l=[]; ms_l=[]
    for k, lx in enumerate(leader_x):
        ref = np.array([lx, 0, -5.0, 0, 0, 0, 0, 0, 0])
        ref_e= ref[:6]
        solver.set(0,'lbx',xs); solver.set(0,'ubx',xs)
        for i in range(N_MPC): solver.set(i,'yref',ref)
        solver.set(N_MPC,'yref',ref_e)
        t0=time.perf_counter(); solver.solve(); ms=( time.perf_counter()-t0)*1e3
        uc=solver.get(0,'u')
        uc=np.clip(uc,-MAX_ACC,MAX_ACC)
        vn=xs[3:6]+uc*DT; spd=np.linalg.norm(vn[:2])
        if spd>MAX_SPD: vn[:2]*=MAX_SPD/spd
        pn=xs[:3]+xs[3:6]*DT+0.5*uc*DT**2
        xs=np.concatenate([pn,vn])
        t_l.append(k*DT); px_l.append(xs[0]); vx_l.append(xs[3])
        ax_l.append(uc[0]); err_l.append(abs(xs[0]-lx)); ms_l.append(ms)

    return dict(t=np.array(t_l),px=np.array(px_l),vx=np.array(vx_l),
                ax=np.array(ax_l),poserr=np.array(err_l),solve_ms=np.array(ms_l))

# ── 领队轨迹：同 verify_mpc_step.py line mode ────────────────────────────────
T_SIM=30.0; T=int(T_SIM/DT)
t_arr = np.arange(T)*DT
leader_x = np.where(t_arr<5.0, 0.0,
           np.where(t_arr<25.0, (t_arr-5.0)*1.0, 20.0))

print("Simulating PID...")
pid = simulate_pid_1d(leader_x)

print("Building MPC solver for comparison...")
os.makedirs('/tmp/acados_cmp', exist_ok=True)
mpc = get_mpc_1d(leader_x)

# ── 对比图 ────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(13, 8))
fig.suptitle('MPC vs PID — 1D Position Step Response Comparison\n'
             '(leader: 0→20 m at 1 m/s, then hold)', fontsize=12)

# Tracking error
axes[0,0].plot(pid['t'], pid['poserr'], label='PID (cascaded)', color='tab:orange', lw=1.5)
axes[0,0].plot(mpc['t'], mpc['poserr'], label='MPC (acados, N=20)', color='tab:blue', lw=1.5)
axes[0,0].axhline(0.5, ls='--', c='gray', lw=0.8, label='0.5 m threshold')
axes[0,0].set_title('Tracking error  pos_err [m]')
axes[0,0].set_ylabel('pos_err [m]'); axes[0,0].legend(fontsize=8); axes[0,0].grid(True,alpha=.3)

# Position vs leader
axes[0,1].plot(pid['t'], leader_x, 'k--', lw=1, label='leader')
axes[0,1].plot(pid['t'], pid['px'], label='PID', color='tab:orange', lw=1.5)
axes[0,1].plot(mpc['t'], mpc['px'], label='MPC', color='tab:blue', lw=1.5)
axes[0,1].set_title('X position [m]')
axes[0,1].set_ylabel('x [m]'); axes[0,1].legend(fontsize=8); axes[0,1].grid(True,alpha=.3)

# Speed
axes[1,0].plot(pid['t'], np.abs(pid['vx']), label='PID |vx|', color='tab:orange', lw=1.5)
axes[1,0].plot(mpc['t'], np.abs(mpc['vx']), label='MPC |vx|', color='tab:blue', lw=1.5)
axes[1,0].axhline(MAX_SPD, ls='--', c='red', lw=0.8, label=f'max_speed {MAX_SPD}m/s')
axes[1,0].set_title('Speed  |vx| [m/s]')
axes[1,0].set_ylabel('m/s'); axes[1,0].legend(fontsize=8); axes[1,0].grid(True,alpha=.3)
axes[1,0].set_xlabel('t [s]')

# Acceleration / control effort
axes[1,1].plot(pid['t'], pid['ax'], label='PID ax', color='tab:orange', lw=1.5)
axes[1,1].plot(mpc['t'], mpc['ax'], label='MPC ax', color='tab:blue', lw=1.5)
axes[1,1].axhline(MAX_ACC, ls='--', c='red', lw=0.8, label=f'±{MAX_ACC}m/s²')
axes[1,1].axhline(-MAX_ACC, ls='--', c='red', lw=0.8)
axes[1,1].set_title('Control effort  ax [m/s²]')
axes[1,1].set_ylabel('m/s²'); axes[1,1].legend(fontsize=8); axes[1,1].grid(True,alpha=.3)
axes[1,1].set_xlabel('t [s]')

fig.tight_layout(rect=(0,0,1,0.95))
out = os.path.join(OUTDIR, 'mpc_vs_pid_compare.png')
fig.savefig(out, dpi=150); plt.close(fig)

# ── 指标摘要 ─────────────────────────────────────────────────────────────────
half = T // 2
for name, d in [('PID', pid), ('MPC', mpc)]:
    settle = next((d['t'][i] for i in range(T) if d['poserr'][i]<0.5), None)
    print(f"[{name}]  settle={settle:.1f}s  "
          f"2nd-half mean_err={np.mean(d['poserr'][half:]):.3f}m  "
          f"max_speed={np.max(np.abs(d['vx'])):.2f}m/s  "
          + (f"solve={np.mean(d['solve_ms']):.2f}ms" if 'solve_ms' in d else "solve=<0.01ms (PID)"))

print(f"\nFigure saved: {out}")
