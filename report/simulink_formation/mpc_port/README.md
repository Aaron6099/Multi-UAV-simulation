# mpc_port — ROS MPC 完整移植 Simulink 闭环验证

把 `mpc_control/mpc_node.py` 的 MPC **算法本体**完整移植到 MATLAB/Simulink，
在 6DOF+串级 PID 被控对象（与 PID 基线同一植物）上闭环，剥离 ROS/EKF/通信
工程噪声，单独验证算法稳定性。

## 真机部署映射（每文件 = 一个真机检验单元）

| 入口文件 | 机数 | 运动 | 真机对应 |
|---|---|---|---|
| `run_mpc_solo1_hover/line/circle.m` | 1 | 悬停/直线/圆周 | 单机首飞三连 |
| `run_mpc_pair2_hover/line/circle.m` | 2 | 同上 | 双机编队 |
| `run_mpc_trio3_hover/line/circle.m` | 3 | 同上 | 三机编队 |
| `run_mpc_cross5_hover/line/circle.m` | 5 | 同上 | 五机十字编队 |

真机测哪个场景，先跑对应文件确认 `VERDICT: PASS/REVIEW`。
输出：`report/figures/mpc_<formation>_<mode>.png` + 本目录 `results.csv`。

## 移植对照（→ mpc_node.py）

| 本目录 | 移植自 | 说明 |
|---|---|---|
| `formation_cfg.m` | scenarios.yaml defaults+formations | 参数单一真值（N=30, q_vel=2, max_accel=4 等） |
| `leader_state.m` | leader_node | hover/line(端点减速)/circle(向心加速度广播) |
| `mpc_precompute.m` | `DoubleIntegratorMPC._setup_ocp` | 双积分精确离散 + 凝聚矩阵 + 常量约束 |
| `mpc_solve_rti.m` | `DoubleIntegratorMPC.solve` (SQP_RTI) | 单次 GN 迭代；碰撞/编队残差在上拍预测处线性化 |
| `qp_admm.m` | (acados HPIPM 的替身) | OSQP 风格 ADMM；本机无优化工具箱 |
| `mpc_swarm_step.m` | `control_loop` | 参考二阶预测/邻居预测交换(一拍延迟)/速度合成(xy=MPC, z=纯P)/限幅 |
| `build_plant_io.m` `build_swarm_model.m` | — | 母版 slx → IO 化 → n 机闭环模型(50Hz Interpreted MATLAB Fcn) |
| `scenario_run.m` | verify_formation.py 判定口径 | 指标+三级 verdict(PASS/REVIEW/FAIL)+图 |

## 与 ROS 端的已知一致性

line/circle 的稳态 track_err≈0.5–1.1m 为**共模滞后**（form_err≈0，队形完好）——
与 Ubuntu 实跑 trio3 circle 1.129m 同源（速度内环无前馈滞后），判 REVIEW 非缺陷。
编队反馈、碰撞避免、邻居降级（队形推断）行为与 ROS 端同构。

## 注意

- 跑前无需手动 init：`scenario_run` 自动把 `../init.m` 注入 base 工作区。
- 改 `formation_cfg.m` 参数后直接重跑；改模型结构需删 `uav_plant_io.slx` /
  `mpc_swarm_*.slx` 强制重建（或 `build_*( true)`）。
- 与 scenarios.yaml 的参数同步是手动的：改 yaml 记得改 `formation_cfg.m`。
