# report/data — 仿真飞行 CSV / flight CSVs

放从 Ubuntu 回传的 `diag_monitor.py --log` 生成的飞行 CSV。
Put the flight CSVs (from `diag_monitor.py --log`) copied back from the Ubuntu host here.

- 命名 / naming：`flight_<formation>_<traj>[_<变体>].csv`
  例 / e.g.：`flight_cross5_line.csv`、`flight_grid9_circle.csv`、`flight_cross5_line_v2.5.csv`
- 列 / columns（每秒一行 / one row per second）：
  `t`,
  每机 per‑UAV `d{i}_x, d{i}_y, d{i}_z, d{i}_zerr, d{i}_velxy, d{i}_arm, d{i}_nav, d{i}_mpc, d{i}_solve_ms, d{i}_fallback, d{i}_hover, d{i}_poserr`,
  汇总 `min_spacing, formation_max_err, safety_violations, total_fallbacks, max_solve_ms, leader_x, leader_y, leader_vx, leader_vy`
  > `d{i}_x / d{i}_y`（世界系 NED 北/东）与 `leader_*` 为本阶段新增，用于俯视轨迹图。
- 出图 / plot：`py report/make_figures.py report/data/<csv> --out report/figures`
- 文本体检 / quick text report：`py analyze_flight.py report/data/<csv>`

> CSV 通常很小（每秒一行），可随仓库提交；如不想入库，把 `report/data/*.csv` 加进 `.gitignore`。
