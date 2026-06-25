# PX4 SITL 本地改动备份（px4_overrides）

本目录固化的是对 **PX4-Autopilot v1.14**（`~/PX4-Autopilot-1.14/`）的本地 SITL 定制改动。
这些改动是多机 grid9 仿真能干净起飞 / 真高一致的必要环境，但它们：

- **不在本项目（`mpc_control`）的正常构建路径里**，`colcon build` 不会带上；
- PX4 仓库虽是 git，但**没有我们自己的远程**，改动只活在本地工作树 / PX4 本地 git，一旦
  误 `git checkout` / 重装 PX4 / 机器故障就会**永久丢失**。

因此在此单独备份（patch + 整文件副本），随本项目 git 一起推到远程，作为唯一可靠备份。

> 生成日期：2026-06-25。base = PX4 v1.14 stock airframe `4001_gz_x500` / world `default.sdf`。

---

## 文件清单

| 文件 | 说明 |
|------|------|
| `4001_gz_x500` | 改好的 airframe **整文件副本**，应急可直接 cp 覆盖 |
| `4001_gz_x500.patch` | airframe 相对 PX4 stock 的 `git diff`（精确改动） |
| `default_sdf_max_step.patch` | `default.sdf` 的 `git diff`（仅 1 行物理步长改动，见下方 ⚠️） |

对应的 PX4 路径：

```
airframe:  ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500
world:     Tools/simulation/gz/worlds/default.sdf
```

---

## 改动一：airframe `4001_gz_x500`（3 个 param override）

均为 `param set-default`，治多机 SITL 起飞 / 长跑可靠性，**与控制器无关**：

| param | 值 | 为什么 |
|-------|----|--------|
| `SIM_BAT_MIN_PCT` | `100` | 仿真电池不掉电。stock `SIM_BAT_DRAIN=60` 会 60s 掉到 50% → 长跑触 "Battery unhealthy" 预检失败 → 挡住 re-arm。钉死 100%。（注：实测此项偏红鲱鱼，留着无害） |
| `COM_OBL_RC_ACT` | `5` | offboard 信号丢失时进 **Hold 悬停**（而非降落），各机自主悬停等恢复 |
| `COM_DISARM_PRFLT` | `0` | **关键修复**。关掉 10s 自动预检解锁砍刀。起飞瞬态 offboard setpoint 断续会把 nav 在地面 OFFBOARD↔Hold 来回弹，默认 10s 砍刀会在飞机爬出去之前 disarm 趴地 → retry 死循环。`0` = 永不自动 disarm，给无限时间爬出去 |

### 应用方法

```bash
PX4=~/PX4-Autopilot-1.14
AIR=ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500

# 方式 A：整文件覆盖（最省事）
cp px4_overrides/4001_gz_x500 "$PX4/$AIR"

# 方式 B：打 patch（保留 PX4 端其它改动时用）
git -C "$PX4" apply /path/to/px4_overrides/4001_gz_x500.patch

# 两种方式都要再同步到 build 副本（改 airframe 无需重编，cp 即可）：
cp "$PX4/$AIR" "$PX4/build/px4_sitl_default/etc/init.d-posix/airframes/4001_gz_x500"
```

---

## 改动二：`default.sdf` 物理步长 ⚠️ 待查

```
<max_step_size>0.004</max_step_size>   →   <max_step_size>0.001</max_step_size>
```

**⚠️ 此改动来历不明，未在任何 STATE / 实验记录里登记，固化时一并保留以免丢失，但需复核：**

- 物理步长从 4ms 缩到 1ms。配合 world 里 `real_time_update_rate=250`，典型 Gazebo 下
  实时率上限 ≈ `0.001 × 250 = 0.25`，即**仿真可能慢约 4 倍**（wall-clock 是 sim time 的 4 倍）。
- run4–run6 的 PASS 结果**很可能就是在此 0.001 步长下取得的**——回退到 stock `0.004` 前，
  应确认是否会改变多机起飞 / 收敛行为。
- **下一步（待决）**：弄清这个 0.001 是有意为数值稳定而改、还是某次调试误留；
  若无依赖则回退 `0.004` 可让仿真提速约 4 倍。

### 应用 / 回退

```bash
PX4=~/PX4-Autopilot-1.14
# 应用本备份（设回 0.001）：
git -C "$PX4" apply /path/to/px4_overrides/default_sdf_max_step.patch
# 回退到 stock 0.004：
git -C "$PX4" checkout -- Tools/simulation/gz/worlds/default.sdf
```
