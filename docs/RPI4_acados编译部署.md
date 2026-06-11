# RPi 4B acados 编译部署（真机机载电脑）

前提：RPi 4B（建议 4GB+ 内存）、Ubuntu 22.04 Server **arm64**、已联网、风扇散热已装。
全程 SSH 操作，约 30–50 分钟（含编译）。

## 0. 系统准备

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git python3-pip libblas-dev liblapack-dev

# CPU 锁性能模式（避免编译降频 + 控制循环 50Hz 抖动）
sudo apt install -y cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils
```

2GB 内存版必须先加 swap（4GB/8GB 跳过）：

```bash
sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## 1. 克隆 acados

```bash
cd ~
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
```

> 版本一致性：建议与 x86 仿真机同版本。在仿真机 `cd ~/acados && git log -1 --format=%H`
> 拿到哈希，RPi 上 `git checkout <hash> && git submodule update --recursive --init`。

## 2. CMake 配置（关键：BLASFEO target）

RPi 4 是 Cortex-A72，BLASFEO 没有 A72 专属 target，**选 `ARMV8A_ARM_CORTEX_A57`**（同代架构，社区验证最优）：

```bash
mkdir -p build && cd build
cmake .. \
  -DACADOS_WITH_QPOASES=ON \
  -DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57 \
  -DHPIPM_TARGET=GENERIC \
  -DACADOS_INSTALL_DIR=$HOME/acados
```

若后续编译/运行有非法指令(SIGILL)等异常，退回保守 target 重配：
`-DBLASFEO_TARGET=GENERIC`（慢 2–3 倍但必稳，solve 预算内仍够 trio3）。

## 3. 编译安装（RPi4 约 10–20 分钟）

```bash
make install -j4        # 2GB 内存用 -j2，防 OOM
```

## 4. Python 接口

```bash
pip3 install -e ~/acados/interfaces/acados_template
pip3 install numpy casadi pyyaml matplotlib    # casadi 有 aarch64 wheel，直接装
```

## 5. t_renderer（⚠️ ARM64 最大的坑）

acados 生成 C 代码依赖 `t_renderer` 二进制。x86 上会自动下载，但下载的是
**x86_64 版**，在 RPi 上报 `Exec format error`。必须手动放 arm64 版：

```bash
# 先去 https://github.com/acados/tera_renderer/releases 确认最新版号
wget https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux-arm64 \
     -O ~/acados/bin/t_renderer
chmod +x ~/acados/bin/t_renderer
~/acados/bin/t_renderer --help   # 能打印用法 = 架构对了
```

若该版本没有 arm64 资产，用 Rust 源码编译（~5 分钟）：

```bash
sudo apt install -y cargo
git clone https://github.com/acados/tera_renderer ~/tera_renderer
cd ~/tera_renderer && cargo build --release
cp target/release/t_renderer ~/acados/bin/
```

## 6. 环境变量

```bash
cat >> ~/.bashrc <<'EOF'
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib
EOF
source ~/.bashrc
```

## 7. 自检（acados 官方例子）

```bash
cd ~/acados/examples/acados_python/getting_started
python3 minimal_example_ocp.py
```

首跑会现编 OCP C 代码（RPi 上比 x86 慢，耐心等）。正常 = 打印求解状态无报错。

## 8. 项目 benchmark（部署门槛）

```bash
cd ~ && git clone https://github.com/Aaron6099/Multi-UAV-simulation.git
cd Multi-UAV-simulation
python3 report/verify_mpc_step.py      # 单机阶跃：看 solve mean/max
python3 report/verify_formation.py     # pair2/trio3 编队：更接近实机负载
```

**判定：solve max < 15 ms 才允许上机**（50Hz 周期 20ms，留 25% 余量）。
参考：x86 SITL 实测 0.13–0.32ms；RPi 4 预估 trio3 约 2–5ms，应能过。

## 9. 常见坑速查

| 症状 | 原因 | 处理 |
|---|---|---|
| `Exec format error` | t_renderer 是 x86 版 | 步骤 5 换 arm64 版 |
| 编译中途被 killed | OOM | `-j4`→`-j2` 或加 swap |
| `import acados_template` 失败 | pip 装错解释器 / 环境变量缺 | 确认 `pip3` 与 `python3` 同源；重 source bashrc |
| 运行时找不到 `.so` | LD_LIBRARY_PATH 未生效 | 重登 SSH 再试 |
| 非法指令 SIGILL | BLASFEO target 不兼容 | 步骤 2 退 GENERIC 重编 |
| solve 偶发尖刺 | CPU 降频 | 锁 performance + 查温度（见下） |

温度监控（Ubuntu 上没有 vcgencmd，用 sysfs）：

```bash
watch -n2 'echo $(($(cat /sys/class/thermal/thermal_zone0/temp)/1000))°C'
# 持续 >80°C 会降频 → 检查风扇
```

## 10. 下一步（acados 通过后）

1. **ROS2 Humble**：按官方 apt 源装 `ros-humble-ros-base`（Server 版不需要 desktop）
2. **px4_msgs + 本包**：
   ```bash
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
   git clone https://github.com/PX4/px4_msgs.git -b release/1.14
   git clone https://github.com/Aaron6099/Multi-UAV-simulation.git mpc_control
   cd ~/ros2_ws && colcon build --symlink-install
   ```
3. **MicroXRCEAgent**（real_hardware_launch.py 依赖，可执行名必须是 `MicroXRCEAgent`）：
   ```bash
   git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/xrce_agent
   cd ~/xrce_agent && mkdir build && cd build
   cmake .. && make -j4 && sudo make install && sudo ldconfig
   ```
4. **CycloneDDS + 域号 + 时钟**：
   ```bash
   sudo apt install -y ros-humble-rmw-cyclonedds-cpp chrony
   echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
   echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc      # 全员一致
   ```
5. **CH340 udev 固定名**（`/dev/ttyFC`，免插拔后 ttyUSB0/1 漂移）：
   ```bash
   echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="ttyFC"' \
     | sudo tee /etc/udev/rules.d/99-ch340.rules
   sudo udevadm control --reload && sudo udevadm trigger
   # 之后 launch 用 agent_dev:=/dev/ttyFC
   ```
6. 台架首跑：`ros2 launch mpc_control real_hardware_launch.py drone_id:=0 scenario:=S2_pair2_hover`
   （首次现编 mpc_node 的 OCP，数分钟；**务必台架预编译，别留到外场**）
