# MuJoCo + Nav2 差速底盘

用真实 Nav2 planner/controller 验证 `examples/nav2/Navigator` 的 sender/receiver 任务组合。程序直接使用 MuJoCo C SDK，无界面运行；轮速执行器通过接触与摩擦驱动底盘，位置由物理仿真产生。

`mujoco_bridge` 接收 `/cmd_vel`，发布 `/clock`、`/odom`、TF、`/scan` 和 `/map`。`mujoco_navigator` 等待 Nav2 生命周期进入 ACTIVE，然后调用 `ComputePathToPose` / `FollowPath`，每秒重规划一次。结束或取消后，任务排空，再关闭 Nav2，最后关闭仿真。

第一版使用真值里程计和固定的 `map → odom`，尚未引入 AMCL 或 SLAM。地图由同一 MJCF 场景中的静态、轴对齐盒子生成。模型约定两轮半径一致、轮轴沿 Y、执行器 gear=1；当前 `scene.xml` 满足这些约定。

## 构建

本机验证版本：ROS 2 Jazzy、Nav2 1.3.13、MuJoCo 3.8.0。MuJoCo 头文件与动态库必须版本一致。还需要本项目的 lexec/co2 源码、CMake、Ninja 和 uv。

本机未安装完整 Nav2，下面的脚本从已配置的 Jazzy APT 源下载依赖并提取到 `build/nav2`，不修改系统安装。它使用当前 APT 候选版本，记录到 `versions.json`；依赖解析以当前系统已安装的软件包为基础。

```bash
cd /home/lvyues/code/rclexec/lrclexec
source /opt/ros/jazzy/setup.bash
UV_CACHE_DIR=/tmp/lrclexec-uv-cache uv run --no-project --no-managed-python \
  simulation/mujoco/fetch_nav2.py --directory build/nav2

MUJOCO_PREFIX=/home/lvyues/rebot_env/isaacsim/exts/isaacsim.pip.newton/pip_prebundle/mujoco
cmake --preset debug \
  -DLRCLEXEC_LEXEC_SOURCE_DIR=/home/lvyues/code/exec/lexec \
  -DLRCLEXEC_CO2_SOURCE_DIR=/home/lvyues/code/coro/coro \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DLRCLEXEC_BUILD_NAV2_EXAMPLE=ON \
  -DLRCLEXEC_BUILD_MUJOCO_SIM=ON \
  -DLRCLEXEC_MUJOCO_ROOT="$MUJOCO_PREFIX" \
  -DCMAKE_PREFIX_PATH="$PWD/build/nav2/root/opt/ros/jazzy" \
  -DLRCLEXEC_NAV2_RUNTIME_PREFIX="$PWD/build/nav2/root/opt/ros/jazzy"
cmake --build --preset debug -j3
ctest --preset debug -R '^mujoco_'
```

`LRCLEXEC_BUILD_MUJOCO_SIM` 默认关闭；仿真目标只在构建树中使用，不进入通用库的安装导出。若已有完整 Nav2，可将两个 Nav2 前缀参数改为对应的 Jazzy 安装目录。MuJoCo SDK 可通过 `LRCLEXEC_MUJOCO_ROOT` 或 `mujoco::mujoco` CMake 包提供；本机路径指向已有 SDK，仅使用其中的头文件和动态库。

ASan/UBSan 使用相同配置参数，将 preset 改为 `asan-clang`，然后运行 `cmake --build --preset asan-clang -j3` 和 `ctest --preset asan-clang`。该配置需要 Clang；MuJoCo 3.8.0 的 sanitizer 头文件含 GCC 13 不接受的属性位置。检查覆盖本项目 C++ 代码；预编译的 MuJoCo/Nav2 依赖没有重新插桩。

## 运行和检查

也可以直接运行回归入口：

```bash
UV_CACHE_DIR=/tmp/lrclexec-uv-cache uv run --no-project --no-managed-python \
  simulation/mujoco/run.py --build build/debug \
  --nav2-prefix build/nav2/root/opt/ros/jazzy \
  --logs build/mujoco-runs --case all
```

| 场景 | 检查 |
| --- | --- |
| `straight` | 到达 `(1, 0)`，Action 成功、任务排空、底盘停稳 |
| `obstacle` | 目标 `(4, 0)`；地图隐藏中间障碍物，由激光写入 costmap 后规划并实际绕行 |
| `cancel` | 检测到实际运动后向客户端发 SIGINT，等待 STOPPED、排空和停稳 |
| `startup-cancel` | 不启动 Nav2，在客户端等待生命周期时发 SIGINT，正常退出 |

三个运动场景均检查全程无墙体/障碍物碰撞。成功场景还检查物理位置误差小于 0.15 m、航向误差小于 0.18 rad；绕障场景要求侧向位移超过 0.55 m。独立的 `mujoco_physics` 测试检查轮子前进、转向、零速度制动、激光距离和倾斜射线。

每个场景保存节点日志、`physics.jsonl`、`result.json` 和 `exit.json`。只有 Action、物理状态与进程关闭检查全部通过才写入 `status: passed`；强制终止进程会使测试失败。脚本只管理自己创建的进程组，清理阶段忽略重复中断，保留时钟直到客户端排空和 Nav2 退出。底盘另有 300 ms 指令看门狗。

默认 DDS 域为 220；`all` 对后续场景递增域号。可用 `--domain` 改为其他空闲域；同一输出目录避免同时运行多个回归。
