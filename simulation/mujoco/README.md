# MuJoCo + Nav2 差速底盘

用真实 Nav2 planner/controller 验证 `examples/nav2/Navigator` 的 sender/receiver 任务组合。程序直接使用 MuJoCo C SDK，默认无界面运行，可启用原生窗口；轮速执行器通过接触与摩擦驱动底盘，位置由物理仿真产生。

`mujoco_bridge` 接收 `/cmd_vel`，发布 `/clock`、`/odom`、TF、`/scan` 和 `/map`。`mujoco_navigator` 用 Service sender 先等 AMCL ACTIVE、设置初始位姿并等待定位/TF 就绪，再等 planner/controller ACTIVE，然后调用 `ComputePathToPose` / `FollowPath`，每秒仿真时间重规划一次。启动查询与超时使用墙钟，导航重规划使用节点 ROS 时钟；客户端统计 FollowPath feedback，成功回归要求收到进度。结束或取消后，任务排空，再关闭 Nav2，最后关闭仿真。

默认使用 AMCL 提供 `map → odom`，`--localization truth` 可保留固定变换兼容模式。当前 `/odom` 与 `odom → base_link` 仍来自物理真值，尚未引入轮编码器里程计或 SLAM。地图由同一 MJCF 场景中的静态、轴对齐盒子生成。模型约定两轮半径一致、轮轴沿 Y、执行器 gear=1；当前 `scene.xml` 满足这些约定。

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

## MuJoCo 窗口

完成上面的构建后，下载可选 GLFW 依赖并启用窗口：

```bash
UV_CACHE_DIR=/tmp/lrclexec-uv-cache uv run --no-project --no-managed-python \
  simulation/mujoco/fetch_nav2.py --directory build/nav2 --viewer
cmake --preset debug -DLRCLEXEC_MUJOCO_VIEWER=ON \
  -DLRCLEXEC_GLFW_ROOT="$PWD/build/nav2/root/usr"
cmake --build --preset debug -j3
UV_CACHE_DIR=/tmp/lrclexec-uv-cache uv run --no-project --no-managed-python \
  simulation/mujoco/run.py --build build/debug \
  --nav2-prefix build/nav2/root/opt/ros/jazzy \
  --logs build/mujoco-runs/gui --view --case obstacle
```

窗口显示蓝色底盘、橙色障碍和绿色目标。左键拖动旋转视角，右键拖动平移，滚轮缩放。导航完成后窗口保持打开；按 Esc 或关闭窗口退出。若运动中关闭，脚本先取消并排空 Action、等待停稳，再关闭 Nav2 和仿真。`--view` 默认选择绕障场景，也可指定 `--case straight`；每次显示一个场景。

渲染在主线程读取物理状态快照，使用独立 MuJoCo Data；窗口交互只改变相机。GLFW/MuJoCo GPU 资源在窗口销毁前释放。可视化回归日志中另存最新的 `viewer.ppm` 画面。本机窗口验证使用 X11 `DISPLAY=:1`、GLFW 3.3.10。

## RViz 交互导航

启用上面的 MuJoCo 窗口构建后，用同一入口打开 RViz、Qt 控制窗和仿真：

```bash
source /opt/ros/jazzy/setup.bash
UV_CACHE_DIR=/tmp/lrclexec-uv-cache uv run --no-project --no-managed-python \
  simulation/mujoco/run.py --build build/debug \
  --nav2-prefix build/nav2/root/opt/ros/jazzy \
  --logs build/mujoco-runs/interactive --executor multi --gui
```

GUI 额外需要 `rviz2`、`rclpy`、`python_qt_binding` 和 PyQt5。本机已经安装；控制窗默认使用 `/usr/bin/python3`，可用 `--ros-python` 指定具有这些模块的 Python。配置文件 `navigation.rviz` 随源码保存，每次启动复制到日志目录，RViz 保存视图不会覆盖源码配置。

等待控制窗显示“就绪”，在 RViz 工具栏选择 **2D Goal Pose**，在地图上按住并拖动，设置目的地和最终朝向。地图、激光、代价地图、规划路径与最后启动的目标会同时显示；MuJoCo 的绿色标记也跟随最后启动的目标。到达后会话保持运行，可以继续设置目标。

运动中再次点选会请求停止旧任务；完整排空后才启动最新等待目标。控制窗的“取消当前目标”会清空等待目标并请求停止当前任务，取消应答不代表已经停稳。成功、取消或规划失败后均可重新发送目标。关闭任一窗口或点击“结束仿真”，入口先排空导航、检查物理停稳，再关闭其余进程。

输入是 RViz 标准 `/goal_pose`（`geometry_msgs/PoseStamped`）；仅接受 `map` 坐标系中有限的平面位姿，非零航向四元数会归一化。初始化完成前拒收目标。`/navigation/cancel` 使用 `std_srvs/Trigger`，`/navigation/status` 发布 JSON 状态和目标编号，`/navigation/active_goal` 发布最后启动的目标。当前入口直接组合 planner/controller，不提供 `NavigateToPose` Action 或 BT navigator。

会话仅保留当前任务和最新等待目标。executor 回调只入队或请求停止，主线程等待导航 sender 完成并销毁 operation 后才交接下一目标；原有单目标模式保持默认。远端不返回 Action 终态时仍可能无法完成排空，入口的强制回收会记录为失败。

## 运行和检查

也可以直接运行回归入口：

```bash
UV_CACHE_DIR=/tmp/lrclexec-uv-cache uv run --no-project --no-managed-python \
  simulation/mujoco/run.py --build build/debug \
  --nav2-prefix build/nav2/root/opt/ros/jazzy \
  --logs build/mujoco-runs/multi --executor multi --case all
```

| 场景 | 检查 |
| --- | --- |
| `straight` | 到达 `(1, 0)`，Action 成功、任务排空、底盘停稳 |
| `obstacle` | 目标 `(4, 0)`；地图隐藏中间障碍物，由激光写入 costmap 后规划并实际绕行 |
| `cancel` | 检测到实际运动后向客户端发 SIGINT，等待 STOPPED、排空和停稳 |
| `startup-cancel` | 不启动 Nav2，在客户端等待生命周期时发 SIGINT，正常退出 |
| `session` | 连续目标及朝向、取消停稳后重发、非法目标、规划失败恢复、最新目标替换、运动中退出排空 |

三个运动场景均检查全程无墙体/障碍物碰撞。成功场景还检查物理位置误差小于 0.15 m、航向误差小于 0.18 rad；绕障场景要求侧向位移超过 0.55 m。独立的 `mujoco_physics` 测试检查轮子前进、转向、零速度制动、激光距离和倾斜射线。

`--case session` 使用与 GUI 相同的目标 topic 和取消 service，检查对应目标终态、物理位置、FollowPath 活动目标峰值为 1，以及退出后的任务计数平衡。独立 `mujoco_goal_inbox` 测试固定旧任务尚未排空的边界，验证中间等待目标被覆盖、取消清空等待目标和空闲关闭。控制器位置到达容差为 0.05 m，为定位误差留出余量；物理验收门槛仍为 0.15 m。

每个场景保存节点日志、`physics.jsonl`、`result.json` 和 `exit.json`；AMCL 模式另存 `localization.jsonl`。只有 Action、物理状态与进程关闭检查全部通过才写入 `status: passed`；强制终止进程会使测试失败。脚本只管理自己创建的进程组，清理阶段忽略重复中断，保留时钟直到客户端排空和 Nav2 退出。底盘另有 300 ms 指令看门狗。

默认 DDS 域为 220；`all` 对后续场景递增域号。可用 `--domain` 改为其他空闲域；同一输出目录避免同时运行多个回归。

## AMCL 与执行器验收

`--executor single|multi` 选择桥和客户端的实际 executor，默认 single。Multi 使用四线程 `spin()`；桥的物理状态仍由同一个互斥 callback group 保护。Nav2/AMCL 独立进程继续使用各自原有 executor。`--view --executor multi` 时，客户端仍为 Multi，桥固定为 Single，保证 GLFW 渲染留在主线程。

客户端通过 `/set_initial_pose` 注入 `(0.15, -0.10, 0.08 rad)` 的小幅偏差，位置协方差各为 `0.09 m²`，航向协方差为 `0.04 rad²`。随后调用 `/request_nomotion_update` 支持静止初始化；收到至少 10 个新鲜 AMCL 样本、位置协方差之和低于 `0.02 m²`、航向协方差低于 `0.02 rad²`，且 `map → base_link` TF 可用后才开始导航。初始化最多等待 20 秒。

定位误差评分不参与控制或就绪判断。桥保留最近 20 秒物理历史，按 AMCL 消息的激光时间戳精确匹配，记录估计、真值、误差与协方差。三个运动场景要求就绪后的每个定位样本误差不超过 `0.10 m / 0.10 rad`，同时保留原有物理目标、无碰撞、反馈和排空门槛。结果同时记录初始化全过程最大误差及最后一次定位误差；最后一次定位样本不一定与停稳时间相同。

CTest 持久覆盖 AMCL 四个单目标场景及持续会话 × Single/Multi，以及真值模式直线兼容检查。启动取消不要求 AMCL 样本；持续会话验证交互和任务交接，逐样本 AMCL 误差门槛仍由三个单目标运动场景检查。当前验收仍基于给定静态地图、无噪声激光和真值里程计，不能替代带里程计误差的定位验证。

2026-10-03 的交互版本验收中，Debug 与 Clang ASan/UBSan 的相关仿真回归各通过 13/13。四组持续会话均验证 FollowPath 活动目标峰值为 1，退出后 active/pending 为零，所有子进程正常退出。结果位于 `build/{debug,asan-clang}/mujoco-runs/{single,multi}/amcl/session/`。另在独立 Xvfb 显示中实际操作 RViz 点选、Qt 取消和结束按钮：一次到达、取消后重发、转向中关闭均完成收束，8 个进程退出码为零；截图和日志位于 `build/acceptance/gui-xvfb/`。
