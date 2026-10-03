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

## Qt / MuJoCo 交互工作台

工作台将 MuJoCo 嵌入一个 Qt 主窗口。需要 Qt 5.15 的 Gui、Widgets 开发包；启用测试时还需要 Qt Test。它使用 C++，不依赖 PyQt 或 GLFW。完成基础构建配置后：

```bash
source /opt/ros/jazzy/setup.bash
cmake --preset debug -DLRCLEXEC_MUJOCO_QT=ON
cmake --build --preset debug -j3
UV_CACHE_DIR=/tmp/lrclexec-uv-cache uv run --no-project --no-managed-python \
  simulation/mujoco/run.py --build build/debug \
  --nav2-prefix build/nav2/root/opt/ros/jazzy \
  --logs build/mujoco-runs/interactive --executor multi --gui
```

默认不启动 RViz；需要查看地图、激光和代价地图时添加 `--rviz`。RViz 使用随源码保存的 `navigation.rviz`，每次复制到日志目录，保存视图不会覆盖源码。`--gui` 与独立 GLFW 窗口的 `--view` 互斥；工作台在 Single/Multi 下都使用所选的桥和导航 executor。

左侧管理任务点顺序和场景障碍物，中间显示 MuJoCo，右侧编辑所选对象。点击“＋ 任务点”后，在俯视场景点击位置，或输入 X/Y/朝向，再应用修改；上移、下移和删除改变任务草稿。蓝色箭头表示每点最终朝向，蓝色虚线只表示任务顺序，绿色实线表示 Nav2 实际规划路径。3D 观察支持左拖旋转、右拖平移和滚轮缩放。

“开始任务”按顺序逐点到达，最多 64 点；执行时锁定任务点。“停止任务”清空等待任务并取消当前导航，草稿保留，排空后可编辑重发。任一点失败会停止后续点，可重试原任务的剩余点。取消应答不代表物理已经停稳。关闭工作台或可选 RViz 窗口，入口先排空导航、检查停稳，再关闭其余进程。

“＋ 障碍物”添加固定 0.8 × 0.6 × 0.6 m 的轴对齐箱体，最多 16 个；也可选择已有动态箱体修改 X/Y 或删除。导航中仍可操作。半透明预览不参与物理，只有物理端确认且场景同步后才显示为已应用。越界、与机器人、墙或其他箱体重叠的修改会被拒绝并保留预览；等待确认时锁定对象选择。原有墙体和静态箱体不可编辑。

动态箱体同时进入物理碰撞、激光与渲染，不写入静态地图、不重置机器人；Nav2 通过激光更新代价地图并重规划。移走或删除箱体后，激光清除相应代价，清除存在扫描和代价地图更新延迟。退出不保存任务草稿或箱体布局。

### 会话接口

这些接口用于本仓库的仿真程序，不属于通用库安装导出：

| 接口 | 消息和语义 |
| --- | --- |
| `/navigation/route` | `geometry_msgs/PoseArray`，整个顺序任务一次验证并入队 |
| `/navigation/route_result` | `std_msgs/String` JSON，回显请求 stamp、accepted 和会话状态；Qt 据此关联自己的提交 |
| `/goal_pose` | RViz 标准 `geometry_msgs/PoseStamped`，视为单点任务；新任务请求取消旧任务，排空后启动最新等待任务 |
| `/navigation/cancel` | `std_srvs/Trigger`，取消当前及等待任务 |
| `/navigation/status` | JSON 状态和任务计数，包含当前点、总点数和已完成点数 |
| `/navigation/active_route`、`/navigation/active_goal` | 最后启动的任务快照与当前目标，transient-local |
| `/navigation/path` | `nav_msgs/Path`，当前实际规划路径，任务结束时清空 |
| `/simulation/edit_obstacle` | `visualization_msgs/Marker`，map 坐标系，id 0–15，ADD 为固定尺寸 CUBE，DELETE 删除；ns 和 stamp 关联请求 |
| `/simulation/edit_result` | JSON 回显 client、request_sec/nanosec、slot、applied、revision 和 reason |
| `/simulation/scene` | JSON 物理快照，version、time、revision、qpos 和 mocap_pos，约 33 Hz、transient-local |

目标仅接受 `map` 中有限的平面位姿，非零航向四元数会归一化；初始化完成前拒收。回调只入队或请求停止，主线程等待 sender 完成并销毁 operation 后才交接下一任务；每次只保留当前任务和最新等待任务。远端不返回 Action 终态时仍可能无法完成排空，入口会将强制回收记录为失败。当前直接组合 planner/controller，不提供 `NavigateToPose` Action 或 BT navigator。

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
| `workbench` | 三点顺序到达、中途失败停止后续点、导航中添加箱体后绕行、删除后代价清除、多点取消与退出 |

三个运动场景均检查全程无墙体/障碍物碰撞。成功场景还检查物理位置误差小于 0.15 m、航向误差小于 0.18 rad；绕障场景要求侧向位移超过 0.55 m。独立的 `mujoco_physics` 测试检查轮子前进、转向、零速度制动、激光距离和倾斜射线。

`--case session` 使用与 GUI 相同的目标 topic 和取消 service，检查对应目标终态、物理位置、FollowPath 活动目标峰值为 1，以及退出后的任务计数平衡。独立 `mujoco_goal_inbox` 测试固定旧任务尚未排空的边界，验证中间等待目标被覆盖、取消清空等待目标和空闲关闭。控制器位置到达容差为 0.05 m，为定位误差留出余量；物理验收门槛仍为 0.15 m。

每个场景保存节点日志、`physics.jsonl`、`result.json` 和 `exit.json`；AMCL 模式另存 `localization.jsonl`。只有 Action、物理状态与进程关闭检查全部通过才写入 `status: passed`；强制终止进程会使测试失败。脚本只管理自己创建的进程组，清理阶段忽略重复中断，保留时钟直到客户端排空和 Nav2 退出。底盘另有 300 ms 指令看门狗。

默认 DDS 域为 220；`all` 对后续场景递增域号。可用 `--domain` 改为其他空闲域；同一输出目录避免同时运行多个回归。

## AMCL 与执行器验收

`--executor single|multi` 选择桥和客户端的实际 executor，默认 single。Multi 使用四线程 `spin()`；桥的物理状态仍由同一个互斥 callback group 保护。Nav2/AMCL 独立进程继续使用各自原有 executor。`--view --executor multi` 时，客户端仍为 Multi，桥固定为 Single，保证 GLFW 渲染留在主线程。

客户端通过 `/set_initial_pose` 注入 `(0.15, -0.10, 0.08 rad)` 的小幅偏差，位置协方差各为 `0.09 m²`，航向协方差为 `0.04 rad²`。随后调用 `/request_nomotion_update` 支持静止初始化；收到至少 10 个新鲜 AMCL 样本、位置协方差之和低于 `0.02 m²`、航向协方差低于 `0.02 rad²`，且 `map → base_link` TF 可用后才开始导航。初始化最多等待 20 秒。

定位误差评分不参与控制或就绪判断。桥保留最近 20 秒物理历史，按 AMCL 消息的激光时间戳精确匹配，记录估计、真值、误差与协方差。三个运动场景要求就绪后的每个定位样本误差不超过 `0.10 m / 0.10 rad`，同时保留原有物理目标、无碰撞、反馈和排空门槛。结果同时记录初始化全过程最大误差及最后一次定位误差；最后一次定位样本不一定与停稳时间相同。

CTest 持久覆盖 AMCL 四个单目标场景、持续会话和工作台场景 × Single/Multi，以及真值模式直线兼容检查。启动取消不要求 AMCL 样本；持续会话验证交互和任务交接，逐样本 AMCL 误差门槛仍由三个单目标运动场景检查。当前验收仍基于给定静态地图、无噪声激光和真值里程计，不能替代带里程计误差的定位验证。

Qt 控件另有两种检查，需要可用的 X11 显示，可在独立 Xvfb 中执行：

```bash
mkdir -p build/acceptance/qt-controls
build/debug/simulation/mujoco/gui/mujoco_gui_test \
  simulation/mujoco/scene.xml build/acceptance/qt-controls
# 复用上面的 --gui 命令，添加 --gui-check：真实 Nav2 + Qt 自动交互验收
# 可同时添加 --rviz；测试退出后，入口继续验证任务排空和物理停稳。
```

`mujoco_gui_test` 使用真实物理和模拟导航状态，验证控件、重排、重叠拒绝、延迟确认期间的选择锁定和场景拾取，保存常规与最小尺寸截图。`--gui --gui-check` 使用真实 ROS 客户端、物理和 Nav2，通过 Qt 控件发送多点任务并检查最终位置/朝向、箱体添加/移动/删除、取消和任务中关窗；截图保存在该次日志目录。它要求同时启用 `LRCLEXEC_BUILD_TESTS` 和 `LRCLEXEC_MUJOCO_QT`。

`mujoco_gui_client` 不需要显示，使用真实 ROS 通信固定请求关联、取消服务延迟出现，以及场景和编辑回执先后到达的边界。

2026-10-03 工作台验收：Debug 主回归 40/40，加上新增 GUI 协议测试 1/1；Clang ASan/UBSan 下通过物理、任务队列和 GUI 协议三项检查，以及 Single/Multi 的 `workbench` 真实导航场景。Qt 控件检查通过；独立 Xvfb 中的 Single 默认工作台和 Multi 加 `--rviz` 两组真实交互均通过，分别有 7/8 个子进程退出码为零，无强制回收。日志和截图位于 `build/acceptance/qt-workbench/`。预编译 ROS/Nav2/MuJoCo 和系统 Qt 库未重新插桩。
