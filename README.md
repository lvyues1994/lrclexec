# lrclexec

用 C++17 的 [lexec](https://github.com/lvyues1994/lexec) 将 ROS 2 定时器、Action、Service、Topic 和任务生命周期接入 sender/receiver 组合。第一版依据 *Senders, Receivers, and Robots* 的思路实现，在本机 ROS 2 Jazzy（rclcpp / rclcpp_action 28.1.22）上验证。

## 构建

依赖 CMake 3.25+、Ninja、ROS 2 Jazzy 和 lexec 源码。测试及基础示例额外使用 `example_interfaces`，ROS 时间回归使用 `rosgraph_msgs`，Topic 回归使用 `std_msgs`，生命周期回归使用 `rclcpp_lifecycle`；信号及安装回归使用 uv 执行标准库 Python 脚本。当前接入并由 CI 固定的 lexec 提交为 `e032d551d53d87209b83d07fd172890461db643b`。

```bash
cd /home/lvyues/code/rclexec/lrclexec
source /opt/ros/jazzy/setup.bash
cmake --preset debug -DLRCLEXEC_LEXEC_SOURCE_DIR=/home/lvyues/code/exec/lexec \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build --preset debug -j3
ctest --preset debug
```

ASan / UBSan 使用独立的构建目录：

```bash
cmake --preset asan -DLRCLEXEC_LEXEC_SOURCE_DIR=/home/lvyues/code/exec/lexec \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build --preset asan -j3
ctest --preset asan
```

这里显式选择 ROS 的系统 Python，避免 ament 调用缺少 `catkin_pkg` 的个人 Python 环境。

作为子项目使用时，链接 `lrclexec::lrclexec`。父项目可以预先提供 `lexec::lexec`，或设置 `LRCLEXEC_LEXEC_SOURCE_DIR`；子项目默认关闭测试、示例和安装。

## 安装与 colcon

项目提供 ament 包和可搬移的 CMake 导出：

```bash
cmake --install build/debug --prefix "$PWD/build/install"
```

消费方将该安装前缀加入 `CMAKE_PREFIX_PATH`，然后使用 `find_package(lrclexec CONFIG REQUIRED)` 和 `target_link_libraries(app PRIVATE lrclexec::lrclexec)`。

当前 lexec 尚无安装导出规则，因此从源码构建时默认安装其核心头文件到 `include/lrclexec/vendor`，不安装 lexec runtime。若消费方预先提供 `lexec::lexec`，导出目标会使用该 provider；否则使用随包头文件。整个程序应使用同一个兼容的 lexec provider。若选择 `LRCLEXEC_BUNDLE_LEXEC_HEADERS=OFF`，安装构建需要可由 `find_package(lexec CONFIG)` 找到的 lexec 包。

在当前目录也可以直接构建 ROS 包：

```bash
colcon --log-base build/colcon-log build --paths . \
  --build-base build/colcon --install-base build/colcon-install \
  --cmake-args -DLRCLEXEC_LEXEC_SOURCE_DIR=/home/lvyues/code/exec/lexec \
  -DPython3_EXECUTABLE=/usr/bin/python3
source build/colcon-install/setup.bash
ros2 run lrclexec navigation_example
```

`package.xml` 声明示例使用的消息依赖，其中 Nav2 消息列为运行和测试依赖；默认 CMake 构建不查找它。通用库本身只链接 rclcpp、rclcpp_action、Threads 和 lexec。启用 Nav2 示例时需另行提供 nav2_msgs 与 co2 的构建依赖。项目及上游 lexec 尚未确定许可证，包元数据暂记为 `Unspecified`。

## API

| 入口 | 行为 |
| --- | --- |
| `TimerScheduler{node}` | 创建 executor 队列及专用互斥 callback group；副本共享这两项资源 |
| `TimerScheduler{lifecycleNode}` | 使用 LifecycleNode 的原生节点接口，并持有完整节点；通用库无需链接 rclcpp_lifecycle |
| `scheduler.nodeInterfaces()` | 取得 base、clock、logging、parameters、timers、topics、waitables 接口，兼容两种节点 |
| `TimerScheduler{node, TimerClock::node}` | 定时器使用节点时钟；`use_sim_time=true` 时跟随 `/clock` |
| `lexec::schedule(scheduler)` | 在该 executor 上异步完成 |
| `schedule_after(scheduler, duration)` | 使用所选时钟等待，默认是 steady wall timer；非正时长也经队列异步完成 |
| `call_service(scheduler, client, request)` | 异步等待服务可用并发送请求，返回 `Service::Response::SharedPtr` |
| `wait_message<Message>(scheduler, topic, qos = rclcpp::QoS{1})` | 每次启动独立订阅，返回首条收到的消息副本 `shared_ptr<Message const>` |
| `execute_action(scheduler, client, goal, options = {})` | 从发送 goal 等到远端终态，可观察 feedback 和取消应答 |
| `make_action_server_preempt<Action>(scheduler, scope, name, factory)` | 将返回 sender 的工厂接为 ActionServer；保留最新等待目标，先排空旧任务再启动新任务 |
| `server.close()` | 拒绝新目标、终止等待目标并停止当前任务；关闭任务也由 scope 跟踪 |
| `spin_with_scope(executor, scope, stopToken)` | 收到外部停止后关闭 scope、请求停止，继续 spin 到 join，然后返回或重抛 executor 异常 |
| `SignalStop{stopSource}` | 在普通线程接收 SIGINT/SIGTERM 并请求停止；排空期间继续接收重复信号 |

头文件分别为 `TimerScheduler.h`、`Service.h`、`Topic.h`、`ExecuteAction.h`、`ActionServer.h`、`SpinWithScope.h`、`SignalStop.h`，均位于 `lrclexec/` 下。

普通 Node 的构造、`scheduler.node()` 返回类型及行为保留；访问器现可抛异常，移除了 `noexcept`。对 LifecycleNode 调度器调用 `node()` 会抛 `logic_error`，应使用 `nodeInterfaces()` 或应用持有的 LifecycleNode。自定义 `ExecutionContext` 默认通过 `node()` 提供接口；包装 LifecycleNode 时须覆盖 `nodeInterfaces()` 并转发内部接口。

LifecycleNode 的 unconfigured、inactive 状态允许这些普通定时器、订阅、Service 和 Action 继续执行；适配层不自动注册生命周期回调。若 deactivate/shutdown 要停止业务，应用须显式请求 stop、关闭并排空 scope，之后释放资源；重新激活时创建新的任务 scope。不要在依赖同一 executor 的生命周期回调里阻塞等待 join。

节点时钟定时器沿用 ROS 原生跳变规则：到期前暂停 `/clock` 会暂停等待，前跳越过截止时刻可立即到期；后跳早于 timer 的 `last_call_time` 时从新时间重新计时。首条 `/clock` 前节点时间为零，首次时间跳变也可能触发到期。停止请求和 `schedule()` 的队列投递不依赖时钟推进。

`call_service` 的 request 按值传入，服务发现每 20 ms 按墙钟检查，支持发现期间取消。停止或 `when_any` 超时会移除本次 pending request 并完成本地等待；ROS Service 不支持取消远端执行，迟到响应会被忽略。需要截止时长时，与墙钟 `schedule_after` 组合；SDK 或队列异常走 `set_error(std::exception_ptr)`。

`wait_message` 在 `start` 后创建订阅，默认 QoS 为可靠、volatile、深度 1，可传入 `SensorDataQoS` 或 transient-local 配置。首条消息指 SDK 首次交给回调的消息：QoS 可能丢弃积压消息，也可能提供历史消息。结果复制并独立持有消息内存，兼容 DDS 借用消息；只为首条候选排队一次完成通知。完成闭包运行前 stop 仍可赢，停止或 `when_any` 超时会释放订阅；SDK/队列异常走 `set_error(std::exception_ptr)`。节点及 executor 仍须存活，ROS graph 在 executor 释放临时引用后更新。

Action 客户端的完成通道：

| ROS 结果 | Sender 完成 |
| --- | --- |
| `SUCCEEDED` | `set_value(Action::Result::SharedPtr)` |
| goal 被拒绝 | `set_error(ActionError<Action>{rejected, ...})` |
| `ABORTED` | `set_error(ActionError<Action>{aborted, result})`，保留结果载荷 |
| `CANCELED` | `set_stopped()` |
| 未知结果码 | `set_error(ActionError<Action>{unknownResult, result})` |
| SDK / 队列异常 | `set_error(std::exception_ptr)` |

停止请求发生在发送前时，不发送 goal；发生在接受前时，等到拿到 handle 后发送取消。取消应答只说明请求被处理，sender 仍等待终态。取消与成功竞争时，以服务端最终结果为准；`when_any` 因此会等待落败 Action 清理结束。

`ActionOptions<Action>::feedback` 接收 `shared_ptr<Action::Feedback const>`，`cancelResponse` 接收 SDK 的取消应答，可检查 `return_code`。两者经 executor 队列投递，只在 Action 尚未结束时通知；终态先到时可能跳过迟到的取消应答。回调抛异常会请求取消，等待远端终态后报告异常；sender 完成前会等待正在执行的观察回调退出。Action 应答投递失败时保留应答并最多重投一次，避免在 SDK 锁内调用用户代码；队列持续失败时无法保证继续执行或有界 join。

服务端工厂返回 `set_value(Result::SharedPtr)` 表示成功；error 或 stopped 表示终止。客户端取消映射为 `CANCELED`，服务端抢占映射为 `ABORTED`。旧 sender 的 operation 析构后才调用下一目标的工厂；关闭 scope 会终止尚未启动的目标。工厂内部应使用 sender 表达异步清理，及时响应 stop token。

## 最小接入

```cpp
#include <lrclexec/ExecuteAction.h>
#include <lrclexec/SpinWithScope.h>

// node 和 client 由应用创建，Action 是应用自己的 ROS Action 类型。
auto scheduler = lrclexec::TimerScheduler{node};
auto scope = lexec::counting_scope{};
auto stop = lexec::inplace_stop_source{};
auto work = lrclexec::execute_action(scheduler, client, goal)
    | lexec::then([&](auto result) noexcept {
        // 使用结果。
        stop.request_stop();
    })
    | lexec::upon_error([&](auto const &error) noexcept {
        // 记录 ActionError 或 exception_ptr。
        stop.request_stop();
    })
    | lexec::upon_stopped([&]() noexcept { stop.request_stop(); });
lexec::spawn(std::move(work), scope.get_token());
executor.add_node(node);
lrclexec::spin_with_scope(executor, scope, stop.get_token());
// join 完成后才 shutdown ROS。
```

Linux/POSIX 程序在入口先创建停止源和 `SignalStop`，再初始化 ROS、创建工作线程：

```cpp
auto stop = lexec::inplace_stop_source{};
auto signals = lrclexec::SignalStop{stop};
auto options = rclcpp::InitOptions{};
options.shutdown_on_signal = false;
rclcpp::init(argc, argv, options, rclcpp::SignalHandlerOptions::None);
// 创建节点与任务，spin_with_scope 排空后调用 rclcpp::shutdown()。
```

`SignalStop` 每个进程只能有一个，并且必须在构造线程析构。停止源须活得更久，信号资源须覆盖 join、ROS shutdown 和节点析构。它无法改变已有工作线程的信号 mask。

停止源、scope、ActionServer 以及用户回调借用的对象都必须活到任务收束。`server.close()` 是显式异步关闭；销毁包装对象不能代替关闭和 join。

本机 Jazzy 的原生 Action client/server 析构会移除 callback group 中的 waitable；与 executor 收集实体并发时可能自锁。应用须持有客户端、服务端及其借用对象，到 executor 停止且 spin 线程 join 之后再释放。

节点必须加入持续运行的 executor；LifecycleNode 使用 `executor.add_node(scheduler.nodeInterfaces().get_node_base_interface())`。支持标准 `SingleThreadedExecutor` / `MultiThreadedExecutor`，以及默认单线程、同线程执行定时器的实验性 `rclcpp::experimental::executors::EventsExecutor`。不要在 executor 回调里对依赖同一 executor 的 sender 调用阻塞 `sync_wait`，也不要同时对一个 executor 调用 `spin` 和 `spin_with_scope`。收束期间 ROS context 必须保持有效。远端若拒绝取消或不返回终态，join 会继续等待。

EventsExecutor 回归使用默认 `SimpleEventsQueue` 和 steady/wall 定时器。Jazzy 的 [ROS 仿真时钟问题](https://github.com/ros2/rclcpp/issues/2480) 尚不在支持范围内；默认事件队列也不保证内存有界。本机 28.1.22 还复现了原生定时器的手动 spin 边界：取消后仍持有堆顶 timer，可能阻挡其他 timer，释放它后恢复。使用 `spin_with_scope` 时应及时释放应用取消的原生 timer；本库单次等待会取消并释放自己的 timer。

## 导航模拟与测试

```bash
ROS_DOMAIN_ID=212 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ./build/debug/examples/navigation_example
```

`examples/navigation.cpp` 用本地 Fibonacci Action 验证基础组合。可选的 `examples/nav2` 使用标准 `ComputePathToPose` / `FollowPath` 消息及本地模拟服务器：先规划，再并行跟随路径与延时重规划；新路径完成后取消并排空旧控制任务，然后开始下一轮。

启用 Nav2 示例需要 `nav2_msgs`（本地版本 1.3.13）和 co2 源码（本地提交 `a265e577`）。当前机器的消息包位于一个独立 overlay：

```bash
NAV2_PREFIX=/home/lvyues/code/exec_extend/rclexec/deps/jazzy-root/opt/ros/jazzy
cmake --preset debug -DLRCLEXEC_BUILD_NAV2_EXAMPLE=ON \
  -DLRCLEXEC_CO2_SOURCE_DIR=/home/lvyues/code/coro/coro \
  -DCMAKE_PREFIX_PATH="$NAV2_PREFIX"
cmake --build --preset debug -j3
ctest --preset debug
LD_LIBRARY_PATH="$NAV2_PREFIX/lib:$LD_LIBRARY_PATH" \
  ROS_DOMAIN_ID=216 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ./build/debug/examples/nav2/nav2_navigation_example
```

导航逻辑使用 co2 的 `while` 协程循环，避免递归 sender 链随重规划次数累计。它检查 Action 终态与 Nav2 `error_code`，保留阶段、协议错误种类、业务错误码和消息；遇到错误直接收束。规划器、控制器和检查器的插件 ID 由 `Resources` 配置。

[MuJoCo 差速底盘示例](simulation/mujoco/README.md) 将同一导航逻辑接入真实 Nav2 planner/controller，使用轮子接触动力学、激光和真值里程计完成导航，支持无界面运行和可选原生窗口。可选开关 `LRCLEXEC_BUILD_MUJOCO_SIM=ON` 默认关闭。回归检查直达、激光发现障碍后绕行、运动中取消及启动阶段取消，并核对物理停稳和进程正常退出；构建和运行步骤见示例说明。

回归覆盖定时器到期/取消竞争、接受前取消、取消被拒后成功、远端暂不返回终态时 join 等待、feedback 异常及完成竞态、拒绝与 abort 载荷、超时分支排空、投递失败和恢复回调重入 ROS 客户端、独占资源析构、等待目标覆盖/取消、服务端关闭及 scope join。Service 回归检查取消、超时、迟到响应和 pending request 清理；ROS 时间回归检查暂停、前后跳变及暂停时取消。还检查正常退出与重复信号、移动安装目录、外部 lexec provider、重复及兄弟目录 `find_package`。Nav2 回归执行至少 300 轮跟随，检查控制器峰值为 1、客户端路径资源有界，以及初次规划、等待重规划、重规划进行中的取消。终态在 ROS 中异步传输，成功返回前可能已接受下一次规划，因此计划数可以略多于跟随次数。

Topic 回归分别使用单线程、四线程及 EventsExecutor，并检查普通 DDS 与进程内通信、首条消息、独立订阅、取消/超时、晚到闭包、创建/投递异常、transient-local/SensorDataQoS，以及借用消息在回调结束后的所有权。LifecycleNode 回归检查完整节点的所有权、各生命周期状态下的通信、取消清理及标准 executor 下的 ROS 时钟；EventsExecutor 额外检查队列注册/重新注册、批内异常恢复、Service 迟到响应、scope 异步清理及重复信号退出。安装消费回归包含独立 LifecycleNode 应用。

基础 CTest 使用本机 DDS 域 211–219 和 221–227；MuJoCo/Nav2 回归使用域 220。客户端路径资源的界限不代表整个 ROS/DDS 进程内存恒定：服务端会在超时前缓存终态结果。

## CI

[GitHub Actions 配置](.github/workflows/ci.yml) 在 push、pull request 或手动触发时执行三个 Ubuntu 24.04 / Jazzy 作业：GCC Debug + Fast DDS、Clang ASan/UBSan + Fast DDS、GCC Debug + Cyclone DDS。Actions、lexec、uv 和 ROS APT 源配置包固定版本；作业执行通用库回归及移动安装目录后的消费测试，Nav2/MuJoCo 单独在本地回归。
