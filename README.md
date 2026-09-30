# lrclexec

用 C++17 的 [lexec](https://github.com/lvyues1994/lexec) 将 ROS 2 定时器、Action 和任务生命周期接入 sender/receiver 组合。第一版依据 *Senders, Receivers, and Robots* 的思路实现，在本机 ROS 2 Jazzy（rclcpp / rclcpp_action 28.1.22）上验证。

## 构建

依赖 CMake 3.25+、Ninja、ROS 2 Jazzy 和 lexec 源码。测试及示例额外使用 `example_interfaces`，当前接入的本地 lexec 提交为 `23a800b`。

```bash
cd /home/lvyues/code/rclexec/lrclexec
source /opt/ros/jazzy/setup.bash
cmake --preset debug -DLRCLEXEC_LEXEC_SOURCE_DIR=/home/lvyues/code/exec/lexec
cmake --build --preset debug -j3
ctest --preset debug
```

ASan / UBSan 使用独立的构建目录：

```bash
cmake --preset asan -DLRCLEXEC_LEXEC_SOURCE_DIR=/home/lvyues/code/exec/lexec
cmake --build --preset asan -j3
ctest --preset asan
```

作为子项目使用时，链接 `lrclexec::lrclexec`。父项目可以预先提供 `lexec::lexec`，或设置 `LRCLEXEC_LEXEC_SOURCE_DIR`。这是普通 CMake 库；目前没有 colcon 包、安装或导出配置。

## API

| 入口 | 行为 |
| --- | --- |
| `TimerScheduler{node}` | 创建 executor 队列及专用互斥 callback group；副本共享这两项资源 |
| `lexec::schedule(scheduler)` | 在该 executor 上异步完成 |
| `schedule_after(scheduler, duration)` | 使用 ROS wall timer 等待；非正时长也经队列异步完成 |
| `execute_action(scheduler, client, goal)` | 表示从发送 goal 到服务端返回终态的整个操作 |
| `make_action_server_preempt<Action>(scheduler, scope, name, factory)` | 将返回 sender 的工厂接为 ActionServer；保留最新等待目标，先排空旧任务再启动新任务 |
| `server.close()` | 拒绝新目标、终止等待目标并停止当前任务；关闭任务也由 scope 跟踪 |
| `spin_with_scope(executor, scope, stopToken)` | 收到外部停止后关闭 scope、请求停止，继续 spin 到 join，然后返回或重抛 executor 异常 |

头文件分别为 `TimerScheduler.h`、`ExecuteAction.h`、`ActionServer.h`、`SpinWithScope.h`，均位于 `lrclexec/` 下。

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

应用初始化 ROS 时使用 `rclcpp::SignalHandlerOptions::None`，将退出意图接入外部 stop source，并在 join 后调用 `rclcpp::shutdown()`。停止源、scope、ActionServer 以及用户回调借用的对象都必须活到任务收束。`server.close()` 是显式异步关闭；销毁包装对象不能代替关闭和 join。

节点必须加入持续运行的 executor。支持标准 `SingleThreadedExecutor` / `MultiThreadedExecutor`；不要在 executor 回调里对依赖同一 executor 的 sender 调用阻塞 `sync_wait`，也不要同时对一个 executor 调用 `spin` 和 `spin_with_scope`。收束期间 ROS context 必须保持有效。远端若拒绝取消或不返回终态，join 会继续等待。

## 导航模拟与测试

```bash
ROS_DOMAIN_ID=212 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ./build/debug/examples/navigation_example
```

`examples/navigation.cpp` 用本地 Fibonacci Action 模拟规划器和控制器：跟随当前路径时并行等待再规划；新路径产生后取消并排空旧控制任务，然后跟随新路径，最终到达目标。它验证组合及生命周期语义，尚未连接 Nav2 或真实机器人。

集成测试覆盖定时器到期/取消竞争、接受前取消、拒绝与 abort 载荷、超时分支排空、投递失败、独占资源析构、等待目标覆盖/取消、服务端关闭及 scope join。CTest 使用本机 DDS 域 211 / 212。

目前只提供 Action 结果通道；feedback、Service、Topic、ROS LifecycleNode、仿真时间定时器和实验性 EventsExecutor 尚未适配。
