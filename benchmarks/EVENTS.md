# Jazzy EventsExecutor 动态订阅复现

本机 rclcpp 28.1.22 的 EventsExecutor 在销毁订阅并创建同地址的新订阅时，可能保留失效实体缓存，漏装新订阅的消息通知。`events_subscription_probe.cpp` 只依赖原生 rclcpp/std_msgs，可独立于 lexec/lrclexec 复现。它是手动诊断工具，故障复现返回非零，不加入默认 CTest 通过项。

## 运行

启用 `LRCLEXEC_BUILD_BENCHMARKS=ON` 后：

```bash
source /opt/ros/jazzy/setup.bash
cmake --build build/debug --target lrclexec_events_subscription_probe -j3
ROS_DOMAIN_ID=229 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  build/debug/benchmarks/lrclexec_events_subscription_probe events
```

最后一个参数可改为 `single` 或 `multi`。返回码：`0` 表示替换后正常收到消息，`2` 表示已复现 SDK 通知缺失，`3` 表示 Events 实验未命中地址复用，`1` 表示实验条件失败，`124/125` 表示 dispatch/executor 异常。地址由分配器决定，不能保证每次命中；未命中不能判定 SDK 已修复。

## 证据

探针先用一个永久订阅和一个动态订阅各接收一条消息，确认 DDS 发现和初始通知正常。暂停发布后，在同一个 executor 回调内销毁旧订阅、有界重建新订阅以命中旧 `rcl_subscription_t*`。随后由独立 `std::thread` 持续发布序列号，排除 ROS 发布 timer 的影响。

本机 5 次 Events 实验均观察到：

- old/new handle 相同，旧订阅 weak owner 已失效。
- 初始消息通知已安装，替换订阅的通知为空；该字段通过派生类只读观察，没有覆盖 SDK handler。
- DDS 显示 1 个 publisher、2 个 subscriptions 已匹配；永久订阅正常收消息。
- 新订阅回调为零，但在同 executor 线程手动 `take()` 能读取消息。
- 显式移除并重新加入整个节点后，通知重新注册，回调恢复。

Single 正常收到 235 条新消息；Multi 在同地址复用条件下正常收到 236 条。原始七份日志及当时源码保存在 `build/measurements/sdk-topic/`。SDK 的 `ExecutorEntitiesCollection::update_entities` 仅比较 raw handle key；同 key 时不替换旧 weak owner，Events 的 `subscriptions.update()` 又只在新增分支安装通知，和探针结果一致。

整节点 remove/add 仅用于确认缓存机制，会影响该节点的所有实体，不能作为适配层的透明修复。本次没有修改系统 SDK，也没有通过延迟释放实体或永久保留旧订阅绕过问题。当前导航与长压使用标准 Single/Multi。

此前混合长压的 Topic timeout 缺少当时的 handle/notifier 记录；本探针确认了一个能够独立导致消息等待不完成的 SDK 缺陷，尚不能证明那次历史 timeout 必然同因。timer 地址复用是另一个已确认的实例，见 [Lifecycle 诊断](../examples/lifecycle/README.md#eventsexecutor-已知限制)。
