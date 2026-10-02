# LifecycleNode 的业务任务收束

这个有限流程示例在 Single/Multi executor 上验证 `configure → activate → deactivate → 异步排空 → activate`。节点、scheduler 和任务 scope 由外部 `Controller` 持有，生命周期回调只捕获它的 weak_ptr，避免节点所有权环。

```bash
source /opt/ros/jazzy/setup.bash
./build/debug/examples/lifecycle/lifecycle_example --executor single --cycles 3
./build/debug/examples/lifecycle/lifecycle_example --executor multi --cycles 3
```

activate 为每一代任务创建新 counting_scope。deactivate、shutdown 和 error 回调关闭 scope、启动 join、请求 stop，然后立即返回；同一 executor 继续运行一个忽略旧 stop token 的 10 ms 清理 sender。外部控制流程等待 join，并核对旧 operation 的资源已释放，才允许再次 activate 或 cleanup。

示例断言排空中的 activate/cleanup 返回 FAILURE、业务资源峰值为 1，并分别从 unconfigured、inactive、active 进入 finalized。它还验证首次 activate 失败后的重试、控制操作抛异常后的排空，以及最终节点 weak_ptr 失效。不要在生命周期回调中同步等待依赖同一 executor 的 join。

这里关闭了 ROS 远程 lifecycle 通信服务，所有转换由应用经 scheduler 队列串行发起；它展示业务任务的所有权与收束策略。若接入远程 transition 服务，需要将服务请求和这些控制操作统一串行化。

## EventsExecutor 已知限制

本机 Jazzy rclcpp 28.1.22 中，销毁旧 timer 后立刻创建清理 timer，可能复用同一个 `rcl_timer_t*` 地址。SDK entity collection 只比较地址，此时仍保留旧 timer 的失效 weak_ptr，新 timer 未加入 TimersManager，scope 因而无法排空。通知 guard condition 没有“刷新已完成”的确认语义，队列投递也不能代替这个确认。

保留直接诊断入口；期限耗尽返回 124，不能当作通过：

```bash
./build/debug/examples/lifecycle/lifecycle_example --executor events --cycles 1
# 或显式加入 CTest：默认测试只承诺 Single/Multi 的完整流程。
cmake --preset debug -DLRCLEXEC_TEST_EVENTS_LIFECYCLE=ON
ctest --test-dir build/debug -R '^lifecycle_session_events$' --output-on-failure
cmake --preset debug -DLRCLEXEC_TEST_EVENTS_LIFECYCLE=OFF
```

此诊断保留正常的失败判定，没有将超时改写为成功。当前 SDK 下不建议用 Events 执行这类连续动态 timer 清理链；需要 SDK 修复后再验证并扩大支持范围。
