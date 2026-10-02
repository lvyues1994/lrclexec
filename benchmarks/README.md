# 压力测试与性能基线

`lrclexec_load` 在真实 ROS executor 上运行。`post` 测量通用队列投递，`schedule` 测量经 counting_scope 启动的 sender；`mixed` 循环执行 schedule、已挂载 timer 的取消、1 ms 到期/取消竞争、Service 和独立 Topic 等待。Service/Topic 的业务结果为 42，墙钟超时为 2 秒。

独立 Release 构建用于测量，Debug 和 ASan/UBSan 用于检查正确性：

```bash
source /opt/ros/jazzy/setup.bash
cmake --preset release-bench \
  -DLRCLEXEC_LEXEC_SOURCE_DIR=/home/lvyues/code/exec/lexec \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build --preset release-bench -j3
UV_CACHE_DIR=/tmp/lrclexec-uv-cache uv run --no-project --no-managed-python \
  benchmarks/run.py --binary build/release-bench/benchmarks/lrclexec_load \
  --stage baseline --seconds 10 --warmup 2 --repeats 3 \
  --output build/measurements/baseline
UV_CACHE_DIR=/tmp/lrclexec-uv-cache uv run --no-project --no-managed-python \
  benchmarks/run.py --binary build/release-bench/benchmarks/lrclexec_load \
  --stage soak --seconds 600 --warmup 2 --executors single multi \
  --output build/measurements/soak
```

每个输出目录必须是新目录。runner 顺序执行各配置，保存二进制 SHA-256、Git 状态、CPU/亲和性/governor、RMW、ROS 包版本、CMake cache，以及每次运行的 JSONL、stderr 和汇总。基线对三个 executor 分别运行 `post`、`schedule`，使用 1 producer/window 1 和 4 producers/window 256；长压使用 4 producers/window 32。`--executors events` 可单独调查实验性 executor 的长期行为，失败仍返回非零，并保存原始输出。

| 指标 | 口径 |
| --- | --- |
| 吞吐 | 完成数 / 从阶段开始到停止接纳并完成业务收束的时间，含线程启动、记账、scope join 和最终队列投递；未测量 SDK 内部事件队列是否为空 |
| post / schedule 延迟 | 提交前到完成回调进入；不含接纳窗口前的等待 |
| timer_cancel 延迟 | stop 请求前到外部线程观察完成，含线程唤醒 |
| 其他 mixed 延迟 | 启动前到外部线程观察终态，含本地 DDS 和线程唤醒 |
| P50/P95/P99 | 固定 1024 桶直方图的桶上界；每个二进制指数段分 16 桶，最大值单独精确记录 |
| inflight / peak_inflight | 已接纳而未完成的业务操作数；窗口是测试程序的背压限制 |
| RSS | Linux `/proc/self/status` 的 VmRSS，约每秒采样 |

warmup 与 measure 分别记账。测试要求每次接纳恰好完成一次、error/重复完成为零、终态 inflight 为零，且峰值不超过窗口；结束后等待 scope join。timer 的取消完成期限为 3 秒；整体无进展或超过阶段结束 10 秒会触发 watchdog，保留失败现场并非正常收束。退出码 124 表示期限耗尽，125 表示 executor 或重复完成异常。

所有 producer 共用一个 scheduler 和互斥 callback group。Multi 使用四个 executor 线程，不能据此声称同一 scheduler 的任务并行。数据包含仪器、分配器和 SDK 开销；CPU governor、后台负载、RMW、构建类型都会影响结果。RSS 平台或一次长压通过不能证明没有泄漏，业务窗口也不等于 SDK 内部队列深度；默认 EventsQueue 不保证内存有界。Jazzy 28.1.22 的 Events 长压已有 Topic 超时，详见[本机记录](RESULTS.md)，尚不能作为稳定性通过项。

启用 `LRCLEXEC_BUILD_BENCHMARKS=ON` 和测试后，CTest 还验证三个 executor 的短期测量协议及直方图与排序分位数的一致性，不设机器相关的性能门槛。

## Action 专项长压

`lrclexec_action_load` 使用真实 `execute_action` 和抢占式 ActionServer。每条任务链循环提交九个目标，覆盖成功、客户端取消、A→B→C 抢占（B 的工厂不运行）、feedback 后取消、goal 拒绝、携带结果的 abort，以及取消被拒后等待成功终态。启动阶段先确认协议服务端缺席，再创建并发现它；不向缺席服务端提交无法保证排空的 Action。

```bash
UV_CACHE_DIR=/tmp/lrclexec-uv-cache uv run --no-project --no-managed-python \
  benchmarks/run.py --binary build/release-bench/benchmarks/lrclexec_action_load \
  --stage action --seconds 600 --executors single multi \
  --output build/measurements/action
```

runner 使用四条独立任务链，每条有自己的节点、scheduler 和互斥 callback group；Multi 实际运行四线程 `spin()`。程序也可直接指定 `--lanes 1` 做最小实验。每条链固定最多三个未完成目标，检查业务资源峰值为一、交接无重叠、最终释放。预期 rejected/aborted 单独统计，不算测试错误；每轮应有 3 value、2 stopped、1 rejected、3 aborted。feedback 载荷及取消 `ERROR_REJECTED` 均检查，observer 在终态后不能继续调用。

每个协议阶段等待最多 5 秒；失败立即输出阶段并异常退出，保留在途对象直到进程结束。正常结束关闭并 join 客户端/服务端 scope，再停止并 join executor，最后释放原生 Action owners。`join_seconds` 测量完整循环结束后的空闲关闭；P99/max 包含本地 DDS、测量及人为控制的清理门，不代表单次队列投递延迟。runner 限制总时长并保存超时部分输出，超时强制回收不属于正常排空。Action stage 不单独预热，数据包含从启动后的第一轮开始的全过程。

RSS 包含原生 SDK 的结果缓存；Jazzy 默认结果缓存期限为 10 秒。业务资源归零、RSS 平台和一次长压通过均不能证明 SDK 无泄漏。CTest 的 `action_load_single_smoke` / `action_load_multi_smoke` 验证所有场景及计数，不设吞吐门槛。Events 的动态订阅故障复现另见 [SDK 诊断](EVENTS.md)。
