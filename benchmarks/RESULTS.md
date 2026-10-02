# 本机测量记录：2026-10-02

环境：Intel i7-13700KF，允许 CPU 0–23，governor 为 powersave；Linux 7.0.0-34 / glibc 2.39；GCC 13.3.0、Release `-O3 -DNDEBUG`，Jazzy rclcpp/rclcpp_action 28.1.22，Fast DDS。未固定 CPU 核或频率。最终测量二进制 SHA-256：

```text
2926c3104495c71516e2c888c061448edccb749eaa003f94a95cb3c2717f995a
```

## 顺序基线

每次预热 1 秒、测量 5 秒，每个配置重复三次，共 36 次，全部完成计数与业务收束检查。期间未运行本任务的构建或其他回归。吞吐列为三次运行的中位数及最小/最大值，单位千次/秒；P99 列为三次桶上界的中位数，单位微秒。指标定义见[工具说明](README.md)。

| Executor | producers / window | 负载 | 吞吐中位数（范围） | P99 上界中位数 |
| --- | --- | --- | ---: | ---: |
| Single | 1 / 1 | post | 199.0（198.3–199.6） | 4.096 |
| Single | 1 / 1 | schedule | 187.4（186.4–187.6） | 4.096 |
| Single | 4 / 256 | post | 2301.6（2218.9–2914.9） | 147.456 |
| Single | 4 / 256 | schedule | 1086.8（972.0–1141.6） | 294.912 |
| Multi | 1 / 1 | post | 166.3（164.5–167.6） | 5.632 |
| Multi | 1 / 1 | schedule | 158.1（157.8–158.4） | 6.656 |
| Multi | 4 / 256 | post | 2421.4（2068.0–2773.2） | 212.992 |
| Multi | 4 / 256 | schedule | 932.8（860.4–983.7） | 491.520 |
| Events | 1 / 1 | post | 287.3（282.0–291.0） | 3.072 |
| Events | 1 / 1 | schedule | 257.6（256.3–261.1） | 3.584 |
| Events | 4 / 256 | post | 2350.0（2342.7–2409.2） | 31.744 |
| Events | 4 / 256 | schedule | 1132.2（1051.6–1252.4） | 294.912 |

原始文件在 `build/measurements/baseline-final/`：`environment.json`、36 份 JSONL/stderr 和 `results.json`。环境记录中的二进制 hash 与上述值一致，关键 C++ 源码 hash 已核对。这是短期本机基线，重复间存在波动，不能据此承诺部署性能；Events 的零延时负载通过也不代表动态通信稳定。

## 最终二进制的 10 分钟混合长压

三个 executor 使用独立 DDS 域并发运行；Events 提前失败，标准两个运行完成了请求时长。下列两项 errors、duplicates、最终 inflight 均为 0，业务操作峰值均为 4；每项使用四个 producer，窗口为 32，warmup 2 秒。

| Executor | 测量时长 | 完成数 | RSS 起→末 / 峰值（KiB） |
| --- | ---: | ---: | ---: |
| Single | 600.003 秒 | 3,037,685 | 32,232→89,228 / 89,324 |
| Multi | 600.003 秒 | 3,008,164 | 33,780→91,580 / 91,676 |

第 300 秒以后，RSS 分别落在 89,228–89,324 KiB 和 91,580–91,676 KiB，呈平台；此前有增长。这个观察不能证明不存在泄漏或 SDK 内存有界。长压的吞吐受并发运行影响，不作为 executor 的横向性能比较。

完整计数、各操作的 P50/P95/P99/max、每秒 RSS 和环境分别保存在 `build/measurements/soak-final-single/` 与 `build/measurements/soak-final-multi/`。两份记录的二进制及全部关键 C++/CMake 源码 hash 已核对一致。

## Events 失败证据

最终二进制的混合长压请求 600 秒，在测量阶段 130.931 秒提前结束：691,830 次完成、1 次 error、0 次重复完成、最终 inflight 为 0、峰值为 4。stderr 明确记录 `producer=1 kind=topic outcome=-2`，确认走了 2 秒 timeout 分支；根因尚未定位。记录在 `build/measurements/soak-final-events/`，失败汇总保存在 `results.json`，未计入通过项。

另一个独立复现是 Lifecycle 清理 timer 地址复用：本机只读 SDK 诊断同时观察到 collection 的 timer key `expired=1`，以及同地址的 live timer `cancelled=0`，随后 scope 超时，退出码 124。日志与临时探针源码保存在 `build/measurements/sdk-lifecycle/`。该 SDK 缺陷已确认；它不能直接解释上述 Topic 超时。连续清理流程的支持范围和复现入口见 [Lifecycle 说明](../examples/lifecycle/README.md#eventsexecutor-已知限制)。

## 正确性检查

Debug 与 Clang 18.1.3 ASan/UBSan 各通过 28/28 项全量回归，包含 Nav2/MuJoCo 无界面测试。最后补充 timer 挂载屏障异常收束后，三个测量 smoke 在两种构建下再次通过；临时 ASan/UBSan 探针确认异常传播前完成取消排空。Cyclone DDS 的新增六项回归也通过。runner 的非零退出、timeout/部分输出留档、半行 JSON 失败汇总和 CTest stderr 展示已分别验证。GitHub Actions 配置已更新，本轮未在远端运行。

## 2026-10-03：Action、AMCL 与 Events 定位补充

本轮 Action Release 二进制 SHA-256 为 `c12337e08d3edd3c984294b2d35b9eb96d81b2884a4896c7a15751924d3d104b`，两组记录的二进制、`action_load.cpp` 和 `Metrics.h` hash 已与最终文件核对。Single/Multi 各使用四条独立任务链、不同 DDS 域并发执行 600 秒；期间也进行了构建和回归，吞吐不用于横向性能结论。

| Executor | 时长（秒） | 完成目标 | value / stopped / rejected / aborted | RSS 起→末 / 峰值（KiB） |
| --- | ---: | ---: | --- | ---: |
| Single | 600.067 | 756,153 | 252,051 / 168,034 / 84,017 / 252,051 | 40,220→49,504 / 49,576 |
| Multi | 600.065 | 768,645 | 256,215 / 170,810 / 85,405 / 256,215 | 40,608→53,300 / 53,588 |

两组均无重复完成，最终 inflight 为 0、客户端峰值为 12，每条链业务资源峰值为 1、scope join 后为 0，没有资源交接重叠。feedback 分别为 169,071 / 171,147 次，stderr 均为空。P99 桶上界分别为 6.554 / 5.243 ms，最大端到端延迟为 23.163 / 14.927 ms，包含 DDS 和人为清理门。空闲关闭并 join 的耗时分别为 0.361 / 0.057 ms，不能当作活动目标取消耗时。

第 300 秒以后的 RSS 范围为 49,504–49,528 KiB / 52,988–53,588 KiB；没有把平台形态解释为无泄漏证明。原始环境、采样、分类计数和汇总保存在 `build/measurements/action-soak-single/`、`build/measurements/action-soak-multi/`。

Debug、Clang ASan/UBSan 全量回归各通过 35/35，新增的两个 Action smoke 在本地 Cyclone DDS 0.10.5 / rmw_cyclonedds_cpp 2.2.4 overlay 下也通过。首次 Cyclone 启动失败属于缺少 overlay 动态库路径，补齐环境后重跑通过；没有修改系统安装。严格定位日志解析另外验证了完整坏行会失败、末尾写入中的半行可忽略，并重新读取了全部 AMCL 评分日志。

两种构建各覆盖 AMCL 四场景 × Single/Multi 和真值模式兼容检查，所有子进程正常退出。定位就绪后，Debug 的最大位置/航向误差为 0.07453 m / 0.00773 rad，ASan/UBSan 为 0.07988 m / 0.00873 rad；均满足预先设置的 0.10 m / 0.10 rad 门槛，且保留物理目标、绕障、无碰撞和取消停稳检查。日志位于 `build/{debug,asan-clang}/mujoco-runs/`。这仍是真值里程计、无噪声激光和给定地图条件下的验收。

Events 原生订阅实验连续 5 次复现同地址替换后通知丢失，Single/Multi 对照正常；收入仓库后的手动目标也得到 Single/Multi 退出 0、Events 退出 2。故障证据与退出码见 [SDK 诊断](EVENTS.md)。本轮没有修改 SDK，也没有把故障复现算作通过测试；历史混合长压 timeout 与本缺陷是否同因仍缺当时的直接证据。
