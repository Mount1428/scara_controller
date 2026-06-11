# SCARA 机械臂下位机（STM32）设计文档

## 目录

1. [系统概述](#1-系统概述)
2. [固件架构](#2-固件架构)
3. [通信协议规范](#3-通信协议规范)
4. [上位机需求](#4-上位机需求)
5. [关键设计决策](#5-关键设计决策)
6. [性能与资源](#6-性能与资源)
7. [附录](#7-附录)

---

## 1. 系统概述

### 1.1 硬件平台

| 项目 | 规格 |
|------|------|
| MCU | STM32F103C8 (Cortex-M3) |
| 主频 | 72MHz |
| FPU | 无（软件浮点仿真） |
| Flash | 64KB |
| SRAM | 20KB |
| 串口 | USART1 (TX: PA9, RX: PA10) |
| 串口适配器 | CH340 USB-Serial |

### 1.2 电机驱动接口

| 信号 | 电机0 | 电机1 | 说明 |
|------|-------|-------|------|
| 脉冲 (PUL) | PA0 | PA1 | 上升沿有效，最小脉冲宽度 6μs |
| 方向 (DIR) | PA2 | PA3 | Low=正向, High=反向 |
| 使能 (ENA) | PA4 | PA5 | Low=使能, High=关闭 |

### 1.3 软件架构分层

```
main.c (HAL初始化, while(1)调用user_loop)
  |
  +-- user_main.cpp (主循环: UART接收解析 + 命令分发)
  |     |
  |     +-- protocol.hpp (帧结构定义, as_bytes序列化)
  |     +-- ring_buffer.hpp (UART RX环形缓冲区)
  |     +-- motor.hpp (Motor驱动 + Controller运动队列)
  |           |
  |           +-- polinomial_profile.hpp (三次多项式轨迹规划)
  |           +-- spsc_queue.hpp (无锁SPSC队列)
  |
  +-- config.hpp (电机/运动学参数)
  +-- type_def.hpp (MotorConfig结构体)
  +-- math_utils.hpp (平方根/角度转换)
```

### 1.4 中断与优先级

| 中断 | 触发 | 频率/条件 | 优先级 | 用途 |
|------|------|-----------|--------|------|
| TIM1 | 定时器更新 | 10kHz (100μs) | 0 (最高) | 运动控制周期 |
| USART1 RX | DMA IDLE | 不定 | 1 | UART接收完成 |
| DMA1_CH5 | 传输完成 | UART TX完成 | 2 | 发送完成回调 |
| DMA1_CH4 | 传输完成 | UART RX完成 | 2 | 接收完成回调 |

---

## 2. 固件架构

### 2.1 主循环 (user_loop)

`user_loop()` 每次调用处理 **一条** 完整命令帧，然后 `__WFI()` 等待中断：

```
user_loop()
  ├─ peek 1 byte → 是否 0xAA?
  ├─ peek sizeof(Header) → header.is_valid()?
  ├─ switch(cmd)
  │   ├─ Motion(0x01)       → 创建PolinomialProfile → add_segment → Ack/Nack
  │   ├─ EmergencyStop(0x02) → clear queue, disable motors → Ack
  │   ├─ QueryStatus(0x03)   → 读取current_step, 转换为角度 → StatusResponse
  │   └─ default             → pop(1),  Nack(InvalidCommand)
  └─ __WFI()  (无数据时休眠)
```

### 2.2 运动控制定时器 (TIM1 @ 10kHz)

每次中断调用 `controller.update_handler(100)`，其逻辑：

```
update_handler(dt_us=100)
  ├─ segment_queue_.front() → 取出当前段
  ├─ elapsed_time_us_ >= segment.duration_us?
  │   ├─ YES: pop队列, 修正到最终步数, elapsed_time_us_ = 0
  │   └─ NO:  计算 elapsed_time_ms, 调用 x_profile.current_step(ms)
  │            计算目标步数, 调用 motor.request_step(step)
  ├─ step_once(): 发送脉冲→等待~6μs→清除脉冲
```

### 2.3 UART 接收 (DMA + IDLE 中断)

采用双缓冲乒乓接收，配合环形缓冲区：

```
UART RX线
  │
  ├─ DMA双缓冲 [2][512]         ← HAL_UARTEx_ReceiveToIdle_DMA
  │   ├─ 缓冲0 使用中
  │   └─ 缓冲1 备用
  │
  └─ uart_rx_event_callback()   ← IDLE中断触发
       ├─ std::exchange(cnt) 切换缓冲区
       ├─ 重新启动 DMA 接收
       └─ 将已满缓冲区推入 RingBuffer<1023>
              │
              └─ user_loop() 逐字节解析
```

### 2.4 UART 发送 (DMA + 忙标志)

非阻塞发送，通过原子标志避免冲突：

```
uart_send(data)
  ├─ while (uart_tx_busy) 等待前一次发送完成
  ├─ uart_tx_busy = true
  ├─ memcpy → uart_tx_raw_buffer[64]  (静态缓冲区)
  ├─ HAL_UART_Transmit_DMA
  └─ TX_COMPLETE回调: uart_tx_busy = false
```

**限制**：单次发送最大 64 字节。当前所有响应帧 ≤ 12 字节，无影响。

### 2.5 帧解析流程

```
uart_rx_buffer → peek(1) → 0xAA?
  ├─ NO:  pop(1), 丢弃
  └─ YES: peek(sizeof(Header))
            ├─ head1 != 0x55? → pop(1), 丢弃 (重新从0xAA对齐)
            └─ header.is_valid()
                  ├─ 根据cmd解析完整帧
                  │   ├─ 解析成功 → pop(size) 弹出, 执行
                  │   └─ 解析失败 → pop(1), 丢弃 (字节对齐重试)
                  └─ cmd未知 → pop(sizeof(Header)), Nack(InvalidCommand)
```

### 2.6 核心模块说明

#### 2.6.1 PolinomialProfile

三次多项式轨迹规划：

```
f(t) = a0 + a1·t + a2·t² + a3·t³

边界条件:
  f(0) = 0                (相对位移为0)
  f(T) = displacement     (总位移)
  f'(0) = start_speed     (起始速度)
  f'(T) = end_speed       (结束速度)

系数计算 (所有运算为 float, 软件仿真):
  a1 = start_speed
  a2 = (3·D - (2·v0 + v1)·T) / T²
  a3 = ((v0 + v1)·T - 2·D) / T³
    其中 D = end_step - start_step, T = process_time
```

**限制**：每次 ISR 调用 `current_step(ms)` 包含 5 次浮点乘加 + 1 次 `lround`（浮点取整），在无 FPU 的 Cortex-M3 上约 300-500 周期，占 100μs 周期中的 ~0.4-0.7μs。

#### 2.6.2 SpscQueue (无锁队列)

- 容量：128 (Controller<128>)
- 长度必须为 2 的幂（使用 bitwise AND 环绕）
- 单生产者（user_loop / add_segment）、单消费者（TIM1 ISR）
- `push`: 写 buffer[tail], tail = next(tail)
- `pop`: head = next(head)
- `front`: 返回 buffer[head] 的拷贝
- `back`: 返回 buffer[prev(tail)] 的拷贝

#### 2.6.3 RingBuffer (环形缓冲区)

- 容量：1023 字节
- 使用 `Capacity + 1` 存储区，以区分满和空
- 支持 power-of-2 优化（bitwise AND 环绕）
- `push`: 写数据到 tail，支持环绕
- `peek`: 从 head 读数据，不消费
- `pop`: 移动 head

#### 2.6.4 Motor

- `rad_to_step(rad)`: `lround(rad × pulse_per_rev / 2π) - offset_step_`
- `step_to_rad(step)`: `(step + offset_step_) × 2π / pulse_per_rev`
- `request_step(step)`: 每次 ±1 步向目标逼近，改变方向时自动换向
- `in_limit(rad)`: 检查角度是否在软限位内

---

## 3. 通信协议规范

### 3.1 物理层

| 参数 | 值 |
|------|-----|
| 接口 | USART1, 8N1 |
| 波特率 | 921600 bps |
| 流控制 | 无 |
| 字节序 | Little-Endian (与 Cortex-M3 一致) |

### 3.2 帧结构

所有帧以 4 字节 Header 开始：

```
  Byte 0: 0xAA      (帧头标识0)
  Byte 1: 0x55      (帧头标识1)
  Byte 2: cmd        (命令码)
  Byte 3: 0x00      (填充字节)
```

不使用 CRC（CRC 字节保留为 0x00，当前未校验）。

### 3.3 帧类型定义

#### 3.3.1 AckFrame (4 字节)

上位机发送命令后，下位机成功执行返回。

```
偏移  大小  字段      值/说明
0     4    Header     cmd=0xF0 (Ack)
```

**总长度**: 4 字节

#### 3.3.2 NackFrame (5 字节)

命令执行失败返回，带原因码。

```
偏移  大小  字段      值/说明
0     4    Header     cmd=0xF1 (Nack)
4     1    reason     原因码: 0x01=InvalidCommand, 0x02=InvalidData, 0x03=ExecutionFailed
```

**总长度**: 5 字节

#### 3.3.3 MotionCommandFrame (24 字节)

上位机发送运动段指令。

```
偏移  大小  字段           说明
0     4    Header         cmd=0x01 (Motion)
4     4    target_angle[0]  电机0目标角度 (float, rad)
8     4    target_angle[1]  电机1目标角度 (float, rad)
12    4    target_speed[0]  电机0目标末端速度 (float, rad/s)
16    4    target_speed[1]  电机1目标末端速度 (float, rad/s)
20    4    process_time     运动段持续时间 (float, s)
```

**总长度**: 24 字节

**约束**：
- `target_angle[0]` 必须在 [1.5708, 3.6652] rad 内 (90°~210°)
- `target_angle[1]` 必须在 [-0.5236, 1.5708] rad 内 (-30°~90°)
- `process_time` 必须 > 0，且不能导致 `current_step` 计算溢出
- 超出限位时返回 Nack(InvalidData)

#### 3.3.4 EmergencyStopFrame (4 字节)

立即停止所有运动并清空队列。

```
偏移  大小  字段      值/说明
0     4    Header     cmd=0x02 (EmergencyStop)
```

**响应**: AckFrame (成功) 或 NackFrame (失败)

#### 3.3.5 QueryStatusFrame (4 字节)

查询当前两轴角度。

```
偏移  大小  字段      值/说明
0     4    Header     cmd=0x03 (QueryStatus)
```

#### 3.3.6 StatusResponseFrame (12 字节)

查询状态响应。

```
偏移  大小  字段           说明
0     4    Header         cmd=0xF2 (StatusResponse)
4     4    angle[0]       电机0当前绝对角度 (float, rad)
8     4    angle[1]       电机1当前绝对角度 (float, rad)
```

### 3.4 通信流程

#### 3.4.1 正常命令-响应

上位机发送 MotionFrame:

```
上位机                         下位机
  |                               |
  |--- MotionCommandFrame(24B) -->|
  |                               |-- 解析帧头校验
  |                               |-- 检查角度限位
  |                               |-- 构造 PolinomialProfile
  |                               |-- push 到 segment_queue
  |                               |-- 成功 → AckFrame(4B)
  |<-- AckFrame(4B)  ------------|
  |                               |
  |  (下个周期发送下一条指令)      |
```

失败场景：

```
  |--- MotionCommandFrame(24B) -->|
  |                               |-- angle[0] 超限
  |<-- NackFrame(5B, InvalidData)-|
  |                               |
  |  (上位机重试/调整参数)         |
```

#### 3.4.2 查询状态

```
上位机                         下位机
  |                               |
  |--- QueryStatusFrame(4B) ---->|
  |<-- StatusResponseFrame(12B) --|
  |                               |
```

#### 3.4.3 紧急停止

```
上位机                         下位机
  |                               |
  |--- EmergencyStopFrame(4B) -->|
  |                               |-- disable motors
  |                               |-- clear segment_queue
  |<-- AckFrame(4B)  ------------|
```

---

## 4. 上位机需求

### 4.1 基本原则

1. **指令同步**：每条指令都必须等待下位机的 ACK/NACK/response 后才能发送下一条指令。超时视为设备丢失。
2. **段间延时控制**：根据运动段 `process_time` 控制指令发送间隔，在保证队列充分利用的前提下尽可能减少 NACK 发生。
3. **主动速度限制**：上位机必须在发送前主动限制关节电机速度，不能依赖下位机限位检查作为速度保护。

### 4.2 指令同步模型

```
发送 MotionCommandFrame
  └─ 等待 ACK / NACK / 超时 (timeout)
       ├─ ACK:    发送下一条指令
       ├─ NACK:   根据 reason 决定重试策略
       │   ├─ InvalidData:   调整参数后重试
       │   ├─ ExecutionFailed:检查状态后重试
       │   └─ InvalidCommand: 检查协议版本
       └─ 超时:
            ├─ 发送 QueryStatusFrame 尝试恢复同步
            │   ├─ 收到有效响应: 恢复通信
            │   └─ 再次超时: 判定设备丢失
            └─ 重试耗尽: 判定设备丢失
```

**超时配置建议**：

| 参数 | 值 | 说明 |
|------|-----|------|
| response_timeout | 500ms | 单次 ACK/NACK 等待超时 |
| resync_timeout | 200ms | 恢复同步等待超时 |
| resync_retries | 3 | 恢复同步最大重试次数 |
| device_lost | 5s 无有效响应 | 判定设备丢失 |

**说明**：921600 bps 下 24 字节帧传输约 0.2ms，下位机处理 << 1ms。500ms 超时远大于正常响应时间，仅作为异常检测阈值。

### 4.3 段间延时控制策略

运动段队列深度为 128，为避免队列满导致 NACK，上位机应根据当前队列占用率和 `process_time` 控制发指令节奏。

**推荐策略**：

```
队列管理参数:
  - MAX_QUEUE = 128 (下位机 SpscQueue 容量)
  - HIGH_WATERMARK = 96  (75% 队列占用警戒线)
  - LOW_WATERMARK = 32   (25% 队列占用低水位)

算法:
  1. 维护一个"虚拟队列"，记录已发送但未完成的运动段总时间
  2. 每条指令发送前:
     a. 虚拟队列总时间 = 所有进行中段剩余时间之和
     b. 如果虚拟队列中段数 < HIGH_WATERMARK:
         立即发送 (不需等待)
     c. 如果虚拟队列中段数 >= HIGH_WATERMARK:
         等待到虚拟队列总时间 < LOW_WATERMARK 后再发送
  3. 每条指令收到 ACK 后:
     a. 将该段加入虚拟队列
     b. update: 虚拟队列总时间 += process_time
  4. TIM1 10kHz 中断驱动: 定期从虚拟队列头部扣除已消耗时间

简化方案 (无需虚拟队列):
  1. 发送一条指令后，等待 process_time × 0.8 再发送下一条
  2. 收到 NACK(ExecutionFailed) 时，等待 process_time × 0.5 再重试
  3. 优点: 实现简单，无需维护队列状态
  4. 缺点: 吞吐量低于虚拟队列方案，管道利用率约 80%
```

**原理**：`process_time` 是运动段从开始到结束的时间。由于队列深度为 128，上位机可以连续发送 128 条指令而无需等待，但队列满后继续发送会收到 NACK。通过控制发送间隔与 `process_time` 匹配，保持队列在低水位运行。

### 4.4 主动速度限制

**下位机不保证速度安全**。上位机必须在发送前确保所有关节的速度不超过安全阈值。

**约束条件**：

| 关节 | 脉冲当量 | 推荐最大速度 | 推荐最大加速度 (间接) |
|------|---------|-------------|-------------------|
| 电机0 (大臂) | 1600 pulse/rev | 2.0 rad/s | 通过 process_time + 速度差值控制 |
| 电机1 (小臂) | 1600 pulse/rev | 2.0 rad/s | 同上 |

上位机应实现：

```
1. 发送前检查:
   target_speed[0] <= MAX_SPEED_0 (2.0 rad/s)
   target_speed[1] <= MAX_SPEED_1 (2.0 rad/s)

2. 速度曲线约束:
   相邻运动段之间的速度跳变不应过大：
     |new_speed[0] - last_speed[0]| <= MAX_ACCEL_0 × process_time
     |new_speed[1] - last_speed[1]| <= MAX_ACCEL_1 × process_time
   其中 MAX_ACCEL 推荐值: 10.0 rad/s² (根据机械结构动态调整)

3. 笛卡尔空间映射约束:
   如果上位机从笛卡尔空间规划轨迹，需逆解到关节空间后校验
   各关节速度不能超过上述限制

4. 错误处理:
   超出限制 → 不发送，返回错误给调用方
```

### 4.5 通信安全

| 场景 | 处理方式 |
|------|---------|
| 发送后超时无响应 | QueryStatus 尝试恢复，3 次失败判定设备丢失 |
| 收到 NACK(InvalidData) | 检查参数后重试，最多 3 次 |
| 收到 NACK(ExecutionFailed) | 查询当前状态，根据状态决定是否重试 |
| 帧解析错位 | 丢弃错误字节，找到下一个 0xAA 重新对齐 |

### 4.6 启动与初始化

```
上位机启动流程:
  1. 打开串口 (921600, 8N1)
  2. 清空串口缓冲区 (drain)
  3. 发送 QueryStatusFrame
  4. 等待 StatusResponseFrame:
     ├─ 500ms 内收到 → 设备就绪
     └─ 超时 → 重试 3 次 → 报告设备丢失
  5. 进入正常命令循环
```

### 4.7 错误恢复流程

```
通信失步检测:
  - 连续 3 次 ACK/NACK 超时
  - 连续收到无法解析的数据

恢复步骤:
  1. drain 串口缓冲区 (2ms)
  2. 发送 QueryStatusFrame
  3. 等待 200ms:
     ├─ 收到 StatusResponseFrame → 同步恢复
     └─ 超时 → retry 3 次
          └─ 全部超时 → 报告设备丢失, 停止发送
  4. 恢复后:
     - 重新初始化虚拟队列状态
     - 发送 QueryStatus 获取当前角度
     - 重新规划轨迹起点
```

---

## 5. 关键设计决策

### 5.1 为什么在 user_loop 中逐帧解析而非中断中解析

- **实时性**：中断应尽量短，帧解析涉及逻辑判断和 Profile 构造（大量浮点运算），不适合在中断上下文执行
- **阻塞风险**：`__WFI()` 在无数据时休眠，CPU 大部分时间空闲
- **确定性**：TIM1 ISR 优先级最高，user_loop 命令处理不会被运动控制中断打断

### 5.2 为什么使用三次多项式轨迹

- 速度连续（柔性冲击）
- 可保证初末速度可控（速度约束）
- 计算量相对可接受（5 次浮点乘加 + lround）

### 5.3 为什么 TX 使用静态缓冲区而非 DMA 直接发送

- 简化内存管理，避免动态分配
- 静态缓冲区保证 DMA 传输期间数据不会被修改
- 当前所有响应帧 ≤ 12 字节，64 字节缓冲区足够

### 5.4 队列深度 128 的考量

- 每段 Segment 包含 2 个 PolinomialProfile，约 64 字节
- 128 × 64 = 8KB，加上其他开销，SRAM 20KB 中约 40%
- 在 921600 bps × 1s / 24B ≈ 4800 帧/秒的理论吞吐下，128 段约 26ms 缓冲

### 5.5 无 CRC 的原因

- USART 硬件层提供每字节奇偶校验（当前未启用）
- 物理层误码率在短距离 USB 串口场景下极低
- CRC 字节已保留在协议中（padding 位置），可后续启用

### 5.6 上电偏移 (init_offset) 的作用

- 编码器/限位开关安装完成后，记录机械零位对应的角度
- `rad_to_step` 减去偏移量，使 step 0 对应机械零位
- `step_to_rad` 加回偏移量，输出相对于机械零位的绝对角度

---

## 6. 性能与资源

### 6.1 CPU 负载估算

| 任务 | 每周期耗时 | 频率 | CPU 占用 |
|------|-----------|------|---------|
| TIM1 ISR (update_handler) | ~2-3μs | 10kHz | 2-3% |
| 脉冲清除 (step_once) | ~6μs NOP | 10kHz | ~6% |
| user_loop (处理命令) | ~10μs | 视指令频率 | <1% |
| UART RX DMA | ~0.5μs | 每512B | <0.1% |
| UART TX DMA | ~0.3μs | 每帧 | <0.1% |
| **总计** | | | **~9-10%** |

### 6.2 内存使用

| 区域 | 用途 | 大小 |
|------|------|------|
| .text + .rodata | 代码 + 常量 | ~10-15KB (估算) |
| .data + .bss | 全局变量 | ~4KB |
| 堆栈 | | ~1KB |
| segment_queue | 128 × Segment | ~8KB |
| uart_rx_buffer | RingBuffer | 1024B |
| uart_rx_raw_buffer | DMA 双缓冲 | 2×512B = 1024B |
| uart_tx_raw_buffer | DMA 发送缓冲 | 64B |
| **总计** | | **~25-30KB** (Flash: ~15KB, SRAM: ~14KB) |

### 6.3 吞吐量

| 场景 | 吞吐量 |
|------|--------|
| 理论 UART 最大 | 921600 bps / 192B(帧+应答) ≈ 4800 命令/s |
| 实测 (连续发送) | ~456 命令/s (受上位机调度 + 串口驱动延迟限制) |
| 主要瓶颈 | 上位机 Python 串口驱动 ~1ms 调度延迟 |
| | CH340 硬件 ~1-2ms 延迟 |

---

## 7. 附录

### 7.1 关键参数汇总

| 参数 | 值 | 说明 |
|------|-----|------|
| UART 波特率 | 921600 | |
| UART RX 缓冲区 | 1023 字节 | RingBuffer |
| UART TX 缓冲区 | 64 字节 | 静态 DMA |
| DMA 双缓冲 | 2 × 512 字节 | 乒乓接收 |
| 运动队列深度 | 128 | SpscQueue |
| 控制周期 | 100μs | TIM1 |
| 脉冲当量 | 1600 pulse/rev | 电机 |
| 脉冲宽度 | ~6μs | 延时清除 |
| 电机0限位 | [90°, 210°] | 软限位 |
| 电机1限位 | [-30°, 90°] | 软限位 |

### 7.2 已知限制与改进方向

| 限制 | 影响 | 改进方向 |
|------|------|---------|
| 无 CRC 校验 | 极端噪声下可能误判帧 | 启用 CRC-8，校验所有帧 |
| float 在 ISR 中计算 | 无 FPU 下增加 ISR 延迟 | 使用 fixmath 定点数替代 |
| rad/step 转换每次计算 | 额外的浮点运算 | 预编译时计算转换常数 |
| __WFI() 休眠 | 每次唤醒需几十个周期 | 可接受，功耗非关键 |
| 协议无序列号 | 无法区分重复帧 | 增加 1 字节帧序列号 |

### 7.3 帧格式对照速查表

```
All frames have 4-byte header: [0xAA, 0x55, cmd, 0x00]

MotionCommandFrame  (24B): HEAD + angle[2] + speed[2] + process_time
AckFrame            ( 4B): HEAD
NackFrame           ( 5B): HEAD + reason
EmergencyStopFrame  ( 4B): HEAD
QueryStatusFrame    ( 4B): HEAD
StatusResponseFrame (12B): HEAD + angle[2]

cmd values:
  0x01  Motion
  0x02  EmergencyStop
  0x03  QueryStatus
  0xF0  Ack
  0xF1  Nack
  0xF2  StatusResponse

reason values (Nack only):
  0x01  InvalidCommand
  0x02  InvalidData
  0x03  ExecutionFailed
```
