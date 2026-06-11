# 设计报告参考资料

本文档提供编写设计报告时可直接引用的技术数据、公式推导、时序分析及协议定义。内容来源于实际固件代码分析，确保与实现一致。

---

## 1. 硬件参数表

### 1.1 MCU 与通信

```yaml
MCU:
  型号: STM32F103C8
  内核: ARM Cortex-M3
  主频: 72 MHz
  FPU: 无 (软件浮点仿真)
  Flash: 64 KB
  SRAM: 20 KB

串口:
  外设: USART1
  引脚: TX=PA9, RX=PA10
  模式: 异步 8N1
  波特率: 921600 bps
  流控制: 无
  接收: DMA + 空闲中断 (IDLE)
  发送: DMA (非阻塞)

定时器:
  TIM1: 控制周期 100μs (10kHz), 优先级 0 (最高)
```

### 1.2 电机接口

| 信号 | 功能 | 电机0 (J1) | 电机1 (J2) | 有效电平 |
|------|------|-----------|-----------|---------|
| PUL | 步进脉冲 | PA0 | PA1 | 上升沿有效 |
| DIR | 方向 | PA2 | PA3 | Low=正向 |
| ENA | 使能 | PA4 | PA5 | Low=使能 |

电机驱动参数：

| 参数 | 值 |
|------|-----|
| 脉冲当量 | 1600 pulse/rev |
| 最小脉冲宽度 | 6μs |
| 驱动器细分 | 未使用 (整步) |

### 1.3 关节约束

| 关节 | 角度限位 (min) | 角度限位 (max) | 上电偏移 |
|------|---------------|---------------|---------|
| 电机0 (大臂) | 90° (1.571 rad) | 210° (3.665 rad) | 210° |
| 电机1 (小臂) | -30° (-0.524 rad) | 90° (1.571 rad) | -30° |

### 1.4 运动学参数

| 符号 | 含义 | 值 |
|------|------|-----|
| br | 大臂长度 | 0.12 m |
| lr | 小臂长度 | 0.20 m |
| d | 基座偏距 | 0.10 m |

---

## 2. 通信协议 — 帧字节定义

### 2.1 帧头 (Header) — 4 字节

所有帧共享同一头部结构：

```
偏移  类型      字段      值
0     uint8_t   head0     0xAA
1     uint8_t   head1     0x55
2     uint8_t   cmd       命令码
3     uint8_t   padding   0x00 (填充)
```

```c
struct Header {
    uint8_t head0 = 0xAA;
    uint8_t head1 = 0x55;
    uint8_t cmd;       // 命令码
    uint8_t padding = 0;
};
```

### 2.2 AckFrame — 4 字节

```
字节偏移: [0] [1] [2] [3]
          0xAA 0x55 0xF0 0x00
```

### 2.3 NackFrame — 5 字节

```
字节偏移: [0] [1] [2] [3] [4]
          0xAA 0x55 0xF1 0x00 reason
```

| reason | 含义 |
|--------|------|
| 0x01 | InvalidCommand — 未识别的命令码 |
| 0x02 | InvalidData — 数据校验失败 (角度超限等) |
| 0x03 | ExecutionFailed — 执行失败 (队列满等) |

### 2.4 MotionCommandFrame — 24 字节

```
字节偏移: [0-3]  [4-7]    [8-11]   [12-15]  [16-19]  [20-23]
字段:     Header  angle[0] angle[1] speed[0] speed[1] process_time
类型:     4B      float    float    float    float    float
```

`process_time` 单位：秒 (s)，类型：`float`

### 2.5 EmergencyStopFrame — 4 字节

```
字节偏移: [0] [1] [2] [3]
          0xAA 0x55 0x02 0x00
```

### 2.6 QueryStatusFrame — 4 字节

```
字节偏移: [0] [1] [2] [3]
          0xAA 0x55 0x03 0x00
```

### 2.7 StatusResponseFrame — 12 字节

```
字节偏移: [0-3]  [4-7]    [8-11]
字段:     Header  angle[0] angle[1]
类型:     4B      float    float
```

### 2.8 命令码汇总

```c
enum class Command : uint8_t {
    None           = 0x00,
    Motion         = 0x01,  // 上位机 → 下位机
    EmergencyStop  = 0x02,  // 上位机 → 下位机
    QueryStatus    = 0x03,  // 上位机 → 下位机
    Ack            = 0xF0,  // 下位机 → 上位机
    Nack           = 0xF1,  // 下位机 → 上位机
    StatusResponse = 0xF2,  // 下位机 → 上位机
};
```

### 2.9 串口字节时序

在 921600 bps、8N1 模式下：

```
每字节传输时间 = 10 bit / 921600 bps ≈ 10.85 μs
  (1 start + 8 data + 1 stop)

帧传输时间:
  MotionCommandFrame (24B): 24 × 10.85 ≈ 260 μs
  AckFrame            ( 4B):  4 × 10.85 ≈  43 μs
  NackFrame           ( 5B):  5 × 10.85 ≈  54 μs
  StatusResponseFrame (12B): 12 × 10.85 ≈ 130 μs

往返延迟 (Motion → Ack):
  上位机发送 24B  (260μs)
  + 下位机处理    (<50μs)
  + 下位机发送 4B (43μs)
  ≈ 350-400 μs  (纯传输+处理时间，不含CH340/USB驱动延迟)
```

---

## 3. 三次多项式轨迹规划

### 3.1 原理

采用三阶多项式 (cubic polynomial) 规划关节空间点对点运动，保证位置和速度连续：

```
  θ(t) = a₀ + a₁·t + a₂·t² + a₃·t³
  
  边界条件:
    θ(0)    = θ_start     (起始位置)
    θ(T)    = θ_end       (终止位置)
    θ'(0)   = ω_start     (起始速度)
    θ'(T)   = ω_end       (终止速度)
```

### 3.2 系数推导

令位移 `D = θ_end - θ_start`，总时间 `T`：

```
  由 θ(0) = 0 (相对位移):
    a₀ = 0
  
  由 θ'(0) = ω_start:
    a₁ = ω_start
  
  由 θ(T) = D 和 θ'(T) = ω_end:
    a₁·T + a₂·T² + a₃·T³ = D
    a₁ + 2·a₂·T + 3·a₃·T² = ω_end
  
  代入 a₁ 解得:
    a₂ = (3·D - (2·ω_start + ω_end)·T) / T²
    a₃ = ((ω_start + ω_end)·T - 2·D) / T³
```

### 3.3 实际代码实现

```cpp
// 构造函数 (user/polinomial_profile.hpp:24-49)
PolinomialProfile(start_step, end_step, process_time, start_speed, end_speed) {
    a1 = start_speed;
    a2 = (3 * displacement - (2 * start_speed + end_speed) * T) / (T * T);
    a3 = ((start_speed + end_speed) * T - 2 * displacement) / (T * T * T);
    offset_step_ = start_step;
    steps_ = displacement;
}

// 每 100μs ISR 中调用
int32_t current_step(float ms) {
    float ms2 = ms * ms;
    float ms3 = ms2 * ms;
    return lround(a1*ms + a2*ms2 + a3*ms3) + offset_step_;
}
```

### 3.4 浮点运算量统计

| 方法 | 浮点乘加 | 浮点除法 | lround | 用途 |
|------|---------|---------|--------|------|
| 构造函数 | 9 | 2 | 0 | 段建立时调用 |
| current_step | 5 | 0 | 1 | 每 100μs 调用 |
| rad_to_step | 2 | 0 | 1 | 段建立时调用 |

STM32F103C8 软件浮点估算：
- 单次 `float` 乘法: ~10-20 周期
- 单次 `float` 加法: ~10-20 周期
- 单次 `float` 除法: ~30-50 周期
- 单次 `lround` (float→int): ~40-60 周期
- **每次 current_step 估算**: ~150-250 周期 ≈ 2-3.5μs @72MHz

---

## 4. 固件状态与流程

### 4.1 主循环状态图

```
                   ┌─────────────┐
                   │   __WFI()   │
                   │  (等待中断)  │
                   └──────┬──────┘
                          │ UART RX 数据到达
                          │ (DMA IDLE 中断)
                          ▼
              ┌─────────────────────┐
              │  uart_rx_event_     │
              │  callback()         │
              │  切换 DMA 缓冲区    │
              │  push → RingBuffer  │
              └─────────┬───────────┘
                        │
                        ▼
              ┌─────────────────────┐
              │  peek(1) == 0xAA?   │
              ├─────YES──────┬──NO──┤
              │              │      │
              ▼              ▼      │
        ┌──────────┐   pop(1) ──────┤
        │peek(4B)  │   (丢弃)       │
        │is_valid? │                │
        ├─YES──┬NO─┤                │
        │      │    │               │
        ▼      │    ▼               │
   ┌──────┐   │  pop(1) ────────────┤
   │switch│   │  (重对齐)           │
   │ cmd  │   │                     │
   ├──┬───┘   │                     │
   │  │       │                     │
┌──┘  ▼       │                     │
│ Motion ──→ 构造 Profile ──→ Ack/Nack
│ EStop  ──→ clear queue ──→ Ack
│ Query  ──→ read angles ──→ StatusResponse
│ default ──→ pop(4) ──→ Nack
└── 返回循环起始
```

### 4.2 运动控制 ISR (TIM1 @ 10kHz)

```
每 100μs 触发:
  1. front() 取当前段
     └─ 队列空 → 直接返回
  
  2. elapsed >= duration_us?
     ├─ YES: pop(), 修正到最终步数, elapsed=0
     └─ NO:  计算目标步数 (current_step), request_step(±1)
  
  3. step_once():
     ├─ 设 PUL=High (发出脉冲)
     ├─ 延时 ~6μs (NOP 忙等)
     └─ 设 PUL=Low (清除脉冲)
```

### 4.3 UART 接收 DMA 乒乓机制

```
初始化:
  缓冲0 ← HAL_UARTEx_ReceiveToIdle_DMA
  缓冲1 ← 空闲

帧到达 (IDLE 中断):
  1. index = exchange(cnt, 1-cnt)  // 切换索引
  2. 缓冲[index] 已满 → 启动接收 缓冲[1-index]
  3. push(缓冲[index], size) → RingBuffer

环形缓冲区:
  - 容量: 1023 字节
  - 生产者: UART IDLE 中断
  - 消费者: user_loop 主循环
  - 无锁 (单生产者/单消费者, atomic head/tail)
```

### 4.4 UART 发送流程

```
uart_send(span<byte> data):
  1. spin: while (tx_busy)  // 等待前一次完成
  2. tx_busy = true
  3. memcpy(data → tx_buffer, min(data.size, 64))
  4. HAL_UART_Transmit_DMA()
  5. 返回 (不等待发送完成)

TX_COMPLETE 回调:
  1. tx_busy = false
```

---

## 5. 内存布局

### 5.1 全局数据结构

| 符号 | 类型 | 大小 | 说明 |
|------|------|------|------|
| `uart_rx_raw_buffer` | `uint8_t[2][512]` | 1024 B | DMA 双缓冲 |
| `uart_rx_buffer` | `RingBuffer<1023>` | 1024 B + 8 B (atomic) | 接收环形缓冲 |
| `uart_tx_raw_buffer` | `uint8_t[64]` | 64 B | 发送 DMA 缓冲 |
| `motor_0` | `Motor` | ~40 B | 电机0实例 |
| `motor_1` | `Motor` | ~40 B | 电机1实例 |
| `controller` | `Controller<128>` | ~8 KB + 8 B | 含 SpscQueue<Segment,128> |

### 5.2 Segment 对象大小

```cpp
struct Segment {
    uint32_t duration_us;       // 4 B
    PolinomialProfile x_profile; // ~24 B (a1,a2,a3,offset_step,steps,end_speed,valid)
    PolinomialProfile y_profile; // ~24 B
};
// ≈ 52 B, 对齐后约 64 B
```

SpscQueue<128>: 128 × ~64 B + 2 × atomic_size_t ≈ 8208 B

### 5.3 总内存估算

| 区域 | 估算大小 |
|------|---------|
| segment_queue | 8208 B |
| uart_rx_raw_buffer | 1024 B |
| uart_rx_buffer | 1032 B |
| uart_tx_raw_buffer | 64 B |
| 其他全局变量 | ~500 B |
| 栈 | ~1024 B |
| **合计** | **~14 KB (SRAM)** |
| 可用 SRAM | 20 KB |
| 余量 | ~6 KB |

---

## 6. 设计权衡分析

### 6.1 轨迹规划：三次多项式 vs S曲线

| 方面 | 三次多项式 (当前) | S曲线 |
|------|-----------------|-------|
| 计算量 | 低 (5 mul + 1 round) | 高 (需分段判断) |
| 速度连续 | 是 | 是 |
| 加速度连续 | 否 (线性变化) | 是 |
| 加加速度 (Jerk) | 常数 | 有界 |
| 适用场景 | 点对点、速度可衔接 | 高速高精度 |
| Cortex-M3 可行性 | 已验证 | 需定点数优化 |

**选用理由**：三次多项式在计算量和平滑性之间取得最佳平衡。对于 SCARA 应用场景，末端速度可衔接（相邻段速度匹配），无急停冲击问题。

### 6.2 命令解析：主循环 vs 中断

| 方案 | 优点 | 缺点 |
|------|------|------|
| 主循环解析 (当前) | 不阻塞 ISR；可 __WFI() 休眠；调试方便 | 响应延迟取决于循环周期 |
| 中断中解析 | 响应快；无主循环延迟 | 阻塞其他中断；大量浮点运算导致 ISR 过长 |

**选用理由**：Motion 命令包含浮点密集型 Profile 构造（~μs 级），不应在中断上下文中执行。主循环在无数据时 `__WFI()` 休眠，不浪费 CPU。

### 6.3 发送方式：忙等待 vs DMA

| 方案 | 优点 | 缺点 |
|------|------|------|
| DMA + 忙标志 (当前) | 非阻塞；CPU 利用率高 | 最差 64B 缓冲区限制 |
| HAL_UART_Transmit (阻塞) | 简单；无缓冲区限制 | 阻塞主循环 ~43μs (Ack) |
| DMA + 链表/循环队列 | 无缓冲区限制 | 复杂度高；场景不需要 |

**选用理由**：所有响应帧 ≤ 12 字节，64B 缓冲区足够。非阻塞 DMA 使发送期间 CPU 可继续处理命令。

### 6.4 队列深度：128 的设计依据

```
推算:
  运动段最短 process_time = 0.1s
  10kHz 控制周期 = 0.0001s
  每段更新次数 = 0.1 / 0.0001 = 1000 次

  队列 128 段 = 最短 12.8s 的连续运动缓冲
  SRAM 占用 ~8 KB ≈ 40%

权衡:
  深度增大 → 更多 SRAM, 但允许更长前瞻
  深度减小 → SRAM 节省, 但上位机需更频繁发送

结论: 128 在 SRAM 约束和缓冲能力之间取得平衡
```

### 6.5 环形缓冲区 1023 的设计依据

```
  921600 bps = 115,000 字节/秒
  DMA 每次触发 ~256 字节 (半缓冲)
  两次 DMA 触发间隔 ≈ 256 / 115000 ≈ 2.2 ms

  RingBuffer 1023 = ~8.9ms 数据缓冲
  足够覆盖主循环最大响应延迟

  选择 1023 而非 1024:
    RingBuffer 实现使用 Capacity+1 区分满/空
    storage = 1024, capacity = 1023
    1024 是 2 的幂 → 可使用位运算优化
```

---

## 7. 时序分析

### 7.1 控制时序

```
       100μs              100μs              100μs
TIM1:  ├────────┤─────────├────────┤─────────├────────┤
       │ ISR    │         │ ISR    │         │ ISR    │
       │ ~3μs   │         │ ~3μs   │         │ ~3μs   │
       └────────┘         └────────┘         └────────┘
       
User                                            ┌──────┐
Loop:                                           │ cmd  │
       ┌──────────────WFI───────────────......──┤proc  │
                                                └──────┘
                                                
UART                                            │24B   │
RX:                                         .....│RX    │
                                                └──────┘
```

### 7.2 运动段生命周期

```
上位机发送 MotionFrame ──→ 下位机 Ack ──→ segment_queue push
                                                │
                                         等待 TIM1 调度
                                                │
               ┌────────────────────────────────┘
               ▼
         update_handler: front()
               │
         elapsed += 100μs
         current_step(elapsed_ms)
               │
         循环直到 elapsed >= duration_us
               │
         pop() → 该段完成
               │
         开始下一段 (或无段 → 停止)
```

### 7.3 关键路径延迟

```
上位机 → UART → 下位机 → 处理 → 响应 → UART → 上位机

Motion 命令:
  24B 传输:      260 μs  (UART)
  DMA 中断延迟:   ~5 μs  (NVIC)
  ISR:            ~2 μs  (DMA IDLE)
  RingBuffer push: ~1 μs
  user_loop 调度:  0-100 μs (取决于当前状态)
  帧解析:          ~2 μs
  Profile 构造:   ~5 μs (浮点运算)
  Ack 发送:       43 μs (4B DMA)
  ─────────────────────────────────
  总计:         ~320-420 μs (理论最小值)

实际观测 (含 CH340 + Python):
  ~2-5 ms (受上位机调度和 USB 驱动延迟影响)
```

---

## 8. 上位机协议实现伪代码

### 8.1 指令同步

```python
class ScaraController:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, 921600, timeout=0.5)
        self.cmd_seq = 0
    
    def send_motion(self, angles, speeds, process_time):
        """发送运动指令，等待 ACK/NACK"""
        for retry in range(3):
            self._send_frame(MOTION_CMD, pack(
                '<4f', angles[0], angles[1], speeds[0], speeds[1], process_time
            ))
            response = self._read_frame(timeout=0.5)
            
            if response is None:
                # 超时，尝试恢复同步
                if not self._resync():
                    raise DeviceLostError()
                continue
            
            if response.cmd == ACK:
                return True
            elif response.cmd == NACK:
                reason = response.data[0]
                if reason == INVALID_DATA:
                    raise InvalidParameterError()
                elif reason == EXECUTION_FAILED:
                    continue  # 重试
                    
        raise MaxRetryError()
    
    def _resync(self) -> bool:
        """尝试恢复通信同步"""
        self._drain()
        for _ in range(3):
            self._send_frame(QUERY_CMD, b'')
            resp = self._read_frame(timeout=0.2)
            if resp and resp.cmd == STATUS_RESP:
                return True
        return False
```

### 8.2 段间延时控制

```python
class MotionPipeline:
    def __init__(self):
        self.virtual_queue_time = 0.0  # 虚拟队列总剩余时间
        self.segment_count = 0
    
    def can_send(self, process_time: float) -> bool:
        """判断是否可以发送新段"""
        return self.segment_count < 96  # HIGH_WATERMARK = 96
    
    def on_send(self, process_time: float):
        self.virtual_queue_time += process_time
        self.segment_count += 1
    
    def on_ack(self):
        pass  # 不需要额外操作
    
    def update(self, dt: float):
        """定时调用，从虚拟队列扣除已消耗时间"""
        self.virtual_queue_time = max(0, self.virtual_queue_time - dt)
        # segment_count 可近似通过 virtual_queue_time / avg_process_time 估算
```

### 8.3 速度限制

```python
def validate_motion(angles, speeds, process_time):
    """发送前校验"""
    # 关节限位检查
    if not (90 <= degrees(angles[0]) <= 210):
        raise OutOfRangeError("J1 out of range")
    if not (-30 <= degrees(angles[1]) <= 90):
        raise OutOfRangeError("J2 out of range")
    
    # 速度限制检查
    MAX_SPEED = 2.0  # rad/s
    if abs(speeds[0]) > MAX_SPEED or abs(speeds[1]) > MAX_SPEED:
        raise SpeedLimitError(f"Speed exceeds {MAX_SPEED} rad/s")
    
    # 加速度约束 (相邻段速度差)
    if last_speeds is not None:
        MAX_ACCEL = 10.0  # rad/s²
        jerk_0 = abs(speeds[0] - last_speeds[0]) / process_time
        jerk_1 = abs(speeds[1] - last_speeds[1]) / process_time
        if jerk_0 > MAX_ACCEL or jerk_1 > MAX_ACCEL:
            raise AccelLimitError(f"Acceleration exceeds {MAX_ACCEL} rad/s²")
    
    # process_time 合理性检查
    if process_time <= 0:
        raise InvalidParameterError("process_time must > 0")
    if process_time < 0.05:
        raise InvalidParameterError("process_time too short for reliable execution")
```

---

## 9. 代码结构速查

```
user/
├── user_main.cpp          主循环 + UART 收发 + 命令分发
├── user_main.h            C 接口头文件 (user_init, user_loop)
├── protocol.hpp           帧结构 + 序列化
├── polinomial_profile.hpp 三次多项式轨迹
├── motor.hpp              Motor + Controller
├── ring_buffer.hpp        SPSC 环形缓冲区
├── spsc_queue.hpp         SPSC 固定容量队列
├── config.hpp             电机 + 运动学参数
├── type_def.hpp           MotorConfig 结构体
└── math_utils.hpp         工具函数 (sqrt, deg/rad 转换)

Core/Src/
├── main.c                 入口 (HAL_Init → user_init → while{user_loop})
├── stm32f1xx_it.c         中断处理 (TIM1, USART1, DMA)
├── tim.c                  TIM 初始化
├── usart.c                USART 初始化
├── gpio.c                 GPIO 初始化
└── dma.c                  DMA 初始化
```

---

## 10. 常见问题

### 10.1 如何计算当前电机角度？

```python
# 下位机返回的是相对于机械零位的绝对角度 (rad)
# 可直接用于逆解计算

# QueryStatus 获取:
response = send_query()
angle_j1 = response.angle[0]  # rad
angle_j2 = response.angle[1]  # rad

# 转换为度:
deg_j1 = angle_j1 * 180 / math.pi
deg_j2 = angle_j2 * 180 / math.pi
```

### 10.2 如何规划一段连续运动？

```
单段: target_angle + target_speed + process_time
     → 下位机从当前位置平滑运动到 target_angle
     → 到达时速度为 target_speed

多段连续:
  段1: angle=[1.7, 0.5], speed=[0.5, 0.3], time=0.3s
  段2: angle=[2.0, 1.0], speed=[0.3, 0.2], time=0.4s
       ↑ 段1 末端速度 = 段2 起始速度 (速度连续)
```

### 10.3 出现 NACK 如何处理？

| NACK 原因 | 可能原因 | 处理 |
|-----------|---------|------|
| InvalidCommand | 协议版本不匹配 | 检查 cmd 值 |
| InvalidData | 角度超限 / process_time 无效 | 检查参数范围 |
| ExecutionFailed | 队列满 / 段无效 | 等待后重试 |

### 10.4 通信超时如何处理？

```
1. drain 串口缓冲区 (读 2ms)
2. 发送 QueryStatusFrame
3. 等待 200ms:
   - 收到 StatusResponse → 恢复
   - 超时 → 重复步骤 2-3，最多 3 次
4. 全部失败 → 关闭串口 → 报告设备丢失
```
