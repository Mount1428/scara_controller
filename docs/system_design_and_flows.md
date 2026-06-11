# SCARA 机械臂下位机控制系统设计目标、功能模块与流程分析

## 摘要

本文档系统阐述 SCARA 机械臂下位机控制系统的设计目标、功能模块划分、各模块实现方式以及模块内部与模块间的处理流程。系统基于 STM32F103 微控制器，采用分层架构实现通信协议解析、轨迹规划、运动队列管理和步进电机控制。通过 DMA 双缓冲接收机制保障通信效率，通过单生产者单消费者无锁队列实现主循环与定时器中断间的数据传递，通过三次多项式插值实现关节空间平滑轨迹生成。文档以流程图形式呈现各功能模块的内部逻辑和模块间的协作关系。

**关键词：** SCARA 机械臂；STM32F103；控制系统设计；功能模块；流程图

---

## 1. 设计目标

### 1.1 总体目标

本系统作为 SCARA 机械臂的下位机控制器，其总体设计目标是接收上位机发送的关节空间运动指令，在微控制器端完成轨迹插值与步进电机脉冲输出，实现双关节的协调运动控制。系统定位为"关节空间下位机执行器"，而非完整的运动学求解器。

### 1.2 具体设计指标

| 设计指标 | 目标值 | 说明 |
|---|---|---|
| 控制轴数 | 2 轴 | SCARA 机械臂大臂和小臂关节 |
| 通信波特率 | 921600 bps | 高速串口通信 |
| 控制周期 | 100 μs (10 kHz) | TIM1 定时器中断周期 |
| 最大单轴步频 | 10 kHz | 受控制周期约束 |
| 每转脉冲数 | 3200 | 步进电机细分设置 |
| 运动段队列深度 | 127 段 | SPSC 队列有效容量 |
| UART 接收缓冲 | 1023 字节 | 环形缓冲区容量 |
| 运动段最大时间 | 理论无限制 | 受浮点精度约束 |

### 1.3 设计约束

1. **硬件约束**：MCU 无 FPU，浮点运算通过软件仿真实现；
2. **实时性约束**：脉冲生成必须在定时器中断中完成，确保时序精度；
3. **通信约束**：协议采用二进制固定帧格式，无 CRC 校验；
4. **控制约束**：系统为开环步进控制，位置反馈来自内部步数计数而非外部传感器。

---

## 2. 系统架构

### 2.1 整体架构概览

下位机系统采用分层架构设计，自下而上分为硬件抽象层、设备驱动层、服务支撑层和应用逻辑层。各层之间通过明确的接口进行交互，实现关注点分离和模块解耦。

```mermaid
graph TB
    subgraph "应用逻辑层 Application Layer"
        A1[协议解析<br/>user_loop]
        A2[轨迹规划<br/>PolinomialProfile]
        A3[运动控制<br/>Controller]
        A4[电机驱动<br/>Motor]
    end

    subgraph "服务支撑层 Service Layer"
        S1[通信服务<br/>RingBuffer/SPSC Queue]
        S2[定时服务<br/>TIM1 100us]
        S3[配置服务<br/>config.hpp]
    end

    subgraph "设备驱动层 Device Driver Layer"
        D1[UART 驱动<br/>USART1]
        D2[DMA 驱动<br/>DMA1 CH4/CH5]
        D3[定时器驱动<br/>TIM1]
        D4[GPIO 驱动<br/>PA0-PA5]
    end

    subgraph "硬件抽象层 HAL Layer"
        H1[STM32 HAL 库]
        H2[CMSIS]
    end

    subgraph "硬件层 Hardware"
        HW1[STM32F103C8<br/>Cortex-M3 @ 72MHz]
        HW2[USART1<br/>PA9-TX PA10-RX]
        HW3[步进电机驱动器<br/>PUL/DIR/ENA]
    end

    A1 --> S1
    A2 --> A3
    A3 --> A4
    A4 --> D4

    S1 --> D1
    S1 --> D2
    S2 --> D3

    D1 --> H1
    D2 --> H1
    D3 --> H1
    D4 --> H1

    H1 --> H2
    H2 --> HW1
    HW1 --> HW2
    HW1 --> HW3

    style A1 fill:#fff3e0
    style A2 fill:#fff3e0
    style A3 fill:#fff3e0
    style A4 fill:#fff3e0
    style S1 fill:#e1f5fe
    style S2 fill:#e1f5fe
    style S3 fill:#e1f5fe
    style D1 fill:#e8f5e8
    style D2 fill:#e8f5e8
    style D3 fill:#e8f5e8
    style D4 fill:#e8f5e8
    style H1 fill:#f3e5f5
    style H2 fill:#f3e5f5
    style HW1 fill:#ffebee
    style HW2 fill:#ffebee
    style HW3 fill:#ffebee
```

### 2.2 各层职责说明

| 层次 | 职责 | 主要组件 |
|---|---|---|
| **硬件层** | 物理设备，包括 MCU、外设和执行机构 | STM32F103C8、USART1、步进电机驱动器 |
| **硬件抽象层** | 提供统一的硬件访问接口，屏蔽底层差异 | STM32 HAL 库、CMSIS |
| **设备驱动层** | 封装具体外设的操作，提供设备级服务 | UART 驱动、DMA 驱动、定时器驱动、GPIO 驱动 |
| **服务支撑层** | 提供通用服务，如数据缓冲、定时调度、配置管理 | RingBuffer、SPSC Queue、TIM1 调度、config |
| **应用逻辑层** | 实现业务逻辑，包括协议处理、轨迹规划和运动控制 | user_loop、PolinomialProfile、Controller、Motor |

### 2.3 模块依赖关系架构图

下图展示各功能模块之间的依赖关系和数据流向：

```mermaid
graph LR
    subgraph "外部接口"
        PC[上位机<br/>串口]
        MOTOR[步进电机<br/>驱动器]
    end

    subgraph "通信模块"
        UART[UART-DMA<br/>双缓冲]
        RING[RingBuffer<br/>环形缓冲]
    end

    subgraph "协议模块"
        PARSE[帧解析<br/>user_loop]
        PROTO[协议定义<br/>protocol.hpp]
    end

    subgraph "控制模块"
        CTRL[Controller<br/>运动队列]
        QUEUE[SPSC Queue<br/>无锁队列]
    end

    subgraph "轨迹模块"
        PROF[PolinomialProfile<br/>三次多项式]
    end

    subgraph "电机模块"
        MOT[Motor<br/>步进控制]
        GPIO[GPIO<br/>PUL/DIR/ENA]
    end

    subgraph "定时模块"
        TIM[TIM1<br/>100us 周期]
    end

    PC -->|串口数据| UART
    UART -->|DMA 中断| RING
    RING -->|peek/pop| PARSE
    PARSE -->|帧结构| PROTO
    PARSE -->|构造轨迹| PROF
    PROF -->|Segment| CTRL
    CTRL -->|入队| QUEUE
    TIM -->|周期中断| CTRL
    CTRL -->|出队执行| MOT
    MOT -->|单步请求| GPIO
    GPIO -->|PUL/DIR/ENA| MOTOR

    QUEUE -.->|主循环写<br/>中断读| CTRL

    style PC fill:#e1f5fe
    style MOTOR fill:#e8f5e8
    style UART fill:#fff3e0
    style RING fill:#fff3e0
    style PARSE fill:#fff3e0
    style PROTO fill:#fff3e0
    style CTRL fill:#e8f5e8
    style QUEUE fill:#e8f5e8
    style PROF fill:#f3e5f5
    style MOT fill:#e8f5e8
    style GPIO fill:#e8f5e8
    style TIM fill:#ffebee
```

### 2.4 执行上下文架构图

系统存在三个并发执行的上下文，各自承担不同职责：

```mermaid
graph TB
    subgraph "主循环上下文 Main Loop"
        M1[user_loop<br/>协议解析]
        M2[轨迹构造]
        M3[入队 add_segment]
    end

    subgraph "UART 中断上下文"
        U1[DMA 接收完成]
        U2[缓冲区切换]
        U3[push 到 RingBuffer]
    end

    subgraph "TIM1 中断上下文 优先级最高"
        T1[update_handler]
        T2[轨迹插值]
        T3[request_step]
        T4[step_once 脉冲输出]
    end

    subgraph "共享资源"
        R1[RingBuffer<br/>原子头尾指针]
        R2[SPSC Queue<br/>原子头尾指针]
        R3[Motor::current_step<br/>单写者]
    end

    U1 --> U2
    U2 --> U3
    U3 -->|写入| R1
    R1 -->|读取| M1

    M1 --> M2
    M2 --> M3
    M3 -->|写入| R2

    R2 -->|读取| T1
    T1 --> T2
    T2 --> T3
    T3 --> T4
    T3 -->|写入| R3

    style M1 fill:#fff3e0
    style M2 fill:#fff3e0
    style M3 fill:#fff3e0
    style U1 fill:#e1f5fe
    style U2 fill:#e1f5fe
    style U3 fill:#e1f5fe
    style T1 fill:#ffebee
    style T2 fill:#ffebee
    style T3 fill:#ffebee
    style T4 fill:#ffebee
    style R1 fill:#e8f5e8
    style R2 fill:#e8f5e8
    style R3 fill:#e8f5e8
```

### 2.5 中断优先级与实时性保障

| 中断源 | 优先级 | 抢占/子优先级 | 周期/触发条件 | 实时性要求 |
|---|---|---|---|---|
| TIM1 更新 | 最高 | 0/0 | 100 μs | 硬实时：脉冲时序必须精确 |
| USART1 IDLE | 中 | 1/0 | 数据帧到达 | 软实时：允许少量延迟 |
| DMA1 CH4/CH5 | 中 | 0/0 | 传输完成 | 软实时：配合 UART |
| SysTick | 最低 | 默认 | 1 ms | 非实时：仅用于 HAL 时基 |

---

## 3. 功能模块划分

系统功能模块可划分为以下六个主要部分：

```mermaid
graph TB
    subgraph "应用层"
        A[协议解析模块]
        B[轨迹规划模块]
    end

    subgraph "控制层"
        C[运动控制模块]
        D[定时器调度模块]
    end

    subgraph "驱动层"
        E[通信驱动模块]
        F[电机驱动模块]
    end

    A --> B
    B --> C
    C --> F
    D --> C
    E --> A
```

### 2.1 模块职责概述

| 模块名称 | 主要文件 | 职责描述 |
|---|---|---|
| 通信驱动模块 | `usart.c`、`dma.c`、`user_main.cpp` | UART-DMA 双缓冲接收、环形缓冲区管理、DMA 发送 |
| 协议解析模块 | `protocol.hpp`、`user_main.cpp` | 帧同步、命令分派、数据合法性检查、响应生成 |
| 轨迹规划模块 | `polinomial_profile.hpp` | 三次多项式系数计算、目标步数插值、速度衔接 |
| 运动控制模块 | `motor.hpp` | 运动段队列管理、周期调度、电机脉冲控制 |
| 定时器调度模块 | `tim.c`、`stm32f1xx_it.c` | TIM1 100 μs 周期中断、回调注册与转发 |
| 电机驱动模块 | `motor.hpp`、`gpio.c` | GPIO 电平控制、角度步数转换、限位检查 |

---

## 3. 功能模块详细设计与实现

### 3.1 通信驱动模块

#### 3.1.1 设计目标

实现高效可靠的串口数据收发，满足 921600 bps 波特率下的实时通信需求，避免主循环阻塞在数据等待上。

#### 3.1.2 实现方式

**接收路径**：采用 DMA + IDLE 中断 + 双缓冲 + 环形缓冲四层机制。

- 硬件层：USART1 配置为 921600-8N1，DMA1_Channel5 负责接收（`Core/Src/usart.c:43-51`）；
- DMA 层：使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 启动接收，IDLE 中断触发回调（`user/user_main.cpp:32-36`）；
- 缓冲层：双缓冲 `uart_rx_raw_buffer[2][512]` 交替接收，避免数据覆盖（`user/user_main.cpp:21-22`）；
- 应用层：接收数据推入 `RingBuffer<1023>`，由主循环消费（`user/user_main.cpp:38-46`）。

**发送路径**：采用 DMA + 忙标志机制。

- 静态缓冲区 `uart_tx_raw_buffer[64]` 暂存待发数据；
- `uart_tx_busy` 原子标志防止并发发送；
- TX complete 回调清除忙标志（`user/user_main.cpp:48-63`）。

#### 3.1.3 接收流程图

```mermaid
flowchart TD
    A[USART1 RX 引脚] --> B[DMA1_Channel5 搬运]
    B --> C{IDLE 中断触发?}
    C -->|否| B
    C -->|是| D[uart_rx_event_callback]
    D --> E[获取当前缓冲区索引]
    E --> F[切换到备用缓冲区]
    F --> G[重启 DMA 接收]
    G --> H[将数据推入 RingBuffer]
    H --> I[返回等待下次中断]

    style D fill:#e1f5fe
    style H fill:#e8f5e8
```

#### 3.1.4 发送流程图

```mermaid
flowchart TD
    A[uart_send<T>] --> B{uart_tx_busy?}
    B -->|是| C[忙等待]
    C --> B
    B -->|否| D[设置 uart_tx_busy = true]
    D --> E[memcpy 到发送缓冲区]
    E --> F[HAL_UART_Transmit_DMA]
    F --> G[等待 TX complete 中断]
    G --> H[清除 uart_tx_busy]

    style D fill:#fff3e0
    style F fill:#e1f5fe
```

### 3.2 协议解析模块

#### 3.2.1 设计目标

实现二进制固定帧协议的同步、解析和命令分派，支持 Motion、EmergencyStop、QueryStatus 三种命令，并返回 ACK/NACK/StatusResponse 响应。

#### 3.2.2 实现方式

协议定义于 `user/protocol.hpp`，采用 `#pragma pack(push, 1)` 保证 1 字节对齐。帧头固定为 `0xAA 0x55`，Header 占 4 字节（含 padding）。命令码采用 `uint8_t` 枚举，帧长度根据命令类型固定。

解析逻辑实现于 `user_loop()`，采用逐字节窥视的帧同步策略：先检查首字节是否为 `0xAA`，再读取完整 Header 验证帧头有效性，最后根据命令码分派处理（`user/user_main.cpp:91-228`）。

#### 3.2.3 帧同步流程图

```mermaid
flowchart TD
    A[user_loop 开始] --> B{RingBuffer 有数据?}
    B -->|否| C[__WFI 等待中断]
    C --> A
    B -->|是| D[peek 1 字节]
    D --> E{是 0xAA?}
    E -->|否| F[pop 1 字节丢弃]
    F --> A
    E -->|是| G[peek sizeof Header]
    G --> H{Header 完整?}
    H -->|否| I[等待更多数据]
    I --> A
    H -->|是| J{header.is_valid?}
    J -->|否| K[pop 1 字节丢弃]
    K --> A
    J -->|是| L[读取完整帧]
    L --> M{命令类型}
    M -->|Motion| N[处理运动命令]
    M -->|EmergencyStop| O[处理急停命令]
    M -->|QueryStatus| P[处理查询命令]
    M -->|其他| Q[pop Header, 返回 NACK]

    style E fill:#e8f5e8
    style J fill:#e8f5e8
    style N fill:#fff3e0
    style O fill:#ffcdd2
    style P fill:#e1f5fe
```

#### 3.2.4 Motion 命令处理流程图

```mermaid
flowchart TD
    A[收到 Motion 命令] --> B[读取 MotionCommandFrame]
    B --> C[pop 帧数据]
    C --> D{浮点合法性检查}
    D -->|失败| E[返回 NACK: InvalidData]
    D -->|通过| F{软限位检查}
    F -->|超限| E
    F -->|通过| G[读取队列末段末速度]
    G --> H{队列为空?}
    H -->|是| I[起始速度 = 0]
    H -->|否| J[起始速度 = 末段末速度]
    I --> K[构造 x/y 三次多项式轨迹]
    J --> K
    K --> L{轨迹有效?}
    L -->|否| E
    L -->|是| M[入队 Controller::add_segment]
    M --> N{入队成功?}
    N -->|否| O[返回 NACK: ExecutionFailed]
    N -->|是| P[返回 ACK]

    style D fill:#e8f5e8
    style F fill:#e8f5e8
    style L fill:#e8f5e8
    style N fill:#e8f5e8
    style P fill:#c8e6c9
    style E fill:#ffcdd2
    style O fill:#ffcdd2
```

### 3.3 轨迹规划模块

#### 3.3.1 设计目标

在关节空间内生成平滑的轨迹曲线，保证位置连续且端点速度满足指定约束，支持多段轨迹的连续衔接。

#### 3.3.2 实现方式

轨迹规划由 `PolinomialProfile` 类实现，支持 Linear 和 Cubic 两种插值类型。当前工程实际使用 Cubic 模式。构造函数根据起始步数、终止步数、运动时间、起始速度和终止速度计算三次多项式系数（`user/polinomial_profile.hpp:30-73`）。

三次多项式形式为：

$$
q(t) = q_s + a_1 t + a_2 t^2 + a_3 t^3
$$

系数由边界条件 $q(0)=q_s$、$q(T)=q_e$、$\dot q(0)=v_s$、$\dot q(T)=v_e$ 确定。

#### 3.3.3 轨迹生成流程图

```mermaid
flowchart TD
    A[输入参数] --> B[起始步数 q_s]
    A --> C[终止步数 q_e]
    A --> D[运动时间 T]
    A --> E[起始速度 v_s]
    A --> F[终止速度 v_e]

    B --> G{检查 T > 0}
    C --> G
    D --> G
    E --> G
    F --> G

    G -->|否| H[标记无效]
    G -->|是| I[计算系数 a1 = vs]
    I --> J[计算系数 a2]
    J --> K[计算系数 a3]
    K --> L[保存 offset_step = qs]
    L --> M[保存 steps = qe - qs]

    N[current_step 调用] --> O{检查有效性}
    O -->|无效| P[返回起始步数]
    O -->|有效| Q{时间 t > 0?}
    Q -->|否| P
    Q -->|是| R{插值类型}
    R -->|Linear| S[round(a1*t) + offset]
    R -->|Cubic| T[round(a1*t + a2*t^2 + a3*t^3) + offset]

    style G fill:#e8f5e8
    style O fill:#e8f5e8
    style Q fill:#e8f5e8
    style S fill:#e1f5fe
    style T fill:#fff3e0
```

#### 3.3.4 系数计算公式

$$
a_1 = v_s
$$

$$
a_2 = \frac{3(q_e - q_s) - (2v_s + v_e)T}{T^2}
$$

$$
a_3 = \frac{(v_s + v_e)T - 2(q_e - q_s)}{T^3}
$$

### 3.4 运动控制模块

#### 3.4.1 设计目标

管理运动段队列，在定时器中断中按时间推进轨迹，调度两个电机同步执行步进脉冲输出。

#### 3.4.2 实现方式

`Controller<Capacity>` 模板类管理 `SpscQueue<Segment, 128>` 运动段队列。每个 Segment 包含持续时间和两轴轨迹（`user/motor.hpp:123-131`）。主循环调用 `add_segment()` 入队，TIM1 中断调用 `update_handler()` 出队执行（`user/motor.hpp:138-188`）。

`Motor` 类负责角度到步数的转换和 GPIO 脉冲输出。`request_step()` 每次最多推进一步，通过 PUL/DIR/ENA 引脚驱动步进电机（`user/motor.hpp:49-78`）。

#### 3.4.3 Controller::update_handler 流程图

```mermaid
flowchart TD
    A[TIM1 中断触发] --> B[update_handler dt=100us]
    B --> C{队列有运动段?}
    C -->|否| D[返回]
    C -->|是| E[获取队首 Segment]
    E --> F{elapsed_time >= duration?}
    F -->|是| G[pop 队首段]
    G --> H[重置 elapsed_time = 0]
    H --> I[修正到终点步数]
    I --> J[step_once 输出脉冲]
    J --> D
    F -->|否| K[elapsed_time += dt]
    K --> L[计算 elapsed_time_ms]
    L --> M[x_profile.current_step]
    M --> N[y_profile.current_step]
    N --> O[motor_x.request_step]
    O --> P[motor_y.request_step]
    P --> J

    style C fill:#e8f5e8
    style F fill:#e8f5e8
    style G fill:#fff3e0
    style J fill:#e1f5fe
```

#### 3.4.4 Motor::request_step 流程图

```mermaid
flowchart TD
    A[request_step 目标步数] --> B{目标步数在限位内?}
    B -->|否| C[返回 false]
    B -->|是| D{目标 == 当前?}
    D -->|是| C
    D -->|否| E[enable 使能电机]
    E --> F{目标 > 当前?}
    F -->|是| G[set_direction 正向]
    F -->|否| H[set_direction 反向]
    G --> I[current_step++]
    H --> J[current_step--]
    I --> K[set_pulse 高电平]
    J --> K
    K --> L[返回 true]

    style B fill:#e8f5e8
    style D fill:#e8f5e8
    style F fill:#e8f5e8
    style K fill:#e1f5fe
    style L fill:#c8e6c9
```

#### 3.4.5 step_once 脉冲宽度保证

```mermaid
flowchart TD
    A[step_once] --> B{任一电机更新?}
    B -->|否| C[返回]
    B -->|是| D[双层循环 __NOP]
    D --> E[约 6us 延时]
    E --> F[motor_x.set_pulse 低电平]
    F --> G[motor_y.set_pulse 低电平]
    G --> C

    style B fill:#e8f5e8
    style D fill:#fff3e0
    style F fill:#e1f5fe
    style G fill:#e1f5fe
```

### 3.5 定时器调度模块

#### 3.5.1 设计目标

提供精确的 100 μs 周期中断，作为运动控制的时间基准，确保轨迹插值和脉冲输出的时序精度。

#### 3.5.2 实现方式

TIM1 配置为 72 MHz / 72 分频 / 100 周期 = 100 μs 中断周期（`Core/Src/tim.c:43-46`）。中断优先级设为 0（最高），确保脉冲输出不受其他中断干扰（`Core/Src/tim.c:83-84`）。中断处理函数通过 HAL 回调机制转发到 `controller.update_handler(100)`（`user/user_main.cpp:81-88`）。

#### 3.5.3 定时器配置参数

| 参数 | 值 | 说明 |
|---|---|---|
| 时钟源 | 72 MHz | HSE + PLL ×9 |
| 预分频器 | 72 - 1 = 71 | 计数频率 1 MHz |
| 周期 | 100 - 1 = 99 | 中断周期 100 μs |
| 中断优先级 | 0, 0 | 最高优先级 |

### 3.6 电机驱动模块

#### 3.6.1 设计目标

将关节角度转换为步进电机脉冲序列，通过 GPIO 引脚控制 PUL/DIR/ENA 信号，实现单步精确控制。

#### 3.6.2 实现方式

`Motor` 类封装角度到步数的转换逻辑和 GPIO 操作。转换公式为：

$$
step = round\left( \theta \cdot \frac{N}{2\pi} \right) - offset
$$

其中 $N=3200$ 为每转脉冲数，$offset$ 为上电偏移步数（`user/motor.hpp:80-83`）。

GPIO 引脚分配：

| 信号 | 电机 0 | 电机 1 |
|---|---|---|
| PUL | PA0 | PA1 |
| DIR | PA2 | PA3 |
| ENA | PA4 | PA5 |

---

## 4. 模块间协作流程

### 4.1 系统启动流程

```mermaid
flowchart TD
    A[系统上电] --> B[HAL_Init]
    B --> C[SystemClock_Config 72MHz]
    C --> D[MX_GPIO_Init]
    D --> E[MX_DMA_Init]
    E --> F[MX_TIM1_Init 100us]
    F --> G[MX_USART1_UART_Init 921600]
    G --> H[user_init]
    H --> I[设置波特率]
    I --> J[注册 UART RX 回调]
    J --> K[注册 UART TX 回调]
    K --> L[启动 DMA 接收]
    L --> M[注册 TIM1 回调]
    M --> N[启动 TIM1 中断]
    N --> O[进入主循环]
    O --> P[user_loop]

    style A fill:#e8f5e8
    style H fill:#fff3e0
    style O fill:#e1f5fe
```

### 4.2 完整运动控制流程

```mermaid
sequenceDiagram
    participant PC as 上位机
    participant UART as UART-DMA
    participant RingBuf as RingBuffer
    participant MainLoop as user_loop
    participant Queue as SPSC Queue
    participant TIM1 as TIM1 中断
    participant Motor as 电机驱动

    PC->>UART: 发送 MotionCommandFrame
    UART->>RingBuf: DMA 接收 + IDLE 中断
    RingBuf->>MainLoop: peek/pop 解析帧
    MainLoop->>MainLoop: 数据合法性检查
    MainLoop->>MainLoop: 软限位检查
    MainLoop->>MainLoop: 构造三次多项式轨迹
    MainLoop->>Queue: add_segment 入队
    MainLoop->>PC: 返回 ACK

    loop 每 100us
        TIM1->>Queue: 获取队首 Segment
        Queue-->>TIM1: 返回 Segment
        TIM1->>TIM1: 计算目标步数
        TIM1->>Motor: request_step
        Motor->>Motor: GPIO 脉冲输出
    end
```

### 4.3 急停处理流程

```mermaid
sequenceDiagram
    participant PC as 上位机
    participant MainLoop as user_loop
    participant Controller as Controller
    participant Motor as 电机

    PC->>MainLoop: 发送 EmergencyStopFrame
    MainLoop->>Controller: emergency_stop
    Controller->>Motor: enable(false) 禁用电机
    Controller->>Controller: 清空运动队列
    Controller->>Controller: 重置时间计数
    Controller-->>MainLoop: 完成
    MainLoop->>PC: 返回 ACK
```

### 4.4 状态查询流程

```mermaid
sequenceDiagram
    participant PC as 上位机
    participant MainLoop as user_loop
    participant Motor as Motor
    participant Protocol as Protocol

    PC->>MainLoop: 发送 QueryStatusFrame
    MainLoop->>Motor: motor_0.current_step()
    Motor-->>MainLoop: 返回当前步数
    MainLoop->>Motor: motor_1.current_step()
    Motor-->>MainLoop: 返回当前步数
    MainLoop->>Motor: step_to_rad 转换
    MainLoop->>Protocol: 构造 StatusResponseFrame
    MainLoop->>PC: 返回两轴角度
```

---

## 5. 数据流与并发模型

### 5.1 执行上下文划分

系统存在三个主要执行上下文，各自承担不同职责：

| 执行上下文 | 优先级 | 主要职责 | 共享资源 |
|---|---|---|---|
| 主循环 | 最低 | 协议解析、轨迹构造、入队 | RingBuffer (读), SPSC Queue (写) |
| UART/DMA 中断 | 中 | 数据接收、缓冲切换 | RingBuffer (写), DMA 缓冲区 |
| TIM1 中断 | 最高 | 轨迹执行、脉冲输出 | SPSC Queue (读), Motor 状态 |

### 5.2 无锁并发机制

| 共享资源 | 生产者 | 消费者 | 同步机制 |
|---|---|---|---|
| RingBuffer | UART 中断 | 主循环 | 原子头尾指针，单生产者单消费者 |
| SPSC Queue | 主循环 | TIM1 中断 | 原子头尾指针，容量为 2 的幂 |
| uart_tx_busy | 主循环 | TX 中断 | 原子布尔标志 |
| Motor::current_step_ | TIM1 中断 | 主循环 (读) | 单写者，允许短暂不一致 |

### 5.3 数据流图

```mermaid
flowchart LR
    subgraph "输入"
        A[上位机串口]
    end

    subgraph "接收路径"
        B[DMA 双缓冲]
        C[RingBuffer]
    end

    subgraph "主循环"
        D[帧解析]
        E[轨迹构造]
        F[运动段入队]
    end

    subgraph "实时路径"
        G[SPSC Queue]
        H[TIM1 调度]
        I[电机脉冲]
    end

    subgraph "输出"
        J[步进电机]
    end

    A --> B
    B --> C
    C --> D
    D --> E
    E --> F
    F --> G
    G --> H
    H --> I
    I --> J

    style A fill:#e1f5fe
    style J fill:#e8f5e8
```

---

## 6. 总结

本文档系统阐述了 SCARA 机械臂下位机控制系统的设计目标、功能模块划分和实现方式。系统采用分层架构，将通信驱动、协议解析、轨迹规划、运动控制和电机驱动划分为独立模块，通过 RingBuffer 和 SPSC Queue 实现模块间的数据传递，通过定时器中断保障实时性。

各模块内部流程和模块间协作关系通过 Mermaid 流程图直观呈现，为系统理解、维护和扩展提供参考。当前系统已实现基本的关节空间运动控制功能，未来可在此基础上扩展运动学求解、闭环控制和更高级的轨迹规划算法。
