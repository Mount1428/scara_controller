# SCARA 机械臂下位机控制程序实现与处理流程分析

## 摘要

本文面向 `scara_controller` 工程的当前源码，对其软件结构、通信机制、运动控制算法、实时调度方式以及上位机测试支撑进行系统性分析。该工程是一个面向 SCARA 双关节机械臂的 STM32 下位机固件，底层由 STM32CubeMX 生成 HAL 初始化代码，上层在 `user/` 目录中以 C++23 实现通信协议解析、运动段队列管理、三次多项式轨迹生成以及步进电机 GPIO 脉冲控制。系统运行时以 `main()` 为入口，完成 GPIO、DMA、USART1 和 TIM1 初始化后调用 `user_init()` 注册通信与定时器回调，并在主循环中持续执行 `user_loop()`。通信层采用 USART1、DMA、IDLE 接收事件、双缓冲与环形缓冲组合，以二进制固定帧完成运动控制、急停和状态查询；控制层采用 TIM1 周期中断驱动，在 100 μs 周期内依据轨迹曲线生成关节目标步数，并通过 PUL/DIR/ENA 引脚驱动两路步进电机。分析表明，当前固件更准确地说是“SCARA 机械臂关节空间下位机执行器”，其源码中保留了 SCARA 几何参数，但尚未实现完整的正运动学或逆运动学求解。

**关键词：** SCARA 机械臂；STM32F103；下位机控制；UART DMA；三次多项式轨迹；步进电机；实时控制

---

## 1. 工程背景与总体定位

本工程位于 `D:/20209/project/scara_kinematic/scara_controller`，从构建文件、外设初始化代码和用户层实现可以判断，其目标平台为 STM32F103 系列微控制器，程序形态为裸机嵌入式固件，而非桌面应用或仿真软件。顶层构建文件 `CMakeLists.txt` 设置 C 标准为 C17，C++ 标准为 C++23，并生成名为 `scara_controller` 的可执行目标；该目标链接 `cmake/stm32cubemx` 中的 STM32CubeMX 生成库，同时将 `user/user_main.cpp` 与 `user/` 下的头文件纳入工程（`CMakeLists.txt:10-17`、`CMakeLists.txt:25-43`、`CMakeLists.txt:49-67`）。构建预设采用 Ninja 生成器，并通过 `cmake/gcc-arm-none-eabi.cmake` 指定 ARM 嵌入式交叉编译工具链（`CMakePresets.json:3-38`）。

从系统功能上看，工程核心目标是接收上位机发送的双关节运动指令，并在微控制器端完成关节目标角度到步进电机脉冲序列的转换。通信入口为 USART1，运动执行入口为 TIM1 周期中断，机械臂控制相关代码集中于 `user/` 目录。工程虽然命名中包含 SCARA 运动学，但当前固件并未在下位机端对末端笛卡尔坐标执行逆运动学求解；相反，上位机或其他规划模块应当先将期望末端位置换算为两轴关节角，再以二进制帧发送给下位机。下位机主要承担关节空间轨迹插值、运动段排队、步进电机脉冲输出与状态反馈。

---

## 2. 软件架构与模块划分

### 2.1 层次化结构

工程可以划分为四个主要层次：

| 层次 | 主要文件 | 职责 |
|---|---|---|
| 构建与工具链层 | `CMakeLists.txt`、`CMakePresets.json`、`cmake/` | 配置 C/C++ 标准、交叉编译工具链、CubeMX 生成源文件与链接目标 |
| HAL 外设层 | `Core/Src/main.c`、`gpio.c`、`dma.c`、`tim.c`、`usart.c`、`stm32f1xx_it.c` | 完成时钟、GPIO、DMA、USART、TIM 等底层硬件初始化与中断转发 |
| 用户业务层 | `user/user_main.cpp`、`protocol.hpp`、`motor.hpp`、`polinomial_profile.hpp`、`ring_buffer.hpp`、`spsc_queue.hpp` | 实现通信解析、命令分派、轨迹规划、运动队列和电机控制 |
| 测试辅助层 | `test/comm_test.py`、`test/protocol_test.py` | 提供上位机串口协议测试、延迟测试、压力测试和角度可视化工具 |

该架构体现了典型嵌入式工程的“底层初始化自动生成、上层业务手写实现”的组织方式。`Core/` 和 `Drivers/` 主要由 STM32CubeMX 与 HAL 框架生成，通常不直接承载业务决策；`user/` 目录则包含控制器的实质性实现。

### 2.2 主入口与运行骨架

系统入口位于 `Core/Src/main.c`。`main()` 的执行顺序为：调用 `HAL_Init()` 初始化 HAL 框架和 SysTick，随后调用 `SystemClock_Config()` 配置系统时钟，再依次初始化 GPIO、DMA、TIM1 和 USART1，之后进入用户初始化函数 `user_init()`，最后在无限循环中调用 `user_loop()`（`Core/Src/main.c:67-107`）。系统时钟配置采用 HSE 作为 PLL 输入，并以 PLL ×9 得到 72 MHz 主频，同时 APB1 二分频、APB2 不分频（`Core/Src/main.c:115-152`）。

这一运行骨架可以形式化表示为：

```text
main()
  ├─ HAL_Init()
  ├─ SystemClock_Config()
  ├─ MX_GPIO_Init()
  ├─ MX_DMA_Init()
  ├─ MX_TIM1_Init()
  ├─ MX_USART1_UART_Init()
  ├─ user_init()
  └─ while (1)
       └─ user_loop()
```

其中，`main()` 不直接解析任何协议，也不直接发出电机脉冲，而是将所有业务处理转交给 `user_init()` 和 `user_loop()`。这使得 HAL 初始化逻辑与用户控制逻辑保持了较清晰的边界。

---

## 3. 硬件抽象与配置参数实现

### 3.1 电机配置结构

电机参数由 `MotorConfig` 结构体统一描述（`user/type_def.hpp:10-34`）。该结构体包含每转脉冲数 `pulse_per_rev`、三类 GPIO 引脚元组、软限位区间 `limit` 以及上电初始偏移 `init_offset`。其中 GPIO 引脚元组的形式为：

```text
(GPIO_TypeDef* port, uint16_t pin, bool active_high)
```

即每个引脚不仅描述端口和引脚号，还描述该信号是否高电平有效。该字段直接影响 `Motor::enable()`、`Motor::set_pulse()` 与 `Motor::set_direction()` 中实际写入 GPIO 的电平逻辑。`MotorConfig::in_limit()` 用于判断输入关节角是否处于软限位范围内，当前实现采用简单闭区间判断，即 `rad >= limit.first && rad <= limit.second`（`user/type_def.hpp:21-33`）。

### 3.2 全局配置参数

全局参数定义于 `user/config.hpp`。通信配置中，`g_baudRate` 设置为 921600，`g_uartBufferSize` 设置为 512（`user/config.hpp:10-12`）。两路电机的每转脉冲数均为 3200，电机 0 使用 PA0/PA2/PA4 分别作为 PUL/DIR/ENA，电机 1 使用 PA1/PA3/PA5 分别作为 PUL/DIR/ENA（`user/config.hpp:15-29`）。软限位与初始偏移如下：

| 电机 | 每转脉冲数 | PUL | DIR | ENA | 软限位 | 初始偏移 |
|---|---:|---|---|---|---|---|
| 电机 0 | 3200 | PA0 | PA2 | PA4 | 70° 至 200° | 200° |
| 电机 1 | 3200 | PA1 | PA3 | PA5 | -20° 至 110° | -20° |

此外，`config.hpp` 中定义了 `g_br = 0.12f`、`g_lr = 0.2f`、`g_d = 0.1f` 三个 SCARA 几何参数，分别对应大臂长度、小臂长度和基座偏距（`user/config.hpp:33-36`）。然而，当前源码中未见这些参数参与正运动学或逆运动学求解，因而它们更接近预留的机械结构参数，而非当前控制闭环中的实际计算输入。

需要注意的是，`g_updateDurationUs` 被定义为 50（`user/config.hpp:31`），但 TIM1 的实际初始化周期为 100 μs，且 `user_init()` 中传入 `controller.update_handler(100)`（`Core/Src/tim.c:43-46`、`user/user_main.cpp:81-88`）。因此，在描述当前有效控制周期时，应以 TIM1 初始化参数和回调实参为准，即当前运动更新周期为 100 μs。

### 3.3 GPIO 初始化与电平状态

GPIO 初始化位于 `Core/Src/gpio.c`。程序使能 GPIOC、GPIOD 和 GPIOA 时钟后，将 PA0 与 PA1 初始置为高电平，将 PA2、PA3、PA4、PA5 初始置为低电平（`Core/Src/gpio.c:47-57`）。随后 PA0、PA1 被配置为高速推挽输出，PA2 至 PA5 被配置为低速推挽输出（`Core/Src/gpio.c:58-70`）。

在用户层，`Motor` 按照配置中的 `active_high` 字段决定真实输出电平。例如 `enable(true)` 会根据 `active_high` 计算有效电平并调用 `HAL_GPIO_WritePin()`（`user/motor.hpp:29-37`）。由于 `config.hpp` 中 ENA 的 `active_high` 字段均为 `true`，故从代码逻辑看，使能时 ENA 输出高电平。若硬件实际为低电平使能，则应调整配置字段而非仅依赖注释说明。本文后续分析均以源码逻辑为准。

---

## 4. 通信协议设计与帧结构实现

### 4.1 命令与响应类型

通信协议定义于 `user/protocol.hpp`。命令码采用 `uint8_t` 枚举表示，其中 `Motion` 为 0x01，`EmergencyStop` 为 0x02，`QueryStatus` 为 0x03，响应帧包括 `Ack`、`Nack` 和 `StatusResponse`，分别为 0xF0、0xF1 和 0xF2（`user/protocol.hpp:8-20`）。错误原因由 `Reason` 枚举描述，包括无效命令、无效数据和执行失败（`user/protocol.hpp:22-28`）。

协议采用固定帧头 `0xAA 0x55`。所有帧结构体均被 `#pragma pack(push, 1)` 包裹，以避免编译器插入额外结构体填充（`user/protocol.hpp:51-124`）。帧头结构 `Header` 由四个字节构成，分别为 `head0`、`head1`、`cmd` 和 `padding`，其中 `padding` 用于保持头部和命令码对齐到 4 字节边界（`user/protocol.hpp:53-71`）。

### 4.2 帧格式

当前固件中主要帧格式如下：

| 帧类型 | 命令码 | 结构组成 | 理论长度 |
|---|---:|---|---:|
| `AckFrame` | 0xF0 | `Header` | 4 B |
| `NackFrame` | 0xF1 | `Header + Reason` | 5 B |
| `StatusResponseFrame` | 0xF2 | `Header + float angle[2]` | 12 B |
| `MotionCommandFrame` | 0x01 | `Header + float target_angle[2] + float target_speed[2] + float process_time` | 24 B |
| `QueryStatusFrame` | 0x03 | `Header` | 4 B |
| `EmergencyStopFrame` | 0x02 | `Header` | 4 B |

`MotionCommandFrame` 是核心输入帧，其字段包括两轴目标角度、两轴目标速度和运动段执行时间（`user/protocol.hpp:101-108`）。字段单位分别为 rad、rad/s 和 s。状态响应帧返回两轴当前绝对角度，单位为 rad（`user/protocol.hpp:88-99`）。

### 4.3 字节序列化策略

协议文件提供模板函数 `as_bytes()`，通过 `reinterpret_cast` 将任意帧结构映射为 `std::span<const std::byte>`（`user/protocol.hpp:126-130`）。虽然当前 `user_main.cpp` 中发送函数并未直接调用 `as_bytes()`，而是使用 `memcpy` 将结构体内容复制到 DMA 发送缓冲，但两者本质上都依赖结构体的内存布局。因此，该协议隐含依赖以下条件：

1. MCU 与上位机均按小端序解释浮点数；
2. `float` 为 IEEE 754 单精度 4 字节表示；
3. 结构体使用 1 字节对齐；
4. 上位机打包格式必须包含 `Header` 中的 padding 字节。

`test/comm_test.py` 中的协议常量显式说明 C++ 端 `Header` 为 4 字节，并使用 `<BBBxff`、`<BBBxfffff` 等 Python `struct` 格式在三字节头后插入一个 pad 字节，因而与当前固件协议保持一致（`test/comm_test.py:90-107`）。相较之下，`test/protocol_test.py` 仍将 `HEADER_FMT`、`ACK_FMT`、`QUERY_FMT` 等定义为三字节格式（`test/protocol_test.py:31-47`），更像旧版脚本或未同步版本，使用时需要注意与当前固件帧头不一致的问题。

---

## 5. UART-DMA 接收与发送机制

### 5.1 USART1 与 DMA 初始化

USART1 初始化位于 `Core/Src/usart.c`。固件将 USART1 配置为 921600 波特率、8 位数据位、1 位停止位、无校验、收发模式、无硬件流控、16 倍过采样（`Core/Src/usart.c:43-51`）。USART1 TX 使用 PA9 复用推挽输出，RX 使用 PA10 输入（`Core/Src/usart.c:73-86`）。DMA 方面，USART1_RX 链接 DMA1_Channel5，方向为外设到内存；USART1_TX 链接 DMA1_Channel4，方向为内存到外设，二者均使用字节对齐和内存地址递增（`Core/Src/usart.c:88-119`）。

DMA 控制器初始化见 `Core/Src/dma.c`。程序开启 DMA1 时钟，并分别使能 DMA1_Channel4 与 DMA1_Channel5 中断（`Core/Src/dma.c:39-53`）。中断转发由 `Core/Src/stm32f1xx_it.c` 完成，其中 DMA1_Channel4 调用 `HAL_DMA_IRQHandler(&hdma_usart1_tx)`，DMA1_Channel5 调用 `HAL_DMA_IRQHandler(&hdma_usart1_rx)`，USART1 全局中断调用 `HAL_UART_IRQHandler(&huart1)`（`Core/Src/stm32f1xx_it.c:205-259`）。

### 5.2 双缓冲接收流程

用户层定义了两个 UART 接收原始缓冲区：

```text
uart_rx_raw_buffer[2][g_uartBufferSize]
```

以及一个容量为 1023 的环形缓冲区 `uart_rx_buffer`（`user/user_main.cpp:21-24`）。接收启动函数 `start_uart_receive()` 调用 `HAL_UARTEx_ReceiveToIdle_DMA()`，以当前原始缓冲区启动 DMA 接收（`user/user_main.cpp:32-36`）。当 UART 接收空闲事件触发时，`uart_rx_event_callback()` 首先通过 `std::exchange` 取得当前缓冲区索引并切换到另一块缓冲区，然后立即重新启动 DMA 接收，最后将刚接收到的 `size` 字节推入环形缓冲区（`user/user_main.cpp:38-46`）。

该设计的处理流程可表示为：

```text
USART1 RX 数据流
  └─ HAL_UARTEx_ReceiveToIdle_DMA()
       └─ IDLE/RX event callback
            ├─ 切换 raw buffer 0/1
            ├─ 立即重启 DMA 接收
            └─ push 到 RingBuffer<1023>
                 └─ user_loop() 逐字节解析完整帧
```

这种设计的优点在于，DMA 可以在后台搬运串口数据，IDLE 事件可以在不固定帧长的情况下触发批量接收，而双缓冲可以减少重新启动 DMA 时覆盖尚未处理数据的风险。需要指出的是，`RingBuffer::push()` 在空间不足时返回 `false`，但当前回调未检查该返回值（`user/ring_buffer.hpp:66-88`、`user/user_main.cpp:43-46`）。因此，当主循环处理速度低于串口输入速度并导致环形缓冲区溢出时，新接收数据可能被静默丢弃。

### 5.3 环形缓冲区实现

`RingBuffer<Capacity>` 采用 `Capacity + 1` 的内部存储空间，以空出一个槽位区分满状态与空状态（`user/ring_buffer.hpp:16-19`）。头尾指针为 `std::atomic_size_t`，适用于中断回调生产数据、主循环消费数据的单生产者/单消费者场景（`user/ring_buffer.hpp:35-37`）。

其关键操作包括：

- `push(span<const byte>)`：检查剩余空间，按是否跨越尾部将数据分两段写入，并更新 `tail_`（`user/ring_buffer.hpp:66-88`）；
- `peek(span<byte>)`：在不移动 `head_` 的情况下读取指定长度数据，用于协议解析时先判断完整帧是否可用（`user/ring_buffer.hpp:90-111`）；
- `pop(count)`：消费指定字节数并移动 `head_`（`user/ring_buffer.hpp:113-122`）。

由于 `user_loop()` 在解析前大量使用 `peek()`，只有确认完整帧或确认无效字节后才调用 `pop()`，该策略有利于处理半包数据和帧边界恢复。

### 5.4 DMA 发送流程

发送侧定义了 `uart_tx_busy` 原子标志和 `uart_tx_raw_buffer[64]` 静态缓冲区（`user/user_main.cpp:25-26`）。模板函数 `uart_send<T>()` 要求 `sizeof(T) <= g_uartBufferSize`，随后等待前一次 DMA 发送完成，将结构体数据复制到静态缓冲区，并调用 `HAL_UART_Transmit_DMA()` 启动发送（`user/user_main.cpp:48-63`）。发送完成回调在 `user_init()` 中注册，并在 TX complete 时将 `uart_tx_busy` 置为 `false`（`user/user_main.cpp:71-79`）。

需要注意，`uart_tx_raw_buffer` 实际长度为 64 字节，但模板约束使用的是 `g_uartBufferSize = 512`。当前所有响应帧最大仅 12 字节，因此不会触发问题；但若未来新增大于 64 字节的响应帧，应同步修改约束或发送缓冲区长度。

---

## 6. 主循环命令解析与状态机处理

### 6.1 用户初始化

`user_init()` 是用户层运行前的初始化函数，主要完成三项工作（`user/user_main.cpp:65-89`）：

1. 按配置值重新设置 USART1 波特率并调用 `HAL_UART_Init()`；
2. 注册 UART RX event 回调和 TX complete 回调，并启动首次 DMA 接收；
3. 注册 TIM1 周期回调，使 TIM1 每次更新中断调用 `controller.update_handler(100)`，然后启动 TIM1 基础定时器中断。

该函数建立了两个异步入口：UART 接收回调负责将外部命令搬运到环形缓冲区，TIM1 回调负责周期性推进运动控制。

### 6.2 帧同步与命令分派

`user_loop()` 是主循环中的协议解析函数。其第一步是从环形缓冲区窥视一个字节，如果该字节不是 `0xAA`，则丢弃一个字节以寻找下一可能帧头（`user/user_main.cpp:91-97`、`user/user_main.cpp:217-221`）。如果首字节为 `0xAA`，函数继续尝试读取完整 `Header`。当 `Header::is_valid()` 确认 `head0 == 0xAA && head1 == 0x55` 后，程序依据 `cmd` 字段进入不同分支（`user/user_main.cpp:98-108`）。

这一过程构成了一个轻量帧同步状态机：

```text
RingBuffer
  ├─ peek 1 byte
  │    ├─ 非 0xAA：pop(1)，继续寻找帧头
  │    └─ 是 0xAA：peek Header
  │          ├─ Header 不完整：保留数据，等待后续字节
  │          ├─ Header 无效：pop(1)，重新同步
  │          └─ Header 有效：switch(cmd)
  │                 ├─ Motion
  │                 ├─ EmergencyStop
  │                 ├─ QueryStatus
  │                 └─ default
```

该策略能够在收到噪声字节或错误帧头后通过逐字节丢弃恢复同步。对于半包数据，`peek()` 会失败但不会移动读指针，从而等待后续 DMA 数据进入缓冲区。

### 6.3 Motion 命令处理

当命令码为 `Motion` 时，`user_loop()` 尝试读取完整 `MotionCommandFrame`。若完整帧已经到达，则先从环形缓冲区弹出该帧，再执行数据合法性检查（`user/user_main.cpp:105-119`）。当前合法性检查包括：目标角度必须为 normal 浮点数，目标速度不能为 NaN，运动时间必须为 normal 浮点数。随后程序调用两个电机配置的 `in_limit()` 检查目标角度是否超出软限位（`user/user_main.cpp:121-127`）。若任一检查失败，固件返回 `NackFrame{InvalidData}`。

在通过数据检查后，程序将 `process_time` 从秒转换为毫秒，读取控制器当前的最终目标步数，并尝试从队列末端运动段取得末速度作为新运动段的起始速度（`user/user_main.cpp:129-145`）。若队列为空，则新段起始速度设为 0。之后程序构造两条三次多项式轨迹：

```text
x_profile: last_target_x → motor_0.rad_to_step(target_angle[0])
y_profile: last_target_y → motor_1.rad_to_step(target_angle[1])
```

其中终止速度由 `rad_per_sec_to_step_per_ms()` 将 rad/s 转换为 step/ms（`user/user_main.cpp:147-159`）。若轨迹对象无效，则返回 `NackFrame{InvalidData}`；若控制器队列入队失败，则返回 `NackFrame{ExecutionFailed}`；否则返回 `AckFrame`（`user/user_main.cpp:160-177`）。

该分支体现了下位机的核心处理链路：

```text
MotionCommandFrame
  ├─ 浮点合法性检查
  ├─ 软限位检查
  ├─ 角度 rad → 步数 step
  ├─ 速度 rad/s → step/ms
  ├─ 构造 x/y 三次多项式轨迹
  ├─ 生成 Segment(duration_us, x_profile, y_profile)
  ├─ 入 SPSC 运动队列
  └─ 返回 ACK 或 NACK
```

### 6.4 EmergencyStop 与 QueryStatus

`EmergencyStop` 分支读取完整急停帧后，调用 `controller.emergency_stop()`，弹出帧并返回 ACK（`user/user_main.cpp:180-190`）。该动作不需要额外参数，属于最高层面的安全控制命令。

`QueryStatus` 分支读取查询帧后，将两个电机当前步数分别转换为弧度角，并以 `StatusResponseFrame` 返回（`user/user_main.cpp:192-201`）。换言之，状态反馈并非来自外部编码器或闭环测量，而是来自固件内部维护的 `current_step_`。因此其语义是“控制器认为当前已输出到的位置”，而非独立传感器测得的机械实际位置。

未知命令分支会弹出一个 `Header` 大小的数据，并返回 `NackFrame{InvalidCommand}`（`user/user_main.cpp:203-207`）。

---

## 7. 轨迹规划算法实现

### 7.1 多项式轨迹类

轨迹规划由 `user/polinomial_profile.hpp` 中的 `PolinomialProfile` 类实现。类名保留源码写法 `Polinomial`。该类支持 `Linear` 和 `Cubic` 两种插值类型，内部使用 `float` 作为标量类型（`user/polinomial_profile.hpp:10-19`）。构造函数输入包括插值类型、起始步数、终止步数、运动总时间、起始速度和终止速度（`user/polinomial_profile.hpp:30-34`）。若运动时间为 0，则对象被标记为无效（`user/polinomial_profile.hpp:37-42`）。

### 7.2 线性插值

当类型为 `Linear` 时，类计算恒定速度系数：

$$
a_1 = \frac{q_e - q_s}{T}
$$

其中 \(q_s\) 为起始步数，\(q_e\) 为终止步数，\(T\) 为运动时间。此时二次项和三次项均为 0，末速度等于恒定速度（`user/polinomial_profile.hpp:44-52`）。当前 `user_main.cpp` 中实际选择的是 `Cubic`，因此线性模式属于保留能力。

### 7.3 三次多项式插值

当类型为 `Cubic` 时，轨迹函数可表示为：

$$
q(t) = q_s + a_1 t + a_2 t^2 + a_3 t^3
$$

其中源码通过 `offset_step_` 保存 \(q_s\)，并将多项式主体设置为相对位移。边界条件为：

$$
q(0)=q_s,
\quad q(T)=q_e,
\quad \dot q(0)=v_s,
\quad \dot q(T)=v_e
$$

对应源码中的系数计算为（`user/polinomial_profile.hpp:54-69`）：

$$
a_1 = v_s
$$

$$
a_2 = \frac{3(q_e-q_s)-(2v_s+v_e)T}{T^2}
$$

$$
a_3 = \frac{(v_s+v_e)T-2(q_e-q_s)}{T^3}
$$

在运行时，`current_step(ms)` 根据当前毫秒时间计算目标步数。若时间小于等于 0，则返回起始步数；若对象无效，也返回起始步数；若为三次模式，则计算 `a1*ms + a2*ms^2 + a3*ms^3` 并四舍五入到整数步数（`user/polinomial_profile.hpp:90-112`）。

### 7.4 速度衔接

`PolinomialProfile::end_speed()` 返回轨迹段终止速度（`user/polinomial_profile.hpp:85-88`）。`user_loop()` 在添加新运动段前会读取控制器队列末端段的 `end_speed()`，作为新段的起始速度（`user/user_main.cpp:133-145`）。这种设计可以在运动队列连续入段时保持速度边界条件上的衔接，减少分段轨迹之间的突变。

---

## 8. 电机驱动与控制器实现

### 8.1 Motor 类：角度、步数与 GPIO 的映射

`Motor` 类定义于 `user/motor.hpp`，是底层执行单元。构造函数根据配置中的初始偏移计算 `offset_step_`，并将角度软限位换算为步数限位（`user/motor.hpp:18-27`）。角度到步数的转换公式为（`user/motor.hpp:80-83`）：

$$
step = round\left( \theta \cdot \frac{N}{2\pi} \right) - offset
$$

其中 \(\theta\) 为弧度角，\(N\) 为每转脉冲数。步数到角度的反变换为（`user/motor.hpp:85-88`）：

$$
\theta = (step + offset) \cdot \frac{2\pi}{N}
$$

角速度到步进速度的转换为（`user/motor.hpp:90-93`）：

$$
v_{step/ms} = \omega_{rad/s} \cdot \frac{N}{2\pi} \cdot 10^{-3}
$$

通过这些变换，固件将通信层中的物理量单位 rad、rad/s 转换为实时控制层使用的 step、step/ms。

### 8.2 单步请求机制

`Motor::request_step(step)` 是电机动作的核心函数（`user/motor.hpp:49-78`）。该函数首先检查目标步数是否落在步数软限位内，若超限则拒绝请求。若目标步数与当前步数不同，则使能电机，根据目标步数与当前步数的大小关系设置方向，并仅将 `current_step_` 增加或减少 1。随后函数将 PUL 引脚置为有效电平，并返回 `true` 表示本周期需要输出脉冲。若目标步数与当前步数相等，则返回 `false`。

这种设计意味着，即使轨迹在某一周期计算出的目标步数与当前位置相差多个步，电机一次中断周期也最多推进一步。它本质上构成了一个按目标位置追踪的限速离散执行器，最大步频受 TIM1 周期限制。当前 TIM1 周期为 100 μs，因此单轴理论最高请求步频约为 10 kHz。

### 8.3 Controller 类：运动段队列与周期调度

`Controller<Capacity>` 同样定义于 `user/motor.hpp`，用于管理两个电机和运动段队列。运动段 `Segment` 包含持续时间 `duration_us` 以及 x/y 两轴轨迹 `x_profile`、`y_profile`（`user/motor.hpp:123-131`）。工程实例化时使用 `Controller<128>`，内部队列为 `SpscQueue<Segment, 128>`（`user/user_main.cpp:19-30`、`user/motor.hpp:220-224`）。由于 SPSC 队列通过保留一个空槽判断满状态，其实际可容纳运动段数为 127。

控制器的周期函数 `update_handler(dt_us)` 由 TIM1 中断回调调用（`user/user_main.cpp:81-88`）。其执行流程如下（`user/motor.hpp:138-176`）：

1. 从运动段队列取得队首段指针；
2. 若队列为空，则不执行动作；
3. 若当前段已达到或超过持续时间，则弹出队首段、重置段内计时，并请求两个电机修正到该段最终步数；
4. 若当前段尚未完成，则累加 `elapsed_time_us_`，换算为毫秒；
5. 根据 x/y 轨迹计算当前时刻目标步数；
6. 分别调用两个电机的 `request_step()`；
7. 调用 `step_once()` 统一撤销脉冲。

该流程可抽象为：

```text
TIM1 update interrupt
  └─ Controller::update_handler(100)
       ├─ front_ptr() 获取当前 Segment
       ├─ elapsed_time_us_ >= duration_us ?
       │    ├─ 是：pop Segment，修正到终点步数
       │    └─ 否：elapsed_time_us_ += 100
       │          ├─ x_profile.current_step(t)
       │          ├─ y_profile.current_step(t)
       │          ├─ motor_x.request_step(target_x)
       │          └─ motor_y.request_step(target_y)
       └─ step_once(updated_x, updated_y)
```

### 8.4 脉冲宽度保证

`step_once()` 在任一电机本周期需要更新时执行双层空循环，并在循环中调用 `__NOP()`，随后将两个电机的 PUL 引脚置为无效电平（`user/motor.hpp:228-239`）。注释说明该延时用于满足电机驱动器至少 6 μs 的脉冲宽度要求。由于系统主频为 72 MHz，内层循环 72 次约对应 1 μs，外层循环 6 次约对应 6 μs。

该实现具有简单直接的优点，但它会在 TIM1 中断上下文中忙等待约 6 μs。对于 100 μs 的控制周期而言，该开销可接受，但仍占用一定中断时间。若未来提高控制频率或增加更多轴数，可以考虑使用硬件定时器输出比较或 PWM 方式生成脉冲，以降低 CPU 忙等待占比。

### 8.5 急停处理

`Controller::emergency_stop()` 首先调用两个电机的 `enable(false)`，然后循环弹出队列中所有运动段，重置段内时间，并将控制器的目标步数更新为当前电机步数（`user/motor.hpp:200-212`）。该实现的语义是立即停止后续轨迹执行，并使后续新增运动段从停止时控制器记录的位置继续规划。急停并不会通过传感器校准机械实际位置，因此若电机在急停前后发生丢步，系统内部位置仍可能与机械实际位置存在偏差。

---

## 9. SPSC 队列与实时并发模型

### 9.1 队列数据结构

`SpscQueue<T, Capacity>` 是一个固定容量单生产者单消费者队列（`user/spsc_queue.hpp:11-117`）。模板通过静态断言要求容量大于 0、元素类型可平凡复制、`std::atomic_size_t` 始终无锁，并要求容量为 2 的幂（`user/spsc_queue.hpp:14-17`）。其头尾索引均为原子变量，环绕通过位与操作实现（`user/spsc_queue.hpp:19-31`）。

队列提供 `push()`、`emplace()`、`pop()`、`front()`、`front_ptr()` 和 `back()` 等接口（`user/spsc_queue.hpp:36-115`）。在本工程中，主循环作为生产者调用 `controller.add_segment()` 将运动段放入队列；TIM1 中断作为消费者调用 `update_handler()` 获取并弹出队首段。这种并发模型避免了动态内存分配和复杂锁机制，符合嵌入式实时控制需求。

### 9.2 中断与主循环协同

系统至少包含三个重要执行上下文：

1. 主循环上下文：执行 `user_loop()`，解析帧并添加运动段；
2. UART/DMA 回调上下文：将接收数据从 DMA 原始缓冲推入环形缓冲区；
3. TIM1 中断上下文：周期性推进轨迹并输出步进脉冲。

主循环与 UART 回调之间通过 `RingBuffer` 传递字节流；主循环与 TIM1 中断之间通过 `SpscQueue` 传递运动段。发送侧通过 `uart_tx_busy` 原子变量协调 DMA 发送状态。这种设计将较耗时的协议解析放在主循环，将必须准时执行的脉冲生成放在定时器中断，从而在结构上区分了非实时任务与硬实时任务。

---

## 10. 定时器与实时控制周期

TIM1 初始化中，预分频器设为 `72 - 1`，周期设为 `100 - 1`（`Core/Src/tim.c:43-46`）。在 72 MHz 时钟下，预分频后计数频率为 1 MHz，每计数一次为 1 μs；周期为 100 个计数，因此更新中断周期为 100 μs，即 10 kHz。TIM1 更新中断在 `stm32f1xx_it.c` 中转发给 HAL 定时器中断处理函数（`Core/Src/stm32f1xx_it.c:233-245`），用户回调则在 `user_init()` 中注册为 `controller.update_handler(100)`（`user/user_main.cpp:81-88`）。

该周期直接决定轨迹采样频率和最大单步输出频率。每个周期中，每个电机最多执行一次 `current_step_` 的加一或减一，并输出一次脉冲。因此，当轨迹目标在相邻周期之间变化超过一步时，控制器不会一次补齐所有误差，而是以每周期一步的方式追赶目标。该特性使脉冲频率自然受限，但也要求上位机设定运动时间和目标速度时不得超过控制器可实现的步频能力。

---

## 11. 上位机测试脚本与协议验证

### 11.1 `comm_test.py`

`test/comm_test.py` 是与当前固件协议最接近的测试工具。脚本开头说明其功能包括通信延迟测量、帧对齐测试和压力测试（`test/comm_test.py:1-14`）。协议常量部分定义了与 C++ 端一致的命令码、帧头和帧长度，并明确指出 C++ `Header` 为 4 字节（`test/comm_test.py:71-107`）。

脚本中的 `read_frame()` 先读取 4 字节头部，检查帧头是否为 `0xAA 0x55`，再根据命令码读取不同长度的响应体（`test/comm_test.py:159-177`）。`query_status()` 发送查询帧并解析状态响应，返回两轴角度（`test/comm_test.py:180-187`）。延迟测试函数 `measure_latency()` 连续发送 `QueryStatus` 并统计往返时间，支持预热、重试、超时和错误统计（`test/comm_test.py:209-260`）。

该脚本不仅用于功能验证，也为论文式说明提供了上位机协议交互模型：即上位机以二进制固定格式打包命令，经串口发送至下位机，再按响应命令码解析 ACK、NACK 或状态数据。

### 11.2 `protocol_test.py`

`test/protocol_test.py` 提供了更丰富的操作接口，包括运动命令、状态查询、到达验证以及基于 matplotlib 的实时角度曲线绘制。`LiveAnglePlot` 类维护时间序列和两轴角度序列，并在图中绘制目标角参考线（`test/protocol_test.py:64-107`）。

然而，该脚本的基础头部格式定义为 `<BBB>`，即三字节头部（`test/protocol_test.py:31-47`），与当前固件中四字节 `Header` 不完全一致。因此，如果将该脚本用于当前固件，需要先同步 ACK、NACK、Query、EmergencyStop 等帧格式，否则会出现帧长度不匹配问题。论文或技术报告中引用测试脚本时，应优先引用 `comm_test.py` 作为当前协议依据。

---

## 12. 数学工具与 SCARA 运动学参数

`user/math_utils.hpp` 提供若干基础数学工具，包括快速倒平方根 `inv_sqrt()`、基于倒平方根的 `sqrt()`、弧度转角度 `rad_to_deg()` 和角度转弧度 `deg_to_rad()`（`user/math_utils.hpp:8-43`）。`inv_sqrt()` 使用经典快速倒平方根初值 `0x5f3759df`，并通过 Newton-Raphson 迭代提升近似精度（`user/math_utils.hpp:10-26`）。

当前工程中，`deg_to_rad()` 直接参与电机配置，将角度制软限位转换为弧度制参数（`user/config.hpp:20-29`）。而 `sqrt()`、`rad_to_deg()` 等函数更多体现为预留工具。结合 `config.hpp` 中 SCARA 几何参数的存在可以推断，工程可能计划在未来加入运动学计算；但就当前源码而言，下位机并没有实现如下典型 SCARA 逆运动学过程：

$$
(x, y) \rightarrow (\theta_1, \theta_2)
$$

也没有实现：

$$
(\theta_1, \theta_2) \rightarrow (x, y)
$$

因此，本文将当前固件定义为关节空间控制器，而不是完整运动学求解器。

---

## 13. 可靠性、边界条件与实现约束

从工程实现看，系统已经具备较完整的通信、队列化运动和急停机制，但仍存在若干实现层面的约束，需要在使用或扩展时注意。

第一，协议没有 CRC 或校验和字段。当前帧同步主要依赖 `0xAA 0x55` 帧头和固定长度解析。当串口数据发生位错误但仍形成合法帧头时，下位机可能无法识别载荷错误。若后续用于噪声较强的现场环境，可以考虑增加长度字段、序号字段和 CRC 校验。

第二，`Motion` 命令的浮点合法性检查较严格地要求目标角度和运动时间为 normal 数值，这会拒绝 0 这样的非 normal 但常见数值；同时目标速度仅检查 NaN，未显式拒绝无穷大（`user/user_main.cpp:111-119`）。如果上位机可能发送 0 rad、0 rad/s 或异常无穷值，则应根据实际协议语义调整合法性判断。

第三，状态反馈来自内部步数计数而非外部位置传感器（`user/user_main.cpp:198-200`、`user/motor.hpp:95-98`）。因此，该系统属于开环步进控制。如果电机堵转、丢步或机械回差显著，固件内部状态与实际关节角可能发生偏离。

第四，UART 接收环形缓冲区溢出时，`push()` 的失败结果未被回调处理（`user/user_main.cpp:43-46`）。在高频发送运动指令或压力测试时，如果主循环无法及时消费数据，可能出现静默丢帧。

第五，运动队列容量为模板参数 128，但由于环形队列保留一个空槽，其最大有效入队运动段数为 127（`user/spsc_queue.hpp:23-31`、`user/spsc_queue.hpp:51-65`）。当队列满时，`controller.add_segment()` 返回 `false`，上层将响应 `NackFrame{ExecutionFailed}`（`user/motor.hpp:178-188`、`user/user_main.cpp:166-175`）。

第六，脉冲宽度通过中断中的忙等待实现（`user/motor.hpp:228-239`）。该方式实现简单，但占用中断执行时间。若未来增加轴数、提高步频或引入更多实时任务，建议迁移到硬件定时器比较输出或 DMA 驱动脉冲序列。

---

## 14. 总体处理流程归纳

综合上述分析，当前固件的完整处理流程可概括为以下闭环：

```text
系统上电
  └─ main()
       ├─ HAL / 时钟 / GPIO / DMA / USART / TIM 初始化
       ├─ user_init()
       │    ├─ 注册 UART RX event callback
       │    ├─ 注册 UART TX complete callback
       │    ├─ 启动 USART1 DMA 接收
       │    ├─ 注册 TIM1 period callback
       │    └─ 启动 TIM1 100 μs 周期中断
       └─ while (1)
            └─ user_loop()
                 ├─ 从 RingBuffer 帧同步
                 ├─ 解析 Motion / EmergencyStop / QueryStatus
                 ├─ Motion: 角度限位检查 → 三次轨迹 → Segment 入队
                 ├─ EmergencyStop: 电机失能 → 清空运动队列
                 └─ QueryStatus: current_step → rad → StatusResponse

异步执行路径 A：USART1 DMA/IDLE
  └─ 接收字节批次 → 双缓冲切换 → push 到 RingBuffer

异步执行路径 B：TIM1 100 μs 中断
  └─ Controller::update_handler(100)
       ├─ 读取当前 Segment
       ├─ 按 elapsed_time 计算目标步数
       ├─ 每轴最多推进一步
       ├─ 输出 PUL/DIR/ENA GPIO
       └─ 延时约 6 μs 后撤销脉冲
```

从控制思想上看，上位机负责将任务分解为目标关节角、终止速度和执行时间；下位机负责将这些关节空间命令转换为连续、可排队的步进电机脉冲序列。通信层保证命令输入与状态输出，轨迹层保证段内目标步数平滑变化，控制器层保证固定周期调度，电机层保证 GPIO 符合驱动器时序要求。

---

## 15. 结论

本文对 `scara_controller` 工程的当前源码进行了分层分析。研究表明，该工程构建了一个基于 STM32F103、USART1-DMA 通信和 TIM1 周期调度的双关节 SCARA 下位机控制系统。其软件结构清晰地分为 HAL 外设初始化层、用户协议解析层、轨迹规划层、运动队列层和电机 GPIO 驱动层。系统通过固定二进制帧接收运动、急停和查询命令，通过三次多项式在关节空间内生成平滑步数目标，并在 100 μs 定时中断中以单步请求方式驱动两路步进电机。

与此同时，当前源码也显示出明确的边界：下位机尚未实现完整 SCARA 正逆运动学；通信协议尚无 CRC 校验；位置反馈基于内部步数计数而非闭环传感器；部分配置注释与实际逻辑存在不一致。因而，在论文或工程报告中应将其严谨表述为“面向 SCARA 双关节的关节空间下位机执行控制器”。若未来需要扩展为完整机器人控制系统，可在现有架构基础上进一步加入运动学求解、轨迹可行性约束、通信校验、闭环位置反馈以及硬件定时器脉冲生成机制。
