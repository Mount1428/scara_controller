# SCARA Controller

SCARA 机械臂底层控制器固件，基于 STM32F103C8 (Cortex-M3, 72MHz) 微控制器，通过 UART 接收上位机发送的关节空间运动指令，执行三次多项式轨迹插补，驱动两个步进电机完成运动控制。

## 系统架构

```
上位机 (PC) --[UART 921600bps]--> STM32F103C8 --[GPIO PUL/DIR/ENA]--> 步进电机驱动器 --> 步进电机
```

**三个并发执行上下文：**

| 上下文 | 优先级 | 周期 | 职责 |
|--------|--------|------|------|
| TIM1 ISR | 最高 | 100us (10kHz) | 轨迹插补计算、步进脉冲输出 |
| UART/DMA ISR | 中 | 事件驱动 | 串口数据接收、双缓冲 DMA |
| 主循环 (user_loop) | 最低 | WFI 空闲唤醒 | 帧解析、命令分发、轨迹规划入队 |

## 通信协议

二进制帧格式，帧头 `0xAA 0x55`：

| 命令 | 方向 | 长度 | 说明 |
|------|------|------|------|
| Motion | PC -> MCU | 24B | 双轴目标角度、速度、运动时间 |
| EmergencyStop | PC -> MCU | 4B | 紧急停止 |
| QueryStatus | PC -> MCU | 4B | 查询当前关节角度 |
| Ack | MCU -> PC | 4B | 确认应答 |
| Nack | MCU -> PC | 5B | 否定应答（含错误码） |
| StatusResponse | MCU -> PC | 12B | 当前双轴角度 (float) |

## 轨迹规划

采用三次多项式插补：`q(t) = a1*t + a2*t^2 + a3*t^3 + offset`

通过边界条件（起止位置 + 起止速度）计算系数，支持线性和三次模式。每 100us 在 TIM1 中断中插值一次，输出目标步数并驱动 GPIO 脉冲。

## 硬件参数

| 参数 | 值 |
|------|-----|
| MCU | STM32F103C8 (64KB Flash, 20KB SRAM) |
| 时钟 | 72MHz (HSE + PLL) |
| 串口 | USART1, 921600 baud, CH340 USB 转串口 |
| 步进电机 | 3200 脉冲/圈 (16 细分) |
| 电机控制引脚 | PA0-PA5 (两轴 PUL/DIR/ENA) |
| 串口引脚 | PA9 (TX), PA10 (RX) |

## 目录结构

```
├── Core/                 # STM32 HAL 初始化代码 (CubeMX 生成)
│   ├── Inc/              # HAL 头文件
│   └── Src/              # HAL 源文件 (main.c, gpio, uart, dma, tim)
├── Drivers/              # ST HAL 库 (厂商代码)
├── user/                 # 应用层代码 (手写)
│   ├── user_main.cpp     # 主循环：UART 帧解析与命令分发
│   ├── config.hpp        # 全局配置 (电机参数、串口波特率、运动学参数)
│   ├── protocol.hpp      # 二进制协议定义
│   ├── motor.hpp         # Motor 类 (步进控制) + Controller 类 (队列调度)
│   ├── polinomial_profile.hpp  # 三次多项式轨迹规划器
│   ├── ring_buffer.hpp   # 无锁 SPSC 环形缓冲区 (UART RX)
│   ├── spsc_queue.hpp    # 无锁 SPSC 队列 (运动段)
│   └── math_utils.hpp    # 数学工具函数
├── test/                 # 上位机测试脚本 (Python)
├── docs/                 # 设计文档
├── cmake/                # CMake 工具链配置
└── scara_controller.ioc  # STM32CubeMX 工程文件
```

## 构建

**前置条件：**
- CMake >= 3.22
- Ninja
- arm-none-eabi-gcc 工具链

**编译：**

```bash
cmake --preset Debug    # 或 Release
cmake --build build/Debug
```

产出文件：`build/Debug/scara_controller.elf`

## 上位机测试

```bash
cd test
pip install pyserial matplotlib
python protocol_test.py COMx    # 协议测试 + 实时角度绘图
python comm_test.py COMx        # 通信质量测试 (延迟、帧对齐、压力测试)
```

## 设计文档

- [系统设计文档](docs/design.md)
- [系统架构与流程图](docs/system_design_and_flows.md)
- [代码实现分析](docs/code_implementation_analysis.md)
- [电气接线图](docs/electrical_schematic.md)
- [三次多项式插补推导](docs/cubic_polynomial_interpolation_derivation.md)

## 许可证

MIT License
