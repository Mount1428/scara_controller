#include <user_main.h>

#include <utility>
#include <cstring>

#include <math_utils.hpp>
#include <polynomial_profile.hpp>
#include <protocol.hpp>
#include <ring_buffer.hpp>
#include <spsc_queue.hpp>

#include <motor.hpp>

#include <config.hpp>

#include <main.h>
#include <usart.h>

using Controller = user::Controller<128>;

static uint8_t uart_rx_raw_cnt = 0;
static uint8_t uart_rx_raw_buffer[2][user::config::g_uartBufferSize];
static user::RingBuffer<1023> uart_rx_buffer;

static std::atomic_bool uart_tx_busy{false};
static uint8_t uart_tx_raw_buffer[64];

static user::Motor motor_0(user::config::g_motorConfig_0),
    motor_1(user::config::g_motorConfig_1);
static Controller controller(motor_0, motor_1);

inline void start_uart_receive()
{
    // 启动UART接收，使用中断方式，每次接收256字节
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_raw_buffer[uart_rx_raw_cnt], user::config::g_uartBufferSize);
}

inline void uart_rx_event_callback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint8_t index = std::exchange(uart_rx_raw_cnt, 1 - uart_rx_raw_cnt); // 获取当前使用的缓冲区索引，并切换到另一个缓冲区
    start_uart_receive();                                                // 继续接收下一批数据

    // 这里的size是本次接收完成的字节数，可能小于或等于我们设置的接收长度
    // 将接收到的数据从HAL的缓冲区复制到我们的环形缓冲区
    uart_rx_buffer.push({reinterpret_cast<const std::byte *>(uart_rx_raw_buffer[index]), size});
}

template <typename T>
    requires(sizeof(T) <= user::config::g_uartBufferSize)
inline void uart_send(const T &data)
{
    constexpr std::size_t send_size = sizeof(T);

    while (uart_tx_busy.load(std::memory_order_acquire))
    {
        // 等待上一次发送完成
    }

    uart_tx_busy.store(true, std::memory_order_release);
    // 将数据复制到发送缓冲区
    std::memcpy(uart_tx_raw_buffer, std::addressof(data), send_size);
    HAL_UART_Transmit_DMA(&huart1, uart_tx_raw_buffer, send_size);
}

void user_init()
{
    // 配置串口波特率
    huart1.Init.BaudRate = user::config::g_baudRate;
    HAL_UART_Init(&huart1);

    // 初始化串口接收
    HAL_UART_RegisterRxEventCallback(&huart1, uart_rx_event_callback);
    HAL_UART_RegisterCallback(
        &huart1, HAL_UART_TX_COMPLETE_CB_ID,
        [](UART_HandleTypeDef *huart)
        {
            uart_tx_busy.store(false, std::memory_order_release); // 发送完成后标记为非忙碌
        });
    start_uart_receive(); // 启动第一次接收

    // 初始化定时器控制任务
    HAL_TIM_RegisterCallback(
        &htim1, HAL_TIM_PERIOD_ELAPSED_CB_ID,
        [](TIM_HandleTypeDef *htim)
        {
            controller.update_handler(user::config::g_updateDurationUs);
        });
    HAL_TIM_Base_Start_IT(&htim1); // 启动定时器中断
}

void user_loop()
{
    if (std::byte head_0; uart_rx_buffer.peek({&head_0, 1}))
    {
        // 检查帧头
        if (head_0 == static_cast<std::byte>(user::Header::FrameHead0))
        {
            if (user::Header header; uart_rx_buffer.peek({reinterpret_cast<std::byte *>(&header), sizeof(header)}))
            {
                if (header.is_valid())
                {
                    // 根据命令类型处理数据
                    switch (static_cast<user::Command>(header.cmd))
                    {
                    case user::Command::Motion:
                        if (user::MotionCommandFrame frame;
                            uart_rx_buffer.peek({reinterpret_cast<std::byte *>(&frame), sizeof(frame)}))
                        {
                            uart_rx_buffer.pop(sizeof(frame)); // 弹出这个命令帧

                            // 验证数据格式
                            if (!std::isfinite(frame.target_angle[0]) || !std::isfinite(frame.target_angle[1]) ||
                                !std::isfinite(frame.target_speed[0]) || !std::isfinite(frame.target_speed[1]) ||
                                !std::isfinite(frame.process_time) || frame.process_time <= 0.0f)
                            {
                                // 数据格式错误
                                uart_send(user::NackFrame{user::Reason::InvalidData}); // 发送一个NACK响应
                                break;
                            }

                            // 检查是否超限
                            if (!motor_0.config().in_limit(frame.target_angle[0]) ||
                                !motor_1.config().in_limit(frame.target_angle[1]))
                            {
                                uart_send(user::NackFrame{user::Reason::InvalidData}); // 发送一个NACK响应
                                break;
                            }

                            // 处理轨迹
                            std::uint32_t duration_ms = std::lround(frame.process_time * 1e3f);
                            std::int32_t last_target_x, last_target_y, last_speed_x, last_speed_y;

                            controller.target_step(last_target_x, last_target_y);
                            if (auto current_segment_opt = controller.final_segment();
                                current_segment_opt.has_value())
                            {
                                const auto &current_segment = current_segment_opt.value();
                                last_speed_x = current_segment.x_profile.end_speed();
                                last_speed_y = current_segment.y_profile.end_speed();
                            }
                            else
                            {
                                last_speed_x = 0;
                                last_speed_y = 0;
                            }

                            constexpr user::PolynomialProfile::Type profile_type = user::PolynomialProfile::Type::Cubic;
                            user::PolynomialProfile x_profile(
                                profile_type,
                                last_target_x, motor_0.rad_to_step(frame.target_angle[0]),
                                duration_ms,
                                last_speed_x,
                                motor_0.rad_per_sec_to_step_per_ms(frame.target_speed[0]));
                            user::PolynomialProfile y_profile(
                                profile_type,
                                last_target_y, motor_1.rad_to_step(frame.target_angle[1]),
                                duration_ms,
                                last_speed_y,
                                motor_1.rad_per_sec_to_step_per_ms(frame.target_speed[1]));
                            if (!x_profile.is_valid() || !y_profile.is_valid())
                            {
                                uart_send(user::NackFrame{user::Reason::InvalidData}); // 发送一个NACK响应
                                break;
                            }

                            // 处理运动控制命令
                            if (!controller.add_segment(
                                    Controller::Segment{
                                        .duration_us = duration_ms * 1000, // 转换为微秒
                                        .x_profile = x_profile,
                                        .y_profile = y_profile}))
                            {
                                uart_send(user::NackFrame{user::Reason::ExecutionFailed}); // 发送一个NACK响应
                                break;
                            }

                            uart_send(user::AckFrame{}); // 发送一个ACK响应
                        }
                        break;
                    case user::Command::EmergencyStop:
                        if (user::EmergencyStopFrame frame;
                            uart_rx_buffer.peek({reinterpret_cast<std::byte *>(&frame), sizeof(frame)}))
                        {
                            // 处理紧急停止命令
                            controller.emergency_stop();
                            uart_rx_buffer.pop(sizeof(frame)); // 弹出这个命令帧

                            // 发送一个ACK响应
                            uart_send(user::AckFrame{});
                        }
                        break;
                    case user::Command::QueryStatus:
                        // 处理查询状态命令
                        if (user::QueryStatusFrame frame;
                            uart_rx_buffer.peek({reinterpret_cast<std::byte *>(&frame), sizeof(frame)}))
                        {
                            uart_rx_buffer.pop(sizeof(frame)); // 弹出这个命令帧
                            uart_send(user::StatusResponseFrame{
                                motor_0.step_to_rad(motor_0.current_step()),
                                motor_1.step_to_rad(motor_1.current_step())});
                        }
                        break;
                    default:
                        // 处理未知命令
                        uart_rx_buffer.pop(sizeof(header));                       // 弹出这个命令帧
                        uart_send(user::NackFrame{user::Reason::InvalidCommand}); // 发送NACK响应
                        break;
                    }
                }
                else
                {
                    // 如果帧头不正确，丢弃这个字节
                    uart_rx_buffer.pop(1);
                }
            }
        }
        else
        {
            // 如果帧头不正确，丢弃这个字节
            uart_rx_buffer.pop(1);
        }
    }
    else
    {
        // 没有数据可读，可以在这里执行其他任务或进入低功耗模式
        __WFI(); // 等待中断
    }
}