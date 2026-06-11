#pragma once

#include <tuple>

#include <type_def.hpp>
#include <math_utils.hpp>

namespace user::config
{
    // comm config
    constexpr std::size_t g_baudRate = 921'600;   // 串口波特率
    constexpr std::size_t g_uartBufferSize = 512; // 串口收发缓冲区大小

    // motor config
    inline const MotorConfig g_motorConfig_0 = {
        .pulse_per_rev = 3200.0f,
        .pul_pin = {GPIOA, GPIO_PIN_0, true}, // 脉冲引脚：PA0，假设高电平有效
        .dir_pin = {GPIOA, GPIO_PIN_2, true}, // 方向引脚：PA2，低电平正向
        .ena_pin = {GPIOA, GPIO_PIN_4, true}, // 使能引脚：PA4，低电平有效
        // .limit = {deg_to_rad(210.0f), deg_to_rad(70.0f)},
        .limit = {deg_to_rad(70.0f), deg_to_rad(200.0f)},
        .init_offset = deg_to_rad(200.0f)};
    inline const MotorConfig g_motorConfig_1 = {
        .pulse_per_rev = 3200.0f,
        .pul_pin = {GPIOA, GPIO_PIN_1, true}, // 脉冲引脚：PA1，假设高电平有效
        .dir_pin = {GPIOA, GPIO_PIN_3, true}, // 方向引脚：PA3，低电平正向
        .ena_pin = {GPIOA, GPIO_PIN_5, true}, // 使能引脚：PA5，低电平有效
        .limit = {deg_to_rad(-20.0f), deg_to_rad(110.0f)},
        .init_offset = deg_to_rad(-20.0f)};

    constexpr std::size_t g_updateDurationUs = 50;

    // kinematic config
    constexpr float g_br = 0.12f; // 大臂长度
    constexpr float g_lr = 0.2f;  // 小臂长度
    constexpr float g_d = 0.1f;   // 基座与大臂连接点的水平距离
} // namespace user::config
