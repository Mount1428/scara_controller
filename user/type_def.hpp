#pragma once

#include <cstdint>
#include <tuple>

#include <stm32f1xx_hal.h>

namespace user
{
    struct MotorConfig
    {
        float pulse_per_rev; // 每转脉冲数

        std::tuple<GPIO_TypeDef *, uint16_t, bool> pul_pin; // 脉冲引脚（GPIO端口 | 引脚号 | 有效电平）
        std::tuple<GPIO_TypeDef *, uint16_t, bool> dir_pin; // 方向引脚（GPIO端口 | 引脚号 | 有效电平）
        std::tuple<GPIO_TypeDef *, uint16_t, bool> ena_pin; // 使能引脚（GPIO端口 | 引脚号 | 有效电平）

        std::pair<float, float> limit; // 软限位，单位为rad (min, max)
        float init_offset;             // 上电时偏移位置，单位为rad

        constexpr bool in_limit(float rad) const noexcept
        {
            // if (limit.first < limit.second)
            // {
                return rad >= limit.first && rad <= limit.second; // 在区间内返回true，否则返回false
            // }
            // else // 如果limit.first >= limit.second，说明区间跨越了0度（如[-30°, 90°]），需要特殊处理
            // {
            //     return (rad >= limit.first - 2 * std::numbers::pi_v<float> || rad <= limit.second) &&
            //            rad >= -std::numbers::pi_v<float> &&
            //            rad <= std::numbers::pi_v<float>; // 在区间内返回true，否则返回false，同时确保输入在合理范围内
            // }
        }
    };
}
