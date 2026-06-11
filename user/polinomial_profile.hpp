#pragma once

#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <cmath>

namespace user
{
    class PolinomialProfile
    {
        using Scalar = float;

    public:
        enum class Type
        {
            Linear, ///< 线性插值
            Cubic   ///< 三次多项式插值
        };

        constexpr PolinomialProfile() = default;

        /**
         * @brief 计算三次多项式轨迹参数
         * @param displacement 运动位移（单位：脉冲）
         * @param process_time 运动总时间（单位：毫秒）
         * @param start_speed  起始速度（单位：脉冲/毫秒）
         * @param end_speed    结束速度（单位：脉冲/毫秒）
         */
        PolinomialProfile(Type type,
                          std::int32_t start_step,
                          std::int32_t end_step,
                          std::uint64_t process_time,
                          Scalar start_speed, Scalar end_speed) noexcept
            : type_(type)
        {
            if (process_time == 0)
            {
                // 无时间，无法计算速度，错误状态
                valid_ = false;
                return;
            }

            if (type == Type::Linear)
            {
                // 线性插值，速度不变
                a1 = (end_step - start_step) / static_cast<Scalar>(process_time);
                a2 = 0;
                a3 = 0;

                end_speed_ = a1; // 线性插值的结束速度等于恒定速度
            }

            if (type == Type::Cubic)
            {
                // 计算三次多项式系数
                // 设多项式为 f(t) = a0 + a1*t + a2*t^2 + a3*t^3
                // 边界条件：
                // f(0) = 0
                // f(process_time) = displacement
                // f'(0) = start_speed
                // f'(process_time) = end_speed

                a1 = start_speed;
                a2 = (3.0f * (end_step - start_step) - (2.0f * start_speed + end_speed) * process_time) / (process_time * process_time);
                a3 = ((start_speed + end_speed) * process_time - 2.0f * (end_step - start_step)) / (process_time * process_time * process_time);

                end_speed_ = end_speed; // 三次多项式插值的结束速度由参数直接指定
            }

            offset_step_ = start_step;
            steps_ = end_step - start_step;
        }

        bool is_valid() const noexcept
        {
            return valid_;
        }

        std::int32_t steps() const noexcept
        {
            return steps_ + offset_step_;
        }

        Scalar end_speed() const noexcept
        {
            return end_speed_;
        }

        std::int32_t current_step(Scalar ms) const noexcept
        {
            if (!valid_)
            {
                return offset_step_; // 无效状态，保持在起始位置
            }

            if (ms <= 0)
            {
                return offset_step_; // 时间未开始，保持在起始位置
            }

            if (type_ == Type::Linear)
            {
                return std::lround(a1 * ms) + offset_step_; // 线性插值直接计算
            }
            else // if (type_ == Type::Cubic)
            {
                Scalar ms_2 = static_cast<Scalar>(ms * ms);
                Scalar ms_3 = ms_2 * static_cast<Scalar>(ms);

                return std::lround(a1 * ms + a2 * ms_2 + a3 * ms_3) + offset_step_;
            }
        }

    private:
        Type type_{Type::Linear}; // 插值类型
        Scalar a1{}, a2{}, a3{};  // 三次多项式系数(a0固定为0<-初始位移为0)

        std::int32_t offset_step_{}; // 起始步数偏移（用于计算绝对目标步数）
        std::int32_t steps_{};       // 总步数
        Scalar end_speed_{};         // 结束速度

        bool valid_{true}; // 是否有效
    };
}