#pragma once

#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <cmath>

namespace user
{
    class PolynomialProfile
    {
        using Scalar = float;

    public:
        enum class Type : std::uint8_t
        {
            Linear, ///< 线性插值
            Cubic   ///< 三次多项式插值
        };

        constexpr PolynomialProfile() = default;

        PolynomialProfile(Type type,
                          std::int32_t start_step,
                          std::int32_t end_step,
                          std::uint64_t process_time,
                          Scalar start_speed, Scalar end_speed) noexcept
            : type_(type)
        {
            if (process_time == 0)
            {
                valid_ = false;
                return;
            }

            Scalar T = static_cast<Scalar>(process_time);
            Scalar D = static_cast<Scalar>(end_step - start_step);

            if (type == Type::Linear)
            {
                a1 = D / T;
                a2 = 0;
                a3 = 0;
                end_speed_ = a1;
            }
            else // Cubic
            {
                a1 = start_speed;
                a2 = (3.0f * D - (2.0f * start_speed + end_speed) * T) / (T * T);
                a3 = ((start_speed + end_speed) * T - 2.0f * D) / (T * T * T);
                end_speed_ = end_speed;
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
                return offset_step_;
            }

            if (ms <= 0)
            {
                return offset_step_;
            }

            if (type_ == Type::Linear)
            {
                return std::lround(a1 * ms) + offset_step_;
            }
            else // Cubic
            {
                return std::lround(ms * (a1 + ms * (a2 + ms * a3))) + offset_step_;
            }
        }

    private:
        Scalar a1{}, a2{}, a3{};         // 多项式系数
        std::int32_t offset_step_{};     // 起始步数
        std::int32_t steps_{};           // 总步数
        Scalar end_speed_{};             // 结束速度
        Type type_{Type::Linear};        // 插值类型
        bool valid_{true};               // 是否有效
    };
}
