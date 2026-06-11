#pragma once

#include <bit>
#include <cstdint>
#include <numbers>

namespace user
{
    template <std::size_t iterations = 1>
        requires(iterations > 0)
    inline constexpr float inv_sqrt(float x) noexcept
    {
        std::uint32_t i = std::bit_cast<std::uint32_t>(x);
        i = 0x5f3759df - (i >> 1);
        float f = std::bit_cast<float>(i);

        for (std::size_t iter = 0; iter < iterations; iter++)
        {
            f *= 1.5f - 0.5f * x * (f * f);
        }

        return f;
    }

    template <std::size_t iterations = 1>
        requires(iterations > 0)
    inline constexpr float sqrt(float x) noexcept
    {
        return x * inv_sqrt<iterations>(x);
    }

    constexpr float rad_to_deg(float rad) noexcept
    {
        return rad * 180.0f / std::numbers::pi_v<float>;
    }

    constexpr float deg_to_rad(float deg) noexcept
    {
        return deg * std::numbers::pi_v<float> / 180.0f;
    }
}
