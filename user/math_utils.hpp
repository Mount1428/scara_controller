#pragma once

#include <cstdint>
#include <numbers>

namespace user
{
    template <std::size_t iterations = 1>
        requires(iterations > 0)
    inline constexpr float inv_sqrt(float x) noexcept
    {
        union
        {
            float f;
            uint32_t i;
        } u;
        u.f = x;
        u.i = 0x5f3759df - (u.i >> 1); // Initial guess

        for (std::size_t iter = 0; iter < iterations; iter++)
        {
            u.f *= 1.5f - 0.5f * x * (u.f * u.f); // Newton-Raphson iteration
        }

        return u.f;
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
