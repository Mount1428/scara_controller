#pragma once

#include <cstddef>
#include <cstdint>

namespace user
{
    inline void fast_copy(
        const std::byte *__restrict__ src,
        std::size_t size,
        std::byte *__restrict__ dst) noexcept
    {
        if (size == 0)
            return;

        // 8 字节拷贝路径（仅 64 位平台）
        if constexpr (sizeof(void *) >= 8)
        {
            // 对齐 dst 到 8 字节边界
            while (size > 0 && (reinterpret_cast<std::uintptr_t>(dst) & 7) != 0)
            {
                *dst++ = *src++;
                --size;
            }

            // 4×8 = 32 字节展开拷贝
            auto *src64 = reinterpret_cast<const std::uint64_t *>(src);
            auto *dst64 = reinterpret_cast<std::uint64_t *>(dst);
            while (size >= 32)
            {
                dst64[0] = src64[0];
                dst64[1] = src64[1];
                dst64[2] = src64[2];
                dst64[3] = src64[3];
                src64 += 4;
                dst64 += 4;
                size -= 32;
            }
            while (size >= 8)
            {
                *dst64++ = *src64++;
                size -= 8;
            }

            src = reinterpret_cast<const std::byte *>(src64);
            dst = reinterpret_cast<std::byte *>(dst64);
        }

        // 4 字节拷贝路径
        {
            // 对齐 dst 到 4 字节边界
            while (size > 0 && (reinterpret_cast<std::uintptr_t>(dst) & 3) != 0)
            {
                *dst++ = *src++;
                --size;
            }

            // 4×4 = 16 字节展开拷贝
            auto *src32 = reinterpret_cast<const std::uint32_t *>(src);
            auto *dst32 = reinterpret_cast<std::uint32_t *>(dst);
            while (size >= 16)
            {
                dst32[0] = src32[0];
                dst32[1] = src32[1];
                dst32[2] = src32[2];
                dst32[3] = src32[3];
                src32 += 4;
                dst32 += 4;
                size -= 16;
            }
            while (size >= 4)
            {
                *dst32++ = *src32++;
                size -= 4;
            }

            src = reinterpret_cast<const std::byte *>(src32);
            dst = reinterpret_cast<std::byte *>(dst32);
        }

        // 尾部逐字节拷贝
        while (size > 0)
        {
            *dst++ = *src++;
            --size;
        }
    }
}
