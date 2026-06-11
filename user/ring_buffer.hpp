#pragma once

#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <span>
#include <atomic>

namespace user
{
    template <std::size_t Capacity>
    class RingBuffer
    {
        static_assert(Capacity > 0, "Capacity must be greater than 0");

        static consteval std::size_t storage_capacity() noexcept
        {
            return Capacity + 1; // One slot is used to distinguish full vs empty
        }

        static constexpr std::size_t normal_index(std::size_t index) noexcept
        {
            if constexpr ((storage_capacity() & (storage_capacity() - 1)) == 0)
            {
                // If capacity is a power of 2, we can use bitwise AND for wrap-around
                return index & (storage_capacity() - 1);
            }
            else
            {
                // Otherwise, use modulo operator
                return index % storage_capacity();
            }
        }

        std::byte buffer_[storage_capacity()]{};
        std::atomic_size_t head_{0};
        std::atomic_size_t tail_{0};

    public:
        constexpr RingBuffer() = default;

        constexpr std::size_t size() const noexcept
        {
            if constexpr ((storage_capacity() & (storage_capacity() - 1)) == 0)
            {
                // If capacity is a power of 2, we can use bitwise AND for wrap-around
                return (tail_ + storage_capacity() - head_) & (storage_capacity() - 1);
            }
            else
            {
                // Otherwise, use modulo operator
                return (tail_ + storage_capacity() - head_) % storage_capacity();
            }
        }

        consteval std::size_t capacity() const noexcept
        {
            return storage_capacity() - 1;
        }

        constexpr std::size_t free_space() const noexcept
        {
            return capacity() - size();
        }

        constexpr bool push(const std::span<const std::byte> item) noexcept
        {
            if (item.size() > free_space())
            {
                return false; // Not enough space
            }

            // Write item to buffer with wrap-around
            std::size_t first_chunk = std::min(item.size(), storage_capacity() - tail_);
            for (std::size_t i = 0, current_tail = tail_; i < first_chunk; ++i)
            {
                buffer_[current_tail + i] = item[i];
            }
            
            std::size_t remaining = item.size() - first_chunk;
            for (std::size_t i = 0; i < remaining; ++i)
            {
                buffer_[i] = item[first_chunk + i];
            }

            tail_ = normal_index(tail_ + item.size());
            return true;
        }

        constexpr bool peek(std::span<std::byte> out) const noexcept
        {
            if (out.size() > size())
            {
                return false; // Not enough data
            }

            // Read item from buffer with wrap-around
            std::size_t first_chunk = std::min(out.size(), storage_capacity() - head_);
            for (std::size_t i = 0, current_head = head_; i < first_chunk; ++i)
            {
                out[i] = buffer_[current_head + i];
            }
            
            std::size_t remaining = out.size() - first_chunk;
            for (std::size_t i = 0; i < remaining; ++i)
            {
                out[first_chunk + i] = buffer_[i];
            }

            return true;
        }

        constexpr bool pop(std::size_t count) noexcept
        {
            if (count > size())
            {
                return false; // Not enough data to pop
            }

            head_ = normal_index(head_ + count);
            return true;
        }
    };
}