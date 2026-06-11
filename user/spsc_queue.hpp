#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <optional>

namespace user
{
    template <typename T, std::size_t Capacity>
    class SpscQueue
    {
        static_assert(Capacity > 0, "Capacity must be greater than 0");
        static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");
        static_assert(std::atomic_size_t::is_always_lock_free, "std::atomic_size_t must be lock-free");
        static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of 2 for optimal performance");

        std::atomic_size_t head_{0};
        std::atomic_size_t tail_{0};
        T buffer_[Capacity]{};

        static constexpr std::size_t next_index(std::size_t index) noexcept
        {
            return (index + 1) & (Capacity - 1); // Wrap around using bitwise AND
        }

        static constexpr std::size_t prev_index(std::size_t index) noexcept
        {
            return (index - 1) & (Capacity - 1); // Wrap around using bitwise AND
        }

    public:
        constexpr SpscQueue() = default;

        bool push(const T &item) noexcept
        {
            std::size_t current_tail = tail_.load(std::memory_order_relaxed);
            std::size_t next_tail = next_index(current_tail);

            if (next_tail == head_.load(std::memory_order_acquire))
            {
                return false; // Queue is full
            }

            buffer_[current_tail] = item; // Copy item into buffer
            tail_.store(next_tail, std::memory_order_release);
            return true;
        }

        template <typename... Args>
        bool emplace(Args &&...args) noexcept
        {
            std::size_t current_tail = tail_.load(std::memory_order_relaxed);
            std::size_t next_tail = next_index(current_tail);

            if (next_tail == head_.load(std::memory_order_acquire))
            {
                return false; // Queue is full
            }

            std::construct_at<T, Args...>(&buffer_[current_tail], std::forward<Args>(args)...);
            tail_.store(next_tail, std::memory_order_release);
            return true;
        }

        bool pop() noexcept
        {
            std::size_t current_head = head_.load(std::memory_order_relaxed);

            if (current_head == tail_.load(std::memory_order_acquire))
            {
                return false; // Queue is empty
            }

            head_.store(next_index(current_head), std::memory_order_release);
            return true;
        }

        std::optional<T> front() const noexcept
        {
            std::size_t current_head = head_.load(std::memory_order_acquire);

            if (current_head == tail_.load(std::memory_order_acquire))
            {
                return std::nullopt; // Queue is empty
            }

            return buffer_[current_head]; // Return a copy of the front item
        }

        std::optional<std::add_pointer_t<const T>> front_ptr() const noexcept
        {
            std::size_t current_head = head_.load(std::memory_order_acquire);

            if (current_head == tail_.load(std::memory_order_acquire))
            {
                return std::nullopt; // Queue is empty
            }

            return &buffer_[current_head]; // Return a copy of the front item
        }

        std::optional<T> back() const noexcept
        {
            std::size_t current_tail = tail_.load(std::memory_order_acquire);

            if (current_tail == head_.load(std::memory_order_acquire))
            {
                return std::nullopt; // Queue is empty
            }

            std::size_t last = prev_index(current_tail);
            return buffer_[last]; // Return a copy of the back item
        }

        std::optional<std::add_pointer_t<const T>> back_ptr() const noexcept
        {
            std::size_t current_tail = tail_.load(std::memory_order_acquire);

            if (current_tail == head_.load(std::memory_order_acquire))
            {
                return std::nullopt; // Queue is empty
            }

            std::size_t last = prev_index(current_tail);
            return &buffer_[last];
        }
    };
} // namespace user
