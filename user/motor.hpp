#pragma once

#include <tuple>
#include <optional>

#include <tim.h>

#include <type_def.hpp>
#include <polynomial_profile.hpp>
#include <spsc_queue.hpp>
#include <protocol.hpp>

namespace user
{
    class Motor
    {
    public:
        explicit Motor(const MotorConfig &config) noexcept
            : config_(config)
        {
            offset_step_ = (config.init_offset) * config.pulse_per_rev / (2.0f * std::numbers::pi_v<float>);

            limit_[0] = rad_to_step(config.limit.first);
            limit_[1] = rad_to_step(config.limit.second);

            set_pulse(false);
        }

        void enable(bool en) noexcept
        {
            auto &[port, pin, active_high] = config_.ena_pin;
            bool real_ena_level = en ? active_high : !active_high;

            // 这里可以添加实际的GPIO控制代码来设置使能
            // HAL_GPIO_WritePin(..., ena_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(port, pin, real_ena_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        void set_pulse(bool level) noexcept
        {
            auto &[port, pin, active_high] = config_.pul_pin;
            bool real_pul_level = level ? active_high : !active_high;

            // 这里可以添加实际的GPIO控制代码来设置脉冲
            // HAL_GPIO_WritePin(..., pul_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(port, pin, real_pul_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        bool request_step(std::int32_t step) noexcept
        {
            if (step < limit_[0] || step > limit_[1])
            {
                return false; // 超出范围，拒绝请求
            }

            if (step != current_step_)
            {
                // 使能电机
                enable(true);

                // 更改电机方向
                if (step > current_step_)
                {
                    set_direction(true); // 正向
                    ++current_step_;     // 每次请求只改变一步
                }
                else
                {
                    set_direction(false); // 反向
                    --current_step_;      // 每次请求只改变一步
                }

                set_pulse(true); // 立即设置脉冲以响应步数请求
                return true;     // 请求成功
            }

            return false; // 没有变化，无需请求
        }

        std::int32_t rad_to_step(float rad) const noexcept
        {
            return std::lround(rad * config_.pulse_per_rev / (2.0f * std::numbers::pi_v<float>)) - offset_step_;
        }

        float step_to_rad(std::int32_t step) const noexcept
        {
            return (step + offset_step_) * (2.0f * std::numbers::pi_v<float>) / config_.pulse_per_rev;
        }

        float rad_per_sec_to_step_per_ms(float rad_s) const noexcept
        {
            return rad_s * config_.pulse_per_rev / (2.0f * std::numbers::pi_v<float>)*1e-3f;
        }

        std::int32_t current_step() const noexcept
        {
            return current_step_;
        }

        const MotorConfig &config() const noexcept
        {
            return config_;
        }

    private:
        MotorConfig config_;

        std::int32_t current_step_{0}; ///< 当前步数
        std::int32_t limit_[2]{};      ///< 步数限制（根据config_.limit计算得到）
        std::int32_t offset_step_{0};  ///< 初始偏移步数（根据config_.init_offset计算得到）

        void set_direction(bool dir_level) noexcept
        {
            auto &[port, pin, active_high] = config_.dir_pin;
            bool real_dir_level = dir_level ? active_high : !active_high;

            // 这里可以添加实际的GPIO控制代码来设置方向
            // HAL_GPIO_WritePin(..., dir_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(port, pin, real_dir_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    };

    template <std::size_t Capacity>
    class Controller
    {
    public:
        struct Segment
        {
            std::uint32_t duration_us; // 运动段持续时间（单位：微秒）
            PolynomialProfile x_profile, y_profile;
        };

        explicit Controller(Motor &motor_x, Motor &motor_y) noexcept
            : motor_x_(motor_x), motor_y_(motor_y)
        {
        }

        void update_handler(uint32_t dt_us) noexcept
        {
            // 这里可以从segment_queue_中取出当前的运动段，并根据dt_us更新电机状态
            if (auto segment_opt = segment_queue_.front_ptr();
                segment_opt.has_value())
            {
                const Segment &segment = *segment_opt.value();

                // 如果当前段已完成，弹出队列
                if (elapsed_time_us_ >= segment.duration_us)
                {
                    segment_queue_.pop();
                    elapsed_time_us_ = 0; // 重置时间计数器

                    // 修正步数误差
                    bool motor_x_updated = motor_x_.request_step(segment.x_profile.steps());
                    bool motor_y_updated = motor_y_.request_step(segment.y_profile.steps());

                    step_once(motor_x_updated, motor_y_updated);
                    return;
                }
                else
                {
                    // 更新当前段的执行时间
                    elapsed_time_us_ += dt_us;
                    float elapsed_time_ms = static_cast<float>(elapsed_time_us_) * 1e-3f;

                    // 计算当前段的目标步数
                    std::int32_t target_step_x = segment.x_profile.current_step(elapsed_time_ms);
                    std::int32_t target_step_y = segment.y_profile.current_step(elapsed_time_ms);

                    // 请求电机执行步数
                    bool motor_x_updated = motor_x_.request_step(target_step_x);
                    bool motor_y_updated = motor_y_.request_step(target_step_y);

                    step_once(motor_x_updated, motor_y_updated);
                }
            }
        }

        bool add_segment(const Segment &segment) noexcept
        {
            // 这里可以添加逻辑来将新的运动段添加到segment_queue_中
            if (segment_queue_.emplace(segment))
            {
                target_step_x_ = segment.x_profile.steps();
                target_step_y_ = segment.y_profile.steps();
                return true;
            }
            return false;
        }

        std::optional<Segment> current_segment() const noexcept
        {
            return segment_queue_.front();
        }

        auto final_segment() const noexcept
        {
            return segment_queue_.back_ptr();
        }

        void emergency_stop() noexcept
        {
            // 紧急停止，立即停止所有运动并清空队列
            motor_x_.enable(false);
            motor_y_.enable(false);

            while (segment_queue_.pop())
                ; // 清空队列

            elapsed_time_us_ = 0; // 重置时间计数器
            target_step_x_ = motor_x_.current_step();
            target_step_y_ = motor_y_.current_step();
        }

        void target_step(std::int32_t &x, std::int32_t &y) const noexcept
        {
            x = target_step_x_;
            y = target_step_y_;
        }

    private:
        Motor &motor_x_, &motor_y_;

        SpscQueue<Segment, Capacity> segment_queue_{};
        std::uint32_t elapsed_time_us_{}; // 当前运动段已执行的时间（单位：微秒）

        std::int32_t target_step_x_{}, target_step_y_{};

        static void delay_us(std::uint32_t us) noexcept
        {
            // DWT 周期计数器精确延时
            std::uint32_t start = DWT->CYCCNT;
            std::uint32_t ticks = us * (SystemCoreClock / 1'000'000);
            while ((DWT->CYCCNT - start) < ticks)
                ;
        }

        void step_once(bool motor_x_updated, bool motor_y_updated) noexcept
        {
            if (motor_x_updated || motor_y_updated)
            {
                delay_us(6); // 脉冲宽度至少 6µs
                motor_x_.set_pulse(false);
                motor_y_.set_pulse(false);
            }
        }
    };
}
