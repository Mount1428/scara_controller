#pragma once

#include <cstdint>
#include <span>

namespace user
{
    enum class Command : std::uint8_t
    {
        None = 0x00, ///< 无命令

        Motion = 0x01,        ///< 运动控制命令
        EmergencyStop = 0x02, ///< 紧急停止命令

        QueryStatus = 0x03, ///< 查询下位机状态命令

        Ack = 0xF0,           ///< 通用ACK响应
        Nack = 0xF1,          ///< 通用NACK响应
        StatusResponse = 0xF2 ///< 下位机状态响应
    };

    enum class Reason : std::uint8_t
    {
        None = 0x00,           ///< 无错误
        InvalidCommand = 0x01, ///< 无效命令
        InvalidData = 0x02,    ///< 数据格式错误
        ExecutionFailed = 0x03 ///< 执行失败（如运动超限）
    };

    inline constexpr const char *command_to_string(Command cmd)
    {
        switch (cmd)
        {
        case Command::Motion:
            return "Motion";
        case Command::EmergencyStop:
            return "EmergencyStop";
        case Command::QueryStatus:
            return "QueryStatus";
        case Command::StatusResponse:
            return "StatusResponse";
        case Command::Ack:
            return "Ack";
        case Command::Nack:
            return "Nack";
        default:
            return "Unknown";
        }
    }

#pragma pack(push, 1) // 确保结构体按1字节对齐，避免编译器添加填充字节

    struct Header
    {
        static constexpr std::uint8_t FrameHead0 = 0xAA; ///< 固定帧头0xAA
        static constexpr std::uint8_t FrameHead1 = 0x55; ///< 固定帧头0x55

        std::uint8_t head0{0xAA}; ///< 固定帧头0xAA
        std::uint8_t head1{0x55}; ///< 固定帧头0x55
        std::uint8_t cmd;         ///< 命令码
        std::uint8_t padding{};   ///< 填充字节，保持帧头和命令码对齐到4字节边界

        constexpr Header() : cmd(static_cast<std::uint8_t>(Command::None)) {}

        explicit constexpr Header(Command command) : cmd(static_cast<std::uint8_t>(command)) {}

        constexpr bool is_valid() const
        {
            return head0 == 0xAA && head1 == 0x55;
        }
    };

    struct AckFrame final
    {
        Header header{Command::Ack};

        constexpr AckFrame() = default;
    };

    struct NackFrame final
    {
        Header header{Command::Nack};
        Reason reason; ///< NACK原因

        explicit constexpr NackFrame(Reason r) : reason(r) {}
    };

    struct StatusResponseFrame final
    {
        Header header{Command::StatusResponse};
        float angle[2]; ///< 当前两轴绝对角度（单位rad）

        constexpr StatusResponseFrame() = default;

        constexpr StatusResponseFrame(float angle_0, float angle_1) noexcept
            : angle{angle_0, angle_1}
        {
        }
    };

    struct MotionCommandFrame final
    {
        Header header{Command::Motion};

        float target_angle[2]; ///< 目标两轴绝对角度（单位rad）
        float target_speed[2]; ///< 目标两轴绝对速度（单位rad/s）
        float process_time;    ///< 预计该运动段运动时间（单位s）
    };

    struct QueryStatusFrame final
    {
        Header header{Command::QueryStatus};

        constexpr QueryStatusFrame() = default;
    };

    struct EmergencyStopFrame final
    {
        Header header{Command::EmergencyStop};

        constexpr EmergencyStopFrame() = default;
    };

#pragma pack(pop)

    template <typename T>
    inline std::span<const std::byte> as_bytes(const T &value = {}) noexcept
    {
        return {reinterpret_cast<const std::byte *>(&value), sizeof(T)};
    }
}
