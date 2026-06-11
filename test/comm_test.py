"""
下位机通信测试工具
==================
功能:
  1. 通信延迟测量 — 连续发送 QueryStatus 指令，统计 RTT
  2. 帧对齐测试   — 发送已知数据验证帧边界，注入错误测试恢复
  3. 压力测试     — 连续发送大量 Motion 指令，检查处理能力

用法:
  python comm_test.py --port COM3 --latency --count 100
  python comm_test.py --port COM3 --align
  python comm_test.py --port COM3 --stress --motions 500
  python comm_test.py --port COM3 --all
"""

from __future__ import annotations

import argparse
import math
import statistics
import struct
import sys
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Optional, TypeVar


# ──────────────────────────────────────────────
# 重试辅助
# ──────────────────────────────────────────────

@dataclass
class RetryConfig:
    max_retries: int = 3       # 最大重试次数（0 = 不重试）
    delay_s: float = 0.0       # 重试间隔（秒）


T = TypeVar("T")


def with_retry(
    fn: Callable[[], T],
    cfg: RetryConfig,
    is_retryable: Callable[[Exception], bool] = lambda e: isinstance(e, (TimeoutError, ValueError)),
    pre_retry: Callable[[], None] | None = None,
) -> tuple[T, int]:
    """
    执行 fn，失败时按 cfg 自动重试。
    pre_retry 在每次重试前调用（可用于 drain 残留字节）。
    返回 (result, retries_consumed)。
    重试用尽后抛出最后一次异常。
    """
    last_exc: Exception | None = None
    for attempt in range(cfg.max_retries + 1):
        try:
            return fn(), attempt
        except Exception as e:
            last_exc = e
            if not is_retryable(e) or attempt == cfg.max_retries:
                raise
            if pre_retry is not None:
                pre_retry()
            if cfg.delay_s > 0:
                import time as _time
                _time.sleep(cfg.delay_s)
    # unreachable, but keep type checker happy
    raise last_exc  # type: ignore[misc]


# ──────────────────────────────────────────────
# 协议常量 & 打包（与 C++ 端严格对齐）
# ──────────────────────────────────────────────

class Command(IntEnum):
    Motion = 0x01
    EmergencyStop = 0x02
    QueryStatus = 0x03
    Ack = 0xF0
    Nack = 0xF1
    StatusResponse = 0xF2


class Reason(IntEnum):
    InvalidCommand = 0x01
    InvalidData = 0x02
    ExecutionFailed = 0x03


FRAME_HEAD0 = 0xAA
FRAME_HEAD1 = 0x55

# C++ Header = 4 bytes: head0(1) + head1(1) + cmd(1) + padding(1)
HEADER_FMT = "<BBBB"
ACK_FMT    = "<BBBB"        # AckFrame = Header = 4 bytes
NACK_FMT   = "<BBBBB"       # NackFrame = Header + Reason(1) = 5 bytes
STATUS_FMT = "<BBBxff"      # StatusResponse = Header(4) + 2 floats(8) = 12 bytes
MOTION_FMT = "<BBBxfffff"   # MotionCommand = Header(4) + 5 floats(20) = 24 bytes
QUERY_FMT  = "<BBBB"        # QueryStatusFrame = Header = 4 bytes
ESTOP_FMT  = "<BBBB"        # EmergencyStopFrame = Header = 4 bytes

ACK_SIZE    = struct.calcsize(ACK_FMT)      # 4
NACK_SIZE   = struct.calcsize(NACK_FMT)     # 5
STATUS_SIZE = struct.calcsize(STATUS_FMT)   # 12
MOTION_SIZE = struct.calcsize(MOTION_FMT)   # 24
QUERY_SIZE  = struct.calcsize(QUERY_FMT)    # 4
ESTOP_SIZE  = struct.calcsize(ESTOP_FMT)    # 4


def pack_ack() -> bytes:
    return struct.pack(ACK_FMT, FRAME_HEAD0, FRAME_HEAD1, Command.Ack, 0)

def pack_nack(reason: Reason) -> bytes:
    return struct.pack(NACK_FMT, FRAME_HEAD0, FRAME_HEAD1, Command.Nack, 0, int(reason))

def pack_query_status() -> bytes:
    return struct.pack(QUERY_FMT, FRAME_HEAD0, FRAME_HEAD1, Command.QueryStatus, 0)

def pack_emergency_stop() -> bytes:
    return struct.pack(ESTOP_FMT, FRAME_HEAD0, FRAME_HEAD1, Command.EmergencyStop, 0)

def pack_motion(
    abs_angle0: float, abs_angle1: float,
    target_speed0: float, target_speed1: float,
    process_time: float,
) -> bytes:
    return struct.pack(
        MOTION_FMT,
        FRAME_HEAD0, FRAME_HEAD1, Command.Motion,
        abs_angle0, abs_angle1, target_speed0, target_speed1, process_time,
    )


# ──────────────────────────────────────────────
# 串口读写工具
# ──────────────────────────────────────────────

def _read_exact(ser, size: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    buf = bytearray()
    while len(buf) < size:
        chunk = ser.read(size - len(buf))
        if chunk:
            buf.extend(chunk)
            continue
        if time.monotonic() >= deadline:
            break
    if len(buf) != size:
        raise TimeoutError(f"expected {size} bytes, got {len(buf)}")
    return bytes(buf)


@dataclass
class Frame:
    cmd: int
    payload: bytes  # 完整帧（含头部）


def read_frame(ser, timeout_s: float) -> Frame:
    """读取一帧，返回 Frame(cmd, full_frame_bytes)。抛出 TimeoutError / ValueError。"""
    header = _read_exact(ser, 4, timeout_s)  # C++ Header = 4 字节
    head0, head1, cmd, _padding = struct.unpack(HEADER_FMT, header)
    if (head0, head1) != (FRAME_HEAD0, FRAME_HEAD1):
        raise ValueError(
            f"bad frame head: got {head0:#04x} {head1:#04x}, "
            f"expected {FRAME_HEAD0:#04x} {FRAME_HEAD1:#04x}"
        )

    if cmd == Command.Ack:
        return Frame(cmd, header)
    if cmd == Command.Nack:
        return Frame(cmd, header + _read_exact(ser, 1, timeout_s))  # reason
    if cmd == Command.StatusResponse:
        rest_size = STATUS_SIZE - 4
        return Frame(cmd, header + _read_exact(ser, rest_size, timeout_s))

    raise ValueError(f"unexpected response command 0x{cmd:02X}")


def query_status(ser, timeout_s: float) -> tuple[float, float]:
    """发送 QueryStatus，返回 (angle0, angle1)。"""
    ser.write(pack_query_status())
    frame = read_frame(ser, timeout_s)
    if frame.cmd != Command.StatusResponse:
        raise RuntimeError(f"expected StatusResponse, got cmd=0x{frame.cmd:02X}")
    _head0, _head1, _cmd, angle0, angle1 = struct.unpack(STATUS_FMT, frame.payload)
    return angle0, angle1


# ──────────────────────────────────────────────
# 1. 通信延迟测量
# ──────────────────────────────────────────────

@dataclass
class LatencyResult:
    samples: int
    min_ms: float
    max_ms: float
    avg_ms: float
    median_ms: float
    p95_ms: float
    p99_ms: float
    timeouts: int
    errors: int
    retries: int = 0           # 总重试次数
    retried_samples: int = 0   # 发生过重试的样本数


def measure_latency(
    ser,
    count: int,
    timeout_s: float,
    warmup: int = 5,
    retry_cfg: RetryConfig = RetryConfig(max_retries=3),
) -> LatencyResult:
    """
    连续发送 count 次 QueryStatus，测量往返时间。
    前 warmup 次为预热，不计入统计。
    失败时自动重试（受 retry_cfg 控制）。
    """
    latencies: list[float] = []
    timeouts = 0
    errors = 0
    total_retries = 0
    retried_samples = 0
    total = count + warmup

    print(f"\n{'='*50}")
    print(f"延迟测量: {count} 次 QueryStatus  (最大重试 {retry_cfg.max_retries} 次)")
    print(f"{'='*50}")

    for i in range(total):
        is_warmup = i < warmup
        seq = i - warmup + 1
        label = "warmup" if is_warmup else f"{seq:>4}/{count}"

        start = time.monotonic()
        retries_used = 0
        try:
            def _do_query() -> Frame:
                ser.write(pack_query_status())
                return read_frame(ser, timeout_s)

            def _on_latency_retry() -> None:
                _drain_input(ser, 0.002)

            frame, retries_used = with_retry(
                _do_query, retry_cfg, pre_retry=_on_latency_retry,
            )
            elapsed = (time.monotonic() - start) * 1000  # ms

            if frame.cmd == Command.StatusResponse:
                if not is_warmup:
                    latencies.append(elapsed)
                    total_retries += retries_used
                    if retries_used > 0:
                        retried_samples += 1
                note = f" (重试{retries_used})" if retries_used > 0 else ""
                if (i + 1) % max(1, total // 20) == 0 or i == total - 1:
                    print(f"  [{label}] {elapsed:.3f} ms  OK{note}")
            else:
                if not is_warmup:
                    errors += 1
                print(f"  [{label}] unexpected cmd=0x{frame.cmd:02X}  ERROR")

        except TimeoutError:
            if not is_warmup:
                timeouts += 1
            print(f"  [{label}] TIMEOUT (重试{retries_used}次后放弃)")
        except (ValueError, RuntimeError) as e:
            if not is_warmup:
                errors += 1
            print(f"  [{label}] {e}  ERROR (重试{retries_used}次后放弃)")

    if not latencies:
        return LatencyResult(
            samples=0, min_ms=0, max_ms=0, avg_ms=0,
            median_ms=0, p95_ms=0, p99_ms=0,
            timeouts=timeouts, errors=errors,
            retries=total_retries, retried_samples=retried_samples,
        )

    latencies.sort()
    n = len(latencies)
    return LatencyResult(
        samples=n,
        min_ms=latencies[0],
        max_ms=latencies[-1],
        avg_ms=sum(latencies) / n,
        median_ms=latencies[n // 2],
        p95_ms=latencies[int(n * 0.95)],
        p99_ms=latencies[int(n * 0.99)],
        timeouts=timeouts,
        errors=errors,
        retries=total_retries,
        retried_samples=retried_samples,
    )


def print_latency_result(r: LatencyResult) -> None:
    print(f"\n{'─'*50}")
    print(f"延迟统计 ({r.samples} 个有效样本)")
    print(f"{'─'*50}")
    if r.samples == 0:
        print("  无有效数据")
        return
    print(f"  最小值:  {r.min_ms:>8.3f} ms")
    print(f"  最大值:  {r.max_ms:>8.3f} ms")
    print(f"  平均值:  {r.avg_ms:>8.3f} ms")
    print(f"  中位数:  {r.median_ms:>8.3f} ms")
    print(f"  P95:     {r.p95_ms:>8.3f} ms")
    print(f"  P99:     {r.p99_ms:>8.3f} ms")
    if r.timeouts:
        print(f"  超时:    {r.timeouts}")
    if r.errors:
        print(f"  错误:    {r.errors}")
    if r.retries > 0:
        print(f"  重试:    {r.retries} 次 (涉及 {r.retried_samples}/{r.samples} 个样本)")


# ──────────────────────────────────────────────
# 2. 帧对齐测试
# ──────────────────────────────────────────────

class AlignmentTestResult:
    def __init__(self) -> None:
        self.stray_bytes_detected = 0
        self.recovery_success = False
        self.frames_matched = 0
        self.frames_mismatched = 0
        self.details: list[str] = []


def test_frame_alignment(
    ser,
    timeout_s: float,
    iterations: int = 50,
    retry_cfg: RetryConfig = RetryConfig(max_retries=3),
) -> AlignmentTestResult:
    """
    帧对齐测试：
    1. 连续发送 QueryStatus，检查每次响应是否完整、无残留字节
    2. 注入随机垃圾数据后发送有效帧（自动重试），测试帧同步恢复能力
    3. 半帧头部后发有效帧，测试协议鲁棒性
    """
    result = AlignmentTestResult()

    print(f"\n{'='*50}")
    print(f"帧对齐测试  (最大重试 {retry_cfg.max_retries} 次)")
    print(f"{'='*50}")

    # ── 测试 1: 连续正常通信，检测残留字节 ──
    print("\n--- 测试 1: 连续 QueryStatus，检测帧一致性 ---")
    for i in range(iterations):
        def _do_query() -> Frame:
            ser.write(pack_query_status())
            return read_frame(ser, timeout_s)

        try:
            frame, _retries = with_retry(_do_query, retry_cfg)
        except (TimeoutError, ValueError) as e:
            result.details.append(f"[{i}] read failed after retry: {e}")
            result.frames_mismatched += 1
            continue

        # 检查长度是否与预期一致
        if frame.cmd == Command.StatusResponse:
            if len(frame.payload) != STATUS_SIZE:
                result.stray_bytes_detected += 1
                result.details.append(
                    f"[{i}] StatusResponse 长度异常: "
                    f"expected {STATUS_SIZE}, got {len(frame.payload)}"
                )
                result.frames_mismatched += 1
                continue
            result.frames_matched += 1
        elif frame.cmd == Command.Nack:
            result.frames_matched += 1
        else:
            result.frames_mismatched += 1
            result.details.append(f"[{i}] 非预期响应 cmd=0x{frame.cmd:02X}")

        if (i + 1) % max(1, iterations // 5) == 0:
            print(f"  [{i+1}/{iterations}]  OK  (匹配: {result.frames_matched}, "
                  f"异常: {result.frames_mismatched})")

    # ── 测试 2: 注入垃圾数据，测试同步恢复 ──
    print("\n--- 测试 2: 垃圾注入后帧同步恢复 ---")
    garbage_lengths = [1, 3, 7, 15, 31, 63]
    for n_bytes in garbage_lengths:
        garbage = bytes([0x00] * n_bytes)
        ser.write(garbage)

        def _do_recovery() -> Frame:
            ser.write(pack_query_status())
            return read_frame(ser, timeout_s)

        try:
            frame, _retries = with_retry(_do_recovery, retry_cfg)
            if frame.cmd == Command.StatusResponse:
                result.recovery_success = True
                print(f"  inject {n_bytes:>2} garbage bytes → 恢复成功 (StatusResponse)")
            else:
                result.details.append(
                    f"  inject {n_bytes} garbage bytes → 收到 cmd=0x{frame.cmd:02X}"
                )
                print(f"  inject {n_bytes:>2} garbage bytes → 恢复但收到非预期响应")
        except (TimeoutError, ValueError) as e:
            # 重试用尽后尝试 E-STOP 硬恢复
            result.details.append(
                f"  inject {n_bytes} garbage bytes → 恢复失败: {e}"
            )
            print(f"  inject {n_bytes:>2} garbage bytes → 恢复失败 (尝试 E-STOP)")

            def _do_estop_recovery() -> Frame:
                ser.write(pack_emergency_stop())
                return read_frame(ser, timeout_s)

            try:
                frame, _ = with_retry(_do_estop_recovery, retry_cfg)
                if frame.cmd == Command.Ack:
                    print(f"  → E-STOP 恢复成功")
                    result.recovery_success = True
            except (TimeoutError, ValueError):
                print(f"  → E-STOP 也失败，串口可能失步")
                result.recovery_success = False

    # ── 测试 3: 半帧头部后发有效帧（带重试）──
    print("\n--- 测试 3: 部分头部后发有效帧 ---")
    partial_headers = [
        b"\xAA",                    # 只有 head0
        b"\xAA\x55",               # head0 + head1
        b"\xAA\x55\x03",           # 完整的 3 字节头部（不含 padding）
    ]
    for partial in partial_headers:
        ser.write(partial)
        def _do_partial_recovery() -> Frame:
            ser.write(pack_query_status())
            return read_frame(ser, timeout_s)

        try:
            frame, _retries = with_retry(_do_partial_recovery, retry_cfg)
            status = "OK" if frame.cmd == Command.StatusResponse else f"cmd=0x{frame.cmd:02X}"
            print(f"  partial {partial.hex()} → {status}")
        except (TimeoutError, ValueError) as e:
            print(f"  partial {partial.hex()} → 失败: {e}")

    return result


def print_alignment_result(r: AlignmentTestResult) -> None:
    print(f"\n{'─'*50}")
    print("对齐测试结果")
    print(f"{'─'*50}")
    print(f"  正常帧匹配:   {r.frames_matched}")
    print(f"  帧异常:       {r.frames_mismatched}")
    print(f"  残留字节:     {r.stray_bytes_detected}")
    print(f"  垃圾恢复:     {'成功' if r.recovery_success else '失败'}")

    if r.stray_bytes_detected > 0 or r.frames_mismatched > 0:
        print(f"\n  详细信息:")
        for d in r.details[-20:]:
            print(f"    {d}")


# ──────────────────────────────────────────────
# 3. 压力测试
# ──────────────────────────────────────────────

@dataclass
class StressTestResult:
    total: int
    accepted: int
    rejected: int
    timeouts: int
    errors: int
    duration_s: float
    throughput: float = 0.0  # commands/sec
    retries: int = 0


def _drain_input(ser, drain_timeout_s: float = 0.002) -> int:
    """
    清空串口接收缓冲区中的残留字节。
    返回丢弃的字节数。用于重试前恢复帧同步。
    """
    drained = 0
    saved_to = ser.timeout
    ser.timeout = 0.01  # 短超时，避免阻塞
    deadline = time.monotonic() + drain_timeout_s
    try:
        while time.monotonic() < deadline:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                break
            drained += len(chunk)
    finally:
        ser.timeout = saved_to
    return drained


def _resync_with_query(ser, timeout_s: float) -> bool:
    """
    通过 QueryStatus 尝试重新同步通信。
    成功返回 True。
    """
    for _ in range(3):
        ser.write(pack_query_status())
        try:
            frame = read_frame(ser, timeout_s)
            if frame.cmd == Command.StatusResponse:
                return True
        except (TimeoutError, ValueError):
            _drain_input(ser, 0.1)
    return False


def stress_test(
    ser,
    motion_count: int,
    timeout_s: float,
    angle_range: tuple[float, float] = (1.0, 2.0),
    speed_range: tuple[float, float] = (0.5, 1.0),
    process_time: float = 0.3,
    retry_cfg: RetryConfig = RetryConfig(max_retries=3),
) -> StressTestResult:
    """
    压力测试：逐条发送 motion_count 条运动指令，发一条等一条响应。
    失败时自动重试：
      1. 先 drain 接收缓冲区（清除可能残留的旧响应）
      2. 重新发送指令
      3. 读取响应
    重试用尽后通过 QueryStatus 尝试重新同步。
    """
    import random

    accepted = 0
    rejected = 0
    timeouts = 0
    errors = 0
    total_retries = 0
    rng = random.Random()

    print(f"\n{'='*50}")
    print(f"压力测试: {motion_count} 条 Motion 指令  (最大重试 {retry_cfg.max_retries} 次)")
    print(f"  process_time={process_time}s")
    print(f"{'='*50}")

    # 预生成运动参数
    motions: list[tuple[float, float, float, float, float]] = []
    for _ in range(motion_count):
        motions.append((
            rng.uniform(*angle_range),
            rng.uniform(*angle_range),
            rng.uniform(*speed_range),
            rng.uniform(*speed_range),
            process_time,
        ))

    start_time = time.monotonic()

    for idx in range(motion_count):
        ang0, ang1, spd0, spd1, pt = motions[idx]

        def _send_and_read_one() -> Frame:
            ser.write(pack_motion(ang0, ang1, spd0, spd1, pt))
            return read_frame(ser, timeout_s)

        def _on_stress_retry() -> None:
            _drain_input(ser, 0.002)  # 清空残留字节避免帧错位

        retries_used = 0
        try:
            frame, retries_used = with_retry(
                _send_and_read_one, retry_cfg,
                is_retryable=lambda e: isinstance(e, (TimeoutError, ValueError)),
                pre_retry=_on_stress_retry,
            )
            total_retries += retries_used

            if frame.cmd == Command.Ack:
                accepted += 1
            elif frame.cmd == Command.Nack:
                rejected += 1
            else:
                errors += 1
        except TimeoutError:
            # 重试用尽：尝试重新同步
            timeouts += 1
            if not _resync_with_query(ser, timeout_s):
                errors += 1
        except (ValueError, RuntimeError):
            errors += 1

        if (idx + 1) % max(1, motion_count // 20) == 0 or idx == motion_count - 1:
            elapsed = time.monotonic() - start_time
            progress = (idx + 1) / motion_count * 100
            print(
                f"  [{idx+1:>5}/{motion_count} ({progress:>5.1f}%)]  "
                f"ACK={accepted}  NACK={rejected}  TO={timeouts}  "
                f"ERR={errors}  RT={total_retries}  {elapsed:.1f}s"
            )

    duration = time.monotonic() - start_time
    return StressTestResult(
        total=motion_count,
        accepted=accepted,
        rejected=rejected,
        timeouts=timeouts,
        errors=errors,
        duration_s=duration,
        throughput=motion_count / duration if duration > 0 else 0,
        retries=total_retries,
    )


def print_stress_result(r: StressTestResult) -> None:
    print(f"\n{'─'*50}")
    print("压力测试结果")
    print(f"{'─'*50}")
    print(f"  总数:      {r.total}")
    print(f"  接受:      {r.accepted}")
    if r.total:
        print(f"  接受率:    {r.accepted/r.total*100:.1f}%")
    print(f"  拒绝:      {r.rejected}")
    print(f"  超时:      {r.timeouts}")
    print(f"  错误:      {r.errors}")
    if r.retries:
        print(f"  重试:      {r.retries} 次")
    print(f"  耗时:      {r.duration_s:.2f} s")
    print(f"  吞吐量:    {r.throughput:.1f} 指令/秒")


# ──────────────────────────────────────────────
# 4. 通信质量综合评估
# ──────────────────────────────────────────────

@dataclass
class CommQualityReport:
    latency: LatencyResult
    alignment: AlignmentTestResult
    stress: StressTestResult

    def summary(self) -> str:
        lines = [f"\n{'='*60}", "通信质量综合评估", f"{'='*60}"]

        # 延迟评级
        if self.latency.samples > 0:
            avg = self.latency.avg_ms
            if avg < 1:
                lat_grade = "优秀 (< 1ms)"
            elif avg < 5:
                lat_grade = "良好 (1-5ms)"
            elif avg < 20:
                lat_grade = "一般 (5-20ms)"
            else:
                lat_grade = f"差 ({avg:.1f}ms)"
            lines.append(f"  延迟: {lat_grade}")
            lines.append(f"    平均 {avg:.3f}ms, 最大 {self.latency.max_ms:.3f}ms")
            lines.append(f"    样本 {self.latency.samples}, 超时 {self.latency.timeouts}")
            if self.latency.retries > 0:
                lines.append(f"    重试 {self.latency.retries} 次 ({self.latency.retried_samples} 个样本涉及重试)")

        # 帧对齐评级
        if self.alignment.frames_matched + self.alignment.frames_mismatched > 0:
            total_f = self.alignment.frames_matched + self.alignment.frames_mismatched
            err_rate = self.alignment.frames_mismatched / total_f * 100
            if err_rate == 0 and self.alignment.stray_bytes_detected == 0:
                align_grade = "优秀 (零异常)"
            elif err_rate < 1:
                align_grade = f"良好 (错误率 {err_rate:.2f}%)"
            else:
                align_grade = f"差 (错误率 {err_rate:.2f}%)"
            lines.append(f"  帧对齐: {align_grade}")
            lines.append(f"    匹配 {self.alignment.frames_matched}, "
                         f"异常 {self.alignment.frames_mismatched}, "
                         f"残留字节 {self.alignment.stray_bytes_detected}")

        # 压力评级
        if self.stress.total > 0:
            err_total = self.stress.timeouts + self.stress.errors
            if err_total == 0:
                stress_grade = "优秀 (零错误)"
            elif err_total / self.stress.total < 0.01:
                stress_grade = f"良好 (错误率 {err_total/self.stress.total*100:.2f}%)"
            else:
                stress_grade = f"差 (错误率 {err_total/self.stress.total*100:.2f}%)"
            lines.append(f"  压力: {stress_grade}")
            lines.append(f"    {self.stress.total} 条指令, {self.stress.duration_s:.1f}s, "
                         f"吞吐 {self.stress.throughput:.0f}/s")
            lines.append(f"    ACK {self.stress.accepted}, NACK {self.stress.rejected}, "
                         f"超时 {self.stress.timeouts}, 错误 {self.stress.errors}")
            if self.stress.retries > 0:
                lines.append(f"    重试 {self.stress.retries} 次")

        lines.append(f"{'='*60}")
        return "\n".join(lines)


# ──────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="下位机通信测试工具")
    p.add_argument("--port", required=True, help="串口, 例如 COM3")
    p.add_argument("--baudrate", type=int, default=921600)
    p.add_argument("--timeout", type=float, default=0.05, help="串口读取超时(秒)")

    # 测试选择
    p.add_argument("--latency", action="store_true", help="运行延迟测量")
    p.add_argument("--align", action="store_true", help="运行帧对齐测试")
    p.add_argument("--stress", action="store_true", help="运行压力测试")
    p.add_argument("--all", action="store_true", help="运行全部测试")

    # 延迟参数
    p.add_argument("--count", type=int, default=100, help="延迟测量次数")
    p.add_argument("--warmup", type=int, default=5, help="预热次数")

    # 重试参数
    p.add_argument("--retry", type=int, default=3,
                   help="失败时最大重试次数 (默认 3, 0=不重试)")
    p.add_argument("--retry-delay", type=float, default=0.0,
                   help="重试间隔秒数 (默认 0)")

    # 压力参数
    p.add_argument("--motions", type=int, default=200, help="压力测试指令数")
    p.add_argument("--process-time", type=float, default=0.3,
                   help="每条运动指令的 process_time(秒)")

    # 对齐参数
    p.add_argument("--align-iter", type=int, default=50, help="帧对齐测试迭代次数")

    return p


def main(argv: list[str]) -> int:
    args = build_parser().parse_args(argv)

    if not (args.latency or args.align or args.stress or args.all):
        print("请指定测试类型: --latency / --align / --stress / --all")
        return 1

    try:
        import serial
    except ImportError:
        print("需要 pyserial: pip install pyserial")
        return 1

    retry_cfg = RetryConfig(max_retries=args.retry, delay_s=args.retry_delay)

    with serial.Serial(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
    ) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # 先发一个 QueryStatus 确认通信正常
        print(f"连接 {args.port} @ {args.baudrate} baud")
        print("检测下位机通信...")
        try:
            a0, a1 = query_status(ser, args.timeout)
            print(f"下位机响应: 角度 = {a0:.6f}, {a1:.6f} rad")
        except (TimeoutError, ValueError, RuntimeError) as e:
            print(f"下位机无响应: {e}")
            return 1

        report = CommQualityReport(
            latency=LatencyResult(0, 0, 0, 0, 0, 0, 0, 0, 0),
            alignment=AlignmentTestResult(),
            stress=StressTestResult(0, 0, 0, 0, 0, 0),
        )

        if args.latency or args.all:
            report.latency = measure_latency(
                ser, args.count, args.timeout, args.warmup,
                retry_cfg=retry_cfg,
            )
            print_latency_result(report.latency)

        if args.align or args.all:
            report.alignment = test_frame_alignment(
                ser, args.timeout, args.align_iter,
                retry_cfg=retry_cfg,
            )
            print_alignment_result(report.alignment)

        if args.stress or args.all:
            report.stress = stress_test(
                ser, args.motions, args.timeout,
                process_time=args.process_time,
                retry_cfg=retry_cfg,
            )
            print_stress_result(report.stress)

        print(report.summary())

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
