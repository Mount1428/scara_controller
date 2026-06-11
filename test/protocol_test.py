from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


class Command(IntEnum):
    None_ = 0x00
    Motion = 0x01
    EmergencyStop = 0x02
    QueryStatus = 0x03
    Ack = 0xF0
    Nack = 0xF1
    StatusResponse = 0xF2


class Reason(IntEnum):
    None_ = 0x00
    InvalidCommand = 0x01
    InvalidData = 0x02
    ExecutionFailed = 0x03


FRAME_HEAD0 = 0xAA
FRAME_HEAD1 = 0x55

HEADER_FMT = "<BBB"
ACK_FMT = "<BBB"
NACK_FMT = "<BBBB"
STATUS_FMT = "<BBBxff"
MOTION_FMT = "<BBBxfffff"
QUERY_FMT = "<BBB"
ESTOP_FMT = "<BBB"

ACK_SIZE = struct.calcsize(ACK_FMT)
NACK_SIZE = struct.calcsize(NACK_FMT)
STATUS_SIZE = struct.calcsize(STATUS_FMT)
MOTION_SIZE = struct.calcsize(MOTION_FMT)
QUERY_SIZE = struct.calcsize(QUERY_FMT)
ESTOP_SIZE = struct.calcsize(ESTOP_FMT)


@dataclass
class StatusResponse:
    angle0: float
    angle1: float


@dataclass
class MotionResult:
    accepted: bool
    arrived: Optional[bool] = None
    final_status: Optional[StatusResponse] = None
    reason: Optional[int] = None


class LiveAnglePlot:
    def __init__(self, target0: float, target1: float) -> None:
        try:
            import matplotlib.pyplot as plt  # type: ignore
        except ImportError as exc:
            raise SystemExit("matplotlib is required for live plotting: pip install matplotlib") from exc

        self._plt = plt
        self._times: list[float] = []
        self._angle0: list[float] = []
        self._angle1: list[float] = []
        self._target0 = target0
        self._target1 = target1

        self._plt.ion()
        self._fig, self._ax = self._plt.subplots(figsize=(8, 4.5))
        (self._line0,) = self._ax.plot([], [], label="joint 0")
        (self._line1,) = self._ax.plot([], [], label="joint 1")
        self._target_line0 = self._ax.axhline(target0, linestyle="--", color=self._line0.get_color(), alpha=0.6, label="target 0")
        self._target_line1 = self._ax.axhline(target1, linestyle="--", color=self._line1.get_color(), alpha=0.6, label="target 1")
        self._ax.set_xlabel("time (s)")
        self._ax.set_ylabel("angle (rad)")
        self._ax.grid(True, alpha=0.25)
        self._ax.legend(loc="best")
        manager = getattr(self._fig.canvas, "manager", None)
        if manager is not None and hasattr(manager, "set_window_title"):
            manager.set_window_title("SCARA joint angle live plot")
        self._plt.show(block=False)

    def add_sample(self, elapsed_s: float, status: StatusResponse) -> None:
        self._times.append(elapsed_s)
        self._angle0.append(status.angle0)
        self._angle1.append(status.angle1)

        self._line0.set_data(self._times, self._angle0)
        self._line1.set_data(self._times, self._angle1)
        self._ax.relim()
        self._ax.autoscale_view()
        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()
        self._plt.pause(0.001)

    def finish(self) -> None:
        self._plt.show(block=True)


def pack_ack() -> bytes:
    return struct.pack(ACK_FMT, FRAME_HEAD0, FRAME_HEAD1, Command.Ack)


def pack_nack(reason: Reason) -> bytes:
    return struct.pack(NACK_FMT, FRAME_HEAD0, FRAME_HEAD1, Command.Nack, int(reason))


def pack_query_status() -> bytes:
    return struct.pack(QUERY_FMT, FRAME_HEAD0, FRAME_HEAD1, Command.QueryStatus)


def pack_emergency_stop() -> bytes:
    return struct.pack(ESTOP_FMT, FRAME_HEAD0, FRAME_HEAD1, Command.EmergencyStop)


def pack_motion(abs_angle0: float, abs_angle1: float, target_speed0: float, target_speed1: float, process_time: float) -> bytes:
    return struct.pack(
        MOTION_FMT,
        FRAME_HEAD0,
        FRAME_HEAD1,
        Command.Motion,
        abs_angle0,
        abs_angle1,
        target_speed0,
        target_speed1,
        process_time,
    )


def unpack_header(data: bytes) -> tuple[int, int, int]:
    if len(data) != 3:
        raise ValueError(f"header size must be 3, got {len(data)}")
    return struct.unpack(HEADER_FMT, data)


def unpack_status_response(data: bytes) -> StatusResponse:
    if len(data) != STATUS_SIZE:
        raise ValueError(f"status frame size must be {STATUS_SIZE}, got {len(data)}")
    head0, head1, cmd, angle0, angle1 = struct.unpack(STATUS_FMT, data)
    if (head0, head1) != (FRAME_HEAD0, FRAME_HEAD1) or cmd != int(Command.StatusResponse):
        raise ValueError("invalid status response header")
    return StatusResponse(angle0, angle1)


def _angle_error(target: float, actual: float) -> float:
    return abs(target - actual)


def _status_matches_target(status: StatusResponse, target0: float, target1: float, tolerance_rad: float) -> bool:
    return _angle_error(target0, status.angle0) <= tolerance_rad and _angle_error(target1, status.angle1) <= tolerance_rad


def query_status(ser, timeout_s: float) -> StatusResponse:
    ser.write(pack_query_status())
    response = _read_frame(ser, timeout_s)
    return unpack_status_response(response)


def query_status_until(
    ser,
    total_timeout_s: float,
    read_timeout_s: float,
    poll_interval_s: float,
) -> StatusResponse:
    deadline = time.monotonic() + total_timeout_s
    last_error: Optional[Exception] = None

    while time.monotonic() < deadline:
        try:
            return query_status(ser, read_timeout_s)
        except TimeoutError as exc:
            last_error = exc
            time.sleep(max(0.0, poll_interval_s))

    if last_error is None:
        raise TimeoutError("status query timed out")
    raise TimeoutError(f"status query timed out after {total_timeout_s:.1f}s") from last_error


def segment_arrival_timeout(process_time_s: float, settle_delay_s: float, extra_time_s: float) -> float:
    return max(0.0, process_time_s) + max(0.0, settle_delay_s) + max(0.0, extra_time_s)


def wait_for_arrival(
    ser,
    timeout_s: float,
    target0: float,
    target1: float,
    tolerance_rad: float,
    settle_delay_s: float,
    poll_interval_s: float,
    plotter: Optional[LiveAnglePlot] = None,
) -> tuple[bool, StatusResponse]:
    deadline = time.monotonic() + timeout_s
    start_time = time.monotonic()
    last_status = StatusResponse(float("nan"), float("nan"))
    time.sleep(max(0.0, settle_delay_s))

    while time.monotonic() < deadline:
        try:
            last_status = query_status(ser, timeout_s)
        except TimeoutError:
            time.sleep(max(0.0, poll_interval_s))
            continue

        if plotter is not None:
            plotter.add_sample(time.monotonic() - start_time, last_status)
        if _status_matches_target(last_status, target0, target1, tolerance_rad):
            return True, last_status

        time.sleep(max(0.0, poll_interval_s))

    return False, last_status


def self_test() -> None:
    assert ACK_SIZE == 3, ACK_SIZE
    assert NACK_SIZE == 4, NACK_SIZE
    assert QUERY_SIZE == 3, QUERY_SIZE
    assert ESTOP_SIZE == 3, ESTOP_SIZE
    assert STATUS_SIZE == 12, STATUS_SIZE
    assert MOTION_SIZE == 24, MOTION_SIZE

    ack = pack_ack()
    nack = pack_nack(Reason.InvalidData)
    query = pack_query_status()
    estop = pack_emergency_stop()
    motion = pack_motion(1.25, -0.5, 0.1, -0.2, 0.75)

    assert ack == b"\xAA\x55\xF0"
    assert nack == b"\xAA\x55\xF1\x02"
    assert query == b"\xAA\x55\x03"
    assert estop == b"\xAA\x55\x02"

    head0, head1, cmd = unpack_header(query)
    assert (head0, head1, cmd) == (FRAME_HEAD0, FRAME_HEAD1, int(Command.QueryStatus))

    motion_head = struct.unpack("<BBB", motion[:3])
    assert motion_head == (FRAME_HEAD0, FRAME_HEAD1, int(Command.Motion))

    print("protocol self-test passed")
    print(f"ACK={ACK_SIZE} NACK={NACK_SIZE} STATUS={STATUS_SIZE} MOTION={MOTION_SIZE}")


def _read_exact(ser, size: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    buffer = bytearray()
    while len(buffer) < size:
        chunk = ser.read(size - len(buffer))
        if chunk:
            buffer.extend(chunk)
            continue
        if time.monotonic() >= deadline:
            break
    if len(buffer) != size:
        raise TimeoutError(f"expected {size} bytes, got {len(buffer)}")
    return bytes(buffer)


def _read_frame(ser, timeout_s: float) -> bytes:
    header = _read_exact(ser, 3, timeout_s)
    head0, head1, cmd = unpack_header(header)
    if (head0, head1) != (FRAME_HEAD0, FRAME_HEAD1):
        raise ValueError(f"bad frame head: {header!r}")

    if cmd == int(Command.Ack):
        return header
    if cmd == int(Command.Nack):
        return header + _read_exact(ser, 1, timeout_s)
    if cmd == int(Command.StatusResponse):
        return header + _read_exact(ser, STATUS_SIZE - 3, timeout_s)
    raise ValueError(f"unexpected response command 0x{cmd:02X}")


def serial_test(
    port: str,
    baudrate: int,
    timeout_s: float,
    motion: Optional[tuple[float, float, float, float, float]],
    verify_arrival: bool,
    arrival_tolerance_rad: float,
    settle_delay_s: float,
    poll_interval_s: float,
    live_plot: bool,
    segment_extra_time: float,
    startup_timeout_s: float,
) -> None:
    verify_arrival = verify_arrival or live_plot

    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyserial is required for serial test: pip install pyserial") from exc

    with serial.Serial(port=port, baudrate=baudrate, timeout=timeout_s) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        status = query_status_until(ser, startup_timeout_s, timeout_s, poll_interval_s)
        print(f"status: angle0={status.angle0:.6f} rad, angle1={status.angle1:.6f} rad")

        if motion is not None:
            ser.write(pack_motion(*motion))
            response = _read_frame(ser, timeout_s)
            _, _, cmd = unpack_header(response[:3])
            if cmd == int(Command.Ack):
                print("motion command accepted: ACK")
                plotter = LiveAnglePlot(motion[0], motion[1]) if live_plot else None
                if plotter is not None:
                    plotter.add_sample(0.0, status)
                if verify_arrival:
                    process_time = float(motion[4])
                    arrival_timeout = segment_arrival_timeout(process_time, settle_delay_s, segment_extra_time)
                    arrived, final_status = wait_for_arrival(
                        ser,
                        arrival_timeout,
                        motion[0],
                        motion[1],
                        arrival_tolerance_rad,
                        settle_delay_s,
                        poll_interval_s,
                        plotter=plotter,
                    )
                    print(
                        f"arrival: {arrived} final={final_status.angle0:.6f},{final_status.angle1:.6f} "
                        f"tolerance={arrival_tolerance_rad:.6f} rad"
                    )
                if plotter is not None:
                    plotter.finish()
            elif cmd == int(Command.Nack):
                reason = response[3]
                print(f"motion command rejected: NACK reason=0x{reason:02X}")
            else:
                raise ValueError(f"unexpected motion response: {response!r}")


def serial_batch_test(
    port: str,
    baudrate: int,
    timeout_s: float,
    motions: list[tuple[float, float, float, float, float]],
    verify_arrival: bool,
    arrival_tolerance_rad: float,
    settle_delay_s: float,
    poll_interval_s: float,
    live_plot: bool,
    segment_extra_time: float,
    startup_timeout_s: float,
) -> None:
    verify_arrival = verify_arrival or live_plot

    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyserial is required for serial test: pip install pyserial") from exc

    with serial.Serial(port=port, baudrate=baudrate, timeout=timeout_s) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        status = query_status_until(ser, startup_timeout_s, timeout_s, poll_interval_s)
        print(f"status: angle0={status.angle0:.6f} rad, angle1={status.angle1:.6f} rad")

        passed = 0
        arrived = 0
        for index, motion in enumerate(motions, start=1):
            ser.write(pack_motion(*motion))
            response = _read_frame(ser, timeout_s)
            _, _, cmd = unpack_header(response[:3])
            if cmd == int(Command.Ack):
                passed += 1
                print(f"[{index}] motion accepted: ACK abs_target={motion[0]:.6f},{motion[1]:.6f}")
                if verify_arrival:
                    plotter = LiveAnglePlot(motion[0], motion[1]) if live_plot else None
                    if plotter is not None:
                        plotter.add_sample(0.0, status)
                    process_time = float(motion[4])
                    arrival_timeout = segment_arrival_timeout(process_time, settle_delay_s, segment_extra_time)
                    ok, final_status = wait_for_arrival(
                        ser,
                        arrival_timeout,
                        motion[0],
                        motion[1],
                        arrival_tolerance_rad,
                        settle_delay_s,
                        poll_interval_s,
                        plotter=plotter,
                    )
                    if ok:
                        arrived += 1
                        print(
                            f"[{index}] arrived: status={final_status.angle0:.6f},{final_status.angle1:.6f} "
                            f"tolerance={arrival_tolerance_rad:.6f} rad"
                        )
                    else:
                        print(
                            f"[{index}] not arrived: final_status={final_status.angle0:.6f},{final_status.angle1:.6f} "
                            f"abs_target={motion[0]:.6f},{motion[1]:.6f} tolerance={arrival_tolerance_rad:.6f} rad"
                        )
                    if plotter is not None:
                        plotter.finish()
            elif cmd == int(Command.Nack):
                reason = response[3]
                print(f"[{index}] motion rejected: NACK reason=0x{reason:02X} abs_target={motion[0]:.6f},{motion[1]:.6f}")
            else:
                raise ValueError(f"unexpected motion response: {response!r}")

        if verify_arrival:
            print(f"batch summary: {passed}/{len(motions)} accepted, {arrived}/{passed} arrived")
        else:
            print(f"batch summary: {passed}/{len(motions)} accepted")


def parse_motion(values: list[str]) -> tuple[float, float, float, float, float]:
    if len(values) != 5:
        raise argparse.ArgumentTypeError("motion requires 5 values: absolute_angle0 absolute_angle1 speed0 speed1 process_time")
    return tuple(float(v) for v in values)  # type: ignore[return-value]


def parse_batch_file(path: str) -> list[tuple[float, float, float, float, float]]:
    motions: list[tuple[float, float, float, float, float]] = []
    with open(path, "r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.reader(handle)
        for line_no, row in enumerate(reader, start=1):
            if not row:
                continue
            if row[0].lstrip().startswith("#"):
                continue
            lower_row = [value.strip().lower() for value in row]
            if lower_row[:2] == ["absolute_angle0", "absolute_angle1"]:
                continue
            if len(row) != 5:
                raise argparse.ArgumentTypeError(f"{path}:{line_no}: expected 5 columns, got {len(row)}")
            motions.append(tuple(float(value) for value in row))  # type: ignore[arg-type]
    if not motions:
        raise argparse.ArgumentTypeError(f"{path}: no motion rows found")
    return motions


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SCARA controller protocol test")
    parser.add_argument("--self-test", action="store_true", help="run local protocol packing checks")
    parser.add_argument("--port", help="serial port, for example COM3 or /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=921600, help="serial baudrate")
    parser.add_argument("--timeout", type=float, default=10.0, help="serial read timeout in seconds")
    parser.add_argument(
        "--motion",
        nargs=5,
        metavar=("ABS_ANGLE0", "ABS_ANGLE1", "SPEED0", "SPEED1", "PROCESS_TIME"),
        help="send a motion command using absolute joint angles (rad) after status query",
    )
    parser.add_argument(
        "--batch-file",
        help="CSV rows: absolute_angle0,absolute_angle1,speed0,speed1,process_time for batch regression",
    )
    parser.add_argument(
        "--verify-arrival",
        action="store_true",
        help="poll status after each accepted motion and verify the target position is reached",
    )
    parser.add_argument(
        "--arrival-tolerance-rad",
        type=float,
        default=0.01,
        help="position tolerance in radians for arrival verification",
    )
    parser.add_argument(
        "--settle-delay",
        type=float,
        default=0.2,
        help="delay before starting arrival polling, in seconds",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.1,
        help="poll interval for arrival verification, in seconds",
    )
    parser.add_argument(
        "--live-plot",
        action="store_true",
        help="show a matplotlib window that updates with status samples in real time",
    )
    parser.add_argument(
        "--segment-extra-time",
        type=float,
        default=5.0,
        help="extra seconds to wait after the segment's expected process_time when verifying arrival",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=30.0,
        help="maximum total time to keep retrying the initial status query",
    )
    return parser


def main(argv: list[str]) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if args.self_test or not args.port:
        self_test()
        if not args.port:
            return 0

    if args.batch_file and args.motion:
        raise SystemExit("--batch-file and --motion cannot be used together")

    if args.batch_file:
        motions = parse_batch_file(args.batch_file)
        serial_batch_test(
            args.port,
            args.baudrate,
            args.timeout,
            motions,
            args.verify_arrival,
            args.arrival_tolerance_rad,
            args.settle_delay,
            args.poll_interval,
            args.live_plot,
            args.segment_extra_time,
            args.startup_timeout,
        )
        return 0

    motion = parse_motion(args.motion) if args.motion else None
    serial_test(
        args.port,
        args.baudrate,
        args.timeout,
        motion,
        args.verify_arrival,
        args.arrival_tolerance_rad,
        args.settle_delay,
        args.poll_interval,
        args.live_plot,
        args.segment_extra_time,
        args.startup_timeout,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))