#!/usr/bin/env python3
"""独立 STM32 GOTO / 连续速度 / TRACK 与 XY mapping 跟焦测试。"""
from __future__ import annotations

import argparse
import bisect
import binascii
import csv
import json
import math
from pathlib import Path
import queue
import struct
import threading
import time
from continuous_control import (DEFAULTS, apply_defaults, mode_name, validate_control,
                                MotionProfile, run_continuous)
from mapping_run import (MAPPING_DEFAULTS, mapping_defaults, validate_mapping,
                         mapping_line, run_mapping, analyze_mapping)
from xy_mapping import MappingCurve
from xy_stage import XYStage, StageReader, default_dll_path

PING, PONG, READY, HELLO = 0x01, 0x02, 0x03, 0x04
PARAM_READ, PARAM_VALUE = 0x11, 0x12
STOP, GOTO = 0x21, 0x22
VELOCITY, TRACK = 0x27, 0x29
STATUS_QUERY, STATUS_RESP = 0x40, 0x41
STREAM_START, STREAM_STOP, STREAM_DATA, EVENT = 0x50, 0x51, 0x52, 0x70
RESPONSE, ERROR = 1, 2


def encode_frame(cmd, payload=b"", flags=0):
    raw = bytes((cmd, flags, len(payload))) + payload
    raw += struct.pack("<H", binascii.crc_hqx(raw, 0xFFFF))
    out = bytearray([0])
    code_at, code = 0, 1
    for byte in raw:
        if byte == 0:
            out[code_at] = code
            code_at, code = len(out), 1
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 255:
                out[code_at] = code
                code_at, code = len(out), 1
                out.append(0)
    out[code_at] = code
    return b"\0" + bytes(out) + b"\0"


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0
        self.discard = False

    def feed(self, data):
        frames = []
        for byte in data:
            if byte:
                if not self.discard:
                    self.buffer.append(byte)
                    if len(self.buffer) > 264:
                        self.buffer.clear()
                        self.discard = True
                        self.bad_frames += 1
                continue
            if self.discard:
                self.discard = False
                continue
            if not self.buffer:
                continue
            encoded = bytes(self.buffer)
            self.buffer.clear()
            raw, i = bytearray(), 0
            try:
                while i < len(encoded):
                    code = encoded[i]
                    i += 1
                    if not code or i + code - 1 > len(encoded):
                        raise ValueError("COBS")
                    raw.extend(encoded[i:i + code - 1])
                    i += code - 1
                    if code < 255 and i < len(encoded):
                        raw.append(0)
                if len(raw) < 5 or len(raw) != 5 + raw[2]:
                    raise ValueError("length")
                if binascii.crc_hqx(raw[:-2], 0xFFFF) != struct.unpack("<H", raw[-2:])[0]:
                    raise ValueError("CRC")
                frames.append((raw[0], raw[1], bytes(raw[3:-2])))
            except ValueError:
                self.bad_frames += 1
        return frames


class MotorLink:
    """单线程拥有串口；GOTO 异步确认，位置由 MCU 遥测更新。"""
    def __init__(self, port, baud=921600, sample_ms=5):
        import serial
        self.ser = serial.Serial(port=None, baudrate=baud, timeout=0,
                                 write_timeout=0.1, dsrdtr=False, rtscts=False)
        self.ser.port = port
        self.ser.dtr = self.ser.rts = False
        self.decoder = Decoder()
        self.session = self.token = 0
        self.pending = {}
        self.unconfirmed = {}
        self.replies = {}
        self.samples = []
        self.events = []
        self.latest = None
        self.last_seq = self.last_ms = None
        self.mcu_ms = 0
        self.lost_samples = 0
        self.heartbeat_at = time.perf_counter()
        self.sample_ms = sample_ms
        self.info = {}
        self.max_velocity = None
        self.ser.open()
        try:
            time.sleep(0.4)  # 与 pc_app 相同，等待可能发生的上电复位。
            self.ser.reset_input_buffer()
            self._start_session()
        except BaseException:
            self.close()
            raise

    def _start_session(self):
        """建立一次完整的协议会话；可用于设备运行中复位后的恢复。"""
        self.session = 0
        self.token = 0
        self.pending.clear()
        self.unconfirmed.clear()
        self.replies.clear()
        self.latest = None
        self.last_seq = self.last_ms = None
        self.mcu_ms = 0
        self.heartbeat_at = time.perf_counter()
        self.info = self.request(HELLO, struct.pack("<HI", 3, 7))
        parameter = self.request(PARAM_READ, b"\x04")
        if len(parameter) != 7 or parameter[2] != 4:
            raise RuntimeError("最大速度参数响应无效")
        self.max_velocity = struct.unpack_from("<f", parameter, 3)[0]
        if not math.isfinite(self.max_velocity) or self.max_velocity <= 0:
            raise RuntimeError("固件返回的最大速度无效")
        hz = round(1000 / self.sample_ms)
        if not 1 <= hz <= min(200, self.info["max_stream_hz"]):
            raise ValueError("遥测周期超出当前固件支持范围")
        self.request(STREAM_START, struct.pack("<HB", hz, 1))
        self.until(lambda: self.latest is not None, 1.0)

    def _recover_session(self):
        """STM32 已复位时重新建立会话，不复用旧命令的 ACK/token。"""
        self.events.append(dict(host_abs_s=time.perf_counter(), event="SESSION_RESET", detail=0))
        self.decoder = Decoder()
        self.ser.reset_input_buffer()
        time.sleep(0.15)
        self._start_session()

    def _handle_ready(self):
        if self.session:
            self._recover_session()

    def _poll_frames(self):
        """读取并处理一批帧；返回是否发生会话复位。"""
        reset = False
        for cmd, flags, payload in self.decoder.feed(self.ser.read(min(self.ser.in_waiting, 8192))):
            now = time.perf_counter()
            if cmd == READY:
                if self.session:
                    reset = True
                    break
                continue
            if cmd == HELLO and flags & RESPONSE:
                key = (HELLO, payload[1]) if len(payload) >= 2 else None
                if key not in self.pending:
                    continue
                if flags & ERROR or len(payload) != 19:
                    raise RuntimeError("协议 v3 握手被拒绝；请核对现有上位机与设备")
                proto, fw, caps, max_hz, uptime = struct.unpack_from("<HIIHI", payload, 2)
                if proto != 3 or not payload[0] or caps & 7 != 7:
                    raise RuntimeError("设备未提供 pc_app 所需的协议 v3 遥测/事件/会话能力")
                self.session = payload[0]
                self.replies[key] = dict(proto=proto, fw_ver=fw, caps=caps,
                                         max_stream_hz=max_hz, uptime_ms=uptime)
                self.pending.pop(key)
                continue
            if not payload or payload[0] != self.session or not self.session:
                continue
            if cmd == STREAM_DATA:
                if len(payload) != 25:
                    raise RuntimeError("遥测长度与 pc_app 协议不符")
                seq, ms = struct.unpack_from("<II", payload, 1)
                if self.last_seq is not None:
                    ds = (seq - self.last_seq) & 0xFFFFFFFF
                    if ds == 0 or ds >= 0x80000000:
                        continue
                    self.lost_samples += ds - 1
                    dm = (ms - self.last_ms) & 0xFFFFFFFF
                    if dm >= 0x80000000:
                        raise RuntimeError("STM32 遥测时间倒退")
                    self.mcu_ms += dm
                else:
                    self.mcu_ms = ms
                self.last_seq, self.last_ms = seq, ms
                pos = struct.unpack_from("<f", payload, 17)[0]
                state, error, limits, overflow = payload[21:25]
                if not math.isfinite(pos):
                    raise RuntimeError("位置遥测不是有限数")
                self.latest = dict(host_abs_s=now, mcu_s=self.mcu_ms / 1000,
                                   position_um=pos, seq=seq, state=state,
                                   error=error, flags=limits, overflow=overflow)
                self.samples.append(self.latest.copy())
                if state == 6 or error or limits & 7:
                    raise RuntimeError(f"马达错误/限位/报警：state={state}, error={error}, flags={limits}")
                continue
            if cmd == EVENT:
                if len(payload) != 4:
                    raise RuntimeError("事件长度无效")
                event, detail = payload[2:4]
                self.events.append(dict(host_abs_s=now, event=event, detail=detail))
                if event in (1, 3, 4, 5, 6, 7, 8):
                    raise RuntimeError(f"固件事件：event={event}, detail={detail}")
                continue
            if flags & RESPONSE and len(payload) >= 2:
                request_cmd = {PONG: PING, PARAM_VALUE: PARAM_READ,
                               STATUS_RESP: STATUS_QUERY}.get(cmd, cmd)
                key = (request_cmd, payload[1])
                pending = self.pending.pop(key, None)
                if pending is None:
                    pending = getattr(self, "unconfirmed", {}).pop(key, None)
                if pending is None:
                    continue
                if flags & ERROR:
                    raise RuntimeError(f"命令被拒绝：0x{request_cmd:02x}, token={payload[1]}")
                if pending[1] is not None:
                    pending[1]["ack_abs_s"] = now
                if request_cmd in (PARAM_READ, STREAM_START, STREAM_STOP):
                    self.replies[key] = payload
        return reset

    def send(self, cmd, body=b"", record=None, expect_ack=True):
        if not hasattr(self, "unconfirmed"):
            self.unconfirmed = {}
        self.token = self.token % 255 + 1
        key = (cmd, self.token)
        if key in self.pending:
            raise RuntimeError("待确认命令令牌冲突")
        payload = bytes((self.session, self.token)) + body
        frame = encode_frame(cmd, payload)
        sent = time.perf_counter()
        if record is not None:
            record.update(token=self.token, sent_abs_s=sent, write_end_abs_s=sent,
                          ack_abs_s=None, write_ok=False, ack_required=expect_ack)
        try:
            if self.ser.write(frame) != len(frame):
                raise RuntimeError("串口写入不完整")
        finally:
            written = time.perf_counter()
            if record is not None:
                record["write_end_abs_s"] = written
        if expect_ack:
            self.pending[key] = (written, record)
        else:
            self.unconfirmed[key] = (written, record)
        if record is not None:
            record["write_ok"] = True
        return key

    def until(self, predicate, timeout):
        deadline = time.perf_counter() + timeout
        while not predicate():
            self.poll()
            if time.perf_counter() >= deadline:
                raise TimeoutError("握手、响应或遥测等待超时")
            time.sleep(0.001)

    def request(self, cmd, body=b""):
        key = self.send(cmd, body)
        self.until(lambda: key in self.replies, 2.5)
        return self.replies.pop(key)

    def poll(self):
        if self._poll_frames():
            self._recover_session()
            return
        now = time.perf_counter()
        if self.session and now - self.heartbeat_at >= 0.25:
            self.send(PING)
            self.heartbeat_at = now
        if any(now - sent > 0.5 for sent, _ in self.pending.values()):
            raise TimeoutError("命令 ACK/心跳超时")
        # VELOCITY 成功及 TRACK UPDATE 不回 ACK；仍关联错误响应。
        if hasattr(self, "unconfirmed"):
            self.unconfirmed = {k: v for k, v in self.unconfirmed.items() if now - v[0] <= 0.5}

    def goto(self, position, record=None):
        return self.send(GOTO, struct.pack("<f", position), record)

    def velocity(self, velocity, record=None):
        return self.send(VELOCITY, struct.pack("<f", velocity), record, expect_ack=False)

    def track(self, position, velocity, sequence, start=False, record=None):
        if not self.info.get("caps", 0) & 8:
            raise RuntimeError("固件不支持 TRACK")
        return self.send(TRACK, struct.pack("<BBHff", 1 if start else 2, 0,
                                           sequence, position, velocity), record, expect_ack=start)

    def read_motion_limits(self):
        values = []
        for parameter in (8, 9):
            payload = self.request(PARAM_READ, bytes((parameter,)))
            if len(payload) != 7 or payload[2] != parameter:
                raise RuntimeError("加减速度参数响应无效")
            value = struct.unpack_from("<f", payload, 3)[0]
            if not math.isfinite(value) or value <= 0:
                raise RuntimeError("设备加减速度无效")
            values.append(value)
        return tuple(values)

    def stop(self):
        if self.session and self.ser.is_open:
            return self.send(STOP)
        return None

    def close(self):
        if self.ser.is_open:
            try:
                self.stop()
                if self.session:
                    self.send(STREAM_STOP)
            finally:
                self.ser.close()


def make_plan(start, step, interval_ms, steps, minimum, maximum):
    values = (start, step, interval_ms, minimum, maximum)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("参数必须为有限数")
    if step == 0 or not 5 <= interval_ms <= 10000 or not 1 <= steps <= 10000:
        raise ValueError("步长不能为0；间隔5~10000 ms；步数1~10000")
    if minimum >= maximum or not minimum <= start <= maximum:
        raise ValueError("起点或软限位无效")
    end = start + step * steps
    if not minimum <= end <= maximum:
        raise ValueError(f"终点 {end:.3f} μm 超出软限位 [{minimum}, {maximum}]")
    dt = interval_ms / 1000
    return [dict(step=i, scheduled_send_s=(i - 1) * dt, deadline_s=i * dt,
                 target_um=start + i * step) for i in range(1, steps + 1)]


def run_test(link, cfg, cancel, update=lambda message: None):
    if mode_name(cfg) != "GOTO":
        return run_continuous(link, cfg, cancel, update)
    if link.latest is None or time.perf_counter() - link.latest["host_abs_s"] > 0.2:
        raise RuntimeError("没有新鲜的位置遥测")
    start = link.latest["position_um"]
    plan = make_plan(start, cfg.step_um, cfg.interval_ms, cfg.steps,
                     cfg.min_um, cfg.max_um)
    t0 = time.perf_counter()
    # 保留起点前的样本，以便围住 t=0 做插值。
    sample_begin = max(0, len(link.samples) - 20)
    initial_bad, initial_lost = link.decoder.bad_frames, link.lost_samples
    commands = []
    result = dict(config=vars(cfg).copy(), start_um=start, origin_abs_s=t0,
                  planned=plan, commands=commands, status="running",
                  firmware=link.info, max_velocity_um_s=link.max_velocity)
    next_index = 0
    display_at = 0.0
    end_s = plan[-1]["deadline_s"] + cfg.tail_ms / 1000
    try:
        while time.perf_counter() - t0 < end_s:
            if cancel.is_set():
                result["status"] = "cancelled"
                break
            link.poll()
            now = time.perf_counter()
            if now - link.latest["host_abs_s"] > max(0.2, 3 * link.sample_ms / 1000):
                raise TimeoutError("位置遥测中断")
            if next_index < len(plan) and now - t0 >= plan[next_index]["scheduled_send_s"]:
                late = now - t0 - plan[next_index]["scheduled_send_s"]
                if late >= cfg.interval_ms / 1000:
                    raise RuntimeError("发送晚了一整个周期，已终止以避免补发突发指令")
                rec = plan[next_index].copy()
                commands.append(rec)
                link.goto(rec["target_um"], rec)
                next_index += 1
            if now - display_at >= 0.1:
                update(f"位置 {link.latest['position_um']:.3f} μm；指令 {next_index}/{len(plan)}")
                display_at = now
            cancel.wait(0.001)
        else:
            result["status"] = "complete"
        # 最后一帧之后仍检查所有 GOTO 的 ACK，不把未知确认标为成功。
        if result["status"] == "complete" and any(c["ack_abs_s"] is None for c in commands):
            raise TimeoutError("测试末尾存在未确认的 GOTO")
    except Exception as exc:
        result["status"], result["error"] = "failed", str(exc)
    finally:
        try:
            key = link.stop()
            if key is not None:
                link.until(lambda: key not in link.pending, 0.6)
        except Exception as exc:
            result["stop_error"] = str(exc)
            result["status"] = "failed"
        result["samples"] = link.samples[sample_begin:]
        result["events"] = [e for e in link.events if e["host_abs_s"] >= t0]
        result["bad_frames"] = link.decoder.bad_frames - initial_bad
        result["lost_samples"] = link.lost_samples - initial_lost
    return result


def interpolate(times, positions, at, max_gap):
    i = bisect.bisect_left(times, at)
    if i < len(times) and abs(times[i] - at) < 1e-9:
        return positions[i]
    if i == 0 or i == len(times) or times[i] - times[i - 1] > max_gap:
        return None
    f = (at - times[i - 1]) / (times[i] - times[i - 1])
    return positions[i - 1] + f * (positions[i] - positions[i - 1])


def analyze(result):
    samples = result["samples"]
    cfg, origin = result["config"], result["origin_abs_s"]
    if not samples:
        return dict(steps=[], samples=[], average_speed_um_s=None,
                    valid_step_count=0, measurement_valid=False)
    # 最小收包偏移估计两个时钟的平移；包含未知最小传输时延，非硬同步。
    offset = min(s["host_abs_s"] - s["mcu_s"] for s in samples)
    unique = []
    for s in samples:
        row = s.copy()
        row["time_s"] = s["mcu_s"] + offset - origin
        row["receive_time_s"] = s["host_abs_s"] - origin
        if not unique or row["mcu_s"] > unique[-1]["mcu_s"]:
            unique.append(row)
    ts, zs = [s["time_s"] for s in unique], [s["position_um"] for s in unique]
    gap = 2.5 * cfg["sample_ms"] / 1000
    for i, s in enumerate(unique):
        s["velocity_um_s"] = None
        if i and 0 < ts[i] - ts[i - 1] <= gap:
            s["velocity_um_s"] = (zs[i] - zs[i - 1]) / (ts[i] - ts[i - 1])
    if result.get("reference_source") == "XY_MAPPING":
        return analyze_mapping(result, unique, interpolate, gap)
    z0 = interpolate(ts, zs, 0, gap)
    duration = result.get("reference_duration_s", cfg["steps"] * cfg["interval_ms"] / 1000)
    zend = interpolate(ts, zs, duration, gap)
    average = abs(zend - z0) / duration if z0 is not None and zend is not None else None
    if average is not None and average < 1e-6:
        average = None  # 静止、近零平均速度不产生无穷大或伪零延时。
    direction = 1 if cfg["step_um"] > 0 else -1
    rows = []
    for command in result.get("evaluation_plan", result["commands"]):
        row = command.copy()
        deadline = row["deadline_s"]
        actual = interpolate(ts, zs, deadline, gap)
        error = row["target_um"] - actual if actual is not None else None
        row.update(actual_um=actual, error_um=error,
                   delay_ms=1000 * direction * error / average
                   if error is not None and average is not None else None,
                   absolute_delay_ms=1000 * abs(error) / average
                   if error is not None and average is not None else None,
                   send_time_s=row["sent_abs_s"] - origin if row.get("sent_abs_s") is not None else None,
                   send_lateness_ms=1000 * (row["sent_abs_s"] - origin - row["scheduled_send_s"])
                   if row.get("sent_abs_s") is not None else None,
                   write_duration_ms=1000 * (row["write_end_abs_s"] - row["sent_abs_s"])
                   if row.get("sent_abs_s") is not None else None,
                   ack_time_s=row["ack_abs_s"] - origin if row.get("ack_abs_s") is not None else None)
        rows.append(row)
    if result.get("profile"):
        profile = MotionProfile(**result["profile"])
        for s in unique:
            s["reference_um"], s["reference_velocity_um_s"] = profile.at(s["time_s"])
            s["following_error_um"] = s["reference_um"] - s["position_um"]
    errors = [r["error_um"] for r in rows if r["error_um"] is not None]
    plateau = [s for s in unique if s.get("reference_velocity_um_s") is not None
               and abs(abs(s["reference_velocity_um_s"]) - profile.peak) < 1e-6
               and s["velocity_um_s"] is not None] if result.get("profile") else []
    return dict(samples=unique, steps=rows, average_speed_um_s=average,
                z_at_start_um=z0, z_at_end_um=zend, duration_s=duration,
                clock_offset_s=offset,
                rms_error_um=math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else None,
                max_abs_error_um=max(map(abs, errors)) if errors else None,
                plateau_velocity_ripple_rms_um_s=math.sqrt(sum(
                    (s["velocity_um_s"] - s["reference_velocity_um_s"]) ** 2 for s in plateau) /
                    len(plateau)) if plateau else None,
                valid_step_count=sum(r["actual_um"] is not None for r in rows),
                measurement_valid=(result["status"] in ("complete", "DEMO_SYNTHETIC")
                                   and len(rows) == cfg["steps"]
                                   and all(r["actual_um"] is not None for r in rows)
                                   and average is not None))


def draw(analysis, title, figure=None):
    from matplotlib.figure import Figure
    fig = figure or Figure(figsize=(12, 4.5), layout="constrained")
    fig.clear()
    ax1, ax2, ax3 = fig.subplots(1, 3)
    steps, samples = analysis["steps"], analysis["samples"]
    ax1.plot([r["target_um"] for r in steps],
             [r["actual_um"] if r["actual_um"] is not None else math.nan for r in steps], "o-", ms=3)
    if steps:
        ends = [min(r["target_um"] for r in steps), max(r["target_um"] for r in steps)]
        ax1.plot(ends, ends, "--", color="gray", label="y = x")
        ax1.legend()
    ax1.set(xlabel="Set position (um)", ylabel="Reported position (um)",
            title="At XY sample time" if "xy_position_source" in analysis else "At each step deadline")
    ax2.plot([s["time_s"] for s in samples],
             [s["velocity_um_s"] if s["velocity_um_s"] is not None else math.nan for s in samples],
             label="Reported velocity")
    if samples and "reference_velocity_um_s" in samples[0]:
        ax2.plot([s["time_s"] for s in samples], [s["reference_velocity_um_s"] if s["reference_velocity_um_s"] is not None
                                               else math.nan for s in samples],
                 "--", label="Reference")
        ax2.legend()
    ax2.set(xlabel="Time (s)", ylabel="Velocity (um/s)", title="Position difference / MCU dt")
    ax3.plot([r["deadline_s"] for r in steps],
             [r["delay_ms"] if r["delay_ms"] is not None else math.nan for r in steps], "o-", ms=3)
    ax3.axhline(0, color="gray", linestyle="--")
    ax3.set(xlabel="Time (s)", ylabel="Equivalent lag (ms)", title="Positive = behind; negative = ahead")
    for ax in (ax1, ax2, ax3):
        ax.grid(alpha=0.25)
    fig.suptitle(title)
    return fig


def save_result(result, out_dir):
    analysis = analyze(result)
    output = Path(out_dir) / (time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1000000:06d}")
    output.mkdir(parents=True, exist_ok=False)
    def write_csv(name, rows):
        if rows:
            with (output / name).open("w", encoding="utf-8-sig", newline="") as f:
                keys = list(dict.fromkeys(key for row in rows for key in row))
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                writer.writerows(rows)
    write_csv("telemetry.csv", analysis["samples"])
    write_csv("steps.csv", analysis["steps"])
    write_csv("events.csv", result["events"])
    if result.get("mode", "GOTO") != "GOTO" or result.get("reference_source") == "XY_MAPPING":
        write_csv("control.csv", result["commands"])
    write_csv("xy_mapping.csv", result.get("xy_mapping", []))
    summary = {k: v for k, v in result.items() if k not in
               ("samples", "commands", "planned", "events", "evaluation_plan", "xy_mapping")}
    summary.update({k: v for k, v in analysis.items() if k not in ("samples", "steps")})
    summary["nominal_speed_um_s"] = (None if result.get("reference_source") == "XY_MAPPING" else
                                    abs(result["config"]["step_um"]) * 1000 / result["config"]["interval_ms"])
    summary["position_source"] = "STM32 STREAM_DATA position_um; sensor provenance not established by pc_app"
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    # 原始数据也保留，失败/停止后的试验可以重新计算。
    (output / "raw.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    fig = draw(analysis, result_title(result))
    fig.savefig(output / "curves.png", dpi=160)
    fig.savefig(output / "curves.pdf")
    return output, analysis


def demo_result(cfg):
    """明确标记的合成数据，仅验证界面、分析与导出；不会连接设备。"""
    apply_defaults(cfg)
    mapping_defaults(cfg)
    if cfg.mapping_mode:
        from simulated_motor import mapping_demo
        return mapping_demo(cfg)
    if mode_name(cfg) != "GOTO":
        from simulated_motor import continuous_demo
        return continuous_demo(cfg)
    origin, start = 100.0, 0.0
    plan = make_plan(start, cfg.step_um, cfg.interval_ms, cfg.steps, cfg.min_um, cfg.max_um)
    dt = cfg.interval_ms / 1000
    duration = cfg.steps * dt
    samples, pos = [], start
    period = cfg.sample_ms / 1000
    n = int((duration + cfg.tail_ms / 1000 + 0.1) / period) + 1
    for i in range(n):
        t = -0.1 + i * period
        idx = min(cfg.steps, max(0, math.floor((t - 0.012) / dt) + 1))
        target = start + idx * cfg.step_um
        delta = min(abs(target - pos), 450 * period)
        pos += math.copysign(delta, target - pos) if delta else 0
        samples.append(dict(host_abs_s=origin + t + 0.002, mcu_s=10 + t,
                            position_um=pos, seq=i, state=7 if idx else 0,
                            error=0, flags=8, overflow=0))
    for row in plan:
        row.update(sent_abs_s=origin + row["scheduled_send_s"],
                   write_end_abs_s=origin + row["scheduled_send_s"] + 0.0002,
                   ack_abs_s=origin + row["scheduled_send_s"] + 0.002, token=row["step"] % 255 + 1)
    return dict(config=vars(cfg).copy(), start_um=start, origin_abs_s=origin,
                planned=plan, commands=plan, status="DEMO_SYNTHETIC",
                firmware={}, max_velocity_um_s=450, samples=samples, events=[],
                bad_frames=0, lost_samples=0)


def result_title(result):
    cfg = result["config"]
    speed = (f"XY {cfg['xy_speed']:g} mm/s | mapping" if result.get("reference_source") == "XY_MAPPING"
             else f"{abs(cfg['step_um']) * 1000 / cfg['interval_ms']:g} um/s")
    return f"{result['status']} | {speed} | {result.get('mode', 'GOTO')}"


def gui(args):
    mapping_defaults(args)
    if not args.stage_dll:
        args.stage_dll = default_dll_path()
    import tkinter as tk
    from tkinter import ttk, messagebox, filedialog
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
    root = tk.Tk()
    root.title("STM32 对焦测试 · GOTO / 速度微调 / TRACK · XY mapping")
    root.geometry(f"{min(1380, root.winfo_screenwidth() - 60)}x{min(920, root.winfo_screenheight() - 100)}")
    tasks, messages = queue.Queue(), queue.Queue()
    stage_tasks = queue.Queue()
    stage_state = {"stage": None, "reader": None}
    stage_lock = threading.Lock()
    cancel, shutdown = threading.Event(), threading.Event()
    motor_done = threading.Event()
    status = tk.StringVar(value="未连接；请先填写实际机械软限位")
    frame = ttk.Frame(root, padding=10)
    frame.pack(fill="x")
    fields = {}
    specs = [("port", "串口", args.port or "COM3"), ("interval_ms", "指令间隔 ms", args.interval_ms),
             ("step_um", "每步 μm", args.step_um), ("steps", "步数", args.steps),
             ("sample_ms", "遥测间隔 ms", args.sample_ms), ("tail_ms", "末尾观察 ms", args.tail_ms),
             ("min_um", "软下限 μm", args.min_um), ("max_um", "软上限 μm", args.max_um)]
    for i, (key, label, default) in enumerate(specs):
        ttk.Label(frame, text=label).grid(row=0, column=i)
        var = tk.StringVar(value=str(default))
        fields[key] = var
        ttk.Entry(frame, textvariable=var, width=13).grid(row=1, column=i, padx=3)
    controls = ttk.Frame(root, padding=10)
    controls.pack(fill="x")
    buttons = []
    mode_vars = {key: tk.BooleanVar(value=getattr(args, key)) for key in ("velocity_mode", "position_mode")}
    mapping_enabled = tk.BooleanVar(value=args.mapping_mode)
    checkboxes = []
    def cfg_from_form():
        cfg = argparse.Namespace(**vars(args))
        for key, var in fields.items():
            setattr(cfg, key, var.get().strip() if key in ("port", "mapping_file", "stage_ip", "stage_dll", "xy_source") else
                    int(var.get()) if key == "steps" else float(var.get()))
        for key, var in mode_vars.items():
            setattr(cfg, key, var.get())
        cfg.mapping_mode = mapping_enabled.get()
        validate(cfg)
        return cfg
    def submit(action):
        try:
            cfg = cfg_from_form()
            if action == "move":
                value = float(manual.get())
                if not math.isfinite(value) or not cfg.min_um <= value <= cfg.max_um:
                    raise ValueError("手动位置超出软限位")
            else:
                value = None
            cancel.clear()
            nonlocal_status_hold[0] = 0.0
            for button in buttons + checkboxes:
                button.configure(state="disabled")
            tasks.put((action, cfg, value))
        except Exception as exc:
            messagebox.showerror("参数", str(exc))
    for label, action in [("连接", "connect"), ("开始测试", "run"), ("模拟演示", "demo"), ("断开", "disconnect")]:
        b = ttk.Button(controls, text=label, command=lambda a=action: submit(a))
        b.pack(side="left", padx=4)
        buttons.append(b)
    ttk.Button(controls, text="停止", command=cancel.set).pack(side="left", padx=4)
    manual = tk.StringVar(value="0")
    ttk.Label(controls, text="绝对位置 μm").pack(side="left", padx=8)
    ttk.Entry(controls, textvariable=manual, width=12).pack(side="left")
    b = ttk.Button(controls, text="移动到位置", command=lambda: submit("move"))
    b.pack(side="left", padx=4)
    buttons.append(b)
    xy_control_button = ttk.Button(controls, text="XY位移台控制", command=lambda: open_xy_control())
    xy_control_button.pack(side="left", padx=4)
    buttons.append(xy_control_button)
    presets = ttk.Frame(root, padding=5)
    presets.pack(fill="x")
    def preset(speed):
        try:
            fields["step_um"].set(f"{speed * float(fields['interval_ms'].get()) / 1000:g}")
        except ValueError:
            pass
    ttk.Label(presets, text="目标更新平均速度：").pack(side="left")
    for speed in (200, 400, 600, 800, 1000):
        ttk.Button(presets, text=f"{speed} μm/s", command=lambda s=speed: preset(s)).pack(side="left", padx=3)

    continuous_box = ttk.LabelFrame(root, text="连续模式：分别勾选测试；均不勾选 = 原 GOTO", padding=6)
    continuous_box.pack(fill="x", padx=10, pady=3)
    def choose_mode(key):
        if mode_vars[key].get():
            mode_vars["position_mode" if key == "velocity_mode" else "velocity_mode"].set(False)
    for i, (key, label) in enumerate((("velocity_mode", "方案 A：速度前馈 + 位置误差速度微调"),
                                      ("position_mode", "方案 B：位置提前 + 连续 TRACK"))):
        box = ttk.Checkbutton(continuous_box, text=label, variable=mode_vars[key], command=lambda k=key: choose_mode(k))
        box.grid(row=0, column=i * 3, columnspan=3, sticky="w", padx=5)
        checkboxes.append(box)
    extra = [("control_ms", "连续周期 ms"), ("acceleration_um_s2", "Z参考加速度 μm/s²"),
             ("kp_s", "A位置增益 /s"), ("trim_percent", "A速度微调上限 %"), ("lead_ms", "B提前量 ms"),
             ("deadband_um", "A死区 μm"), ("max_following_um", "误差停止阈值 μm")]
    for i, (key, label) in enumerate(extra):
        ttk.Label(continuous_box, text=label).grid(row=1, column=i)
        fields[key] = tk.StringVar(value=str(getattr(args, key)))
        ttk.Entry(continuous_box, textvariable=fields[key], width=14).grid(row=2, column=i, padx=4)

    stage_box = ttk.LabelFrame(root, text="ZMotion XY 位移台 / 导入 mapping（XY: mm；Z: μm）", padding=6)
    stage_box.pack(fill="x", padx=10, pady=3)
    fields["stage_ip"] = tk.StringVar(value=args.stage_ip)
    fields["stage_dll"] = tk.StringVar(value=args.stage_dll)
    fields["mapping_file"] = tk.StringVar(value=args.mapping_file)
    fields["xy_source"] = tk.StringVar(value=args.xy_source)
    ttk.Label(stage_box, text="IP").grid(row=0, column=0, sticky="w")
    ttk.Entry(stage_box, textvariable=fields["stage_ip"], width=18).grid(row=0, column=1)
    ttk.Label(stage_box, text="zauxdll64.dll").grid(row=0, column=2)
    ttk.Entry(stage_box, textvariable=fields["stage_dll"], width=70).grid(row=0, column=3, columnspan=4, sticky="ew")
    def browse_dll():
        path = filedialog.askopenfilename(title="选择原工程 zauxdll64.dll", filetypes=[("ZMotion DLL", "*.dll")])
        if path:
            fields["stage_dll"].set(path)
    b = ttk.Button(stage_box, text="选择 DLL", command=browse_dll)
    b.grid(row=0, column=7)
    buttons.append(b)
    stage_status = tk.StringVar(value="XY 未连接；默认 MPOS 反馈位置，DPOS 为控制器指令位置")
    mapping_status = tk.StringVar(value="尚未导入 mapping")
    stage_buttons = []
    def submit_stage(action):
        try:
            cfg = cfg_from_form()
            for button in buttons + checkboxes:
                button.configure(state="disabled")
            stage_tasks.put((action, cfg))
        except Exception as exc:
            messagebox.showerror("位移台参数", str(exc))
    for i, (label, action) in enumerate((("连接位移台", "connect"), ("断开位移台", "disconnect"))):
        b = ttk.Button(stage_box, text=label, command=lambda a=action: submit_stage(a))
        b.grid(row=1, column=i, padx=3, pady=4)
        buttons.append(b)
        stage_buttons.append(b)
    def preview_mapping():
        try:
            curve = MappingCurve.load(fields["mapping_file"].get())
            top = tk.Toplevel(root)
            top.title("导入的目标高度 mapping（文件原始距离）")
            fig = Figure(figsize=(10, 4.5), layout="constrained")
            axes = fig.subplots(1, 2 if curve.x_mm is not None else 1)
            if curve.x_mm is not None:
                ax_xy, ax_z = axes
                ax_xy.plot(curve.x_mm, curve.y_mm, "-o", markersize=2.5, linewidth=1,
                           label="imported points")
                ax_xy.scatter((curve.x_mm[0], curve.x_mm[-1]),
                              (curve.y_mm[0], curve.y_mm[-1]),
                              c=("green", "red"), zorder=3, label="start / end")
                ax_xy.set(xlabel="X (mm)", ylabel="Y (mm)",
                          title="Overall XY curve (imported point order)")
                ax_xy.set_aspect("equal", adjustable="datalim")
                ax_xy.legend(loc="best")
            else:
                ax_z = axes
            ax_z.plot(curve.distance_mm, curve.target_z_um, "-o", markersize=2.5,
                      linewidth=1, label="target Z")
            ax_z.set(xlabel="Mapping distance (mm)", ylabel="Target Z (um)",
                     title="Target height curve")
            ax_z.grid(alpha=.25)
            ax_z.legend(loc="best")
            preview = FigureCanvasTkAgg(fig, master=top)
            preview.get_tk_widget().pack(fill="both", expand=True)
            preview.draw()
        except Exception as exc:
            messagebox.showerror("mapping", str(exc))
    def import_mapping():
        path = filedialog.askopenfilename(title="导入物理前馈页面的五列目标曲线",
                                          filetypes=[("Mapping", "*.txt *.csv *.tsv"), ("所有文件", "*.*")])
        if not path:
            return
        try:
            curve = MappingCurve.load(path)
            fields["mapping_file"].set(str(Path(path).resolve()))
            if curve.start_xy_mm and curve.end_xy_mm:
                for key, value in zip(("xy_start_x", "xy_start_y", "xy_end_x", "xy_end_y"),
                                      (*curve.start_xy_mm, *curve.end_xy_mm)):
                    fields[key].set(f"{value:g}")
            mapping_enabled.set(True)
            mapping_status.set(f"{Path(path).name}；{len(curve.distance_mm)}点；"
                               f"Z {min(curve.target_z_um):g}~{max(curve.target_z_um):g} μm；"
                               "完整曲线按比例对应下方 XY 起终点")
            preview_mapping()
        except Exception as exc:
            messagebox.showerror("mapping 导入", str(exc))
    b = ttk.Button(stage_box, text="导入 mapping", command=import_mapping)
    b.grid(row=1, column=2)
    buttons.append(b)
    b = ttk.Button(stage_box, text="预览 mapping", command=preview_mapping)
    b.grid(row=1, column=3)
    buttons.append(b)
    box = ttk.Checkbutton(stage_box, text="用实际 XY + mapping 替换固定步长直线", variable=mapping_enabled)
    box.grid(row=1, column=4, columnspan=4, sticky="w")
    checkboxes.append(box)
    xy_fields = [("xy_start_x", "起点 X mm"), ("xy_start_y", "起点 Y mm"),
                 ("xy_end_x", "终点 X mm"), ("xy_end_y", "终点 Y mm"),
                 ("xy_speed", "扫描线速度 mm/s"), ("xy_acceleration", "XY加速度 mm/s²"),
                 ("xy_sample_ms", "XY读取周期 ms"), ("xy_tolerance_mm", "扫描线容差 mm"),
                 ("z_offset_um", "Z整体偏移 μm"), ("z_speed_limit", "Z速度上限 μm/s"),
                 ("scan_timeout_s", "扫描超时 s")]
    for i, (key, label) in enumerate(xy_fields):
        row, col = 2 + (i // 7) * 2, i % 7
        ttk.Label(stage_box, text=label).grid(row=row, column=col)
        fields[key] = tk.StringVar(value=str(getattr(args, key)))
        ttk.Entry(stage_box, textvariable=fields[key], width=14).grid(row=row + 1, column=col, padx=4)
    ttk.Label(stage_box, text="XY位置来源").grid(row=4, column=4)
    ttk.Combobox(stage_box, textvariable=fields["xy_source"], values=("MPOS", "DPOS"),
                 state="readonly", width=10).grid(row=5, column=4)
    b = ttk.Button(stage_box, text="到扫描起点 XY+Z", command=lambda: submit("prepare"))
    b.grid(row=5, column=5, columnspan=2)
    buttons.append(b)
    b = ttk.Button(stage_box, text="到起点 XY", command=lambda: submit("move_xy_start"))
    b.grid(row=5, column=0, columnspan=2)
    buttons.append(b)
    b = ttk.Button(stage_box, text="到终点 XY", command=lambda: submit("move_xy_end"))
    b.grid(row=5, column=2, columnspan=2)
    buttons.append(b)
    ttk.Label(stage_box, textvariable=mapping_status, wraplength=1250).grid(row=6, column=0, columnspan=8, sticky="w")
    ttk.Label(stage_box, textvariable=stage_status, wraplength=1250).grid(row=7, column=0, columnspan=8, sticky="w")

    def get_stage(cfg):
        with stage_lock:
            stage, reader = stage_state["stage"], stage_state["reader"]
        if stage is None or reader is None:
            raise RuntimeError("请先连接 XY 位移台")
        if stage.source != cfg.xy_source or abs(reader.period_s * 1000 - cfg.xy_sample_ms) > 1e-6:
            raise RuntimeError("XY位置来源/采样周期改变后请重新连接位移台")
        return stage, reader

    def open_xy_control():
        top = tk.Toplevel(root)
        top.title("XY 位移台控制")
        top.geometry("470x260")
        top.transient(root)
        current = tk.StringVar(value="当前坐标：未连接")
        result = tk.StringVar(value="")
        ttk.Label(top, textvariable=current, font=("TkDefaultFont", 11)).pack(pady=10)
        form = ttk.Frame(top)
        form.pack(fill="x", padx=18)
        speed = tk.StringVar(value=str(args.xy_speed))
        target_x = tk.StringVar(value=str(args.xy_start_x))
        target_y = tk.StringVar(value=str(args.xy_start_y))
        for row, (label, variable) in enumerate((("移动速度 mm/s", speed),
                                                   ("目标 X mm", target_x),
                                                   ("目标 Y mm", target_y))):
            ttk.Label(form, text=label).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Entry(form, textvariable=variable, width=16).grid(row=row, column=1, padx=8)

        def move_xy_manual():
            try:
                cfg = cfg_from_form()
                velocity = float(speed.get())
                target = (float(target_x.get()), float(target_y.get()))
                if not math.isfinite(velocity) or not .001 <= velocity <= 500:
                    raise ValueError("移动速度必须在 0.001~500 mm/s")
                if not all(math.isfinite(v) for v in target):
                    raise ValueError("目标坐标必须是有限数")
                cancel.clear()
                for button in buttons + checkboxes:
                    button.configure(state="disabled")
                tasks.put(("move_xy_manual", cfg, {"target": target, "speed": velocity}))
                result.set("已发送移动命令")
            except Exception as exc:
                messagebox.showerror("XY 位移台", str(exc), parent=top)

        ttk.Button(top, text="移动到目标位置", command=move_xy_manual).pack(pady=8)
        def sync_xy_mpos():
            if not messagebox.askyesno(
                    "确认校准", "仅在已经确认机械位置正确且位移台空闲时执行。\n"
                                "是否将当前 MPOS 对齐到 DPOS？", parent=top):
                return
            try:
                cfg = cfg_from_form()
                cancel.clear()
                tasks.put(("sync_xy_mpos", cfg, None))
                result.set("已发送 MPOS 校准命令")
            except Exception as exc:
                messagebox.showerror("XY 校准", str(exc), parent=top)
        ttk.Button(top, text="MPOS 对齐 DPOS", command=sync_xy_mpos).pack(pady=2)
        ttk.Label(top, textvariable=result, foreground="gray").pack()

        def refresh_xy():
            if not top.winfo_exists():
                return
            try:
                with stage_lock:
                    active_stage = stage_state["stage"]
                    active_reader = stage_state["reader"]
                if active_reader is None:
                    current.set("当前坐标：未连接")
                else:
                    sample = active_reader.latest()
                    dpos = active_stage.get_dpositions()
                    idle = active_stage.is_idle()
                    current.set(f"MPOS(校正)：X={sample['x_mm']:.4f}，Y={sample['y_mm']:.4f} mm\n"
                                f"DPOS：X={dpos[0]:.4f}，Y={dpos[1]:.4f} mm；状态={'空闲' if idle else '运动中'}")
            except Exception as exc:
                current.set(f"当前坐标：读取失败（{exc}）")
            top.after(200, refresh_xy)

        refresh_xy()

    def stage_worker():
        stage = reader = None
        try:
            while not shutdown.is_set():
                try:
                    action, cfg = stage_tasks.get(timeout=.1)
                except queue.Empty:
                    if reader:
                        try:
                            sample = reader.latest()
                            messages.put(("xy_status", f"XY {stage.source}：X={sample['x_mm']:.4f} mm，"
                                          f"Y={sample['y_mm']:.4f} mm；DLL读取 {sample['read_duration_s'] * 1000:.1f} ms"))
                        except Exception as exc:
                            messages.put(("xy_status", str(exc)))
                    continue
                try:
                    if reader:
                        reader.stop()
                        reader = None
                    if stage and stage.handle.value:
                        stage.stop()
                        stage.disconnect()
                    stage = None
                    with stage_lock:
                        stage_state.update(stage=None, reader=None)
                    if action == "connect":
                        stage = XYStage(cfg.stage_dll, cfg.xy_source)
                        stage.connect(cfg.stage_ip)
                        reader = StageReader(stage, cfg.xy_sample_ms / 1000)
                        reader.start()
                        with stage_lock:
                            stage_state.update(stage=stage, reader=reader)
                        messages.put(("xy_status", f"XY 已连接 {cfg.stage_ip}；axis 0=X、1=Y；{cfg.xy_source}"))
                    else:
                        messages.put(("xy_status", "XY 已断开"))
                except Exception as exc:
                    if stage and not stage.handle.value:
                        stage = None
                    messages.put(("error", str(exc)))
                finally:
                    messages.put(("ready", None))
        finally:
            # 先让马达线程完成双设备 STOP，再释放 XY 采样器与句柄。
            motor_done.wait()
            try:
                if reader:
                    reader.stop()
                if stage and stage.handle.value:
                    stage.stop()
                    stage.disconnect()
            except Exception as exc:
                messages.put(("error", str(exc)))

    stage_thread = threading.Thread(target=stage_worker, name="StageConnection", daemon=True)
    stage_thread.start()
    ttk.Label(root, textvariable=status, wraplength=1200).pack(fill="x", padx=10)
    figure = Figure(figsize=(12, 4), layout="constrained")
    canvas = FigureCanvasTkAgg(figure, master=root)
    canvas.get_tk_widget().pack(fill="both", expand=True)

    def worker():
        link = None
        report_at = 0.0
        try:
            while not shutdown.is_set():
                try:
                    action, cfg, value = tasks.get(timeout=0.005)
                except queue.Empty:
                    if link:
                        try:
                            if cancel.is_set():
                                link.stop()
                                cancel.clear()
                            link.poll()
                            now = time.perf_counter()
                            if link.latest and now - link.latest["host_abs_s"] > max(0.5, 3 * link.sample_ms / 1000):
                                raise TimeoutError("位置遥测中断，请重新连接")
                            if now - report_at >= 0.2 and link.latest:
                                messages.put(("status", f"位置 {link.latest['position_um']:.3f} μm；"
                                              f"GOTO 当前速度上限 {link.max_velocity:g} μm/s"))
                                report_at = now
                            # 手动操作只需最新位置，试验原始数据在 run_test 内保留。
                            if len(link.samples) > 2000:
                                link.samples = link.samples[-100:]
                        except Exception as exc:
                            messages.put(("error", str(exc)))
                            link.close()
                            link = None
                    continue
                try:
                    if action == "connect":
                        if link:
                            link.close()
                        link = None
                        link = MotorLink(cfg.port, cfg.baud, cfg.sample_ms)
                        messages.put(("status", f"已连接 {cfg.port}；固件 {link.info['fw_ver']}；"
                                      f"GOTO 速度上限 {link.max_velocity:g} μm/s"))
                    elif action == "disconnect":
                        if link:
                            link.close()
                        link = None
                        messages.put(("status", "已断开"))
                    elif action == "demo":
                        messages.put(("result", demo_result(cfg)))
                    elif action == "move_xy_manual":
                        stage, reader = get_stage(cfg)
                        target = value["target"]
                        try:
                            stage.move(target, value["speed"], cfg.xy_acceleration)
                            began = time.perf_counter()
                            while True:
                                if cancel.is_set():
                                    raise RuntimeError("XY 移动已停止")
                                sample = reader.latest()
                                if (math.dist((sample["x_mm"], sample["y_mm"]), target) <= cfg.xy_tolerance_mm
                                        and stage.is_idle()):
                                    messages.put(("status", f"XY 已到达 ({target[0]:g}, {target[1]:g}) mm"))
                                    break
                                if time.perf_counter() - began > cfg.scan_timeout_s:
                                    dpos = stage.get_dpositions()
                                    messages.put(("status", f"XY timeout target={target} MPOS=({sample['x_mm']:.4f},{sample['y_mm']:.4f}) DPOS=({dpos[0]:.4f},{dpos[1]:.4f})"))
                                    raise TimeoutError("XY 移动到目标位置超时")
                                cancel.wait(.005)
                        except BaseException:
                            stage.stop()
                            raise
                    elif action == "sync_xy_mpos":
                        stage, reader = get_stage(cfg)
                        try:
                            aligned = stage.sync_mpos_to_dpos()
                            messages.put(("status", f"XY MPOS 已对齐 DPOS：X={aligned[0]:.4f}，Y={aligned[1]:.4f} mm"))
                        except BaseException:
                            raise
                    elif action in ("move_xy_start", "move_xy_end"):
                        stage, reader = get_stage(cfg)
                        target = ((cfg.xy_start_x, cfg.xy_start_y) if action == "move_xy_start"
                                  else (cfg.xy_end_x, cfg.xy_end_y))
                        try:
                            stage.move(target, cfg.xy_speed, cfg.xy_acceleration)
                            began = time.perf_counter()
                            while True:
                                if cancel.is_set():
                                    raise RuntimeError("XY 移动已停止")
                                sample = reader.latest()
                                if (math.dist((sample["x_mm"], sample["y_mm"]), target) <= cfg.xy_tolerance_mm
                                        and stage.is_idle()):
                                    messages.put(("status", f"XY 已到达 ({target[0]:g}, {target[1]:g}) mm"))
                                    break
                                if time.perf_counter() - began > cfg.scan_timeout_s:
                                    raise TimeoutError("XY 移动到目标位置超时")
                                cancel.wait(.005)
                        except BaseException:
                            stage.stop()
                            raise
                    elif action in ("run", "move", "prepare"):
                        if link is None:
                            raise RuntimeError("请先连接 STM32")
                        if abs(cfg.sample_ms - link.sample_ms) > 1e-6 or cfg.port != link.ser.port:
                            raise RuntimeError("串口或遥测周期改变后请重新连接")
                        if action == "prepare":
                            stage, reader = get_stage(cfg)
                            curve = MappingCurve.load(cfg.mapping_file)
                            line = mapping_line(curve, cfg)
                            z = curve.target_z_um[0] + cfg.z_offset_um
                            if not cfg.min_um <= z <= cfg.max_um:
                                raise ValueError("mapping 起点 Z 超出软限位")
                            if link.latest["state"] != 0:
                                raise RuntimeError("先停止 Z 并等待 IDLE")
                            try:
                                stage.move(line.start, cfg.xy_speed, cfg.xy_acceleration)
                                key = link.goto(z)
                                began = time.perf_counter()
                                while True:
                                    if cancel.is_set():
                                        raise RuntimeError("移动到起点已停止")
                                    link.poll()
                                    sample = reader.latest()
                                    if (math.dist((sample["x_mm"], sample["y_mm"]), line.start) <= cfg.xy_tolerance_mm
                                            and abs(link.latest["position_um"] - z) <= max(.5, cfg.deadband_um)
                                            and key not in link.pending and link.latest["state"] == 0 and stage.is_idle()):
                                        break
                                    if time.perf_counter() - began > cfg.scan_timeout_s:
                                        raise TimeoutError("移动到扫描起点超时")
                                    cancel.wait(.005)
                                messages.put(("status", "XY 与 Z 已到 mapping 起点，可以开始测试"))
                            except BaseException:
                                link.stop()
                                stage.stop()
                                raise
                        elif action == "move":
                            if link.latest["state"] != 0:
                                raise RuntimeError("马达应处于 IDLE；先停止并等待遥测")
                            key = link.goto(value)
                            link.until(lambda: key not in link.pending, 0.6)
                            confirmed = time.perf_counter()
                            link.until(lambda: link.latest["host_abs_s"] > confirmed, 0.5)
                            messages.put(("status", f"已发送 GOTO {value:g} μm"))
                        else:
                            if link.latest["state"] != 0:
                                raise RuntimeError("测试起点必须为 IDLE；先停止并等待遥测")
                            if cfg.mapping_mode:
                                stage, reader = get_stage(cfg)
                                result = run_mapping(link, stage, reader, MappingCurve.load(cfg.mapping_file), cfg, cancel,
                                                     lambda s: messages.put(("status", s)))
                            else:
                                result = run_test(link, cfg, cancel, lambda s: messages.put(("status", s)))
                            messages.put(("result", result))
                except Exception as exc:
                    messages.put(("error", str(exc)))
                finally:
                    messages.put(("ready", None))
        finally:
            try:
                if link:
                    link.close()
            finally:
                motor_done.set()

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    def refresh():
        while not messages.empty():
            kind, value = messages.get_nowait()
            if kind == "ready":
                for button in buttons + checkboxes:
                    button.configure(state="normal")
            elif kind == "xy_status":
                stage_status.set(value)
            elif kind == "error":
                status.set(value)
                nonlocal_status_hold[0] = time.perf_counter() + 8
                messagebox.showerror("马达测试", value)
            elif kind == "status":
                if time.perf_counter() >= nonlocal_status_hold[0]:
                    status.set(value)
            elif kind == "result":
                try:
                    output, analysis = save_result(value, args.output)
                    draw(analysis, result_title(value), figure)
                    canvas.draw_idle()
                    avg = analysis["average_speed_um_s"]
                    status.set(f"{value['status']}；实测平均速度 {avg if avg is not None else '无有效数据'} μm/s；"
                               f"有效点 {analysis['valid_step_count']}/{analysis.get('evaluation_count', value['config']['steps'])}；"
                               f"{value.get('error', '')} 保存：{output}")
                    # 给结果状态留出查看时间。
                    nonlocal_status_hold[0] = time.perf_counter() + 8
                except Exception as exc:
                    messagebox.showerror("结果导出", str(exc))
        if not shutdown.is_set():
            root.after(100, refresh)
    nonlocal_status_hold = [0.0]
    def close():
        cancel.set()
        shutdown.set()
        status.set("正在停止马达并关闭串口…")
        def finish():
            if thread.is_alive() or stage_thread.is_alive():
                root.after(50, finish)
            else:
                root.destroy()
        finish()
    root.protocol("WM_DELETE_WINDOW", close)
    root.after(100, refresh)
    root.mainloop()


def validate(cfg):
    validate_control(cfg)
    validate_mapping(cfg)
    make_plan(0, cfg.step_um, cfg.interval_ms, cfg.steps, -1e12, 1e12)
    if not (math.isfinite(cfg.min_um) and math.isfinite(cfg.max_um) and cfg.min_um < cfg.max_um):
        raise ValueError("软限位无效")
    if not math.isfinite(cfg.sample_ms) or not 5 <= cfg.sample_ms <= 1000:
        raise ValueError("遥测周期须为5~1000 ms")
    if abs(round(1000 / cfg.sample_ms) - 1000 / cfg.sample_ms) > 1e-6:
        raise ValueError("遥测周期应对应整数 Hz，例如5、10、20、50、100 ms")
    if cfg.sample_ms > cfg.interval_ms / 2:
        raise ValueError("遥测周期不得超过指令周期的一半")
    if not math.isfinite(cfg.tail_ms) or not max(100, 2 * cfg.sample_ms) <= cfg.tail_ms <= 10000:
        raise ValueError("末尾观察时间须至少100 ms和两个遥测周期，至多10000 ms")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="", help="例如 COM3")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--interval-ms", type=float, default=50)
    parser.add_argument("--step-um", type=float, default=10)
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--sample-ms", type=float, default=5)
    parser.add_argument("--tail-ms", type=float, default=500)
    parser.add_argument("--min-um", type=float, default=-5000)
    parser.add_argument("--max-um", type=float, default=5000)
    parser.add_argument("--output", default=str(Path(__file__).resolve().parent / "results"))
    parser.add_argument("--cli", action="store_true", help="命令行单次硬件试验")
    parser.add_argument("--demo", action="store_true", help="合成数据，离线导出三张曲线")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--velocity-mode", action="store_true", help="方案 A：速度前馈+位置误差速度微调")
    parser.add_argument("--position-mode", action="store_true", help="方案 B：位置提前+连续 TRACK")
    for key, value in DEFAULTS.items():
        if not key.endswith("mode"):
            parser.add_argument("--" + key.replace("_", "-"), type=float, default=value)
    for key, value in MAPPING_DEFAULTS.items():
        if key == "mapping_mode":
            parser.add_argument("--mapping-mode", action="store_true", help="设置位置来自实际 XY 和导入 mapping")
        else:
            parser.add_argument("--" + key.replace("_", "-"), type=str if isinstance(value, str) else float,
                                default=default_dll_path() if key == "stage_dll" else value)
    args = parser.parse_args()
    if args.list_ports:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            print(p.device, p.description)
        return 0
    validate(args)
    if args.demo:
        result = demo_result(args)
        output, analysis = save_result(result, args.output)
        print(f"合成演示（非硬件测量）：{output}")
        if result.get("error"):
            print(result["error"])
        return 0 if result["status"] == "DEMO_SYNTHETIC" and analysis["measurement_valid"] else 1
    if not args.cli:
        gui(args)
        return 0
    if not args.port:
        parser.error("--cli 必须指定 --port")
    link = stage = reader = None
    try:
        # Ethernet 连接可能阻塞；先连接 XY，再建立 STM32 心跳。
        if args.mapping_mode:
            stage = XYStage(args.stage_dll, args.xy_source)
            stage.connect(args.stage_ip)
            reader = StageReader(stage, args.xy_sample_ms / 1000)
            reader.start()
        link = MotorLink(args.port, args.baud, args.sample_ms)
        if link.latest["state"] != 0:
            raise RuntimeError("测试起点必须为 IDLE")
        print(f"固件信息 {link.info}；当前 GOTO 上限 {link.max_velocity} μm/s")
        cancel = threading.Event()
        import signal
        signal.signal(signal.SIGINT, lambda *_: cancel.set())
        if args.mapping_mode:
            deadline = time.perf_counter() + 1.
            while reader.sample is None and time.perf_counter() < deadline:
                link.poll()
                time.sleep(.001)
            result = run_mapping(link, stage, reader, MappingCurve.load(args.mapping_file), args, cancel, print)
        else:
            result = run_test(link, args, cancel, print)
    finally:
        try:
            if link:
                link.close()
        finally:
            if reader:
                reader.stop()
            if stage:
                try:
                    stage.stop()
                finally:
                    stage.disconnect()
    output, analysis = save_result(result, args.output)
    print(f"{result['status']}；平均速度 {analysis['average_speed_um_s']} μm/s；结果：{output}")
    if result.get("error"):
        print(result["error"])
    return 0 if result["status"] == "complete" and analysis["measurement_valid"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, TimeoutError, ImportError, OSError) as exc:
        raise SystemExit(str(exc))
