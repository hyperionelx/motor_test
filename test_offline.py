"""离线验证；不会打开串口。运行：python -m unittest discover -s test_motor -v"""
import argparse
import importlib
import json
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from motor_test import (Decoder, MotorLink, analyze, demo_result, encode_frame,
                        make_plan, interpolate, validate, GOTO, HELLO, PING,
                        STOP, STREAM_DATA, RESPONSE, ERROR, run_test)
import motor_test
from unittest.mock import patch


def config(**kwargs):
    values = dict(step_um=10., interval_ms=50., steps=20, sample_ms=5.,
                  tail_ms=500., min_um=-5000., max_um=5000.)
    values.update(kwargs)
    return argparse.Namespace(**values)


class FakeSerial:
    def __init__(self):
        self.data = b""
        self.is_open = True
        self.written = []

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, size):
        data, self.data = self.data[:size], self.data[size:]
        return data

    def write(self, data):
        self.written.append(data)
        return len(data)


def fake_link():
    import time
    link = MotorLink.__new__(MotorLink)
    link.ser, link.decoder = FakeSerial(), Decoder()
    link.session, link.token = 7, 0
    link.pending, link.replies = {}, {}
    link.samples, link.events = [], []
    link.latest = link.last_seq = link.last_ms = None
    link.mcu_ms = link.lost_samples = 0
    link.heartbeat_at = time.perf_counter()
    return link


def stream(seq, ms, position=10., session=7, flags=8):
    payload = bytes((session,)) + struct.pack("<II4hf4B", seq, ms, 0, 0, 0, 0,
                                              position, 0, 0, flags, 0)
    return encode_frame(STREAM_DATA, payload)


class ProtocolTests(unittest.TestCase):
    def test_crc_known_vector(self):
        import binascii
        self.assertEqual(binascii.crc_hqx(b"123456789", 0xFFFF), 0x29B1)

    def test_chunked_frames_and_corrupt_crc(self):
        frame = encode_frame(GOTO, b"\x07\x01" + struct.pack("<f", 30.0))
        decoder = Decoder()
        rows = []
        for byte in frame:
            rows.extend(decoder.feed(bytes((byte,))))
        self.assertEqual(rows, [(GOTO, 0, b"\x07\x01" + struct.pack("<f", 30.0))])
        corrupt = bytearray(frame)
        corrupt[-2] ^= 1
        self.assertEqual(decoder.feed(corrupt), [])
        self.assertEqual(decoder.bad_frames, 1)
        self.assertEqual(len(decoder.feed(frame + frame)), 2)

    def test_async_ack_session_token_and_rejection(self):
        link = fake_link()
        record = {}
        key = link.goto(10., record)
        link.ser.data = encode_frame(GOTO, bytes((8, key[1])), RESPONSE)
        link.poll()
        self.assertIn(key, link.pending)
        link.ser.data = encode_frame(GOTO, bytes((7, key[1])), RESPONSE)
        link.poll()
        self.assertNotIn(key, link.pending)
        self.assertIsNotNone(record["ack_abs_s"])
        key = link.goto(30.)
        link.ser.data = encode_frame(GOTO, bytes((7, key[1])), RESPONSE | ERROR)
        with self.assertRaisesRegex(RuntimeError, "命令被拒绝"):
            link.poll()

    def test_stream_wrap_duplicates_loss_and_faults(self):
        link = fake_link()
        link.ser.data = stream(0xFFFFFFFE, 0xFFFFFFFE) + stream(0xFFFFFFFE, 0xFFFFFFFE)
        link.poll()
        self.assertEqual(len(link.samples), 1)
        link.ser.data = stream(1, 3, 20.)
        link.poll()
        self.assertEqual(link.lost_samples, 2)
        self.assertAlmostEqual(link.samples[1]["mcu_s"] - link.samples[0]["mcu_s"], .005)
        link.ser.data = stream(2, 8, flags=9)
        with self.assertRaisesRegex(RuntimeError, "限位"):
            link.poll()

    def test_partial_write_record_can_be_exported(self):
        link = fake_link()
        link.ser.write = lambda data: len(data) - 1
        record = {}
        with self.assertRaisesRegex(RuntimeError, "写入不完整"):
            link.goto(10., record)
        self.assertFalse(record["write_ok"])
        self.assertIn("sent_abs_s", record)
        self.assertIn("write_end_abs_s", record)

    def test_matches_pc_app_wire_bytes(self):
        workspace = Path(__file__).resolve().parents[1]
        source = workspace / "i-ATF PC_0928_original/i-ATF PC_0928_original/i-ATF PC_0925_backup"
        if not (source / "pc_app/comm/protocol.py").exists():
            from xy_stage import REFERENCE_ROOT
            source = REFERENCE_ROOT
        if not (source / "pc_app/comm/protocol.py").exists():
            self.skipTest("独立复制后无 pc_app，跳过原协议比对")
        sys.path.insert(0, str(source))
        # 不向参考工程写入新字节码缓存。
        old_bytecode = sys.dont_write_bytecode
        sys.dont_write_bytecode = True
        try:
            protocol = importlib.import_module("pc_app.comm.protocol")
        finally:
            sys.dont_write_bytecode = old_bytecode
        comparisons = [(HELLO, struct.pack("<HI", 3, 7), "build_hello", ()),
                       (GOTO, struct.pack("<f", 30.), "build_motor_goto", (30.,)),
                       (PING, b"", "build_ping", ()), (STOP, b"", "build_motor_stop", ()),
                       (0x11, b"\x04", "build_param_read", (4,)),
                       (0x50, struct.pack("<HB", 200, 1), "build_stream_start", (200, 1))]
        for cmd, body, name, args in comparisons:
            original = protocol.Protocol()
            original.session = 7
            session = 0 if cmd == HELLO else 7
            self.assertEqual(encode_frame(cmd, bytes((session, 1)) + body),
                             getattr(original, name)(*args))
        payload = bytes((7,)) + struct.pack("<II4hf4B", 1, 1000, 0, 0, 0, 0, 123.5, 0, 0, 8, 0)
        decoded = protocol.Protocol.parse_stream_v3(payload)
        link = fake_link()
        link.ser.data = protocol.Protocol()._build_frame(STREAM_DATA, 0, payload)
        link.poll()
        self.assertEqual(link.latest["position_um"], decoded["position_um"])
        self.assertEqual(link.latest["mcu_s"], decoded["t_ms"] / 1000)


class MeasurementTests(unittest.TestCase):
    def test_one_second_twenty_steps_and_limits(self):
        plan = make_plan(100., 10., 50., 20, -5000., 5000.)
        self.assertEqual(len(plan), 20)
        self.assertEqual(plan[0]["scheduled_send_s"], 0)
        self.assertEqual(plan[-1]["deadline_s"], 1.)
        self.assertEqual(plan[-1]["target_um"], 300.)
        with self.assertRaisesRegex(ValueError, "终点"):
            make_plan(4900., 30., 50., 20, -5000., 5000.)

    def test_linear_motion_known_lag_and_reverse(self):
        for direction in (1, -1):
            result = demo_result(config(step_um=direction * 10.))
            for row in result["samples"]:
                t = row["mcu_s"] - 10
                row["position_um"] = direction * (200 * t - 4)
            analysis = analyze(result)
            self.assertAlmostEqual(analysis["average_speed_um_s"], 200., places=6)
            for row in analysis["steps"]:
                # 最小收包偏移含2ms传输延迟：估计误差4.4um，等效22ms。
                self.assertAlmostEqual(row["delay_ms"], 22., places=5)
            self.assertTrue(analysis["measurement_valid"])

    def test_missing_samples_zero_speed_partial_results(self):
        self.assertIsNone(interpolate([0., 1.], [0., 10.], .5, .02))
        result = demo_result(config())
        for row in result["samples"]:
            row["position_um"] = 0.
        analysis = analyze(result)
        self.assertIsNone(analysis["average_speed_um_s"])
        self.assertTrue(all(r["delay_ms"] is None for r in analysis["steps"]))
        result["status"] = "cancelled"
        result["commands"] = result["commands"][:3]
        self.assertFalse(analyze(result)["measurement_valid"])

    def test_all_speed_presets_and_validation(self):
        for speed in (200, 400, 600, 800, 1000):
            cfg = config(step_um=speed * .05)
            validate(cfg)
            analysis = analyze(demo_result(cfg))
            self.assertEqual(len(analysis["steps"]), 20)
            self.assertTrue(analysis["measurement_valid"])
            json.dumps(analysis, allow_nan=False)
        with self.assertRaises(ValueError):
            validate(config(sample_ms=50))
        with self.assertRaises(ValueError):
            validate(config(step_um=float("nan")))


class ScheduleTests(unittest.TestCase):
    def exercise(self, stall=False, cancel_at=None):
        class Clock:
            now = 100.
            def perf_counter(self):
                return self.now
        clock = Clock()
        class Cancel:
            def is_set(self):
                return cancel_at is not None and clock.now >= 100 + cancel_at
            def wait(self, duration):
                clock.now += duration
        class Link:
            sample_ms = 5
            max_velocity = 3000.
            info = {}
            lost_samples = 0
            decoder = Decoder()
            def __init__(self):
                self.latest = dict(host_abs_s=100., position_um=0.)
                self.samples = [self.latest.copy()]
                self.events = []
                self.stop_count = 0
                self.stalled = False
            def poll(self):
                if stall and not self.stalled and clock.now >= 100.01:
                    clock.now += .1
                    self.stalled = True
                self.latest = dict(host_abs_s=clock.now, position_um=(clock.now - 100) * 200)
                self.samples.append(self.latest.copy())
            def goto(self, position, row):
                row.update(sent_abs_s=clock.now, write_end_abs_s=clock.now,
                           ack_abs_s=clock.now, token=1, write_ok=True)
            def stop(self):
                self.stop_count += 1
                return None
        link = Link()
        with patch.object(motor_test, "time", clock):
            result = run_test(link, config(), Cancel())
        self.assertEqual(link.stop_count, 1)
        return result

    def test_fixed_deadlines_without_accumulated_drift(self):
        result = self.exercise()
        self.assertEqual(result["status"], "complete")
        self.assertEqual(len(result["commands"]), 20)
        for row in result["commands"]:
            late = row["sent_abs_s"] - 100 - row["scheduled_send_s"]
            self.assertGreaterEqual(late, -1e-9)
            self.assertLess(late, .00101)

    def test_stall_aborts_without_burst_and_cancel_stops(self):
        result = self.exercise(stall=True)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(len(result["commands"]), 1)
        result = self.exercise(cancel_at=.12)
        self.assertEqual(result["status"], "cancelled")
        self.assertEqual(len(result["commands"]), 3)


if __name__ == "__main__":
    unittest.main()
