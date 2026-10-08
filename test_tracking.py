"""A/B、mapping 和 XY 原生边界的定向离线验证；从不连接真实硬件。"""
import ctypes
import importlib
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from test_offline import config, fake_link
from motor_test import (analyze, demo_result, encode_frame, ERROR, RESPONSE,
                        VELOCITY, TRACK, validate, save_result)
from continuous_control import MotionProfile, VelocityTrim, validate_control, run_continuous
from mapping_run import mapping_defaults, run_mapping, MappingVelocityControl
from simulated_motor import SimClock, SimCancel, SimMotor, SimStage, SimReader
from xy_mapping import MappingCurve, LineMapping, XYVelocityEstimator
from xy_stage import XYStage, StageReader, REFERENCE_ROOT
import continuous_control
import mapping_run


EXAMPLE = Path(__file__).parent / "examples/mapping_line.txt"


class ProfileTests(unittest.TestCase):
    def test_one_rise_plateau_fall_same_end_both_directions(self):
        for sign in (1, -1):
            p = MotionProfile(100., 100. + sign * 200, 200., 4000., 4000.)
            self.assertAlmostEqual(p.duration_s, 1.05)
            self.assertEqual(p.at(-1), (100., 0.))
            self.assertEqual(p.at(p.duration_s), (100. + sign * 200, 0.))
            self.assertAlmostEqual(p.at(.5)[1], sign * 200)
            for boundary in (p.rise_s, p.rise_s + p.cruise_s):
                self.assertLess(abs(p.at(boundary - 1e-9)[0] - p.at(boundary + 1e-9)[0]), 1e-5)
            vs = [abs(p.at(i * p.duration_s / 1000)[1]) for i in range(1001)]
            changes = [b - a for a, b in zip(vs, vs[1:]) if abs(b - a) > 1e-9]
            self.assertEqual(sum(a > 0 and b < 0 for a, b in zip(changes, changes[1:])), 1)

    def test_short_trip_triangular_and_asymmetric_rates(self):
        p = MotionProfile(0., 1., 200., 100., 200.)
        self.assertEqual(p.cruise_s, 0.)
        self.assertAlmostEqual(p.rise_s / p.fall_s, 2.)
        self.assertEqual(p.at(p.duration_s), (1., 0.))

    def test_exclusive_modes_bad_parameters(self):
        with self.assertRaisesRegex(ValueError, "同时"):
            validate(config(velocity_mode=True, position_mode=True))
        for kwargs in (dict(control_ms=100), dict(trim_percent=51), dict(lead_ms=101),
                       dict(kp_s=math.inf), dict(max_following_um=26)):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                validate(config(**kwargs))

    def test_trim_cap_deadband_slew_and_no_scan_reversal(self):
        cfg = config(velocity_mode=True, control_ms=10., acceleration_um_s2=4000.)
        validate(cfg)
        p = MotionProfile(0, 200, 200, 4000, 4000)
        c = VelocityTrim(cfg, p)
        for t in (0., .01, .02, .03, .04, .05, .06):
            row = c.update(t, p.at(t)[0] - 10.)
            self.assertLessEqual(abs(row["correction_velocity_um_s"]), 40.)
        row = c.update(.5, p.at(.5)[0] + .1)
        self.assertEqual(row["correction_velocity_um_s"], 0.)
        row = c.update(.51, 10000.)
        self.assertGreaterEqual(row["command_velocity_um_s"], 0.)

    def test_continuous_demo_modes_reverse_and_600_speed(self):
        for mode in ("velocity_mode", "position_mode"):
            for sign in (1, -1):
                cfg = config(step_um=sign * 30., **{mode: True})
                result = demo_result(cfg)
                self.assertEqual(result["status"], "DEMO_SYNTHETIC", result.get("error"))
                self.assertTrue(analyze(result)["measurement_valid"])
                self.assertEqual(result["profile"]["end"], sign * 600.)
                self.assertNotIn("GOTO", {r["kind"] for r in result["commands"]})
                if mode == "position_mode":
                    self.assertEqual(sum(r["kind"] == "TRACK_START" for r in result["commands"]), 1)
                    self.assertAlmostEqual(result["commands"][-1]["command_position_um"], sign * 600.)


class WireTests(unittest.TestCase):
    def test_no_success_ack_is_not_a_timeout_but_error_is_caught(self):
        link = fake_link()
        link.info = {"caps": 8}
        row = {}
        key = link.velocity(-600., row)
        self.assertNotIn(key, link.pending)
        self.assertFalse(row["ack_required"])
        link.poll()
        link.ser.data = encode_frame(VELOCITY, bytes((7, key[1])), RESPONSE | ERROR)
        with self.assertRaisesRegex(RuntimeError, "命令被拒绝"):
            link.poll()
        key = link.track(0., 200., 1, start=True)
        self.assertIn(key, link.pending)
        link.ser.data = encode_frame(TRACK, bytes((7, key[1])), RESPONSE)
        link.poll()
        self.assertNotIn(key, link.pending)
        key = link.track(10., 200., 2)
        self.assertNotIn(key, link.pending)

    def test_tracking_capability_guard(self):
        link = fake_link()
        link.info = {"caps": 7}
        with self.assertRaisesRegex(RuntimeError, "不支持"):
            link.track(0, 0, 0)
        self.assertEqual(link.ser.written, [])

    def test_wire_bytes_against_v3_reference_when_present(self):
        path = REFERENCE_ROOT / "pc_app/comm/protocol.py"
        if not path.exists():
            self.skipTest("本机无参考 pc_app 源文件")
        sys.path.insert(0, str(REFERENCE_ROOT))
        old = sys.dont_write_bytecode
        sys.dont_write_bytecode = True
        try:
            proto = importlib.import_module("pc_app.comm.protocol")
        finally:
            sys.dont_write_bytecode = old
            sys.path.remove(str(REFERENCE_ROOT))
        for command, body, name, values in (
                (VELOCITY, struct.pack("<f", -600.), "build_motor_velocity", (-600.,)),
                (TRACK, struct.pack("<BBHff", 1, 0, 0, 140., 0.), "build_motor_track", (1, 0, 140., 0.)),
                (TRACK, struct.pack("<BBHff", 2, 0, 65535, -140., -350.), "build_motor_track", (2, 65535, -140., -350.))):
            p = proto.Protocol()
            p.session = 7
            self.assertEqual(encode_frame(command, bytes((7, 1)) + body), getattr(p, name)(*values))


class MappingTests(unittest.TestCase):
    def test_original_five_column_target_import(self):
        curve = MappingCurve.load(EXAMPLE)
        self.assertEqual(curve.start_xy_mm, (0., -40.))
        self.assertEqual(curve.end_xy_mm, (0., 40.))
        self.assertEqual(curve.x_mm, (0., 0., 0., 0., 0.))
        self.assertEqual(curve.y_mm, (-40., -20., 0., 20., 40.))
        self.assertEqual(curve.at(10), (105., -3.5))
        self.assertEqual(len(curve.source_sha256), 64)

    def test_csv_bom_and_reject_invalid_actual_only_duplicate_nan(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "map.csv"
            path.write_text("distance_mm,target_z_um,actual_z_um\n0,10,999\n1,20,999\n", encoding="utf-8-sig")
            self.assertEqual(MappingCurve.load(path).at(.5)[0], 15.)
            for text in ("distance_mm,actual_z_um\n0,10\n1,20\n",
                         "distance_mm,target_z_um\n0,10\n0,20\n",
                         "distance_mm,target_z_um\n0,10\n1,nan\n",
                         "distance_mm,target_z_um\n1,10\n0,20\n"):
                path.write_text(text, encoding="utf-8")
                with self.assertRaises(ValueError):
                    MappingCurve.load(path)

    def test_import_preserves_curved_xy_point_sequence(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "curved.csv"
            path.write_text(
                "distance_mm,x_mm,y_mm,target_z_um\n"
                "0,0,0,10\n"
                "1,1,2,20\n"
                "2,3,1,30\n",
                encoding="utf-8",
            )
            curve = MappingCurve.load(path)
            self.assertEqual(curve.x_mm, (0., 1., 3.))
            self.assertEqual(curve.y_mm, (0., 2., 1.))
            self.assertEqual(curve.start_xy_mm, (0., 0.))
            self.assertEqual(curve.end_xy_mm, (3., 1.))

    def test_diagonal_projection_scaling_real_speed_and_endpoint_no_extrapolation(self):
        curve = MappingCurve((0., 10.), (0., 100.))
        line = LineMapping(curve, (0., 0.), (6., 8.))
        ref = line.reference(3., 4., 6., 8.)
        self.assertAlmostEqual(ref["target_um"], 50.)
        self.assertAlmostEqual(ref["ideal_velocity_um_s"], 100.)
        self.assertAlmostEqual(line.reference(3., 4., 12., 16.)["ideal_velocity_um_s"], 200.)
        scaled = LineMapping(curve, (0., 0.), (12., 16.))
        self.assertAlmostEqual(scaled.reference(6., 8., 6., 8.)["ideal_velocity_um_s"], 50.)
        end = line.reference(6., 8., 6., 8., lead_s=.1)
        self.assertEqual(end["target_um"], 100.)
        self.assertEqual(end["ideal_velocity_um_s"], 0.)
        with self.assertRaisesRegex(RuntimeError, "离开"):
            line.reference(3., 8.)

    def test_arbitrary_curved_mapping_and_reverse_scan(self):
        curve = MappingCurve((0., 1., 3.), (10., 30., -10.))
        self.assertEqual(curve.at(.5), (20., 20.))
        self.assertEqual(curve.at(2.), (10., -20.))
        line = LineMapping(curve, (5., 0.), (-1., 0.))
        ref = line.reference(4., 0., -2., 0.)
        self.assertEqual(ref["target_um"], 20.)
        self.assertEqual(ref["ideal_velocity_um_s"], 20.)

    def test_xy_velocity_regression_ignores_repeated_sample(self):
        estimator = XYVelocityEstimator()
        for i in range(5):
            vx, vy = estimator.add(dict(host_abs_s=100 + .01 * i, x_mm=i * 2., y_mm=i * -1.))
        self.assertAlmostEqual(vx, 200.)
        self.assertAlmostEqual(vy, -100.)
        self.assertEqual(estimator.add(dict(host_abs_s=100.04, x_mm=999., y_mm=999.)), (vx, vy))

    def test_mapping_simulation_uses_xy_all_three_modes_and_exports(self):
        for mode in (None, "velocity_mode", "position_mode"):
            cfg = config(mapping_file=str(EXAMPLE), mapping_mode=True, xy_start_x=0., xy_start_y=-40.,
                         xy_end_x=0., xy_end_y=40., xy_speed=100., **({mode: True} if mode else {}))
            validate(cfg)
            result = demo_result(cfg)
            self.assertEqual(result["status"], "DEMO_SYNTHETIC", result.get("error"))
            analysis = analyze(result)
            self.assertTrue(analysis["measurement_valid"])
            self.assertTrue(any(abs(r["ideal_velocity_um_s"]) > 100 for r in result["xy_mapping"]))
            for row in result["xy_mapping"]:
                self.assertAlmostEqual(row["target_um"], 140. - (row["y_mm"] + 40.) * 3.5)
            json.dumps(result, allow_nan=False)
            if mode == "position_mode":
                with tempfile.TemporaryDirectory() as folder:
                    out, _ = save_result(result, folder)
                    for file in ("xy_mapping.csv", "control.csv", "steps.csv", "curves.png", "summary.json"):
                        self.assertTrue((out / file).exists())

    def test_zero_z_slope_lag_is_undefined(self):
        cfg = config(mapping_file=str(EXAMPLE), mapping_mode=True)
        mapping_defaults(cfg)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "flat.csv"
            path.write_text("distance_mm,target_z_um\n0,10\n10,10\n")
            cfg.mapping_file = str(path)
            result = demo_result(cfg)
        self.assertTrue(all(r["delay_ms"] is None for r in analyze(result)["steps"]))

    def test_mapping_velocity_changes_direction_only_through_zero(self):
        cfg = config(velocity_mode=True)
        validate(cfg)
        controller = MappingVelocityControl(cfg, 3000, 4000, 4000)
        controller.step(0., 0., 200., 0.)
        before, _ = controller.step(.1, 0., 200., 0.)
        changed, _ = controller.step(.11, 0., -200., 0.)
        self.assertGreaterEqual(changed, 0.)
        self.assertLess(changed, before)


class RuntimeFailureTests(unittest.TestCase):
    def setup_scan(self, mode="velocity_mode"):
        cfg = config(mapping_file=str(EXAMPLE), mapping_mode=True, xy_start_x=0., xy_start_y=-40.,
                     xy_end_x=0., xy_end_y=40., xy_speed=100., **{mode: True})
        validate(cfg)
        clock = SimClock()
        motor = SimMotor(clock, 140.)
        stage = SimStage(clock, (0., -40.))
        stage.stops = 0
        stage.stop = lambda: setattr(stage, "stops", stage.stops + 1)
        return cfg, clock, motor, stage, SimReader(stage)

    def test_xy_read_failure_stops_both_and_preserves_samples(self):
        cfg, clock, motor, stage, reader = self.setup_scan()
        original = reader.latest
        def failing():
            if clock.now > 100.2:
                raise TimeoutError("XY 陈旧")
            return original()
        reader.latest = failing
        with patch.object(mapping_run, "time", clock):
            result = run_mapping(motor, stage, reader, MappingCurve.load(EXAMPLE), cfg, SimCancel(clock))
        self.assertEqual(result["status"], "failed")
        self.assertIn("XY", result["error"])
        self.assertEqual(motor.stops, 1)
        self.assertEqual(stage.stops, 1)
        self.assertTrue(result["samples"])

    def test_cancel_stops_both(self):
        cfg, clock, motor, stage, reader = self.setup_scan("position_mode")
        cancel = SimCancel(clock)
        cancel.is_set = lambda: clock.now > 100.2
        with patch.object(mapping_run, "time", clock):
            result = run_mapping(motor, stage, reader, MappingCurve.load(EXAMPLE), cfg, cancel)
        self.assertEqual(result["status"], "cancelled")
        self.assertEqual((motor.stops, stage.stops), (1, 1))

    def test_continuous_stall_no_burst_and_stop(self):
        clock = SimClock()
        motor = SimMotor(clock)
        original = motor.poll
        def poll():
            if 100.09 < clock.now < 100.12:
                clock.now += .03
            original()
        motor.poll = poll
        cfg = config(velocity_mode=True)
        validate(cfg)
        with patch.object(continuous_control, "time", clock):
            result = run_continuous(motor, cfg, SimCancel(clock))
        self.assertEqual(result["status"], "failed")
        self.assertIn("周期", result["error"])
        self.assertEqual(motor.stops, 1)


class FakeNativeFunction:
    def __init__(self, fn):
        self.fn = fn

    def __call__(self, *args):
        return self.fn(*args)


class NativeBoundaryTests(unittest.TestCase):
    def make_dll(self):
        calls = []
        class Dll:
            pass
        dll = Dll()
        def setter(value):
            def fn(handle, axis, output):
                output._obj.value = value
                calls.append((axis, value))
                return 0
            return fn
        def open_eth(ip, output):
            output._obj.value = 1234
            return 0
        dll.ZAux_OpenEth = FakeNativeFunction(open_eth)
        dll.ZAux_SetTimeOut = FakeNativeFunction(lambda *args: 0)
        for name, val in (("GetAtype", 65), ("GetUnits", 10000.), ("GetMpos", 3.), ("GetDpos", 99.), ("GetIfIdle", -1)):
            setattr(dll, "ZAux_Direct_" + name, FakeNativeFunction(setter(val)))
        for name in ("SetSpeed", "SetAccel", "SetDecel", "MoveAbs", "Single_Cancel"):
            setattr(dll, "ZAux_Direct_" + name, FakeNativeFunction(lambda *args, n=name: calls.append((n, args)) or 0))
        dll.ZAux_Close = FakeNativeFunction(lambda handle: 0)
        return dll, calls

    def test_default_feedback_source_typed_handle_and_single_interpolated_move(self):
        dll, calls = self.make_dll()
        stage = XYStage("dummy.dll", dll=dll)
        stage.connect("192.168.0.11")
        self.assertEqual(stage.get_positions(), (3., 3.))
        self.assertEqual(dll.ZAux_Direct_MoveAbs.argtypes[0], ctypes.c_void_p)
        self.assertEqual(stage.handle.value, 1234)
        stage.move((10., 20.), 5., 1000.)
        moves = [c for c in calls if c[0] == "MoveAbs"]
        self.assertEqual(len(moves), 1)
        self.assertEqual(tuple(moves[0][1][2]), (0, 1))
        self.assertEqual(tuple(moves[0][1][3]), (10., 20.))
        stage.disconnect()
        self.assertIsNone(stage.handle.value)

    def test_axis_unit_mismatch_closes_without_moves(self):
        dll, calls = self.make_dll()
        dll.ZAux_Direct_GetUnits = FakeNativeFunction(lambda h, a, output: setattr(output._obj, "value", 1.) or 0)
        stage = XYStage("dummy.dll", dll=dll)
        with self.assertRaisesRegex(RuntimeError, "单位"):
            stage.connect("192.168.0.11")
        self.assertIsNone(stage.handle.value)
        self.assertFalse(any(c[0] == "MoveAbs" for c in calls))

    def test_reader_stale_error_and_lifetime(self):
        stage = type("Stage", (), {"get_positions": lambda self: (1., 2.)})()
        reader = StageReader(stage, .005)
        reader.start()
        deadline = time.perf_counter() + 1.
        while reader.sample is None and time.perf_counter() < deadline:
            time.sleep(.001)
        self.assertEqual(reader.latest()["x_mm"], 1.)
        reader.stop()
        with self.assertRaisesRegex(RuntimeError, "尚无"):
            reader.latest()
        reader.sample = dict(error="", host_abs_s=time.perf_counter() - 1., read_duration_s=.01)
        with self.assertRaises(TimeoutError):
            reader.latest()

    def test_blocked_reader_cannot_be_closed_until_native_call_returns(self):
        entered, release = threading.Event(), threading.Event()
        def positions(stage):
            entered.set()
            release.wait(2.)
            return 1., 2.
        stage = type("Stage", (), {"get_positions": positions})()
        reader = StageReader(stage)
        reader.start()
        self.assertTrue(entered.wait(1.))
        try:
            with self.assertRaisesRegex(RuntimeError, "不能关闭"):
                reader.stop()
            self.assertIsNone(reader.sample)
        finally:
            release.set()
            reader.thread.join(1.)
        self.assertFalse(reader.thread.is_alive())

    def test_stop_reports_busy_native_handle_without_hanging(self):
        dll, _ = self.make_dll()
        stage = XYStage("dummy.dll", dll=dll)
        entered, release = threading.Event(), threading.Event()
        def hold():
            with stage.lock:
                entered.set()
                release.wait(2.)
        thread = threading.Thread(target=hold)
        thread.start()
        self.assertTrue(entered.wait(1.))
        try:
            with self.assertRaisesRegex(RuntimeError, "停止未确认"):
                stage.stop()
        finally:
            release.set()
            thread.join(1.)


if __name__ == "__main__":
    unittest.main()
