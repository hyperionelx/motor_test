"""带加减速的离线设备模型；结果不能代替硬件验收。"""
import math
from unittest.mock import patch
import continuous_control
import mapping_run
from continuous_control import MotionProfile, clamp
from xy_mapping import MappingCurve


class SimClock:
    now = 100.

    def perf_counter(self):
        return self.now


class SimCancel:
    def __init__(self, clock):
        self.clock = clock

    def is_set(self):
        return False

    def wait(self, seconds):
        self.clock.now += seconds


class SimMotor:
    sample_ms = 5.
    max_velocity = 20000.
    info = dict(proto=3, caps=31, fw_ver=67840)
    lost_samples = 0

    def __init__(self, clock, position=0.):
        self.clock, self.position = clock, position
        self.decoder = type("DecoderCounters", (), {"bad_frames": 0})()
        self.velocity_now = self.requested = 0.
        self.last_tick = clock.now
        self.next_sample = clock.now
        self.anchor = None
        self.gotopos = None
        self.samples, self.events, self.pending = [], [], {}
        self.stops = 0
        self.sample(clock.now - .005)
        self.sample(clock.now)

    def sample(self, t):
        self.latest = dict(host_abs_s=t + .002, mcu_s=t - 90., position_um=self.position,
                           seq=len(self.samples), state=0 if abs(self.velocity_now) < .5 else 1,
                           error=0, flags=8, overflow=0)
        self.samples.append(self.latest.copy())

    def read_motion_limits(self):
        return 20000., 20000.

    def poll(self):
        while self.last_tick + .001 <= self.clock.now + 1e-9:
            self.last_tick += .001
            desired = self.requested
            if self.anchor:
                z, v, at = self.anchor
                desired = v + clamp(20 * (z + v * (self.last_tick - at) - self.position), -300, 300)
            if self.gotopos is not None:
                error = self.gotopos - self.position
                desired = math.copysign(min(20000., math.sqrt(2 * 20000 * abs(error))), error) if abs(error) > .1 else 0.
            self.velocity_now += clamp(desired - self.velocity_now, -20., 20.)
            self.position += self.velocity_now * .001
            if self.last_tick + 1e-9 >= self.next_sample:
                self.sample(self.last_tick)
                self.next_sample = self.last_tick + self.sample_ms / 1000

    def record(self, row, ack=False):
        if row is not None:
            row.update(sent_abs_s=self.clock.now, write_end_abs_s=self.clock.now,
                       ack_abs_s=self.clock.now if ack else None, token=1, write_ok=True, ack_required=ack)

    def velocity(self, velocity, record=None):
        self.requested = velocity
        self.anchor = self.gotopos = None
        self.record(record)

    def track(self, position, velocity, sequence, start=False, record=None):
        self.anchor = (position, velocity, self.clock.now)
        self.gotopos = None
        self.record(record, start)
        return None

    def goto(self, position, record=None):
        self.gotopos, self.anchor = position, None
        self.record(record, True)

    def until(self, predicate, timeout):
        if not predicate():
            raise TimeoutError("模拟响应未完成")

    def stop(self):
        self.stops += 1
        self.requested = self.velocity_now = 0.
        self.anchor = self.gotopos = None
        return None


def continuous_demo(cfg):
    clock = SimClock()
    motor = SimMotor(clock)
    with patch.object(continuous_control, "time", clock):
        result = continuous_control.run_continuous(motor, cfg, SimCancel(clock))
    result["simulation_model"] = "bounded acceleration discrete motor; not firmware/physical measurement"
    if result["status"] == "complete":
        result["status"] = "DEMO_SYNTHETIC"
    return result


class SimStage:
    def __init__(self, clock, start):
        self.clock, self.start = clock, start
        self.profile = None

    def is_idle(self):
        return self.profile is None or self.clock.now - self.origin >= self.profile.duration_s

    def move(self, end, speed, acceleration):
        self.end, self.origin = end, self.clock.now
        length = math.dist(self.start, end)
        self.unit = tuple((b - a) / length for a, b in zip(self.start, end))
        self.profile = MotionProfile(0., length, speed, acceleration, acceleration)

    def stop(self):
        pass


class SimReader:
    def __init__(self, stage, sample_ms=10):
        self.stage, self.sample_ms = stage, sample_ms

    def latest(self):
        stage = self.stage
        t = math.floor((stage.clock.now - 100) * 1000 / self.sample_ms + 1e-6) * self.sample_ms / 1000 + 100
        along = stage.profile.at(t - stage.origin)[0] if stage.profile else 0.
        position = tuple(a + u * along for a, u in zip(stage.start, stage.unit)) if stage.profile else stage.start
        return dict(seq=round((t - 100) * 1000 / self.sample_ms), host_abs_s=t,
                    receive_abs_s=t, read_duration_s=0., x_mm=position[0], y_mm=position[1], error="")


def mapping_demo(cfg):
    curve = MappingCurve.load(cfg.mapping_file)
    clock = SimClock()
    motor = SimMotor(clock, curve.target_z_um[0] + cfg.z_offset_um)
    stage = SimStage(clock, (cfg.xy_start_x, cfg.xy_start_y))
    reader = SimReader(stage, cfg.xy_sample_ms)
    with patch.object(mapping_run, "time", clock):
        result = mapping_run.run_mapping(motor, stage, reader, curve, cfg, SimCancel(clock))
    result["simulation_model"] = "synthetic XY/Motor dynamics; not hardware measurement"
    if result["status"] == "complete":
        result["status"] = "DEMO_SYNTHETIC"
    return result
