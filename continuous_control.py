"""整段梯形参考、速度微调与连续 TRACK；单位 μm、秒。

只使用已核对的 protocol v3 接口，不写入设备参数。
"""
from dataclasses import dataclass, asdict
from contextlib import contextmanager
from functools import wraps
import math
import time


@contextmanager
def high_resolution_timer():
    """Windows 控制循环请求1 ms计时，退出时配对恢复；不能保证实时调度。"""
    import ctypes
    import os
    winmm, enabled = None, False
    if os.name == "nt":
        try:
            winmm = ctypes.WinDLL("winmm")
            enabled = winmm.timeBeginPeriod(1) == 0
        except OSError:
            pass
    try:
        yield enabled
    finally:
        if enabled:
            winmm.timeEndPeriod(1)


def motion_timer(fn):
    @wraps(fn)
    def wrapped(*args, **kwargs):
        with high_resolution_timer():
            return fn(*args, **kwargs)
    return wrapped


DEFAULTS = dict(velocity_mode=False, position_mode=False, control_ms=10.,
                acceleration_um_s2=4000., kp_s=8., trim_percent=20.,
                lead_ms=20., deadband_um=0.3, max_following_um=25.)


def apply_defaults(cfg):
    for key, value in DEFAULTS.items():
        if not hasattr(cfg, key):
            setattr(cfg, key, value)
    return cfg


def mode_name(cfg):
    if getattr(cfg, "velocity_mode", False) and getattr(cfg, "position_mode", False):
        raise ValueError("两种模式请分别测试，不能同时启用")
    return ("VELOCITY_TRIM" if getattr(cfg, "velocity_mode", False) else
            "TRACK_POSITION_LEAD" if getattr(cfg, "position_mode", False) else "GOTO")


def validate_control(cfg):
    apply_defaults(cfg)
    mode = mode_name(cfg)
    for key in DEFAULTS:
        if key.endswith("mode"):
            continue
        if not math.isfinite(getattr(cfg, key)):
            raise ValueError(f"{key} 必须为有限数")
    if not 5 <= cfg.control_ms <= 50:
        raise ValueError("连续控制周期须为5~50 ms，留出150 ms看门狗余量")
    if not 1 <= cfg.acceleration_um_s2 <= 1000000:
        raise ValueError("参考加速度须为1~1000000 μm/s²")
    if not 0 <= cfg.kp_s <= 60 or not 0 <= cfg.trim_percent <= 50:
        raise ValueError("位置增益须为0~60 /s；速度微调须为0~50%")
    if not 0 <= cfg.lead_ms <= 100 or not 0 <= cfg.deadband_um <= 5:
        raise ValueError("位置提前须为0~100 ms；死区须为0~5 μm")
    if not cfg.deadband_um < cfg.max_following_um <= 25:
        raise ValueError("跟随误差停止阈值须大于死区且不超过25 μm")
    if mode != "GOTO" and cfg.sample_ms > cfg.control_ms:
        raise ValueError("连续模式的遥测周期不得超过控制周期")
    return mode


def clamp(x, lo, hi):
    return min(hi, max(lo, x))


@dataclass
class MotionProfile:
    """限制峰值速度，整段仅一段加速和减速；短程自动成为三角形。"""
    start: float
    end: float
    speed: float
    acceleration: float
    deceleration: float

    def __post_init__(self):
        if (not all(math.isfinite(x) for x in asdict(self).values()) or
                min(self.speed, self.acceleration, self.deceleration) <= 0 or self.end == self.start):
            raise ValueError("连续运动参考参数无效")
        self.direction = 1 if self.end > self.start else -1
        distance = abs(self.end - self.start)
        self.peak = min(self.speed, math.sqrt(2 * distance /
                        (1 / self.acceleration + 1 / self.deceleration)))
        self.rise_s = self.peak / self.acceleration
        self.fall_s = self.peak / self.deceleration
        self.cruise_s = max(0., (distance - 0.5 * self.peak *
                                (self.rise_s + self.fall_s)) / self.peak)
        self.duration_s = self.rise_s + self.cruise_s + self.fall_s

    def at(self, t):
        if t <= 0:
            return self.start, 0.
        if t >= self.duration_s:
            return self.end, 0.
        if t < self.rise_s:
            distance, velocity = 0.5 * self.acceleration * t * t, self.acceleration * t
        elif t < self.rise_s + self.cruise_s:
            distance = 0.5 * self.peak * self.rise_s + self.peak * (t - self.rise_s)
            velocity = self.peak
        else:
            remaining = self.duration_s - t
            distance = abs(self.end - self.start) - 0.5 * self.deceleration * remaining * remaining
            velocity = self.deceleration * remaining
        return self.start + self.direction * distance, self.direction * velocity


class VelocityTrim:
    def __init__(self, cfg, profile):
        self.cfg, self.profile = cfg, profile
        self.previous, self.previous_t = 0., 0.

    def update(self, t, position):
        z, v = self.profile.at(t)
        error = z - position
        correction = 0. if abs(error) <= self.cfg.deadband_um else self.cfg.kp_s * error
        bound = self.profile.peak * self.cfg.trim_percent / 100
        correction = clamp(correction, -bound, bound)
        desired = v + correction
        # 扫描中只允许原方向；结束后停住并报告残差，不靠反复倒车消除误差。
        desired = self.profile.direction * clamp(self.profile.direction * desired,
                                                0., self.profile.peak + bound)
        if t >= self.profile.duration_s:
            desired = 0.
        horizon = (self.cfg.control_ms + self.cfg.sample_ms) / 1000 + 0.05
        distance = (self.cfg.max_um - position if desired >= 0 else position - self.cfg.min_um)
        # 软限位前预留控制/遥测延迟及制动距离。
        decel = self.profile.deceleration
        safe_speed = max(0., math.sqrt((decel * horizon) ** 2 +
                                      2 * decel * max(0., distance)) - decel * horizon)
        desired = math.copysign(min(abs(desired), safe_speed), desired)
        dt = max(0., t - self.previous_t)
        rate = self.profile.acceleration if abs(desired) > abs(self.previous) else decel
        commanded = self.previous + clamp(desired - self.previous, -rate * dt, rate * dt)
        # 参考固件16 Hz × 0.08 μm = 1.28 μm/s；更小非零请求会被拒绝。
        if abs(commanded) < 1.28:
            commanded = 0.
        self.previous, self.previous_t = commanded, t
        return dict(reference_um=z, reference_velocity_um_s=v, following_error_um=error,
                    correction_velocity_um_s=correction, command_velocity_um_s=commanded)


def evaluation_plan(profile, count):
    return [dict(step=i, deadline_s=i * profile.duration_s / count,
                 target_um=profile.at(i * profile.duration_s / count)[0])
            for i in range(1, count + 1)]


@motion_timer
def run_continuous(link, cfg, cancel, update=lambda message: None):
    mode = validate_control(cfg)
    if link.latest is None or time.perf_counter() - link.latest["host_abs_s"] > 0.2:
        raise RuntimeError("没有新鲜的位置遥测")
    if link.latest["state"] != 0:
        raise RuntimeError("连续模式必须从 IDLE 启动")
    if mode == "TRACK_POSITION_LEAD" and not link.info.get("caps", 0) & 8:
        raise RuntimeError("设备未声明 TRACK 能力，无法启用方案 B")
    acceleration, deceleration = link.read_motion_limits()
    start = link.latest["position_um"]
    end = start + cfg.step_um * cfg.steps
    if not cfg.min_um <= start <= cfg.max_um or not cfg.min_um <= end <= cfg.max_um:
        raise ValueError("起点或终点超出软限位")
    if mode == "TRACK_POSITION_LEAD" and max(abs(start), abs(end)) > 5000:
        raise ValueError("参考固件 TRACK 绝对位置范围为 ±5000 μm")
    speed = abs(cfg.step_um) * 1000 / cfg.interval_ms
    if speed * (1 + cfg.trim_percent / 100 if mode == "VELOCITY_TRIM" else 1) > link.max_velocity:
        raise ValueError("参考速度及微调余量超过设备速度上限")
    profile = MotionProfile(start, end, speed, min(cfg.acceleration_um_s2, acceleration),
                            min(cfg.acceleration_um_s2, deceleration))
    if profile.peak < 1.28:
        raise ValueError("参考峰值速度低于固件最小运动速度1.28 μm/s")
    # TRACK 固件单次锚点跳变上限100 μm；给调度抖动及提前量留余量。
    if mode == "TRACK_POSITION_LEAD" and profile.peak * cfg.control_ms / 1000 > 50:
        raise ValueError("TRACK 单周期位移过大，请减小连续控制周期")
    if mode == "TRACK_POSITION_LEAD" and profile.peak * cfg.lead_ms / 1000 > cfg.max_following_um:
        raise ValueError("速度×提前时间超过跟随误差阈值，请减小提前量")
    plan = evaluation_plan(profile, cfg.steps)
    commands = []
    t0 = time.perf_counter()
    sample_begin = max(0, len(link.samples) - 20)
    initial_bad, initial_lost = link.decoder.bad_frames, link.lost_samples
    result = dict(config=vars(cfg).copy(), mode=mode, start_um=start, origin_abs_s=t0,
                  planned=plan, evaluation_plan=plan, commands=commands, status="running",
                  profile=asdict(profile), reference_duration_s=profile.duration_s,
                  firmware=link.info, max_velocity_um_s=link.max_velocity,
                  device_acceleration_um_s2=acceleration, device_deceleration_um_s2=deceleration)
    controller = VelocityTrim(cfg, profile)
    period = cfg.control_ms / 1000
    next_tick, display_at = 0, 0.
    try:
        if cancel.is_set():
            result["status"] = "cancelled"
        else:
            if mode == "TRACK_POSITION_LEAD":
                row = dict(kind="TRACK_START", command_position_um=start, command_velocity_um_s=0.)
                commands.append(row)
                key = link.track(start, 0., 0, start=True, record=row)
                link.until(lambda: key not in link.pending, 0.6)
                # 握手不计入运动参考时间；UPDATE 绝不重置轨迹。
                t0 = time.perf_counter()
                result["origin_abs_s"] = t0
            finish = profile.duration_s + cfg.tail_ms / 1000
            while time.perf_counter() - t0 < finish:
                if cancel.is_set():
                    result["status"] = "cancelled"
                    break
                link.poll()
                now = time.perf_counter()
                if now - link.latest["host_abs_s"] > max(0.1, 3 * link.sample_ms / 1000):
                    raise TimeoutError("连续模式位置遥测中断")
                position = link.latest["position_um"]
                if not cfg.min_um <= position <= cfg.max_um:
                    raise RuntimeError("位置越过软件限位")
                t = now - t0
                ref, _ = profile.at(t)
                if abs(ref - position) > cfg.max_following_um:
                    raise RuntimeError("跟随误差超过停止阈值")
                if t >= next_tick * period:
                    late = t - next_tick * period
                    if late >= period:
                        raise RuntimeError("连续控制晚了一整个周期，已停止；不会突发补发")
                    row = dict(kind=mode, scheduled_send_s=next_tick * period, control_time_s=t,
                               reference_um=ref, reference_velocity_um_s=profile.at(t)[1])
                    commands.append(row)
                    if mode == "VELOCITY_TRIM":
                        row.update(controller.update(t, position))
                        link.velocity(row["command_velocity_um_s"], row)
                    else:
                        # 用完整参考在 t+τ 的位置与速度，含末端减速，避免终点多走 vτ。
                        anchor, velocity = profile.at(t + cfg.lead_ms / 1000)
                        row.update(command_position_um=anchor, command_velocity_um_s=velocity,
                                   position_lead_um=anchor - ref, following_error_um=ref - position)
                        link.track(anchor, velocity, (next_tick + 1) & 0xFFFF, record=row)
                    next_tick += 1
                if now - display_at >= 0.1:
                    update(f"{mode}；位置 {position:.3f} μm；误差 {ref - position:.3f} μm")
                    display_at = now
                cancel.wait(0.001)
            else:
                result["status"] = "complete"
    except Exception as exc:
        result["status"], result["error"] = "failed", str(exc)
    finally:
        try:
            key = link.stop()
            if key is not None:
                link.until(lambda: key not in link.pending, 0.6)
        except Exception as exc:
            result["status"], result["stop_error"] = "failed", str(exc)
        result["samples"] = link.samples[sample_begin:]
        result["events"] = [e for e in link.events if e["host_abs_s"] >= t0]
        result["bad_frames"] = link.decoder.bad_frames - initial_bad
        result["lost_samples"] = link.lost_samples - initial_lost
        result["final_error_um"] = end - link.latest["position_um"]
    return result
