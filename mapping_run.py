"""实际 XY→mapping→Z 跟踪，支持原 GOTO 与独立 A/B 模式。"""
from dataclasses import asdict
import math
import time
from continuous_control import apply_defaults, mode_name, clamp, motion_timer
from xy_mapping import LineMapping, XYVelocityEstimator


MAPPING_DEFAULTS = dict(mapping_file="", mapping_mode=False, stage_ip="192.168.0.11",
                        stage_dll="", xy_source="MPOS", xy_start_x=0., xy_start_y=0.,
                        xy_end_x=10., xy_end_y=0., xy_speed=5., xy_acceleration=1000.,
                        xy_sample_ms=10., xy_tolerance_mm=.05, z_offset_um=0.,
                        z_speed_limit=3000., scan_timeout_s=60.)


def mapping_defaults(cfg):
    apply_defaults(cfg)
    for key, value in MAPPING_DEFAULTS.items():
        if not hasattr(cfg, key):
            setattr(cfg, key, value)
    return cfg


def mapping_line(curve, cfg):
    return LineMapping(curve, (cfg.xy_start_x, cfg.xy_start_y),
                       (cfg.xy_end_x, cfg.xy_end_y), cfg.xy_tolerance_mm, cfg.z_offset_um)


def validate_mapping(cfg):
    mapping_defaults(cfg)
    for key in ("xy_start_x", "xy_start_y", "xy_end_x", "xy_end_y", "xy_speed", "xy_acceleration",
                "xy_sample_ms", "xy_tolerance_mm", "z_offset_um", "z_speed_limit", "scan_timeout_s"):
        if not math.isfinite(getattr(cfg, key)):
            raise ValueError(f"{key} 必须为有限数")
    if cfg.xy_source not in ("MPOS", "DPOS"):
        raise ValueError("XY 来源须为 MPOS 或 DPOS")
    if not .001 <= cfg.xy_speed <= 500 or not .001 <= cfg.xy_acceleration <= 100000:
        raise ValueError("XY 速度须为0.001~500 mm/s；加速度为0.001~100000 mm/s²")
    if not 5 <= cfg.xy_sample_ms <= 50 or not .0001 <= cfg.xy_tolerance_mm <= 10:
        raise ValueError("XY 采样周期为5~50 ms；扫描线容差为0.0001~10 mm")
    if not 1.28 <= cfg.z_speed_limit <= 20000 or not 1 <= cfg.scan_timeout_s <= 3600:
        raise ValueError("Z 速度上限为1.28~20000 μm/s；超时为1~3600 s")
    if cfg.mapping_mode and not cfg.mapping_file:
        raise ValueError("请先导入 mapping 文件")


class MappingVelocityControl:
    def __init__(self, cfg, vmax, acceleration, deceleration):
        self.cfg = cfg
        self.vmax = min(vmax, cfg.z_speed_limit)
        self.acceleration = min(acceleration, cfg.acceleration_um_s2)
        self.deceleration = min(deceleration, cfg.acceleration_um_s2)
        self.previous, self.previous_t = 0., None

    def step(self, t, target, ideal_velocity, position, terminal=False):
        error = target - position
        effective = math.copysign(max(0., abs(error) - self.cfg.deadband_um), error)
        bound = min(100., self.vmax) if terminal else abs(ideal_velocity) * self.cfg.trim_percent / 100
        correction = clamp(self.cfg.kp_s * effective, -bound, bound)
        requested = (0. if terminal else ideal_velocity) + correction
        if abs(requested) > self.vmax + 1e-6:
            raise RuntimeError("所需 Z 速度超过上限，请降低 XY 扫描速度")
        # 制动距离与下一次控制之前可能发生的运动都计入软限位。
        distance = self.cfg.max_um - position if requested > 0 else position - self.cfg.min_um
        horizon = (self.cfg.control_ms + self.cfg.sample_ms) / 1000 + .05
        safe = math.sqrt((self.deceleration * horizon) ** 2 + 2 * self.deceleration * max(0., distance)) - self.deceleration * horizon
        desired = math.copysign(min(abs(requested), safe), requested)
        dt = 0. if self.previous_t is None else max(0., t - self.previous_t)
        # 换向前先减速到零，曲面的真实坡度变化仍允许自然换向。
        if self.previous * desired < 0:
            desired = 0.
        rate = self.acceleration if abs(desired) > abs(self.previous) else self.deceleration
        velocity = self.previous + clamp(desired - self.previous, -rate * dt, rate * dt)
        if abs(velocity) < 1.28:
            velocity = 0.
        self.previous, self.previous_t = velocity, t
        return velocity, correction


@motion_timer
def run_mapping(link, stage, reader, curve, cfg, cancel, update=lambda msg: None):
    validate_mapping(cfg)
    mode = mode_name(cfg)
    line = mapping_line(curve, cfg)
    sample = reader.latest()
    if math.dist((sample["x_mm"], sample["y_mm"]), line.start) > cfg.xy_tolerance_mm:
        raise RuntimeError("XY 尚未到设定起点，请先点击“到扫描起点 XY+Z”")
    if not stage.is_idle() or link.latest["state"] != 0:
        raise RuntimeError("XY 和 Z 必须均静止再开始扫描")
    if time.perf_counter() - link.latest["host_abs_s"] > .2:
        raise TimeoutError("Z 遥测陈旧")
    first, _ = curve.at(curve.distance_mm[0])
    if abs(first + cfg.z_offset_um - link.latest["position_um"]) > max(.5, cfg.deadband_um):
        raise RuntimeError("Z 未对齐 mapping 起点，请先点击“到扫描起点 XY+Z”")
    if any(not cfg.min_um <= z + cfg.z_offset_um <= cfg.max_um for z in curve.target_z_um):
        raise ValueError("mapping 高度（含偏移）超出 Z 软限位")
    vmax = min(link.max_velocity, cfg.z_speed_limit)
    max_slope = max(abs((b - a) / (d - c)) for a, b, c, d in
                    zip(curve.target_z_um, curve.target_z_um[1:], curve.distance_mm, curve.distance_mm[1:])) * line.scale
    if max_slope * cfg.xy_speed * (1 + cfg.trim_percent / 100 if mode == "VELOCITY_TRIM" else 1) > vmax:
        raise ValueError("mapping 坡度×XY 速度（含微调余量）超过 Z 上限，请降低 XY 速度")
    if mode == "TRACK_POSITION_LEAD":
        if not link.info.get("caps", 0) & 8:
            raise RuntimeError("设备未声明 TRACK 能力")
        if max(abs(z + cfg.z_offset_um) for z in curve.target_z_um) > 5000:
            raise ValueError("TRACK 高度超出固件 ±5000 μm 范围")
        if max_slope * cfg.xy_speed * cfg.control_ms / 1000 > 50:
            raise ValueError("TRACK 单周期位移过大，请减小控制周期或降低 XY 速度")
        if max_slope * cfg.xy_speed * cfg.lead_ms / 1000 > cfg.max_following_um:
            raise ValueError("Z速度×提前时间超过误差阈值，请减小提前量")
    acceleration, deceleration = link.read_motion_limits()
    if hasattr(stage, "prepare_motion"):
        stage.prepare_motion(cfg.xy_speed, cfg.xy_acceleration)
        link.poll()
    controller = MappingVelocityControl(cfg, link.max_velocity, acceleration, deceleration)
    estimator = XYVelocityEstimator(window_s=max(.04, cfg.xy_sample_ms / 1000 * 3))
    commands, trace = [], []
    t0 = time.perf_counter()
    sample_begin = max(0, len(link.samples) - 20)
    initial_bad, initial_lost = link.decoder.bad_frames, link.lost_samples
    result = dict(config=vars(cfg).copy(), mode=mode, reference_source="XY_MAPPING",
                  mapping=asdict(curve), mapping_scale=line.scale, start_um=link.latest["position_um"],
                  origin_abs_s=t0, commands=commands, planned=[], evaluation_plan=[], xy_mapping=trace,
                  status="running", firmware=link.info, max_velocity_um_s=link.max_velocity)
    sequence, display_at = 0, 0.
    period = (cfg.interval_ms if mode == "GOTO" else cfg.control_ms) / 1000
    next_send = 0.
    last_xy_seq, end_at, stable = None, None, 0
    try:
        if cancel.is_set():
            result["status"] = "cancelled"
            return result
        if mode == "TRACK_POSITION_LEAD":
            row = dict(kind="TRACK_START")
            commands.append(row)
            key = link.track(first + cfg.z_offset_um, 0., 0, start=True, record=row)
            link.until(lambda: key not in link.pending, .6)
        # 跟踪通道先准备，再开始 XY；连接/导入操作本身不启动扫描。
        t0 = time.perf_counter()
        result["origin_abs_s"] = t0
        if hasattr(stage, "prepare_motion"):
            stage.move(line.end, cfg.xy_speed, cfg.xy_acceleration, prepared=True)
        else:
            stage.move(line.end, cfg.xy_speed, cfg.xy_acceleration)
        # 初次运动调用的传输耗时不当作控制调度迟到；后续不重置期限。
        next_send = time.perf_counter() - t0
        while True:
            if cancel.is_set():
                result["status"] = "cancelled"
                break
            link.poll()
            now = time.perf_counter()
            if now - link.latest["host_abs_s"] > max(.1, 3 * link.sample_ms / 1000):
                raise TimeoutError("Z 遥测中断")
            if now - t0 > cfg.scan_timeout_s:
                raise TimeoutError("XY 扫描超时")
            sample = reader.latest()
            vx, vy = estimator.add(sample)
            raw = line.reference(sample["x_mm"], sample["y_mm"], vx, vy)
            # 已到终点且反馈稳定才进入末尾观察，不用预计速度判断完成。
            if sample["seq"] != last_xy_seq:
                at_end = math.dist((sample["x_mm"], sample["y_mm"]), line.end) <= cfg.xy_tolerance_mm
                stable = stable + 1 if at_end and math.hypot(vx, vy) <= .05 else 0
                if stable >= 3 and end_at is None:
                    end_at = now
                    result["reference_duration_s"] = now - t0
                trace.append(dict(**sample, **raw, vx_mm_s=vx, vy_mm_s=vy,
                                  time_s=sample["host_abs_s"] - t0, xy_source=cfg.xy_source))
                last_xy_seq = sample["seq"]
            offset = min(s["host_abs_s"] - s["mcu_s"] for s in link.samples[-100:])
            z_time = link.latest["mcu_s"] + offset
            common = line.reference(sample["x_mm"], sample["y_mm"], vx, vy,
                                    lead_s=z_time - sample["host_abs_s"])
            position = link.latest["position_um"]
            if not cfg.min_um <= position <= cfg.max_um:
                raise RuntimeError("Z 超出软件限位")
            error = common["target_um"] - position
            if abs(error) > cfg.max_following_um:
                raise RuntimeError("Z 跟随误差超过停止阈值")
            t = now - t0
            if t >= next_send:
                if t - next_send >= period:
                    raise RuntimeError("控制线程迟到一个周期，已停止，未补发")
                current = line.reference(sample["x_mm"], sample["y_mm"], vx, vy,
                                         lead_s=now - sample["host_abs_s"])
                row = dict(kind=mode, scheduled_send_s=next_send, control_time_s=t,
                           xy_time_s=sample["host_abs_s"] - t0, x_mm=sample["x_mm"], y_mm=sample["y_mm"],
                           reference_um=current["target_um"], reference_velocity_um_s=current["ideal_velocity_um_s"],
                           following_error_um=error, feedback_um=position)
                commands.append(row)
                if mode == "GOTO":
                    row["command_position_um"] = raw["target_um"]
                    link.goto(row["command_position_um"], row)
                elif mode == "VELOCITY_TRIM":
                    velocity, correction = controller.step(t, common["target_um"], current["ideal_velocity_um_s"],
                                                            position, terminal=end_at is not None)
                    row.update(command_velocity_um_s=velocity, correction_velocity_um_s=correction)
                    link.velocity(velocity, row)
                else:
                    lead = line.reference(sample["x_mm"], sample["y_mm"], vx, vy,
                                          lead_s=now - sample["host_abs_s"] + cfg.lead_ms / 1000)
                    row.update(command_position_um=lead["target_um"], command_velocity_um_s=lead["ideal_velocity_um_s"],
                               position_lead_um=lead["target_um"] - current["target_um"])
                    sequence = (sequence + 1) & 0xFFFF
                    link.track(row["command_position_um"], row["command_velocity_um_s"], sequence, record=row)
                next_send += period
            if now - display_at >= .1:
                update(f"XY ({sample['x_mm']:.3f}, {sample['y_mm']:.3f}) mm；mapping Z {raw['target_um']:.3f} μm；"
                       f"Z {position:.3f} μm；vZ {raw['ideal_velocity_um_s']:.1f} μm/s；{mode}")
                display_at = now
            if end_at is not None and now - end_at >= cfg.tail_ms / 1000:
                result["status"] = "complete"
                break
            cancel.wait(.001)
        if result["status"] == "complete" and any(r.get("ack_required") and r.get("ack_abs_s") is None for r in commands):
            raise TimeoutError("末尾仍有待确认的 GOTO/START")
    except Exception as exc:
        result["status"], result["error"] = "failed", str(exc)
    finally:
        # STOP 与 DLL 取消分别执行；一台停止失败不能跳过另一台。
        try:
            key = link.stop()
            if key is not None:
                link.until(lambda: key not in link.pending, .6)
        except Exception as exc:
            result["status"], result["stop_error"] = "failed", str(exc)
        try:
            stage.stop()
        except Exception as exc:
            result["status"], result["xy_stop_error"] = "failed", str(exc)
        result["reference_duration_s"] = result.get("reference_duration_s", max(.001, time.perf_counter() - t0))
        result["samples"] = link.samples[sample_begin:]
        result["events"] = [e for e in link.events if e["host_abs_s"] >= t0]
        result["bad_frames"] = link.decoder.bad_frames - initial_bad
        result["lost_samples"] = link.lost_samples - initial_lost
        result["final_error_um"] = curve.target_z_um[-1] + cfg.z_offset_um - link.latest["position_um"]
    return result


def analyze_mapping(result, samples, interpolate, z_gap):
    trace, origin = result["xy_mapping"], result["origin_abs_s"]
    ts, zs = [s["time_s"] for s in samples], [s["position_um"] for s in samples]
    xt = [r["host_abs_s"] - origin for r in trace]
    targets = [r["target_um"] for r in trace]
    speeds = [r["ideal_velocity_um_s"] for r in trace]
    xy_gap = max(.03, result["config"]["xy_sample_ms"] / 1000 * 3)
    rows = []
    for i, xy in enumerate(trace, 1):
        t = xy["host_abs_s"] - origin
        actual = interpolate(ts, zs, t, z_gap)
        error = xy["target_um"] - actual if actual is not None else None
        velocity = xy["ideal_velocity_um_s"]
        lag = 1000 * error / velocity if error is not None and abs(velocity) >= .5 else None
        rows.append(dict(step=i, deadline_s=t, target_um=xy["target_um"], actual_um=actual,
                         error_um=error, delay_ms=lag, absolute_delay_ms=abs(lag) if lag is not None else None,
                         x_mm=xy["x_mm"], y_mm=xy["y_mm"], distance_mm=xy["distance_mm"],
                         ideal_velocity_um_s=velocity, send_time_s=None, send_lateness_ms=None,
                         write_duration_ms=None, ack_time_s=None))
    for s in samples:
        s["reference_um"] = interpolate(xt, targets, s["time_s"], xy_gap)
        s["reference_velocity_um_s"] = interpolate(xt, speeds, s["time_s"], xy_gap)
        s["following_error_um"] = (s["reference_um"] - s["position_um"]
                                   if s["reference_um"] is not None else None)
    duration = result["reference_duration_s"]
    valid = [r for r in rows if r["error_um"] is not None and 0 <= r["deadline_s"] <= duration]
    errors = [r["error_um"] for r in valid]
    travel, measured_time = 0., 0.
    for previous, current in zip(samples, samples[1:]):
        dt = current["time_s"] - previous["time_s"]
        overlap = min(duration, current["time_s"]) - max(0., previous["time_s"])
        if overlap > 0 and current["velocity_um_s"] is not None and dt > 0:
            travel += abs(current["position_um"] - previous["position_um"]) * overlap / dt
            measured_time += overlap
    average = travel / measured_time if measured_time else None
    ripple = [s["velocity_um_s"] - s["reference_velocity_um_s"] for s in samples
              if s["velocity_um_s"] is not None and s["reference_velocity_um_s"] is not None
              and 0 < s["time_s"] <= duration]
    return dict(samples=samples, steps=rows, average_speed_um_s=average, duration_s=duration,
                valid_step_count=len(valid), evaluation_count=len([r for r in rows if 0 <= r["deadline_s"] <= duration]),
                measurement_valid=result["status"] in ("complete", "DEMO_SYNTHETIC") and bool(valid)
                                  and all(r["actual_um"] is not None for r in rows if 0 <= r["deadline_s"] <= duration),
                rms_error_um=math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else None,
                max_abs_error_um=max(map(abs, errors)) if errors else None,
                mean_error_um=(sum(errors) / len(errors)) if errors else None,
                error_variance_um2=(sum((e - sum(errors) / len(errors)) ** 2 for e in errors) /
                                    len(errors)) if errors else None,
                reference_velocity_error_rms_um_s=math.sqrt(sum(v * v for v in ripple) / len(ripple)) if ripple else None,
                lag_definition="1000*(mapping target - reported Z)/local signed ideal Z velocity; undefined below 0.5 um/s",
                xy_position_source=result["config"]["xy_source"])
