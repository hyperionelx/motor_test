# AB 模式控制原理与参数影响

本文说明当前 `motor_test.py` 中 A/B 模式的实际控制逻辑，重点针对“导入 mapping 曲线、不联动 XY 的曲线测试”。

## 1. 模式区别

| 模式 | 控制对象 | 周期发送内容 | 特点 |
|---|---|---|---|
| A：速度微调 | Z 速度 | `VELOCITY(速度)` | 理论速度前馈，加位置误差修正 |
| B：位置提前 | Z 位置和速度 | `TRACK(提前位置, 速度)` | 直接给固件未来位置和速度 |

A 不是直接位置控制，终点必须通过速度反馈、减速和终点收敛完成。B 由 `TRACK` 的最终位置决定，通常更适合位置精度要求高的场景。

## 2. 导入曲线的公共参考公式

设：

- `t`：当前测试时间，单位 s；
- `T`：曲线总时间，单位 s；
- `s0`、`s1`：曲线起止距离，单位 mm；
- `z_curve(s)`：mapping 曲线在距离 `s` 处的 Z 值，单位 μm；
- `Zstart`：测试开始时的实际 Z 位置。

曲线进度：

```text
r = clamp(t / T, 0, 1)
s = s0 + (s1 - s0) * r
```

理论目标位置：

```text
Zref(t) = Zstart + z_curve(s) - z_curve(s0)
```

理论速度：

```text
Vref(t) = slope(s) * (s1 - s0) / T
```

其中 `slope(s)` 单位为 μm/mm。`t >= T` 后，理论位置保持最终值 `Zend`，理论速度为 0。

曲线总时间为：

```text
T = step_ms * steps / 1000
```

曲线文件决定 Z 总行程，`step_ms` 和 `steps` 决定执行快慢。

## 3. A 模式：速度微调

### 3.1 位置误差

```text
e = Zref - Zactual
```

`e > 0` 表示实际位置落后，`e < 0` 表示实际位置超前。

死区处理：

```text
e_eff = sign(e) * max(|e| - deadband_um, 0)
```

比例修正请求：

```text
Vcorr_request = kp_s * e_eff
```

### 3.2 正常速度指令

正常修正上限为：

```text
Vcorr_limit = |Vref_peak| * trim_percent / 100
Vcorr = clamp(Vcorr_request, -Vcorr_limit, Vcorr_limit)
Vdesired = Vref + Vcorr
```

当 `|e| > max_following_um` 时，当前代码进入追赶状态，临时允许更大的比例修正，但仍受固件最大速度限制。若：

```text
|e| > max(100 μm, 4 * max_following_um)
```

则停止测试，作为安全保护。

### 3.3 终点提前刹车

不能等到 `t = T` 才发送 0 速度，因为电机仍需要减速距离。设终点方向为 `D`（正向为 1，反向为 -1）：

```text
d = D * (Zend - Zactual)
d_effective = max(0, d - |Vprevious| * control_period)
Vend_limit = sqrt(2 * deceleration * d_effective)
```

当当前指令朝向终点且速度超过 `Vend_limit` 时，提前降低速度：

```text
|Vdesired| <= Vend_limit
```

这部分用于避免曲线已经到达终点、但电机仍带有较大速度而冲过终点。

### 3.4 速度斜率和反向

发送前还会限制每周期的速度变化：

```text
ΔVmax = acceleration * Δt       加速时
ΔVmax = deceleration * Δt       减速时
```

因此速度不会瞬间跳变。反向时要求先经过 0：

```text
正速度 -> 0 -> 负速度
```

低于固件最小有效速度的指令会被置为 0，避免无效微小速度导致终点继续漂移。

## 4. B 模式：位置提前

提前时间：

```text
tlead = t + lead_ms / 1000
```

提前位置和速度：

```text
Zlead = Zref(tlead)
Vlead = Vref(tlead)
```

每个控制周期发送：

```text
TRACK(Zlead, Vlead)
```

提前量用于补偿通信、执行器和固件响应延迟。末端会限制在 `T`，因此最终位置为 `Zend`、速度为 0。

## 5. 参数对运动曲线的影响

### `step_ms`、`steps`

总时间：

```text
T = step_ms * steps / 1000
```

- 总时间减小：理论速度增大，跟随误差和过冲风险增大；
- 总时间增大：运动变慢，通常更容易跟随；
- `steps` 增大且总时间不变：采样更密，轨迹更细；
- `step_ms` 增大：总时间变长，曲线速度降低。

### `control_ms`

连续控制发送周期，单位 ms。

- 减小：响应更快，但串口和 CPU 负担增加；
- 增大：通信压力降低，但延迟和终点刹车距离增大。

控制延迟距离近似为：

```text
d_delay = |V| * control_ms / 1000
```

### `acceleration_um_s2`

软件参考加减速度，实际使用值为：

```text
a_used = min(acceleration_um_s2, firmware_acceleration)
d_used = min(acceleration_um_s2, firmware_deceleration)
```

- 增大：加减速更快，跟随能力增强，但运动冲击更明显；
- 减小：运动更平滑，但可能来不及跟随或刹车；
- 该参数直接影响终点提前刹车公式。

### `kp_s`

A 模式位置误差增益：

```text
Vcorr_request = kp_s * e_eff
```

- 增大：纠偏更强，跟随误差减小；
- 过大：容易出现修正过强或振荡；
- 减小：更平滑，但可能产生较大滞后。

过冲时不应只增大 `kp_s`，应先确认总时间和终点刹车是否足够。

### `trim_percent`

A 模式正常修正速度的比例上限：

```text
Vcorr_limit = Vref_peak * trim_percent / 100
```

- 增大：纠偏能力增强；
- 过大：前馈速度和修正速度叠加更激进；
- 减小：更接近理论速度，但跟随误差更难消除。

### `deadband_um`

A 模式误差死区，单位 μm。

- 增大：小误差不修正，运动更平滑，但最终误差可能变大；
- 减小：终点更精确，但可能频繁发送微小修正。

### `max_following_um`

正常跟随误差阈值，单位 μm。

当前 A 模式超过该值会进入追赶，不会立即停止；超过更高的安全阈值才停止。

- 增大：允许更大的暂时滞后，减少误停；
- 过大：可能掩盖机械或通信异常；
- 减小：保护更严格，但更容易进入追赶。

它不是最终位置精度参数，不能直接消除过冲。

### `lead_ms`

B 模式位置提前量，单位 ms。

- 增大：补偿更多延迟，跟随可能更及时；
- 过大：可能超前并产生过冲；
- 减小：运动更保守，但可能滞后。

该参数只影响 B 模式。

## 6. 调参建议

### A 模式过冲

建议按以下顺序处理：

1. 增大 `step_ms * steps` 的总时间；
2. 确认 `acceleration_um_s2` 不高于真实减速度能力；
3. 适当减小 `kp_s`；
4. 适当减小 `trim_percent`；
5. 保持 `control_ms` 较小；
6. 查看 `endpoint_braking_commands` 是否大于 0。

### A 模式走不完

1. 增大总时间；
2. 适当增大 `kp_s`；
3. 适当增大 `trim_percent`；
4. 查看 `following_recovery_commands`；
5. 查看 `peak_following_error_um` 是否接近安全阈值。

### B 模式滞后或超前

1. 滞后时适当增大 `lead_ms`；
2. 超前或过冲时减小 `lead_ms`；
3. 减小 `control_ms`；
4. 增大总时间，降低理论速度。

## 7. 结果字段

| 字段 | 含义 |
|---|---|
| `final_position_um` | STOP 完成后实际停稳的位置 |
| `final_error_um` | 最终目标位置减去实际停稳位置 |
| `peak_following_error_um` | 测试过程中的最大跟随误差 |
| `following_recovery_commands` | A 模式进入追赶的控制次数 |
| `endpoint_braking_commands` | A 模式触发提前刹车的控制次数 |
| `events` | 固件事件、复位或报警 |
| `error` | 测试提前结束的原因 |

如果 `endpoint_braking_commands > 0`，说明终点提前刹车已经介入；如果 `following_recovery_commands` 很大，说明曲线速度过快或 A 参数纠偏不足。
