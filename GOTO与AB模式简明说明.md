# GOTO、A、B 三种 Z 轴控制方法

## 一、三种方法的核心区别

| 方法 | 发送内容 | 核心含义 |
|---|---|---|
| GOTO | 目标位置 | 告诉电机“去哪里” |
| A 速度微调 | 理论速度 + 误差修正 | 告诉电机“以多快速度走” |
| B 位置提前 | 提前位置 + 速度 | 告诉电机“未来应该在哪里” |

## 二、GOTO 模式

### 核心公式

```text
 Ztarget(i) = Zstart + i × step_um
 i = 1, 2, ..., steps
```

```text
 发送时间(i) = (i - 1) × interval_ms
 总位移 = step_um × steps
```

PC 端按 `interval_ms` 发送目标位置，速度、加速度和减速度主要由 STM32 固件执行。

### 参数影响

| 参数 | 影响 |
|---|---|
| `step_um` | 每次目标位置的增量 |
| `steps` | 目标点数量；总位移为 `step_um × steps` |
| `interval_ms` | GOTO 指令间隔；越小，目标变化越快 |
| `min_um/max_um` | 软件允许的 Z 轴范围 |
| `tail_ms` | 最后一条 GOTO 后的等待时间 |

GOTO 适合单点定位和分段移动。目标点变化太快时，电机可能还没完成上一个目标就收到下一个目标。

## 三、A 模式：速度微调

### 核心公式

```text
 T = step_ms × steps / 1000
 Zref(t) = Zstart + Zcurve(t / T) - Zcurve(0)
 Vref(t) = dZref(t) / dt
```

```text
 e = Zref - Zactual
 e_eff = sign(e) × max(|e| - deadband_um, 0)
 Vcorr = kp_s × e_eff
 |Vcorr| ≤ |Vref_peak| × trim_percent / 100
 Vcommand = Vref + Vcorr
```

A 模式是“理论速度前馈 + 位置误差修正”，不是直接位置控制。

### 终点提前刹车

不能等到终点才发送 0 速度，因为电机需要减速距离：

```text
 d = |Zend - Zactual|
 Vstop = sqrt(2 × deceleration × d)
 |Vcommand| ≤ Vstop
```

### 参数影响

| 参数 | 增大后的主要影响 |
|---|---|
| `step_ms × steps` | 总时间变长、理论速度降低，更容易跟随，过冲通常减小 |
| `control_ms` | 控制更新变慢，延迟和刹车距离增加 |
| `kp_s` | 误差修正更强；过大可能振荡或过冲 |
| `trim_percent` | 允许更大的速度修正；过大可能运动激进 |
| `deadband_um` | 小误差不修正；过大时终点误差可能增加 |
| `acceleration_um_s2` | 加减速更快；过大可能冲击和过冲增加 |
| `max_following_um` | 正常跟随误差阈值，不是最终位置精度参数 |

典型问题链路：

```text
 速度过高 → 跟不上 → 误差增大 → 追赶 → 终点过冲
```

## 四、B 模式：位置提前

### 核心公式

```text
 tlead = t + lead_ms / 1000
 Zlead = Zref(tlead)
 Vlead = Vref(tlead)
```

每个控制周期发送：

```text
 TRACK(Zlead, Vlead)
```

`lead_ms` 用来补偿通信、处理和电机响应延迟。

### 参数影响

| 参数 | 增大后的主要影响 |
|---|---|
| `lead_ms` | 补偿更多延迟；过大可能超前和过冲 |
| `control_ms` | 更新变慢，跟随响应变慢 |
| `acceleration_um_s2` | 影响速度变化和跟随能力 |
| `max_following_um` | 允许的当前位置误差范围 |
| `step_ms × steps` | 总时间变长，曲线速度降低，更容易跟随 |

`lead_ms` 太小会滞后，合适可以补偿延迟，太大会超前。

## 五、调参顺序

### GOTO

1. 确认 `step_um × steps` 在软限位内；
2. 目标跳变太快时增大 `interval_ms`；
3. 增加 `tail_ms` 等待真正停稳。

### A

1. 先增加总时间；
2. 跟不上时适当增大 `kp_s` 或 `trim_percent`；
3. 过冲时降低 `kp_s`、`trim_percent` 或曲线速度；
4. 检查 `endpoint_braking_commands`。

### B

1. 先使用较小 `lead_ms`；
2. 滞后时逐步增加；
3. 超前或过冲时减小；
4. 曲线过快时增加总时间。

## 六、结果字段

| 字段 | 含义 |
|---|---|
| `final_position_um` | STOP 完成后的真实停稳位置 |
| `final_error_um` | 最终目标位置减去真实停稳位置 |
| `peak_following_error_um` | 运行期间最大跟随误差 |
| `following_recovery_commands` | A 模式进入追赶的次数 |
| `endpoint_braking_commands` | A 模式提前刹车的次数 |
