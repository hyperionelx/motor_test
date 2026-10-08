# STM32 马达与 XY mapping 跟焦测试

入口 `motor_test.py`。保留 GOTO，新增两个分别勾选的连续模式、ZMotion 位移台连接、mapping 导入及实际 XY 驱动的目标 Z。原工程仅用于核对接口，未修改原 pc_app 或固件。

## 启动

64位 Windows、Python 3.10+（含 Tkinter）：

```powershell
cd "代码所在目录"
python -m pip install -r requirements.txt
python motor_test.py
```

也可双击 `start_test.cmd`，优先使用Python launcher，再尝试Codex自带Python；不自动安装依赖。Codex Python若缺pyserial，连接STM32前需使用同一Python安装requirements。

ZMotion需原工程的 **zauxdll64.dll 与同目录 zmotion.dll**。可“选择 DLL”；程序也尝试当前 `dll/` 与指定的 v3 工程目录。代码包不混入其他来源的DLL。

## 快速操作

1. 关闭原上位机的同串口连接；填写实际允许的Z软限位，连接STM32。
2. 填写位移台IP（原默认 `192.168.0.11`）、DLL路径、XY位置来源，点击“连接位移台”。axis0=X、axis1=Y；核验ATYPE=65、UNITS=10000，不归零、不修改单位。
3. 原物理前馈页面点击“导出最近红点轨迹”保存 `.txt`；本程序“导入 mapping”。自动预览，并把文件XY起终点填入可编辑输入框。
4. 确认XY起终点、扫描线速度、XY加速度、Z偏移和软限位。完整mapping距离区间**按比例**对应这条线；改变线长会改变空间坡度。文件Z是绝对目标高度，默认偏移0。
5. 分别勾选A或B；两框互斥。均不选使用GOTO作基线。
6. “到扫描起点 XY+Z”，等待静止和Z对齐；再“开始测试”。**开始测试会驱动XY沿设定直线移动**，Z同时跟随。连接、导入、预览不启动运动。
7. “停止”请求停止Z和XY，保存已有数据。完成后的三图显示在窗口，输出在 `results/时间戳/`。

默认 **MPOS反馈位置**；DPOS是控制器指令位置，用作与原程序对照的显式选项，不能称为编码器实测。位置来源、XY读取周期、IP/DLL改变后请重连XY；串口或STM32遥测周期改变后重连STM32。

## Mapping 格式

原v3页面导出的UTF-8 BOM/制表符五列表：

```text
# start_xy_mm 0 -40
# end_xy_mm 0 40
distance_mm  x_mm  y_mm  target_z_um  actual_z_um
0            0     -40   140          140.4
40           0       0     0            0.1
80           0      40  -140         -139.9
```

也接受同名表头的CSV/TSV/空白分隔文件，至少需 `distance_mm` 和 `target_z_um`。**只用target_z_um**，忽略实际Z/FES；距离非负、严格递增、至少两点；高度有限。actual_z_um可空白。可导入 `examples/mapping_line.txt` 测试。

截图、实际红点单列、原始FES扫描、含重复/回程距离的时间诊断文件不能唯一指定目标mapping，会报告错误，不猜测列含义。

## 实际XY生成参考

实际XY投影到手动扫描线，按比例对应到文件距离，以分段线性插值得到 `Z_target`。有限窗口回归估计实际沿线速度：

```text
vZ_ideal = mapping局部空间斜率 × 实测XY沿线速度
```

单位为μm/mm、mm/s、μm/s。mapping模式不使用每步位移/步数生成目标或决定完成时刻。XY通过一次协调直线插补命令移动，完成由反馈到达终点及估计速度稳定判定。

- **A：速度前馈＋位置误差速度微调。** 持续 `MOTOR_VELOCITY(0x27)`，加上有死区、百分比限幅的比例位置修正，限制加减速和制动距离，换向先过零。mapping尾段用小幅位置修正收敛终点。
- **B：位置提前＋连续TRACK。** 一次 `TRACK START(0x29)` 后持续UPDATE；按实测沿线速度和提前时间查询预测mapping位置及匹配Z速度。固件连续参考/位置环执行跟踪，边界不外推；没有TRACK能力的设备拒绝启用。
- **均不选：GOTO基线。** 按原指令周期发位置；mapping模式把设置位置替换为实际XY对应Z，仍可能有周期内启停。

默认连续控制10 ms、XY读取10 ms、STM32遥测5 ms；A增益8/s、死区0.3 μm、微调20%；B提前20 ms。提前量是待标定参数，不代表已测定设备延迟。

不启用mapping时，A/B用全程一次加速/匀速/减速的梯形参考，短程用三角形；峰值保持所选目标速度。例如200 μm、200 μm/s、4000 μm/s²总时长1.05 s，原GOTO仍为1 s。评估按各自真实参考时间，导出明确记录。固定直线A尾段停住并报告残差，不追加第二次GOTO。

非线性mapping的坡度变化与XY真实加減速要求Z速度变化，无法强制任意曲面全程恒速。目标是消除每条短GOTO的人为启停。坡度/曲率/延迟超过Z能力时需降低XY速度，不能无限加大补偿。

## 停止与测量

XY读取在独立线程，不堵塞STM32控制；DLL命令等待超时设为50 ms。XY数据超过100 ms或读取耗时过长、离开扫描线、Z遥测中断、误差/软限位越界、调度迟到整周期、固件报警/限位或用户停止，会中止扫描并保存数据。两台设备的停止错误分别记录。若原生调用异常不返回，停止会报告未确认；原生读取未退出时不关闭其句柄。Python/Windows不保证硬实时。

已核对成功的VELOCITY与TRACK UPDATE **不回ACK**；仍关联错误响应。GOTO、TRACK START、STOP等核对会话/令牌/ACK。协议v3：COBS、CRC16、session/token、float32小端。不写设备持久参数。

Z来自STM32 `STREAM_DATA.position_um`，参考固件为马达脉冲计数，不能等同独立位移反馈。本程序也不读取相机清晰度，不能独立证明光学焦点保持。

固定直线保留原延时定义。mapping按XY采样时刻插值Z，计算 `1000×(目标Z−读取Z)/局部带符号理想Z速度`；正值落后，负值超前；理想速度小于0.5 μm/s时延时为空。MCU和PC以最小收包偏移对齐，残留未知传输延迟；不是硬同步延时测量。

## 输出与测试

保留 `steps.csv`、`telemetry.csv`、`summary.json`、`raw.json`、`curves.png/pdf`。新增 `control.csv` 实际发送指令及修正、`xy_mapping.csv` 真实XY时间戳/位置/读取耗时/目标/速度/来源。摘要包含模式、参考时长、RMS/最大误差、速度误差、源文件SHA256和终点残差。mapping平均速度=有效时间内Z绝对位移累计/有效时间；等效延时始终用局部速度。

```powershell
python -m unittest discover -s . -v
python verify_gui.py
python motor_test.py --demo --velocity-mode
python motor_test.py --demo --position-mode
python motor_test.py --demo --mapping-mode --mapping-file examples/mapping_line.txt --xy-start-x 0 --xy-start-y -40 --xy-end-x 0 --xy-end-y 40 --xy-speed 100 --velocity-mode
```

离线输出标为 `DEMO_SYNTHETIC`，不等价于固件或机械。已检查实际DLL加载与所需导出符号，未打开真实设备、未驱动马达。

硬件A/B对照用同一文件、起点、XY速度，依次GOTO/A/B；先低速再提高。B可对比提前0/10/20 ms，A可对比微调10%/20%；比较RMS/最大误差、匀速段波动、终点残差和失败/限位/丢样。高速光学跟焦需另以相机/FES或独立位移反馈验收。

业界方法见 `CONTROL_METHODS.md`；本次编程prompt见 `PROGRAMMING_PROMPT.txt`。
