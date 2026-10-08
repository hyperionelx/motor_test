# 软件验证结果

2026-09-30，使用Codex自带64位Python，在当前独立测试程序完成验证。

命令：

```powershell
python -m unittest discover -s code -v
python code/verify_gui.py
```

结果：**36项unittest全部通过，无跳过**；隐藏窗口GUI验证通过。

覆盖：

- 原GOTO/HELLO/PING/STOP/参数/遥测的协议一致性及CRC、会话、ACK、拒绝、时间回绕、丢帧/报警。
- VELOCITY、TRACK START/UPDATE与用户指定v3工程的实际协议字节一致；成功实时命令无ACK不误超时，错误响应仍捕获。
- 梯形/三角形参考、正反向、速度修正/死区/限幅、换向过零、整周期调度迟到停止且不补发。
- 原五列/BOM/CSV mapping、目标列选择、非等距/非线性插值、斜线/反向XY投影、线长缩放、单位换算、实际XY速度变化、边界不外推、零理想速度时延时留空。
- XY读取失败/陈旧、取消、双设备停止、单位校验失败、64位句柄、一次两轴插补调用、原生调用阻塞时不关闭句柄/停止报告未确认。
- 窗口中的A/B勾选互斥、mapping导入自动填XY元数据、预览、mapping演示、三图与数据导出、有序关闭。

额外读取验证：原工程 `physical_feedforward_trace_track.txt` 的 **3287点**成功导入（起点(0,-40)、终点(0,40) mm），用于mapping模拟跟踪，状态DEMO_SYNTHETIC、分析有效。没有把实际红点当目标曲线。

实际 `zauxdll64.dll` 可加载，Ethernet、MPOS/DPOS、单位/轴类型、插补、停止、命令超时等所需导出符号及ctypes签名均已绑定；**没有连接真实位移台**。

全部新控制演示标记DEMO_SYNTHETIC。离散模拟马达不是实际固件，不据此宣称某模式更优。`verification/continuous/`、`verification/mapping/` 和新增GUI演示是合成数据；旧 `results/` 实验数据保留。

未验证：真实串口、真实XY/Z联动、设备当前固件二进制、持续高XY速度的跟随误差、相机/FES的光学焦点保持。硬件验收仍需完成。代码包不包含厂商DLL；用户需使用现有原工程DLL及实际机械限位。
