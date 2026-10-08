# 连续跟踪方法与接口核对

查阅日期：2026-09-30。资料支撑控制方法，不能证明本设备性能。

1. [Beckhoff Position Control Loop](https://infosys.beckhoff.com/content/1033/tf50x0_tc3_nc_ptp/3443964683.html)：速度前馈与位置跟随误差P修正组合，对应A。PC侧环路延迟不同于驱动内部闭环。
2. [Kollmorgen CSV](https://www.kollmorgen.com/en-us/developer-network/akd-and-ethercat-csv-mode)：主站周期给速度，必要时外层位置环在主站闭合。STM32步进脉冲不等同伺服驱动的速度传感器闭环。
3. [Kollmorgen CSP](https://webhelp.kollmorgen.com/studio/Content/Kollmorgen%20Studio/3%207%20Cyclic%20Synchronous%20Position%20mode%20%28csp%29.htm)：外部连续位置参考，对应B使用现有TRACK锚点/速度参考，不能把普通GOTO重新命名为CSP。
4. [Zaber Stream](https://www.zaber.com/protocol-manual#topic_command_stream)：连续路径排队、段间过渡速度和加速度由控制器规划。本项目只借鉴连续参考原则，没有Zaber硬件。
5. [ZMotion EtherCAT说明](https://www.zmotionglobal.com/support_info_70.html)：MPOS为反馈、DPOS为指令位置。[官方PC手册V2.1.4](https://www.zmotionglobal.com/upload/Zmotion%20PC%20Programming%20Manual%20V2.1.4.pdf) 给出Ethernet、反馈、直线插补、停止及 `ZAux_SetTimeOut(handle,uint32 timems)` API。

本实现用 `Δz≈vZ×τ` 作延迟的一阶预测；τ须实测/对照标定。坡度变化时重新查询预测mapping与速度，不延长曲线边界，不能保证任意高XY速度下无误差。

参考用户指定 `i-ATF PC_0928_original_v3/i-ATF PC_0928_original/i-ATF PC_0925_backup`：

- `pc_app/control/translation_stage.py`：Ethernet、axis0/1、ATYPE65、UNITS10000；原代码读取GetDpos。
- `pc_app/ui/main_window.py::_export_physical_feedforward_trace`：含XY元数据的五列表，只读target_z_um。此版本的导出函数把目标高度写为起终高度的线性采样。
- `pc_app/comm/protocol.py`、`protocol_contract.py`：VELOCITY0x27、TRACK0x29与负载布局；新命令已作字节一致性测试。
- 固件 `Core/Src/app.c`：成功VELOCITY不回ACK，TRACK只START回ACK，UPDATE拒绝通过事件；STOP及运动所有权。
- `Core/Src/tracking.c`：连续参考、P修正、锚点跳变限制、150 ms看门狗。本程序使用TRACK v1，未把v2的50 ms软退化规则误用到原50 ms测试周期。
- `Core/Inc/main.h`：最小速度16 Hz×0.08 μm=1.28 μm/s，TRACK范围±5000 μm。

独立代码不依赖原工程导入路径/PySide6，不写持久参数或烧写固件。参考源码不等于证明设备上正运行相同二进制，实际设备能力、参数、ACK/事件仍须匹配。

固定直线A/B有单次梯形参考；mapping随实际XY和空间坡度变化，必要加减速不可省略。光学清晰度及高速硬件跟焦待实测验收。
