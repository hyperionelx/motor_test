import sys
import platform
import ctypes
import os
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QGroupBox, QLabel, QLineEdit, QPushButton, 
    QDoubleSpinBox, QTextEdit, QMessageBox
)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFont

# ======================= 0. 固化的硬件参数常量 =======================
HARDWARE_CONFIG = {
    # 厂家确认该位移台使用 EtherCAT 位置轴。
    # 65 = EtherCAT CSP（周期同步位置）模式。
    "ATYPE": 65,
    # UNITS 由控制器端配置为 10000。本程序只读取校验，不下发修改，
    # 避免 Demo 启动时覆盖行程换算参数。
    "UNITS": 10000.0,
    "ACCEL": 1000.0,    # 加速度
    "DECEL": 1000.0,    # 减速度
    "DEFAULT_SPEED": 5.0  # 默认运行速度 5 mm/s (安全保守初值)
}

# ======================= 1. DLL 加载 =======================
systype = platform.system()
zauxdll = None

try:
    if systype == 'Windows':
        # 不依赖 IDE 的当前工作目录。zmc_python.py 位于“单轴运动例程”，
        # DLL 位于其上一级目录；同时保留当前目录作为兼容路径。
        base_dir = os.path.dirname(os.path.abspath(__file__))
        dll_candidates = [
            os.path.join(base_dir, '..', 'zauxdll64.dll'),
            os.path.join(base_dir, 'zauxdll64.dll'),
            os.path.join(os.getcwd(), 'zauxdll64.dll'),
        ]
        if platform.architecture()[0] == '64bit':
            dll_path = next((p for p in dll_candidates if os.path.isfile(p)), None)
            if dll_path is None:
                raise FileNotFoundError('找不到 zauxdll64.dll')
            zauxdll = ctypes.WinDLL(os.path.abspath(dll_path))
        else:
            raise OSError('当前程序只提供 64 位 DLL，请使用 64 位 Python')
    elif systype == 'Darwin':
        zauxdll = ctypes.CDLL('./zmotion.dylib')
    elif systype == 'Linux':
        zauxdll = ctypes.CDLL('./libbzmotion.so')
except Exception as e:
    print(f"Warning: Failed to load motion dll: {e}")


# ======================= 2. 控制器通信层 =======================
class ZMCWrapper:
    def __init__(self):
        self.handle = ctypes.c_void_p()
        self.is_connected = False
        self.axis_ready = {}

    def connect(self, ip):
        if not zauxdll:
            return -999
        if self.handle.value is not None:
            self.disconnect()
        ip_bytes = ip.encode('utf-8')
        p_ip = ctypes.c_char_p(ip_bytes)
        ret = zauxdll.ZAux_OpenEth(p_ip, ctypes.pointer(self.handle))
        self.is_connected = (ret == 0)
        return ret

    def disconnect(self):
        if not zauxdll or not self.handle.value:
            self.is_connected = False
            return 0
        ret = zauxdll.ZAux_Close(self.handle)
        self.handle = ctypes.c_void_p()
        self.is_connected = False
        self.axis_ready.clear()
        return ret

    def init_axis_hardware(self, iaxis, speed):
        """配置轴类型和运动参数，并返回每一项的控制器错误码。"""
        results = {}

        if HARDWARE_CONFIG["ATYPE"] is not None:
            results["ATYPE"] = zauxdll.ZAux_Direct_SetAtype(
                self.handle, iaxis, HARDWARE_CONFIG["ATYPE"]
            )

        # 不调用 SetUnits：厂家要求保留控制器中已有的 UNITS=10000 设置。
        results["ACCEL"] = zauxdll.ZAux_Direct_SetAccel(
            self.handle, iaxis, ctypes.c_float(HARDWARE_CONFIG["ACCEL"])
        )
        results["DECEL"] = zauxdll.ZAux_Direct_SetDecel(
            self.handle, iaxis, ctypes.c_float(HARDWARE_CONFIG["DECEL"])
        )
        results["SPEED"] = zauxdll.ZAux_Direct_SetSpeed(
            self.handle, iaxis, ctypes.c_float(speed)
        )

        # SetAtype 返回 0 不代表控制器最终采用了该轴类型，必须复读确认。
        atype_ret, actual_atype = self.get_atype(iaxis)
        results["ATYPE_VERIFY"] = atype_ret if actual_atype == HARDWARE_CONFIG["ATYPE"] else -1001
        results["ATYPE_ACTUAL"] = actual_atype
        self.axis_ready[iaxis] = (
            results.get("ATYPE", 0) == 0
            and atype_ret == 0
            and actual_atype == HARDWARE_CONFIG["ATYPE"]
        )
        return results

    def get_atype(self, iaxis):
        value = ctypes.c_int()
        ret = zauxdll.ZAux_Direct_GetAtype(
            self.handle, iaxis, ctypes.byref(value)
        )
        return ret, value.value

    def get_units(self, iaxis):
        value = ctypes.c_float()
        ret = zauxdll.ZAux_Direct_GetUnits(
            self.handle, iaxis, ctypes.byref(value)
        )
        return ret, value.value

    def verify_connection(self, iaxis=0):
        """通过读取控制器实际参数验证通信，而不是只看 OpenEth 返回值。"""
        if not self.is_connected or not self.handle.value:
            return False, {"连接句柄": -1}

        values = {}

        atype = ctypes.c_int()
        ret = zauxdll.ZAux_Direct_GetAtype(
            self.handle, iaxis, ctypes.byref(atype)
        )
        values["ATYPE"] = (ret, atype.value)
        if ret != 0:
            return False, values

        dpos = ctypes.c_float()
        ret = zauxdll.ZAux_Direct_GetDpos(
            self.handle, iaxis, ctypes.byref(dpos)
        )
        values["DPOS"] = (ret, dpos.value)
        if ret != 0:
            return False, values

        units = ctypes.c_float()
        ret = zauxdll.ZAux_Direct_GetUnits(
            self.handle, iaxis, ctypes.byref(units)
        )
        values["UNITS"] = (ret, units.value)
        if ret != 0:
            return False, values

        speed = ctypes.c_float()
        ret = zauxdll.ZAux_Direct_GetSpeed(
            self.handle, iaxis, ctypes.byref(speed)
        )
        values["SPEED"] = (ret, speed.value)
        return ret == 0, values

    def set_speed(self, iaxis, val):
        return zauxdll.ZAux_Direct_SetSpeed(self.handle, iaxis, ctypes.c_float(val))

    def move_rel(self, iaxis, dist):
        return zauxdll.ZAux_Direct_Single_Move(self.handle, iaxis, ctypes.c_float(dist))

    def move_abs(self, iaxis, target_pos):
        return zauxdll.ZAux_Direct_Single_MoveAbs(self.handle, iaxis, ctypes.c_float(target_pos))

    def vmove(self, iaxis, idir):
        return zauxdll.ZAux_Direct_Single_Vmove(self.handle, iaxis, int(idir))

    def stop_axis(self, iaxis):
        return zauxdll.ZAux_Direct_Single_Cancel(self.handle, iaxis, 2)

    def get_dpos(self, iaxis):
        dpos = ctypes.c_float()
        ret = zauxdll.ZAux_Direct_GetDpos(self.handle, iaxis, ctypes.byref(dpos))
        return ret, dpos.value

    def set_zero(self, iaxis):
        return zauxdll.ZAux_Direct_SetDpos(self.handle, iaxis, ctypes.c_float(0.0))

    def is_axis_ready(self, iaxis):
        return self.axis_ready.get(iaxis, False)


# ======================= 3. 单轴精简控制卡片 =======================
class AxisCardWidget(QGroupBox):
    def __init__(self, axis_id, zmc: ZMCWrapper, log_callback, parent=None):
        super().__init__(f"轴 {axis_id} 控制", parent)
        self.axis_id = axis_id
        self.zmc = zmc
        self.log = log_callback
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        # 1. 实时坐标大字显示 (单位: mm)
        self.lbl_pos = QLabel("0.0000 mm")
        self.lbl_pos.setAlignment(Qt.AlignCenter)
        self.lbl_pos.setFont(QFont("Consolas", 18, QFont.Bold))
        self.lbl_pos.setStyleSheet(
            "background-color: #F8F9FA; color: #0288D1; border: 1px solid #B0BEC5; "
            "border-radius: 6px; padding: 6px;"
        )
        layout.addWidget(self.lbl_pos)

        # 2. 清零与单轴停止
        h_btn_layout = QHBoxLayout()
        self.btn_zero = QPushButton("坐标置零")
        self.btn_zero.clicked.connect(self.action_set_zero)
        h_btn_layout.addWidget(self.btn_zero)

        self.btn_stop = QPushButton("🛑 停止")
        self.btn_stop.setStyleSheet("background-color: #E53935; color: white; font-weight: bold;")
        self.btn_stop.clicked.connect(self.action_stop)
        h_btn_layout.addWidget(self.btn_stop)
        layout.addLayout(h_btn_layout)

        # 3. 运行速度设定 (仅开放用户日常需要调节的速度)
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("运行速度(mm/s):"))
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(0.1, 100.0)
        self.spin_speed.setValue(HARDWARE_CONFIG["DEFAULT_SPEED"])
        self.spin_speed.valueChanged.connect(self.action_change_speed)
        speed_layout.addWidget(self.spin_speed)
        layout.addLayout(speed_layout)

        # 4. 点动运行 (按住走，松手停)
        jog_box = QGroupBox("点动 (按住持续移动)")
        jog_layout = QHBoxLayout(jog_box)
        self.btn_jog_neg = QPushButton("◀ 负向 (Jog-)")
        self.btn_jog_neg.pressed.connect(lambda: self.action_vmove(-1))
        self.btn_jog_neg.released.connect(self.action_stop)
        jog_layout.addWidget(self.btn_jog_neg)

        self.btn_jog_pos = QPushButton("正向 (Jog+) ▶")
        self.btn_jog_pos.pressed.connect(lambda: self.action_vmove(1))
        self.btn_jog_pos.released.connect(self.action_stop)
        jog_layout.addWidget(self.btn_jog_pos)
        layout.addWidget(jog_box)

        # 5. 精确位移控制
        pos_box = QGroupBox("定位移动")
        grid = QGridLayout(pos_box)

        grid.addWidget(QLabel("相对移动(mm):"), 0, 0)
        self.spin_rel = QDoubleSpinBox()
        self.spin_rel.setRange(-500.0, 500.0)
        self.spin_rel.setValue(5.0)
        self.spin_rel.setDecimals(4)
        grid.addWidget(self.spin_rel, 0, 1)

        self.btn_rel = QPushButton("执行")
        self.btn_rel.clicked.connect(self.action_move_rel)
        grid.addWidget(self.btn_rel, 0, 2)

        grid.addWidget(QLabel("绝对坐标(mm):"), 1, 0)
        self.spin_abs = QDoubleSpinBox()
        self.spin_abs.setRange(-500.0, 500.0)
        self.spin_abs.setValue(0.0)
        self.spin_abs.setDecimals(4)
        grid.addWidget(self.spin_abs, 1, 1)

        self.btn_abs = QPushButton("定位")
        self.btn_abs.clicked.connect(self.action_move_abs)
        grid.addWidget(self.btn_abs, 1, 2)

        layout.addWidget(pos_box)
        layout.addStretch()

    # --- 动作与槽函数 ---
    def apply_hardware_init(self):
        """下发常量配置"""
        speed = self.spin_speed.value()
        self.zmc.init_axis_hardware(self.axis_id, speed)

    def action_change_speed(self, val):
        if self.zmc.is_connected:
            self.zmc.set_speed(self.axis_id, val)
            self.log(f"轴 {self.axis_id} 速度更新为: {val:.2f} mm/s")

    def update_position(self):
        if self.zmc.is_connected:
            ret, dpos = self.zmc.get_dpos(self.axis_id)
            if ret == 0:
                self.lbl_pos.setText(f"{dpos:.4f} mm")

    def action_move_rel(self):
        if not self.zmc.is_connected:
            return
        if not self.zmc.is_axis_ready(self.axis_id):
            self.log(f"轴 {self.axis_id} 未配置为目标物理轴，禁止运动")
            return
        dist = self.spin_rel.value()
        ret = self.zmc.move_rel(self.axis_id, dist)
        if ret == 0:
            self.log(f"轴 {self.axis_id} 相对移动 {dist} mm，指令已发送")
        else:
            self.log(f"轴 {self.axis_id} 相对移动失败，错误码: {ret}")

    def action_move_abs(self):
        if not self.zmc.is_connected:
            return
        if not self.zmc.is_axis_ready(self.axis_id):
            self.log(f"轴 {self.axis_id} 未配置为目标物理轴，禁止运动")
            return
        target = self.spin_abs.value()
        ret = self.zmc.move_abs(self.axis_id, target)
        if ret == 0:
            self.log(f"轴 {self.axis_id} 移动到绝对坐标 {target} mm，指令已发送")
        else:
            self.log(f"轴 {self.axis_id} 绝对移动失败，错误码: {ret}")

    def action_vmove(self, direction):
        if not self.zmc.is_connected:
            return
        if not self.zmc.is_axis_ready(self.axis_id):
            self.log(f"轴 {self.axis_id} 未配置为目标物理轴，禁止点动")
            return
        ret = self.zmc.vmove(self.axis_id, direction)
        if ret != 0:
            self.log(f"轴 {self.axis_id} 点动失败，错误码: {ret}")

    def action_stop(self):
        if not self.zmc.is_connected:
            return
        ret = self.zmc.stop_axis(self.axis_id)
        if ret == 0:
            self.log(f"轴 {self.axis_id} 停止")
        else:
            self.log(f"轴 {self.axis_id} 停止失败，错误码: {ret}")

    def action_set_zero(self):
        if not self.zmc.is_connected:
            return
        self.zmc.set_zero(self.axis_id)
        self.log(f"轴 {self.axis_id} 坐标已清零")


# ======================= 4. 主窗体 =======================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.zmc = ZMCWrapper()
        self.setWindowTitle("3 轴运动控制工作台")
        self.resize(960, 580)

        self.init_ui()

        # 刷新位置的定时器 (100ms)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_axes_positions)

    def init_ui(self):
        central = QWidget()
        main_layout = QVBoxLayout(central)

        # 1. 顶部控制条 (IP连接 + 全局急停)
        top_group = QGroupBox("系统通信")
        top_layout = QHBoxLayout(top_group)

        top_layout.addWidget(QLabel("控制器 IP:"))
        self.ip_input = QLineEdit("192.168.0.11")
        self.ip_input.setMaximumWidth(140)
        top_layout.addWidget(self.ip_input)

        self.btn_connect = QPushButton("连接控制器")
        self.btn_connect.clicked.connect(self.toggle_connect)
        top_layout.addWidget(self.btn_connect)

        self.lbl_status = QLabel("● 未连接")
        self.lbl_status.setStyleSheet("color: red; font-weight: bold;")
        top_layout.addWidget(self.lbl_status)

        top_layout.addStretch()

        self.btn_all_stop = QPushButton("🛑 全轴急停 (EMERGENCY STOP)")
        self.btn_all_stop.setStyleSheet(
            "background-color: #B71C1C; color: white; font-weight: bold; font-size: 13px; padding: 6px 16px;"
        )
        self.btn_all_stop.clicked.connect(self.stop_all)
        top_layout.addWidget(self.btn_all_stop)

        main_layout.addWidget(top_group)

        # 2. 中间 3 栏轴控制卡片
        axes_layout = QHBoxLayout()
        self.axis_cards = []
        for i in range(3):
            card = AxisCardWidget(axis_id=i, zmc=self.zmc, log_callback=self.log)
            self.axis_cards.append(card)
            axes_layout.addWidget(card)
        main_layout.addLayout(axes_layout)

        # 3. 底部日志
        main_layout.addWidget(QLabel("操作日志:"))
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(80)
        main_layout.addWidget(self.log_text)

        self.setCentralWidget(central)

    def log(self, text):
        self.log_text.append(f">> {text}")

    def toggle_connect(self):
        if not self.zmc.is_connected:
            ip = self.ip_input.text().strip()
            self.log(f"正在连接 {ip}...")
            ret = self.zmc.connect(ip)
            if ret == 0:
                self.log("网络连接返回成功，正在读取控制器参数验证通信...")
                verified, values = self.zmc.verify_connection(0)
                self.log(f"轴 0 控制器返回内容: {values}")

                if not verified:
                    self.zmc.disconnect()
                    self.lbl_status.setText("● 通信验证失败")
                    self.lbl_status.setStyleSheet("color: red; font-weight: bold;")
                    self.log("连接句柄存在，但无法正常读取控制器内容，已断开")
                    QMessageBox.critical(
                        self, "通信验证失败",
                        f"已连接到 {ip}，但读取控制器轴 0 参数失败。\n"
                        f"返回内容: {values}\n请检查控制器型号、连接方式和 SDK DLL。"
                    )
                    return

                self.lbl_status.setText("● 已连接/通信正常")
                self.lbl_status.setStyleSheet("color: green; font-weight: bold;")
                self.btn_connect.setText("断开连接")
                self.log(f"已连接至控制器 {ip}，轴 0 通信验证成功")

                # 连接成功后初始化运动参数；每一步都检查返回值。
                for card in self.axis_cards:
                    results = self.zmc.init_axis_hardware(
                        card.axis_id, card.spin_speed.value()
                    )
                    failed = {
                        name: code for name, code in results.items()
                        if name not in ("ATYPE_ACTUAL",) and code != 0
                    }
                    atype = results.get("ATYPE_ACTUAL", -1)
                    if failed:
                        self.log(
                            f"轴 {card.axis_id} 配置失败: {failed}，"
                            f"实际 ATYPE={atype}，目标 ATYPE={HARDWARE_CONFIG['ATYPE']}"
                        )
                    else:
                        self.log(f"轴 {card.axis_id} 配置完成，当前 ATYPE={atype}")
                        units_ret, units = self.zmc.get_units(card.axis_id)
                        if units_ret != 0:
                            self.log(
                                f"轴 {card.axis_id} 读取 UNITS 失败，错误码: {units_ret}"
                            )
                        elif abs(units - HARDWARE_CONFIG["UNITS"]) > 0.0001:
                            self.log(
                                f"警告：轴 {card.axis_id} 当前 UNITS={units}，"
                                f"厂家要求为 {HARDWARE_CONFIG['UNITS']}"
                            )
                        else:
                            self.log(
                                f"轴 {card.axis_id} 当前 UNITS={units}，未修改控制器换算参数"
                            )

                self.timer.start(100)
            else:
                self.log(f"连接失败! 错误码: {ret}")
                QMessageBox.critical(self, "连接失败", f"无法连接控制器 (错误码: {ret})\n请检查电脑本机IP(需设为 192.168.0.X)及网线！")
        else:
            self.timer.stop()
            self.zmc.disconnect()
            self.lbl_status.setText("● 未连接")
            self.lbl_status.setStyleSheet("color: red; font-weight: bold;")
            self.btn_connect.setText("连接控制器")
            self.log("已断开连接")

    def update_axes_positions(self):
        for card in self.axis_cards:
            card.update_position()

    def stop_all(self):
        if self.zmc.is_connected:
            for card in self.axis_cards:
                card.action_stop()
            self.log("【警告】已触发全轴急停！")

    def closeEvent(self, event):
        if self.zmc.is_connected:
            self.timer.stop()
            self.zmc.disconnect()
        event.accept()


# ======================= 入口 =======================
if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
