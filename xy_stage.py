"""ZMotion Ethernet / axis 0=X、1=Y。DLL I/O 与 STM32 控制线程分离。"""
import ctypes
import math
import os
from pathlib import Path
import threading
import time


REFERENCE_ROOT = Path("C:/Users/muyas/Desktop/i-ATF PC_0925_backup/新建文件夹/"
                      "i-ATF PC_0928_original_v3/i-ATF PC_0928_original/i-ATF PC_0925_backup")


def default_dll_path():
    candidates = [Path(__file__).resolve().parent / "zmotion/zauxdll64.dll",
                  Path(__file__).resolve().parent / "dll/zauxdll64.dll",
                  REFERENCE_ROOT / "pc_app/translation stage/zauxdll64.dll"]
    return str(next((p for p in candidates if p.is_file()), candidates[0]))


class XYStage:
    def __init__(self, dll_path, source="MPOS", dll=None):
        if source not in ("MPOS", "DPOS"):
            raise ValueError("XY 位置来源须为 MPOS 或 DPOS")
        self.source = source
        self.handle = ctypes.c_void_p()
        self.lock = threading.RLock()
        self.mpos_offset = (0.0, 0.0)
        self.dll_path = str(Path(dll_path).resolve())
        self.dll_dirs = []
        if dll is None:
            if os.name != "nt" or ctypes.sizeof(ctypes.c_void_p) != 8:
                raise RuntimeError("ZMotion 需要64位 Windows Python")
            path = Path(self.dll_path)
            if not path.is_file():
                raise RuntimeError("找不到 zauxdll64.dll，请选择原工程的 DLL（同目录需含 zmotion.dll）")
            if hasattr(os, "add_dll_directory"):
                self.dll_dirs.append(os.add_dll_directory(str(path.parent)))
            dll = ctypes.WinDLL(str(path))
        self.dll = dll
        h, axis, f = ctypes.c_void_p, ctypes.c_int, ctypes.c_float
        specs = {"ZAux_OpenEth": [ctypes.c_char_p, ctypes.POINTER(h)], "ZAux_Close": [h],
                 "ZAux_SetTimeOut": [h, ctypes.c_uint32],
                 "ZAux_Direct_MoveAbs": [h, axis, ctypes.POINTER(axis), ctypes.POINTER(f)],
                 "ZAux_Direct_Single_Cancel": [h, axis, axis]}
        for name in ("GetMpos", "GetDpos", "GetUnits"):
            specs["ZAux_Direct_" + name] = [h, axis, ctypes.POINTER(f)]
        for name in ("GetAtype", "GetIfIdle"):
            specs["ZAux_Direct_" + name] = [h, axis, ctypes.POINTER(axis)]
        for name in ("SetSpeed", "SetAccel", "SetDecel"):
            specs["ZAux_Direct_" + name] = [h, axis, f]
        for name, args in specs.items():
            fn = getattr(dll, name, None)
            if fn is None:
                raise RuntimeError(f"DLL 缺少 {name}")
            fn.argtypes, fn.restype = args, ctypes.c_int32

    def call(self, name, *args):
        if not self.handle.value:
            raise RuntimeError("位移台未连接")
        ret = getattr(self.dll, name)(self.handle, *args)
        if ret != 0:
            raise RuntimeError(f"{name} 返回错误码 {ret}")

    def connect(self, ip):
        with self.lock:
            ret = self.dll.ZAux_OpenEth(str(ip).strip().encode("ascii"), ctypes.byref(self.handle))
            if ret or not self.handle.value:
                raise RuntimeError(f"连接 ZMotion 失败：{ret}")
            try:
                self.call("ZAux_SetTimeOut", ctypes.c_uint32(50))
                for axis in (0, 1):
                    atype, units = ctypes.c_int(), ctypes.c_float()
                    self.call("ZAux_Direct_GetAtype", axis, ctypes.byref(atype))
                    self.call("ZAux_Direct_GetUnits", axis, ctypes.byref(units))
                    if atype.value != 65 or not math.isclose(units.value, 10000., abs_tol=1.):
                        raise RuntimeError(f"axis {axis} 类型/单位与参考设备不符：ATYPE={atype.value}, UNITS={units.value}")
                # MPOS 是编码器坐标，DPOS 是控制器指令坐标。记录连接时的固定零点偏置，
                # 只在软件反馈层补偿，不改写控制器坐标。
                mpos = self._get_positions("Mpos")
                dpos = self._get_positions("Dpos")
                self.mpos_offset = tuple(mpos[i] - dpos[i] for i in range(2))
                self.get_positions()
            except BaseException:
                self.disconnect()
                raise

    def get_positions(self):
        if self.source == "DPOS":
            return self._get_positions("Dpos")
        raw = self._get_positions("Mpos")
        return tuple(raw[i] - self.mpos_offset[i] for i in range(2))

    def get_raw_mpositions(self):
        return self._get_positions("Mpos")

    def get_dpositions(self):
        return self._get_positions("Dpos")

    def sync_mpos_to_dpos(self):
        """在轴空闲时把编码器反馈坐标 MPOS 对齐到当前指令坐标 DPOS。"""
        with self.lock:
            if not self.is_idle():
                raise RuntimeError("只能在 XY 位移台空闲时校准 MPOS")
            dpos = self._get_positions("Dpos")
            fn = getattr(self.dll, "ZAux_Direct_SetMpos", None)
            if fn is None:
                raise RuntimeError("DLL 缺少 ZAux_Direct_SetMpos")
            if not getattr(fn, "argtypes", None):
                fn.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
                fn.restype = ctypes.c_int32
            for axis, value in enumerate(dpos):
                ret = fn(self.handle, axis, ctypes.c_float(value))
                if ret != 0:
                    raise RuntimeError(f"ZAux_Direct_SetMpos 返回错误码 {ret}")
            self.mpos_offset = (0.0, 0.0)
            return dpos

    def _get_positions(self, name):
        with self.lock:
            values = []
            for axis in (0, 1):
                value = ctypes.c_float()
                self.call("ZAux_Direct_Get" + name, axis, ctypes.byref(value))
                if not math.isfinite(value.value):
                    raise RuntimeError("XY 位置非有限数")
                values.append(value.value)
            return tuple(values)

    def is_idle(self):
        with self.lock:
            states = []
            for axis in (0, 1):
                value = ctypes.c_int()
                self.call("ZAux_Direct_GetIfIdle", axis, ctypes.byref(value))
                states.append(value.value == -1)
            return all(states)

    def prepare_motion(self, speed, acceleration):
        if not all(math.isfinite(v) and v > 0 for v in (speed, acceleration)):
            raise ValueError("XY 速度与加速度必须是有限正数")
        with self.lock:
            if not self.is_idle():
                raise RuntimeError("位移台仍在运动")
            for axis in (0, 1):
                for name, value in (("SetSpeed", speed), ("SetAccel", acceleration), ("SetDecel", acceleration)):
                    self.call("ZAux_Direct_" + name, axis, ctypes.c_float(value))

    def move(self, position, speed, acceleration, prepared=False):
        if (len(position) != 2 or not all(math.isfinite(x) for x in (*position, speed, acceleration))
                or speed <= 0 or acceleration <= 0):
            raise ValueError("XY 运动参数无效")
        with self.lock:
            if not self.is_idle():
                raise RuntimeError("位移台仍在运动")
            # 控制器插补轴0为主轴；两轴一次 MoveAbs，沿直线移动。
            if not prepared:
                self.prepare_motion(speed, acceleration)
            axes, positions = (ctypes.c_int * 2)(0, 1), (ctypes.c_float * 2)(*position)
            try:
                self.call("ZAux_Direct_MoveAbs", 2, axes, positions)
            except BaseException:
                self.stop()
                raise

    def stop(self):
        if not self.lock.acquire(timeout=.3):
            raise RuntimeError("XY DLL 正被未返回调用占用，停止未确认")
        try:
            errors = []
            for axis in (0, 1):
                try:
                    self.call("ZAux_Direct_Single_Cancel", axis, 2)
                except Exception as exc:
                    errors.append(str(exc))
            if errors:
                raise RuntimeError("；".join(errors))
        finally:
            self.lock.release()

    def disconnect(self):
        with self.lock:
            if self.handle.value:
                ret = self.dll.ZAux_Close(self.handle)
                self.handle = ctypes.c_void_p()
                if ret:
                    raise RuntimeError(f"关闭位移台失败：{ret}")


class StageReader:
    def __init__(self, stage, period_s=.01):
        self.stage, self.period_s = stage, period_s
        self.lock = threading.Lock()
        self.cancel = threading.Event()
        self.sample = None
        self.thread = threading.Thread(target=self.run, name="XYReader", daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        seq = 0
        while not self.cancel.is_set():
            begin = time.perf_counter()
            try:
                x, y = self.stage.get_positions()
                end = time.perf_counter()
                sample = dict(seq=seq, host_abs_s=(begin + end) / 2, receive_abs_s=end,
                              read_duration_s=end - begin, x_mm=x, y_mm=y, error="")
            except Exception as exc:
                sample = dict(seq=seq, host_abs_s=begin, receive_abs_s=time.perf_counter(),
                              read_duration_s=time.perf_counter() - begin, error=str(exc))
            with self.lock:
                if not self.cancel.is_set():
                    self.sample = sample
            seq += 1
            self.cancel.wait(max(0., self.period_s - (time.perf_counter() - begin)))

    def latest(self, max_age_s=.1):
        with self.lock:
            sample = self.sample.copy() if self.sample else None
        if sample is None:
            raise RuntimeError("尚无 XY 位置样本")
        if sample["error"]:
            raise RuntimeError("XY 读取失败：" + sample["error"])
        age = time.perf_counter() - sample["host_abs_s"]
        if age < 0 or age > max_age_s or sample["read_duration_s"] > max_age_s:
            raise TimeoutError("XY 位置数据陈旧/读取耗时过长")
        return sample

    def stop(self):
        self.cancel.set()
        self.thread.join(.3)
        with self.lock:
            self.sample = None
        if self.thread.is_alive():
            raise RuntimeError("XY DLL 读取仍未返回，不能关闭或重连其句柄")
