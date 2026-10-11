"""Small, Tkinter-friendly wrapper around the bundled Hikvision MVS SDK.

The motion application deliberately imports this module only when the camera
window is opened.  A missing SDK therefore does not affect the existing Z/XY
features.
"""
from __future__ import annotations

import ctypes
import os
import platform
import sys
import threading
from pathlib import Path

import numpy as np


_ROOT = Path(__file__).resolve().parent
_RUNTIME = _ROOT / "mvsdk" / "runtime"
if os.name == "nt" and _RUNTIME.is_dir() and hasattr(os, "add_dll_directory"):
    _DLL_DIR = os.add_dll_directory(str(_RUNTIME))
else:
    _DLL_DIR = None

from mvsdk import CameraParams_header as CH  # noqa: E402
from mvsdk import MvCamera  # noqa: E402
from mvsdk import MvErrorDefine_const as ERR  # noqa: E402
from mvsdk import PixelType_header as PIX  # noqa: E402


MV_OK = 0
MV_GIGE_DEVICE = 0x1
MV_USB_DEVICE = 0x4
MV_ACCESS_EXCLUSIVE = 1
MONO8 = getattr(PIX, "PixelType_Gvsp_Mono8", 17301505)
MONO10 = getattr(PIX, "PixelType_Gvsp_Mono10", 17825803)
MONO12 = getattr(PIX, "PixelType_Gvsp_Mono12", 17825805)
RGB8 = getattr(PIX, "PixelType_Gvsp_RGB8_Packed", 35127316)
_BAYER = {
    getattr(PIX, "PixelType_Gvsp_BayerRG8", 17301513),
    getattr(PIX, "PixelType_Gvsp_BayerGR8", 17301512),
    getattr(PIX, "PixelType_Gvsp_BayerGB8", 17301514),
    getattr(PIX, "PixelType_Gvsp_BayerBG8", 17301515),
}


def _text(value) -> str:
    try:
        return bytes(value).split(b"\0", 1)[0].decode("ascii", "replace").strip()
    except Exception:
        return ""


def _check(ret, operation: str):
    if ret == MV_OK:
        return
    name = f"0x{int(ret) & 0xFFFFFFFF:08X}"
    for attr in dir(ERR):
        if attr.startswith("MV_E"):
            try:
                if getattr(ERR, attr) == ret:
                    name = attr
                    break
            except Exception:
                pass
    raise RuntimeError(f"{operation}失败 ({name})")


class MVSFrameSource:
    """Thread-safe open/start/grab/parameter interface for one MVS camera."""

    _sdk_lock = threading.Lock()
    _sdk_initialized = False

    def __init__(self, device_index: int = 0):
        self.device_index = int(device_index)
        self._cam = MvCamera()
        self._opened = False
        self._grabbing = False
        self._lock = threading.RLock()
        self.width = 0
        self.height = 0
        self.pixel_type = 0
        self.name = ""

    @classmethod
    def _ensure_sdk(cls):
        with cls._sdk_lock:
            if not cls._sdk_initialized:
                _check(MvCamera.MV_CC_Initialize(), "初始化 MVS SDK")
                cls._sdk_initialized = True

    @staticmethod
    def enumerate_devices():
        MVSFrameSource._ensure_sdk()
        devices = CH.MV_CC_DEVICE_INFO_LIST()
        if MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, devices) != MV_OK:
            return []
        result = []
        for index in range(int(devices.nDeviceNum)):
            info = devices.pDeviceInfo[index].contents
            if info.nTLayerType == MV_GIGE_DEVICE:
                details = info.SpecialInfo.stGigEInfo
            else:
                details = info.SpecialInfo.stUsb3VInfo
            result.append({
                "index": index,
                "model": _text(getattr(details, "chModelName", b"")),
                "serial": _text(getattr(details, "chSerialNumber", b"")),
                "type": "GigE" if info.nTLayerType == MV_GIGE_DEVICE else "USB",
            })
        return result

    def open(self):
        with self._lock:
            if self._opened:
                return
            self._ensure_sdk()
            devices = CH.MV_CC_DEVICE_INFO_LIST()
            _check(MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, devices),
                   "枚举相机")
            count = int(devices.nDeviceNum)
            if count == 0:
                raise RuntimeError("未找到 MVS 相机")
            if not 0 <= self.device_index < count:
                raise RuntimeError(f"相机索引 {self.device_index} 超出范围")
            info = devices.pDeviceInfo[self.device_index].contents
            _check(self._cam.MV_CC_CreateHandle(info), "创建相机句柄")
            try:
                _check(self._cam.MV_CC_OpenDevice(MV_ACCESS_EXCLUSIVE, 0), "打开相机")
                try:
                    packet_size = self._cam.MV_CC_GetOptimalPacketSize()
                    if packet_size > 0:
                        self._cam.MV_CC_SetIntValueEx("GevSCPSPacketSize", packet_size)
                except Exception:
                    pass
                try:
                    self._cam.MV_CC_SetEnumValue("TriggerMode", 0)
                except Exception:
                    pass
                self._read_shape()
                self.name = _text(getattr(
                    info.SpecialInfo.stGigEInfo if info.nTLayerType == MV_GIGE_DEVICE
                    else info.SpecialInfo.stUsb3VInfo, "chModelName", b""))
                self._opened = True
            except Exception:
                try:
                    self._cam.MV_CC_CloseDevice()
                    self._cam.MV_CC_DestroyHandle()
                except Exception:
                    pass
                raise

    def close(self):
        with self._lock:
            self.stop()
            if self._opened:
                try:
                    self._cam.MV_CC_CloseDevice()
                    self._cam.MV_CC_DestroyHandle()
                finally:
                    self._opened = False

    def start(self):
        with self._lock:
            if not self._opened:
                raise RuntimeError("相机尚未连接")
            if not self._grabbing:
                _check(self._cam.MV_CC_StartGrabbing(), "开始取流")
                self._grabbing = True

    def stop(self):
        with self._lock:
            if self._grabbing:
                try:
                    self._cam.MV_CC_StopGrabbing()
                finally:
                    self._grabbing = False

    def _read_shape(self):
        value = CH.MVCC_INTVALUE_EX()
        if self._cam.MV_CC_GetIntValueEx("Width", value) == MV_OK:
            self.width = int(value.nCurValue)
        if self._cam.MV_CC_GetIntValueEx("Height", value) == MV_OK:
            self.height = int(value.nCurValue)
        enum_value = CH.MVCC_ENUMVALUE()
        if self._cam.MV_CC_GetEnumValue("PixelFormat", enum_value) == MV_OK:
            self.pixel_type = int(enum_value.nCurValue)

    def get_exposure(self):
        with self._lock:
            value = CH.MVCC_FLOATVALUE()
            if self._cam.MV_CC_GetFloatValue("ExposureTime", value) == MV_OK:
                return float(value.fCurValue)
        return None

    def set_exposure(self, value: float):
        with self._lock:
            return self._cam.MV_CC_SetFloatValue("ExposureTime", float(value)) == MV_OK

    def grab(self, timeout_ms=100):
        with self._lock:
            if not self._grabbing:
                return None
            frame = CH.MV_FRAME_OUT()
            if self._cam.MV_CC_GetImageBuffer(frame, int(timeout_ms)) != MV_OK:
                return None
            try:
                return self._to_numpy(frame)
            finally:
                self._cam.MV_CC_FreeImageBuffer(frame)

    @staticmethod
    def _to_numpy(frame):
        info = frame.stFrameInfo
        width, height = int(info.nWidth), int(info.nHeight)
        length, pixel_type = int(info.nFrameLen), int(info.enPixelType)
        if not width or not height or not length:
            return None
        pointer = ctypes.cast(frame.pBufAddr, ctypes.POINTER(ctypes.c_ubyte))
        raw = np.ctypeslib.as_array(pointer, shape=(length,)).copy()
        if pixel_type == MONO8:
            return raw[:width * height].reshape(height, width)
        if pixel_type in (MONO10, MONO12):
            pixels = raw[:width * height * 2].view(np.uint16).reshape(height, width)
            return np.right_shift(pixels, 2 if pixel_type == MONO10 else 4).astype(np.uint16)
        if pixel_type == RGB8:
            return raw[:width * height * 3].reshape(height, width, 3)
        if pixel_type in _BAYER:
            # The camera window can still display Bayer data as monochrome;
            # this avoids modifying SDK conversion state while streaming.
            return raw[:width * height].reshape(height, width)
        return raw[:width * height].reshape(height, width)

    @property
    def is_opened(self):
        return self._opened

    @property
    def is_grabbing(self):
        return self._grabbing
