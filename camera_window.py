"""Independent Tkinter camera window for the motor-test application."""
from __future__ import annotations

import csv
import datetime as _dt
import math
import multiprocessing as mp
import queue
import threading
import time
from pathlib import Path

import matplotlib.image as mpimg
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class CameraWindow:
    """MVS preview, exposure control, colormap and bounded image recording."""

    COLORMAPS = ("gray", "viridis", "plasma", "inferno", "magma",
                 "cividis", "turbo", "coolwarm", "jet")

    def __init__(self, parent):
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk

        self.tk = tk
        self.filedialog = filedialog
        self.messagebox = messagebox
        self.ttk = ttk
        self.parent = parent
        self.top = tk.Toplevel(parent)
        self.top.title("相机预览 / 曝光 / 图像保存")
        self.top.geometry("1050x760")
        self.top.protocol("WM_DELETE_WINDOW", self.close)

        self.source = None  # retained as a compatibility/status proxy
        self._camera_process = None
        self._camera_commands = None
        self._camera_events = None
        self._frame_queue = None
        self._save_queue = queue.Queue(maxsize=30)
        self._record_lock = threading.Lock()
        self._stop = threading.Event()
        self._connected = threading.Event()
        self._recording = False
        self._record_until = 0.0
        self._next_save = 0.0
        self._save_count = 0
        self._save_dir = None
        self._metadata_path = None
        self._last_frame = None
        self._last_limits = (0.0, 255.0)
        self._cmap = "gray"
        # Tkinter and Matplotlib share the main thread.  Keep acquisition,
        # preview rendering and disk writing at separate, bounded rates so
        # opening the camera cannot starve the motor-control UI.
        self._acquisition_interval = 1.0 / 30.0
        self._preview_interval = 0.10
        self._last_preview_time = 0.0
        self._record_started = 0.0
        self._motion_armed = False
        self._motion_duration = 0.0
        self._motion_interval = 0.0

        self.status = tk.StringVar(value="相机未连接")
        self.exposure = tk.StringVar(value="")
        self.colormap = tk.StringVar(value=self._cmap)
        self.duration = tk.StringVar(value="10")
        self.interval = tk.StringVar(value="100")
        self.save_path = tk.StringVar(value=str(Path.cwd() / "camera_images"))

        controls = ttk.Frame(self.top, padding=8)
        controls.pack(fill="x")
        self.connect_button = ttk.Button(controls, text="连接相机", command=self.connect)
        self.connect_button.pack(side="left", padx=3)
        self.disconnect_button = ttk.Button(controls, text="断开相机", command=self.disconnect,
                                            state="disabled")
        self.disconnect_button.pack(side="left", padx=3)
        ttk.Label(controls, text="曝光时间 μs").pack(side="left", padx=(16, 3))
        ttk.Entry(controls, textvariable=self.exposure, width=12).pack(side="left")
        self.exposure_button = ttk.Button(controls, text="设置曝光", command=self.set_exposure,
                                          state="disabled")
        self.exposure_button.pack(side="left", padx=3)
        ttk.Label(controls, text="Colormap").pack(side="left", padx=(16, 3))
        ttk.Combobox(controls, textvariable=self.colormap, values=self.COLORMAPS,
                     state="readonly", width=11).pack(side="left")
        self.colormap.trace_add("write", lambda *_: self._on_colormap())

        save_box = ttk.LabelFrame(self.top, text="保存一段时间内的图像", padding=6)
        save_box.pack(fill="x", padx=8, pady=(0, 5))
        ttk.Label(save_box, text="持续时间 s").pack(side="left")
        ttk.Entry(save_box, textvariable=self.duration, width=8).pack(side="left", padx=3)
        ttk.Label(save_box, text="间隔 ms").pack(side="left", padx=(10, 3))
        ttk.Entry(save_box, textvariable=self.interval, width=8).pack(side="left", padx=3)
        ttk.Entry(save_box, textvariable=self.save_path, width=42).pack(side="left", padx=(10, 3))
        ttk.Button(save_box, text="选择目录", command=self.choose_save_dir).pack(side="left", padx=3)
        self.record_button = ttk.Button(save_box, text="开始保存图像", command=self.start_recording,
                                        state="disabled")
        self.record_button.pack(side="left", padx=3)
        self.record_status = tk.StringVar(value="未开始保存")
        ttk.Label(save_box, textvariable=self.record_status, foreground="gray").pack(side="left", padx=8)

        self.figure = Figure(figsize=(10, 6), layout="constrained")
        self.axis = self.figure.add_subplot(111)
        self.axis.set_axis_off()
        self.axis.set_title("未连接相机")
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.top)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=4)
        ttk.Label(self.top, textvariable=self.status, foreground="gray").pack(anchor="w", padx=10, pady=5)

        self._save_thread = threading.Thread(target=self._save_loop, name="camera-image-writer",
                                             daemon=True)
        self._save_thread.start()
        self._poll_frames()

    def connect(self):
        if self._connected.is_set():
            return
        self.connect_button.configure(state="disabled")
        self.status.set("正在连接相机...")
        threading.Thread(target=self._connect_worker, name="camera-connect", daemon=True).start()

    def _legacy_connect_worker(self):
        try:
            from camera_mvs import MVSFrameSource
            source = MVSFrameSource(0)
            source.open()
            source.start()
            if self._stop.is_set():
                source.close()
                return
            self.source = source
            self._connected.set()
            self.exposure_value = source.get_exposure()
            self.top.after(0, self._connected_ui)
            next_frame_at = time.perf_counter()
            while not self._stop.is_set() and self._connected.is_set():
                frame = source.grab(100)
                if frame is None:
                    continue
                try:
                    while True:
                        self._frame_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._frame_queue.put_nowait((time.monotonic(), frame))
                except queue.Full:
                    pass
                next_frame_at += self._acquisition_interval
                delay = next_frame_at - time.perf_counter()
                if delay > 0:
                    self._stop.wait(delay)
                elif delay < -self._acquisition_interval:
                    next_frame_at = time.perf_counter()
        except Exception as exc:
            self._connected.clear()
            try:
                if self.source is not None:
                    self.source.close()
            except Exception:
                pass
            self.source = None
            self.top.after(0, lambda: self._connect_failed(str(exc)))

    def _connect_worker(self):
        try:
            from camera_process import camera_process_main
            context = mp.get_context("spawn")
            self._camera_commands = context.Queue()
            self._camera_events = context.Queue()
            self._frame_queue = context.Queue(maxsize=2)
            process = context.Process(
                target=camera_process_main,
                args=(self._camera_commands, self._frame_queue,
                      self._camera_events, 0),
                name="mvs-camera-process")
            process.daemon = True
            process.start()
            self._camera_process = process
            deadline = time.monotonic() + 15.0
            while not self._stop.is_set() and time.monotonic() < deadline:
                try:
                    event = self._camera_events.get(timeout=.2)
                except queue.Empty:
                    if not process.is_alive():
                        raise RuntimeError("相机进程提前退出")
                    continue
                if event[0] == "connected":
                    self.camera_name = event[1]
                    self.exposure_value = event[2]
                    self._connected.set()
                    self.top.after(0, self._connected_ui)
                    return
                if event[0] == "error":
                    raise RuntimeError(event[1])
            raise TimeoutError("相机连接超时")
        except Exception as exc:
            self._connected.clear()
            self._terminate_camera_process()
            try:
                self.top.after(0, lambda: self._connect_failed(str(exc)))
            except Exception:
                pass

    def _connected_ui(self):
        self.connect_button.configure(state="disabled")
        self.disconnect_button.configure(state="normal")
        self.exposure_button.configure(state="normal")
        self.record_button.configure(state="normal")
        value = getattr(self, "exposure_value", None)
        if value is not None:
            self.exposure.set(f"{value:g}")
        name = getattr(self, "camera_name", "")
        self.status.set(f"相机已连接：{name or 'MVS 相机'}，实时取流中")

    def _connect_failed(self, error):
        self.connect_button.configure(state="normal")
        self.status.set(f"相机连接失败：{error}")
        self.messagebox.showerror("相机连接失败", error, parent=self.top)

    def _legacy_disconnect(self):
        self._connected.clear()
        self._recording = False
        self.cancel_motion_recording()
        self.record_button.configure(text="开始保存图像")
        self.status.set("正在断开相机...")
        source = self.source
        self.source = None
        if source is not None:
            threading.Thread(target=self._close_source, args=(source,), daemon=True).start()
        self.connect_button.configure(state="normal")
        self.disconnect_button.configure(state="disabled")
        self.exposure_button.configure(state="disabled")
        self.record_button.configure(state="disabled")

    def _close_source(self, source):
        try:
            source.close()
        finally:
            self.top.after(0, lambda: self.status.set("相机未连接"))

    def _terminate_camera_process(self):
        process = self._camera_process
        self._camera_process = None
        if process is None:
            return
        try:
            if process.is_alive() and self._camera_commands is not None:
                self._camera_commands.put_nowait(("stop",))
            process.join(timeout=2.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
        except Exception:
            pass
        self._camera_commands = None
        self._camera_events = None
        self._frame_queue = None

    def disconnect(self):
        self._connected.clear()
        self._recording = False
        self.cancel_motion_recording()
        self.record_button.configure(text="开始保存图像")
        self.status.set("正在断开相机...")
        threading.Thread(target=self._terminate_camera_process,
                         name="camera-disconnect", daemon=True).start()
        self.connect_button.configure(state="normal")
        self.disconnect_button.configure(state="disabled")
        self.exposure_button.configure(state="disabled")
        self.record_button.configure(state="disabled")

    def _legacy_set_exposure(self):
        try:
            value = float(self.exposure.get())
            if not math.isfinite(value) or value <= 0:
                raise ValueError("曝光时间必须是大于 0 的数值")
            source = self.source
            if source is None or not source.is_opened:
                raise RuntimeError("请先连接相机")
            if not source.set_exposure(value):
                raise RuntimeError("相机拒绝了该曝光时间")
            actual = source.get_exposure()
            if actual is not None:
                self.exposure.set(f"{actual:g}")
            self.status.set(f"曝光时间已设置为 {actual if actual is not None else value:g} μs")
        except Exception as exc:
            self.messagebox.showerror("设置曝光", str(exc), parent=self.top)

    def set_exposure(self):
        try:
            value = float(self.exposure.get())
            if not math.isfinite(value) or value <= 0:
                raise ValueError("曝光时间必须是大于 0 的数值")
            if not self._connected.is_set() or self._camera_commands is None:
                raise RuntimeError("请先连接相机")
            self._camera_commands.put(("set_exposure", value))
            deadline = time.monotonic() + 2.0
            ok = actual = None
            while time.monotonic() < deadline:
                try:
                    event = self._camera_events.get(timeout=.1)
                except queue.Empty:
                    continue
                if event[0] == "exposure":
                    ok, actual = event[1], event[2]
                    break
                if event[0] == "error":
                    raise RuntimeError(event[1])
            if not ok:
                raise RuntimeError("相机拒绝了该曝光时间")
            if actual is not None:
                self.exposure.set(f"{actual:g}")
            self.status.set(
                f"曝光时间已设置为 {actual if actual is not None else value:g} μs")
        except Exception as exc:
            self.messagebox.showerror("设置曝光", str(exc), parent=self.top)

    def choose_save_dir(self):
        path = self.filedialog.askdirectory(parent=self.top, title="选择图像保存目录")
        if path:
            self.save_path.set(path)

    def start_recording(self):
        if self._recording:
            self._recording = False
            self.record_button.configure(text="开始保存图像")
            self.record_status.set(f"已停止，共排队 {self._save_count} 张")
            return
        try:
            duration = float(self.duration.get())
            interval_ms = float(self.interval.get())
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError("持续时间必须大于 0")
            if not math.isfinite(interval_ms) or interval_ms < 1:
                raise ValueError("保存间隔必须不小于 1 ms")
            directory = Path(self.save_path.get()).expanduser().resolve()
            directory.mkdir(parents=True, exist_ok=True)
            stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            self._save_dir = directory / f"camera_{stamp}"
            self._save_dir.mkdir()
            self._metadata_path = self._save_dir / "frames.csv"
            with self._metadata_path.open("w", newline="", encoding="utf-8-sig") as handle:
                csv.writer(handle).writerow(("index", "saved_at", "elapsed_s", "image_file"))
            self._save_count = 0
            self._recording = True
            now = time.monotonic()
            self._record_started = now
            self._record_until = now + duration
            self._next_save = now
            self.record_button.configure(text="停止保存")
            self.record_status.set(f"保存中：{self._save_dir.name}")
        except Exception as exc:
            self.messagebox.showerror("保存图像", str(exc), parent=self.top)

    def _poll_camera_events(self):
        if self._camera_events is None:
            return
        while True:
            try:
                event = self._camera_events.get_nowait()
            except queue.Empty:
                return
            if event[0] == "error":
                self._connected.clear()
                self.status.set(f"相机进程错误：{event[1]}")
            elif event[0] == "disconnected" and self._connected.is_set():
                self._connected.clear()
                self.status.set("相机已断开")

    def _poll_frames(self):
        self._poll_camera_events()
        try:
            if self._frame_queue is None:
                raise queue.Empty
            timestamp, frame = self._frame_queue.get_nowait()
        except (queue.Empty, AttributeError):
            timestamp = frame = None
        if frame is not None:
            self._last_frame = frame
            self._update_limits(frame)
            self._capture_if_due(timestamp, frame)
            now = time.monotonic()
            if now - self._last_preview_time >= self._preview_interval:
                self._last_preview_time = now
                self._draw_frame(frame)
        if self._recording and time.monotonic() >= self._record_until:
            self._recording = False
            self.record_button.configure(text="开始保存图像")
            self.record_status.set(f"保存完成：{self._save_count} 张")
        if self.top.winfo_exists():
            self.top.after(20, self._poll_frames)

    def arm_for_motion(self):
        """Prepare image recording; actual timing begins at XY motion start."""
        if not self._connected.is_set() or self._camera_process is None:
            raise RuntimeError("请先连接相机")
        with self._record_lock:
            if self._recording or self._motion_armed:
                raise RuntimeError("相机当前已经在保存或等待跟焦开始")
        self.start_recording()
        with self._record_lock:
            if not self._recording:
                return False
            self._motion_duration = self._record_until - self._record_started
            self._motion_interval = max(0.001, float(self.interval.get()) / 1000.0)
            self._recording = False
            self._motion_armed = True
        self.record_button.configure(text="等待跟焦开始")
        self.record_status.set("已准备，等待位移台开始移动...")
        return True

    def start_motion_recording(self):
        """Start an armed capture from the motion worker thread."""
        with self._record_lock:
            if not self._motion_armed or self._stop.is_set():
                return False
            now = time.monotonic()
            self._motion_armed = False
            self._recording = True
            self._record_started = now
            self._record_until = now + self._motion_duration
            self._next_save = now
        return True

    def cancel_motion_recording(self):
        with self._record_lock:
            self._motion_armed = False

    def _update_limits(self, frame):
        array = np.asarray(frame)
        if array.ndim == 3:
            self._last_limits = (0.0, 255.0)
            return
        if array.dtype == np.uint8:
            self._last_limits = (0.0, 255.0)
            return
        finite = array[np.isfinite(array)]
        if finite.size:
            low, high = np.percentile(finite, (1, 99))
            self._last_limits = (float(low), float(high if high > low else low + 1))

    @staticmethod
    def _preview_array(frame):
        """Downsample only the on-screen copy; saved frames stay unchanged."""
        array = np.asarray(frame)
        if array.ndim < 2:
            return array
        height, width = array.shape[:2]
        step = max(1, int(math.ceil(max(height / 720.0, width / 1000.0))))
        return array[::step, ::step]

    def _draw_frame(self, frame):
        array = self._preview_array(frame)
        if not hasattr(self, "_image_artist"):
            self._image_artist = self.axis.imshow(array, cmap=self._cmap, aspect="auto")
            self.axis.set_axis_off()
        else:
            self._image_artist.set_data(array)
        if array.ndim == 2:
            self._image_artist.set_clim(*self._last_limits)
            self._image_artist.set_cmap(self._cmap)
        self.canvas.draw_idle()

    def _on_colormap(self):
        self._cmap = str(self.colormap.get() or "gray")
        if hasattr(self, "_image_artist"):
            self._image_artist.set_cmap(self._cmap)
            self.canvas.draw_idle()

    def _capture_if_due(self, timestamp, frame):
        if not self._recording or timestamp + 1e-9 < self._next_save:
            return
        index = self._save_count + 1
        path = self._save_dir / f"image_{index:06d}_{_dt.datetime.now():%H%M%S_%f}.png"
        item = (np.asarray(frame).copy(), path, index,
                timestamp - self._record_started)
        try:
            self._save_queue.put_nowait(item)
        except queue.Full:
            return
        self._save_count = index
        self._next_save = timestamp + max(0.001, float(self.interval.get()) / 1000.0)
        self.record_status.set(f"保存中：{self._save_count} 张")

    def _save_loop(self):
        import numpy as np
        while not self._stop.is_set() or not self._save_queue.empty():
            try:
                frame, path, index, elapsed = self._save_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                array = np.asarray(frame)
                if array.ndim == 2:
                    low, high = self._last_limits
                    mpimg.imsave(str(path), array, cmap=self._cmap, vmin=low, vmax=high)
                else:
                    mpimg.imsave(str(path), array)
                if self._metadata_path is not None:
                    with self._metadata_path.open("a", newline="", encoding="utf-8-sig") as handle:
                        csv.writer(handle).writerow((
                            index, _dt.datetime.now().isoformat(timespec="milliseconds"),
                            f"{max(0.0, elapsed):.6f}", path.name))
            except Exception:
                pass
            finally:
                self._save_queue.task_done()

    def close(self):
        self._stop.set()
        self._connected.clear()
        self._recording = False
        self.cancel_motion_recording()
        self._terminate_camera_process()
        self.top.destroy()
