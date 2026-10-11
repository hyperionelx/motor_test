"""Independent Tkinter camera window for the motor-test application."""
from __future__ import annotations

import csv
import datetime as _dt
import math
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

        self.source = None
        self._frame_queue = queue.Queue(maxsize=2)
        self._save_queue = queue.Queue(maxsize=30)
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

    def _connect_worker(self):
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
        except Exception as exc:
            self._connected.clear()
            try:
                if self.source is not None:
                    self.source.close()
            except Exception:
                pass
            self.source = None
            self.top.after(0, lambda: self._connect_failed(str(exc)))

    def _connected_ui(self):
        self.connect_button.configure(state="disabled")
        self.disconnect_button.configure(state="normal")
        self.exposure_button.configure(state="normal")
        self.record_button.configure(state="normal")
        value = getattr(self, "exposure_value", None)
        if value is not None:
            self.exposure.set(f"{value:g}")
        source = self.source
        name = getattr(source, "name", "") if source else ""
        self.status.set(f"相机已连接：{name or 'MVS 相机'}，实时取流中")

    def _connect_failed(self, error):
        self.connect_button.configure(state="normal")
        self.status.set(f"相机连接失败：{error}")
        self.messagebox.showerror("相机连接失败", error, parent=self.top)

    def disconnect(self):
        self._connected.clear()
        self._recording = False
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

    def set_exposure(self):
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
            self._record_until = now + duration
            self._next_save = now
            self.record_button.configure(text="停止保存")
            self.record_status.set(f"保存中：{self._save_dir.name}")
        except Exception as exc:
            self.messagebox.showerror("保存图像", str(exc), parent=self.top)

    def _poll_frames(self):
        try:
            timestamp, frame = self._frame_queue.get_nowait()
        except queue.Empty:
            timestamp = frame = None
        if frame is not None:
            self._last_frame = frame
            self._update_limits(frame)
            self._draw_frame(frame)
            self._capture_if_due(timestamp, frame)
        if self._recording and time.monotonic() >= self._record_until:
            self._recording = False
            self.record_button.configure(text="开始保存图像")
            self.record_status.set(f"保存完成：{self._save_count} 张")
        if self.top.winfo_exists():
            self.top.after(20, self._poll_frames)

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

    def _draw_frame(self, frame):
        import numpy as np
        array = np.asarray(frame)
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
                timestamp - (self._record_until - float(self.duration.get())))
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
        source = self.source
        self.source = None
        if source is not None:
            try:
                source.close()
            except Exception:
                pass
        self.top.destroy()
