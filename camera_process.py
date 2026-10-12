"""Subprocess backend for the MVS camera.

The camera SDK and frame conversion live outside the motor-test process so
they cannot compete with the motor worker for the Python GIL.
"""
from __future__ import annotations

import queue
import time


def camera_process_main(command_queue, frame_queue, event_queue, device_index=0):
    source = None
    try:
        from camera_mvs import MVSFrameSource

        source = MVSFrameSource(device_index)
        source.open()
        source.start()
        event_queue.put(("connected", source.name, source.get_exposure()))

        interval = 1.0 / 30.0
        next_frame_at = time.perf_counter()
        running = True
        while running:
            try:
                while True:
                    command = command_queue.get_nowait()
                    name = command[0]
                    if name == "stop":
                        running = False
                        break
                    if name == "set_exposure":
                        ok = source.set_exposure(command[1])
                        actual = source.get_exposure() if ok else None
                        event_queue.put(("exposure", ok, actual))
            except queue.Empty:
                pass

            if not running:
                break
            frame = source.grab(100)
            if frame is not None:
                packet = (time.monotonic(), frame)
                try:
                    while True:
                        frame_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    frame_queue.put_nowait(packet)
                except queue.Full:
                    pass

            next_frame_at += interval
            delay = next_frame_at - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -interval:
                next_frame_at = time.perf_counter()
    except Exception as exc:
        try:
            event_queue.put(("error", str(exc)))
        except Exception:
            pass
    finally:
        if source is not None:
            try:
                source.close()
            except Exception:
                pass
        try:
            event_queue.put(("disconnected",))
        except Exception:
            pass
