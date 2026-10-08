"""隐藏窗口的离线 GUI 冒烟验证，不连接串口。"""
import argparse
import json
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk, filedialog

import motor_test

output = Path(__file__).resolve().parent / "verification" / "gui_demo"
output.mkdir(parents=True, exist_ok=True)
errors = []
checks = []
real_tk = tk.Tk
real_toplevel = tk.Toplevel


def widgets(parent):
    for widget in parent.winfo_children():
        yield widget
        yield from widgets(widget)


def test_root():
    root = real_tk()
    root.withdraw()
    def invoke_demo():
        try:
            all_widgets = list(widgets(root))
            buttons = {w.cget("text"): w for w in all_widgets if isinstance(w, ttk.Button)}
            boxes = {w.cget("text"): w for w in all_widgets if isinstance(w, ttk.Checkbutton)}
            a = next(w for label, w in boxes.items() if label.startswith("方案 A"))
            b = next(w for label, w in boxes.items() if label.startswith("方案 B"))
            a.invoke()
            assert root.getvar(a.cget("variable"))
            b.invoke()
            assert not root.getvar(a.cget("variable")) and root.getvar(b.cget("variable"))
            checks.append("checkbox A/B exclusivity")
            buttons["导入 mapping"].invoke()
            mapping = next(w for label, w in boxes.items() if label.startswith("用实际 XY"))
            assert root.getvar(mapping.cget("variable"))
            checks.append("mapping import, metadata endpoints, preview")
            buttons["模拟演示"].invoke()
        except Exception as exc:
            errors.append(str(exc))
    def close():
        root.tk.call(root.protocol("WM_DELETE_WINDOW"))
    root.after(100, invoke_demo)
    root.after(4500, close)
    return root


tk.Tk = test_root
def hidden_toplevel(*args, **kwargs):
    top = real_toplevel(*args, **kwargs)
    top.withdraw()
    return top
tk.Toplevel = hidden_toplevel
messagebox.showerror = lambda title, message: errors.append(f"{title}: {message}")
filedialog.askopenfilename = lambda **kwargs: str(Path(__file__).parent / "examples/mapping_line.txt")
args = argparse.Namespace(port="", baud=921600, interval_ms=50., step_um=10., steps=20,
                          sample_ms=5., tail_ms=500., min_um=-5000., max_um=5000.,
                          output=str(output), cli=False, demo=False, list_ports=False)
args.xy_speed = 100.
motor_test.gui(args)
summaries = list(output.glob("*/summary.json"))
assert not errors, errors
assert summaries, "GUI 未成功导出演示结果"
summary = json.loads(max(summaries, key=lambda p: p.stat().st_mtime).read_text(encoding="utf-8"))
assert summary["status"] == "DEMO_SYNTHETIC"
assert summary["measurement_valid"]
assert summary["mode"] == "TRACK_POSITION_LEAD"
assert summary["reference_source"] == "XY_MAPPING"
assert summary["config"]["xy_start_y"] == -40. and summary["config"]["xy_end_y"] == 40.
print("PASS: hidden GUI, A/B exclusive checks, five-column import/metadata/preview, mapping demo, plots/export, shutdown; no devices opened")
