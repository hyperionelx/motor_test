"""导入原上位机五列 mapping；按实际 XY 投影求 Z，不按计划时间造 XY。"""
from dataclasses import dataclass
import bisect
import csv
import hashlib
import math
from pathlib import Path


@dataclass(frozen=True)
class MappingCurve:
    distance_mm: tuple
    target_z_um: tuple
    source_path: str = ""
    source_sha256: str = ""
    start_xy_mm: tuple | None = None
    end_xy_mm: tuple | None = None
    x_mm: tuple | None = None
    y_mm: tuple | None = None

    def __post_init__(self):
        if len(self.distance_mm) < 2 or len(self.distance_mm) != len(self.target_z_um):
            raise ValueError("mapping 至少需要两个距离/目标高度点")
        if not all(math.isfinite(v) for v in (*self.distance_mm, *self.target_z_um)):
            raise ValueError("mapping 含 NaN/Inf")
        if self.distance_mm[0] < 0 or any(b <= a for a, b in zip(self.distance_mm, self.distance_mm[1:])):
            raise ValueError("distance_mm 必须非负且严格递增，不能重复或倒序")
        for pair in (self.start_xy_mm, self.end_xy_mm):
            if pair is not None and (len(pair) != 2 or not all(math.isfinite(v) for v in pair)):
                raise ValueError("mapping XY 元数据无效")

        if (self.x_mm is None) != (self.y_mm is None):
            raise ValueError("mapping x_mm/y_mm must be provided together")
        if self.x_mm is not None:
            if len(self.x_mm) != len(self.distance_mm) or len(self.y_mm) != len(self.distance_mm):
                raise ValueError("mapping XY point count must match distance_mm")
            if not all(math.isfinite(v) for v in (*self.x_mm, *self.y_mm)):
                raise ValueError("mapping XY contains NaN/Inf")

    @property
    def span_mm(self):
        return self.distance_mm[-1] - self.distance_mm[0]

    def at(self, distance):
        if not math.isfinite(distance):
            raise ValueError("mapping 查询距离无效")
        distance = min(self.distance_mm[-1], max(self.distance_mm[0], distance))
        i = min(len(self.distance_mm) - 2, max(0, bisect.bisect_right(self.distance_mm, distance) - 1))
        slope = ((self.target_z_um[i + 1] - self.target_z_um[i]) /
                 (self.distance_mm[i + 1] - self.distance_mm[i]))
        return self.target_z_um[i] + slope * (distance - self.distance_mm[i]), slope

    @classmethod
    def load(cls, path):
        path = Path(path).resolve()
        if path.stat().st_size > 20 * 1024 * 1024:
            raise ValueError("mapping 文件超过20 MiB")
        data = path.read_bytes()
        lines = data.decode("utf-8-sig").splitlines()
        meta, body = {}, []
        for line in lines:
            if not line.strip():
                continue
            if line.lstrip().startswith("#"):
                items = line.lstrip()[1:].strip().replace(",", " ").split()
                if len(items) == 3 and items[0] in ("start_xy_mm", "end_xy_mm"):
                    pair = tuple(float(v) for v in items[1:])
                    if not all(math.isfinite(v) for v in pair):
                        raise ValueError("mapping XY 元数据无效")
                    meta[items[0]] = pair
            else:
                body.append(line)
        if not body:
            raise ValueError("mapping 文件为空")
        delimiter = "\t" if "\t" in body[0] else "," if "," in body[0] else None
        rows = list(csv.reader(body, delimiter=delimiter)) if delimiter else [line.split() for line in body]
        header = [name.strip() for name in rows[0]]
        if "distance_mm" not in header or "target_z_um" not in header:
            raise ValueError("请导入目标高度曲线：必须含 distance_mm 和 target_z_um，不能用 actual_z_um/FES 替代")
        di, zi = header.index("distance_mm"), header.index("target_z_um")
        has_xy = "x_mm" in header or "y_mm" in header
        if has_xy and not all(key in header for key in ("x_mm", "y_mm")):
            raise ValueError("mapping x_mm and y_mm must be provided together")
        xi = header.index("x_mm") if has_xy else None
        yi = header.index("y_mm") if has_xy else None
        distances, heights = [], []
        xs, ys = [], []
        for lineno, row in enumerate(rows[1:], 2):
            try:
                distances.append(float(row[di]))
                heights.append(float(row[zi]))
                if has_xy:
                    xs.append(float(row[xi]))
                    ys.append(float(row[yi]))
            except (ValueError, IndexError) as exc:
                raise ValueError(f"mapping 第{lineno}个数据行无效") from exc
        if len(distances) > 200000:
            raise ValueError("mapping 点数超过200000")
        if "start_xy_mm" not in meta and all(key in header for key in ("x_mm", "y_mm")) and len(rows) > 2:
            xi, yi = header.index("x_mm"), header.index("y_mm")
            meta["start_xy_mm"] = (float(rows[1][xi]), float(rows[1][yi]))
            meta["end_xy_mm"] = (float(rows[-1][xi]), float(rows[-1][yi]))
        return cls(tuple(distances), tuple(heights), str(path), hashlib.sha256(data).hexdigest(),
                   meta.get("start_xy_mm"), meta.get("end_xy_mm"),
                   tuple(xs) if has_xy else None, tuple(ys) if has_xy else None)


class LineMapping:
    """将导入曲线的完整距离区间映射到手动设置的扫描线（显式按比例缩放）。"""
    def __init__(self, curve, start, end, tolerance_mm=.05, offset_um=0.):
        self.curve, self.start, self.end = curve, tuple(start), tuple(end)
        self.tolerance_mm, self.offset_um = tolerance_mm, offset_um
        if not all(math.isfinite(v) for v in (*start, *end, tolerance_mm, offset_um)) or tolerance_mm <= 0:
            raise ValueError("扫描线参数无效")
        self.length_mm = math.dist(start, end)
        if self.length_mm < 1e-6:
            raise ValueError("XY 起终点不能相同")
        self.unit = tuple((b - a) / self.length_mm for a, b in zip(start, end))
        self.scale = curve.span_mm / self.length_mm

    def project(self, x, y):
        if not all(math.isfinite(v) for v in (x, y)):
            raise ValueError("XY 位置无效")
        along = (x - self.start[0]) * self.unit[0] + (y - self.start[1]) * self.unit[1]
        cross = abs((x - self.start[0]) * self.unit[1] - (y - self.start[1]) * self.unit[0])
        if cross > self.tolerance_mm or not -self.tolerance_mm <= along <= self.length_mm + self.tolerance_mm:
            raise RuntimeError("XY 位置离开扫描线/有效 mapping 区间")
        return min(self.length_mm, max(0., along)), cross

    def reference(self, x, y, vx=0., vy=0., lead_s=0.):
        along, cross = self.project(x, y)
        path_speed = vx * self.unit[0] + vy * self.unit[1]
        predicted = min(self.length_mm, max(0., along + path_speed * lead_s))
        # 到达边界后不外推曲面，也不给马达继续向外的速度。
        if ((predicted == self.length_mm and path_speed > 0) or (predicted == 0 and path_speed < 0)):
            path_speed = 0.
        z, slope = self.curve.at(self.curve.distance_mm[0] + predicted * self.scale)
        return dict(target_um=z + self.offset_um, ideal_velocity_um_s=slope * self.scale * path_speed,
                    distance_mm=along, mapping_distance_mm=self.curve.distance_mm[0] + along * self.scale,
                    cross_track_mm=cross, path_velocity_mm_s=path_speed)


class XYVelocityEstimator:
    """有限窗口线性回归；仅使用真实且时间不同的 XY 样本。"""
    def __init__(self, window_s=.04):
        self.window_s, self.rows = window_s, []

    def add(self, sample):
        t = sample["host_abs_s"]
        if self.rows and t <= self.rows[-1][0]:
            return self.velocity()
        self.rows.append((t, sample["x_mm"], sample["y_mm"]))
        self.rows = [r for r in self.rows if t - r[0] <= self.window_s]
        return self.velocity()

    def velocity(self):
        if len(self.rows) < 2:
            return 0., 0.
        origin = self.rows[-1][0]
        ts = [r[0] - origin for r in self.rows]
        mean_t = sum(ts) / len(ts)
        denominator = sum((t - mean_t) ** 2 for t in ts)
        if denominator <= 1e-15:
            return 0., 0.
        return tuple(sum((t - mean_t) * r[axis] for t, r in zip(ts, self.rows)) / denominator
                     for axis in (1, 2))
