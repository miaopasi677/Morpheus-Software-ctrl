"""真实几何正向运动学（无 IK / 无外部求解器 / 无 LUT）。

输入 = 标准映射归一化舵机值（经 frame_to_pwm 得 PWM，再换算舵机转角）。
准确性来自 CAD 提取的真实几何（linkage_model.json）：
  translate  线驱/齿条/推杆/open：表皮点沿真实方向平动，行程 = 曲柄半径 × Δθ
  fourbar    连杆：输出摇杆绕真实固定铰转，曲柄角->摇杆角用闭式圆-圆交点（正向）
  revolute   眼/ jaw /颈：绕真实铰链轴转

setup（apply_mechanism_rig.py）用 rest_pose() 摆骨骼；
桥接（morpheus_arkit_bridge.py）用 pose_matrix() 每帧驱动。
"""
import json
import math
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent
GEO = ROOT / "cad_geometry"

_cache = {}


def _load():
    if "model" not in _cache:
        _cache["model"] = json.loads((GEO / "linkage_model.json").read_text(
            encoding="utf-8"))["servos"]
        drv = json.loads((ROOT / "driver_bones_report.json").read_text(encoding="utf-8"))
        _cache["calib"] = drv["calibration"]
    return _cache["model"], _cache["calib"]


def calib():
    return _load()[1]


def world_to_blender(w):
    from mathutils import Vector
    c = calib()
    em = c["eye_mid"]
    f = (w[2] - 1.0, w[0] + 96.84, w[1] - 285.87)
    return Vector((em[0] + f[0] * c["sx"], em[1] + f[1] * c["sy"], em[2] + f[2] * c["sz"]))


def dir_to_blender(d):
    from mathutils import Vector
    c = calib()
    # CAD 方向 (dx,dy,dz) -> face (dz,dx,dy) -> Blender（与 world_to_blender 同置换）
    v = Vector((d[2] * c["sx"], d[0] * c["sy"], d[1] * c["sz"]))
    return v.normalized() if v.length > 1e-9 else Vector((0, 0, 1))


def unit_per_mm():
    return calib()["sz"]


def _instances():
    if "inst" not in _cache:
        _cache["inst"] = json.loads((GEO / "assembly_instances.json").read_text(
            encoding="utf-8"))["instances"]
    return _cache["inst"]


def _instance_center(pred):
    from mathutils import Vector
    hits = [it for it in _instances() if pred(it["file"].lower())]
    if not hits:
        return None
    cs = []
    for it in hits:
        bb = it["world_bbox"]
        cs.append(Vector(((bb["min_mm"][0] + bb["max_mm"][0]) / 2,
                          (bb["min_mm"][1] + bb["max_mm"][1]) / 2,
                          (bb["min_mm"][2] + bb["max_mm"][2]) / 2)))
    return sum(cs, Vector()) / len(cs)


def _drv_rows():
    if "drv_rows" not in _cache:
        drv = json.loads((ROOT / "driver_bones_report.json").read_text(encoding="utf-8"))
        _cache["drv_rows"] = {r["id"]: r for r in drv["rows"]}
    return _cache["drv_rows"]


def build_spec(sid):
    """返回该路正向运动学 spec（Blender 坐标）。"""
    from mathutils import Vector
    model, _ = _load()
    rec = model.get(str(sid)) or {}
    typ = rec.get("type", "open")
    spec = {"id": sid, "type": typ, "mode": "translate",
            "confidence": rec.get("confidence", "low")}
    anchor = rec.get("anchor")
    anchor_b = world_to_blender(anchor["world_mm"]) if anchor else None
    if anchor_b is None:
        # 无 CAD 表皮接触点：用蒙皮绑定时的表皮控制点作锚点代理（仍在表皮上）
        row = _drv_rows().get(sid)
        if row:
            anchor_b = Vector(row["head"])
            spec["confidence"] = "anchor_proxy"
    servo_b = world_to_blender(rec["servo_axis_mm"]) if rec.get("servo_axis_mm") else None
    normal_b = dir_to_blender(rec["plane"]["normal"]) if rec.get("plane") else Vector((0, 0, 1))
    crank_r = rec.get("crank_r_mm") or 0.0
    spec["gain"] = crank_r * unit_per_mm()          # blender unit / rad
    spec["anchor"] = anchor_b
    spec["servo"] = servo_b

    if typ == "linkage" and rec.get("grounds") and rec.get("crank_pin_mm"):
        js = [j for j in rec["joints"] if j.get("world_mm")]
        if len(js) >= 2:
            A = servo_b
            B0 = world_to_blender(rec["crank_pin_mm"])
            C0 = world_to_blender(js[-1]["world_mm"])
            # 输出件的固定铰（钢丝驱六连杆：OT head 的接地螺柱是导线座，不是输出铰）
            last_part = (rec.get("chain_files") or [None])[-1]
            og = [g for g in rec["grounds"] if g.get("part") == last_part]
            D = world_to_blender((og or rec["grounds"])[0]["world_mm"])
            u = dir_to_blender(rec["plane"]["u"])
            v = dir_to_blender(rec["plane"]["v"])
            o = A

            def p2(x):
                d = x - o
                return (d.dot(u), d.dot(v))

            pa, pb, pc, pd = p2(A), p2(B0), p2(C0), p2(D)
            # 杆长必须取平面内投影长度：3D 长度含离面分量，会让 C0 不在交点圆上，
            # 中立位就跳到错误装配支（实测 M27 dtheta=0 -> -110°）。
            spec.update(mode="fourbar", pivot=D, axis=normal_b,
                        fb={"A": pa, "B0": pb, "C0": pc, "D": pd,
                            "Rc": math.dist(pa, pb), "Lc": math.dist(pb, pc),
                            "Rr": math.dist(pc, pd)})
            spec["rest_head"] = D
            spec["rest_tail"] = C0
            return spec

    if typ in ("eye_rev", "neck_rev", "jaw_rev"):
        pivot, axis, ratio = None, normal_b, 1.0
        if typ == "eye_rev":
            ctr = _instance_center(lambda f: "eyeball_mounts" in f)
            pivot = world_to_blender(tuple(ctr)) if ctr is not None else anchor_b
            # 功能方向定轴符号：正视前方=-Y，抬头需绕 -X，右转需绕 -Z
            axis = dir_to_blender((0, 0, -1) if sid == 7 else (0, -1, 0))
        elif typ == "neck_rev":
            ctr = _instance_center(lambda f: "neck" in f and ("platform" in f or "baseplate" in f))
            pivot = world_to_blender(tuple(ctr)) if ctr is not None else servo_b
            axis = dir_to_blender((0, 1, 0) if sid == 31 else (0, 0, 1))
        else:  # jaw
            js = [j for j in rec["joints"] if j.get("world_mm")]
            pivot = world_to_blender(js[-1]["world_mm"]) if js else anchor_b
            if js and js[-1].get("axis"):
                axis = dir_to_blender(js[-1]["axis"])
            if pivot is not None and anchor_b is not None and crank_r:
                arm = (anchor_b - pivot).length
                ratio = crank_r * unit_per_mm() / arm if arm > 1e-6 else 1.0
        spec.update(mode="revolute", pivot=pivot, axis=axis, ratio=ratio,
                    rest_head=pivot, rest_tail=anchor_b)
        return spec

    # translate: cable / rack / pushrod / open
    d = rec.get("drive_dir_world")
    dir_b = dir_to_blender(d) if d else (servo_b - anchor_b).normalized() if (
        servo_b is not None and anchor_b is not None) else Vector((0, 1, 0))
    crank_pin_b = world_to_blender(rec["crank_pin_mm"]) if rec.get("crank_pin_mm") else None
    proj = None
    if typ in ("pushrod", "rack") and crank_pin_b is not None and servo_b is not None \
            and rec.get("plane") and anchor_b is not None and crank_r > 0.05:
        # 精确正向：曲柄销位移在杆轴/轨轴上的投影 = 表皮点位移（符号由几何决定，
        # 不再依赖 dir×gain×dtheta 的人为符号）。
        u = dir_to_blender(rec["plane"]["u"])
        v = dir_to_blender(rec["plane"]["v"])
        r2 = crank_pin_b - servo_b
        rc2 = math.hypot(r2.dot(u), r2.dot(v))
        th0 = math.atan2(r2.dot(v), r2.dot(u))
        b0_2 = servo_b + u * (rc2 * math.cos(th0)) + v * (rc2 * math.sin(th0))
        if typ == "pushrod":
            dir_b = (crank_pin_b - anchor_b).normalized()   # 杆轴：表皮 -> 曲柄销
        else:
            rail = dir_to_blender(d) if d else dir_b
            tang = u * (-r2.dot(v)) + v * (r2.dot(u))       # 曲柄销切向（dtheta>0）
            dir_b = rail * (1.0 if tang.dot(rail) >= 0 else -1.0)
        proj = {"o": servo_b, "u": u, "v": v, "Rc": rc2, "th0": th0, "B0": b0_2}
    elif typ == "pushrod" and crank_pin_b is not None and anchor_b is not None:
        dir_b = (crank_pin_b - anchor_b).normalized()
    if not crank_r:
        spec["gain"] = 8.0 * unit_per_mm()          # 缺曲柄半径时的标称行程
        spec["confidence"] = "low_gain"
    # 符号约定：该路「功能表达式」（最强 ARKit 系数）=1 时，表皮点沿 +dir 运动
    # （线驱/一阶=拉向导线座/舵机侧；推杆=拉向曲柄销）。符号经 frame_to_pwm 实算，
    # 自动兼容 jawOpen 这类负系数通道；PWM 表 direction 已含硬件安装方向。
    import servo_map as sm
    sgn_dir = 1.0 if sm.BY_ID[sid]["direction"] > 0 else -1.0
    sign = 1.0
    if proj is not None and typ == "pushrod":
        s = sm.BY_ID[sid]
        dtest = sgn_dir * 0.5 * max(1, s["pwm_high"] - s["pwm_low"]) * math.pi / 2000.0
        th = proj["th0"] + dtest
        B = proj["o"] + proj["u"] * (proj["Rc"] * math.cos(th)) \
            + proj["v"] * (proj["Rc"] * math.sin(th))
        if (B - proj["B0"]).dot(dir_b) < 0:
            sign = -1.0
    elif proj is None and typ in ("cable", "open"):
        sign = sgn_dir
        # 功能基准：该路最强 ARKit 表达式=1 经 frame_to_pwm 得到的实际转角 dfunc。
        # 让该表达式产生 +dir（拉向导线座/舵机侧）位移 => sign=sign(dfunc)。
        # 例 M23：jawOpen 系数 -1（Start=Upper，只能向 Lower 动），dfunc>0，
        # 故 sign=+1，张嘴=拉向后侧；若只按 direction 取 sign 会反向。
        # 并列最强（如眉 ±0.8、颌左右 ±0.7）或被 clamp 成 0 时语义不明，保持 sgn_dir。
        best, second, bname = 0.0, 0.0, None
        for n, m in sm.ARKIT_TO_SERVO.items():
            c = abs(m.get(sid, 0.0))
            if c > best:
                second, best, bname = best, c, n
            elif c > second:
                second = c
        if bname is not None and best > second + 1e-9:
            vec = [0.0] * len(sm.MODEL_BS_LIST)
            vec[sm.MODEL_BS_LIST.index(bname)] = 1.0
            pf = sm.frame_to_pwm(vec)[sid - 1]
            dfunc = (pf - sm.BY_ID[sid]["pwm_start"]) * math.pi / 2000.0
            if abs(dfunc) > 1e-9:
                sign = 1.0 if dfunc > 0 else -1.0
    spec.update(mode="translate", dir=dir_b, proj=proj, sign=sign, rest_head=anchor_b,
                rest_tail=(anchor_b + dir_b * 0.12) if anchor_b is not None else None)
    return spec


def _circle_intersect(c0, r0, c1, r1, ref):
    """两圆交点中离 ref 较近者（保持装配分支）。容差吸收 JSON  rounding。"""
    dx, dy = c1[0] - c0[0], c1[1] - c0[1]
    d = math.hypot(dx, dy)
    if d < 1e-9 or d > r0 + r1 + 1e-6 or d < abs(r0 - r1) - 1e-6:
        return None
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h2 = max(0.0, r0 * r0 - a * a)
    h = math.sqrt(h2)
    mx, my = c0[0] + a * dx / d, c0[1] + a * dy / d
    p1 = (mx + h * dy / d, my - h * dx / d)
    p2 = (mx - h * dy / d, my + h * dx / d)
    d1 = (p1[0] - ref[0]) ** 2 + (p1[1] - ref[1]) ** 2
    d2 = (p2[0] - ref[0]) ** 2 + (p2[1] - ref[1]) ** 2
    return p1 if d1 <= d2 else p2


def fourbar_dphi(spec, dtheta):
    """曲柄转 dtheta -> 摇杆相对中立转角（闭式正向，增量 marched 保持装配支）。"""
    if abs(dtheta) < 1e-12:
        return 0.0
    fb = spec["fb"]
    th0 = math.atan2(fb["B0"][1] - fb["A"][1], fb["B0"][0] - fb["A"][0])
    phi0 = math.atan2(fb["C0"][1] - fb["D"][1], fb["C0"][0] - fb["D"][0])
    ref = fb["C0"]
    phi = phi0
    steps = max(1, int(abs(dtheta) / 0.02))
    for k in range(1, steps + 1):
        th = th0 + dtheta * k / steps
        B = (fb["A"][0] + fb["Rc"] * math.cos(th), fb["A"][1] + fb["Rc"] * math.sin(th))
        C = _circle_intersect(B, fb["Lc"], fb["D"], fb["Rr"], ref)
        if C is None:
            break
        ref = C
        phi = math.atan2(C[1] - fb["D"][1], C[0] - fb["D"][0])
    d = phi - phi0
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def output_angle(spec, dtheta):
    if spec["mode"] == "fourbar":
        return fourbar_dphi(spec, dtheta)
    if spec["mode"] == "revolute":
        return spec.get("ratio", 1.0) * dtheta
    return 0.0


def pose_matrix(spec, dtheta, rest_matrix):
    """给定舵机转角增量 dtheta(rad) 与骨骼 rest 矩阵，返回目标 armature 矩阵。"""
    from mathutils import Vector, Matrix
    M0 = rest_matrix
    if spec["mode"] == "translate":
        dtheta = spec.get("sign", 1.0) * dtheta
        pr = spec.get("proj")
        if pr:
            th = pr["th0"] + dtheta
            B = pr["o"] + pr["u"] * (pr["Rc"] * math.cos(th)) \
                + pr["v"] * (pr["Rc"] * math.sin(th))
            db = B - pr["B0"]
            disp = spec["dir"] * db.dot(spec["dir"])
        else:
            disp = spec["dir"] * (spec["gain"] * dtheta)
        return Matrix.Translation(disp) @ M0
    head = spec.get("pivot") if spec.get("pivot") is not None else M0.to_translation()
    ang = output_angle(spec, dtheta)
    R = Matrix.Rotation(ang, 4, spec["axis"])
    return Matrix.Translation(head) @ R @ Matrix.Translation(-head) @ M0


def servo_dtheta(sid, pwm_value):
    """PWM -> 相对中立舵机转角(rad)。假设 0..2000 对应 180°（标注为假设）。"""
    import servo_map as sm
    s = sm.BY_ID[sid]
    return (pwm_value - s["pwm_start"]) * (math.pi / 2000.0)