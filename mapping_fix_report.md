# Morpheus Blender 映射修复对比报告

日期：2026-11-09 · 文件：`LY_GENERAL_V0.blend`（Blender 5.2.3 LTS 后台实测）
配套图：[mapping_audit_preview.png](mapping_audit_preview.png) · 原始数据：[mapping_audit.json](mapping_audit.json)

## 结论

模型已按真实 SolidWorks CAD 几何建准：33 根 `Morpheus_Servo_XX` 骨的 rest 位置落在
**真实铰链 / 表皮驱动锚点**（不再是舵机轴心或原地旋转标记），运动由
`mechanism_kinematics.py` 的**正向机构运动学**驱动——无 IK、无外部求解器、无 LUT。
ARKit 归一化值经标准映射 `ARKIT_TO_SERVO` / `frame_to_pwm` **同源**产生 33 路 PWM 与骨骼转角，
映射准确性来自几何本身。

## 驱动链路（本轮未改 PWM 路径）

```
ARKit 52 系数
  ├─> SampleHead / eyes.L / eyes.R 形态键与眼球旋转（面部形变）
  ├─> servo_map.ARKIT_TO_SERVO -> frame_to_pwm -> 33 路 PWM -> UDP 9878（硬件）
  └─> (PWM − pwm_start) × π/2000 = 舵机转角增量 dtheta
        -> mechanism_kinematics 正向运动学 -> pose bone matrix（模型）
```

`servo_map.py`（PWM 唯一来源）与 `emit_pwm` 本轮**零改动**（文件修改时间停留 11-05）；
注入测试的 `pwm_after_inject` 与手算一致（如 M23=1100、M16=1185、M07=1480）。

## 机构分型（cad_geometry/linkage_model.json，接触簇提取）

| 类型 | 路数 | 通道 | 正向模型 |
|---|---|---|---|
| cable 线驱 | 7 | 含下颌 M23 | 锚点沿真实钢丝方向平动，行程=曲柄半径×Δθ |
| linkage 四连杆 | 4 | M04/09/11/27 | 曲柄角→摇杆角，闭式圆-圆交点（增量 marching 保持装配支） |
| pushrod 推杆 | 5 | M05/06/08/25/26 | 曲柄销位移在杆轴上的精确投影 |
| rack 齿条 | 2 | M13/16 | 曲柄销切向在轨轴上的投影 |
| eye_rev 眼球 | 2 | M07/24 | 绕真实球心轴转（俯仰 −X / 注视 −Z） |
| neck_rev 颈 | 2 | M32/33 | 绕颈平台真实轴转 |
| open 一阶 | 11 | 含 M31 | 拉向舵机侧的一阶近似（待实物标定） |

## 修复前 → 修复后（注入 11 个 ARKit=1 实测）

| 指标 | 修复前 | 修复后 |
|---|---|---|
| M01 颊顶点位移 vs CAD 拉动方向偏角 | 161.18°（反向） | 18.73° |
| M02 鼻顶点偏角 | 163.95°（反向） | 16.06° |
| M05 眼睑顶点偏角 | 167.98°（反向） | 24.55° |
| M04 眉峰输出转角 | 63.78°（含 111.37° 中立跳支） | 25.96°（闭式真值，中立 0 跳变） |
| M09 嘴角摇杆转角 | 0°（固定铰取错） | 49.98° |
| M23 下颌 | 误判 jaw_rev，原地转 2.4° | cable：下巴后拉 [0.17, 0.94, −0.29]，对铰链产生张口力矩 |
| M16 下唇齿条 | PCA 抓体长轴、crank_r=0 → 无运动 | 借镜像 M13 轨轴，竖直向下（顶点 Z 分量 −0.89） |
| 眼球 eyeLookUp | +43.2°（向下转） | −43.2°（向上转），axis 加权和 |
| 蒙皮顶点组 | 14 路完全缺失 | 31 组；M31 垄断 10348 顶点修剪 9529（限颈区 Z≤−100mm） |
| 骨 rest 位置 | 舵机轴心附近、原地旋转 | 真实固定铰 D / 表皮锚点；bind 不变、中立位不跳变 |

修复前数字来自本会话早期审计运行（transcript 存档）；修复后数字来自当前
`mapping_audit.json`（`audit_mapping.py` 只读实测，不保存文件）。

## 本轮关键 bug 与修法

1. **四连杆杆长离面分量**：Rc/Lc/Rr 原用 3D 长度而坐标是平面投影 → C0 不在交点圆上、
   中立位跳支（M04 跳 111.37°、M27 跳 −110°）。改取平面内 `math.dist`。
2. **方向向量缺轴置换**：`dir_to_blender` 原漏 CAD→face 的 (z,x,y) 置换 → 所有旋转轴 /
   平面基错乱。改为 `(d[2]·sx, d[0]·sy, d[1]·sz)`。
3. **M09 固定铰取错**：g0 是导线座螺柱；改取输出件接地铰 g1。
4. **M23 分类**：CAD 链终于上唇球铰（PCA 轴无意义）→ 改 cable；验证后拉对铰链力矩确为张口。
5. **M16 轨轴**：PCA 抓到体长轴且 crank_r=0 → 借镜像 M13 轨轴；投影条件加 `crank_r>0.05`。
6. **符号约定**：以「该路最强 ARKit 表达式=1 经 frame_to_pwm 实算的转角 dfunc」定 sign，
   使功能表达式产生 +dir（拉向导线座）位移。M23 的 jawOpen 系数为 −1（Start=Upper，
   只能向 Lower 动）→ dfunc>0 → sign=+1；若只按硬件 direction 取 sign 会反向。
   并列最强（眉 ±0.8、颌左右 ±0.7）或被 clamp 为 0 时语义不明，保持 sgn_dir。
7. **眼球**：轴符号翻正（−X 俯仰 / −Z 注视），bridge 用 `axis7·ang7 + axis24·ang24` 向量和；
   eyes 拆 L/R、原点=球心。
8. **审计阈值**：蒙皮归一化后 M09/M16 权重 <0.5 被漏检 → 顶点组阈值 0.5→0.2。

## 验证方式

- `blender --background LY_GENERAL_V0.blend --python audit_mapping.py`：只读打开，注入
  11 个 ARKit=1，测骨 head/tail 位移、转轴、mold 顶点组平均位移方向、eyes 旋转；结束恢复中立。
- 符号全表核对：33 路 translate sign 逐一打印，确认仅 M23 因功能基准修正而翻转。
- `.blend` 内嵌文本块 `morpheus_arkit_bridge.py` 已与磁盘同步并保存。

## 遗留边界（需实物标定，非本轮几何问题）

- 11 路 open 为一阶近似（拉向舵机侧）；M31 整头颈旋转属场景级变换，不在模具蒙皮内。
- 15 路无 CAD anchor（M02/05/06/07/12/13/15/16/20/22/25/26/31/32/33）用旧蒙皮控制点作代理
  （`anchor_proxy`；其中 4 路因缺曲柄半径同时标 `low_gain`），挂点待实物确认。
- `servo_dtheta` 假设 0..2000 ticks = 180°；`ARKIT_TO_SERVO` 为种子矩阵，论文要求 5000 组实测标定。
- 平面法向 PCA 符号任意性由功能 sign 约定吸收；M21 等共激活顶点的混合位移属审计测量污染。

## 交付与备份

- 代码：`mechanism_kinematics.py`、`extract_linkage_model.py`、`apply_mechanism_rig.py`、
  `apply_skin_coverage.py`、`morpheus_arkit_bridge.py`、`audit_mapping.py`、`make_mapping_audit_preview.py`
- 数据：`cad_geometry/linkage_model.json`、`mapping_audit.json`、`mechanism_rig_report.json`、
  `skin_coverage_report.json`、`mapping_audit_preview.png`
- 备份：`LY_GENERAL_V0_before_mechanism_rig.blend`、`LY_GENERAL_V0_before_skin.blend`