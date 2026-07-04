# pose-correct

`pose-correct` 用三颗固定在底盘上的激光测距仪，对 odin 输出的 `S.x / S.y / S.yaw` 做离线标定和在线纠错。

场景假设：

- 小车在第一象限角落，两面墙分别是 `x=0` 和 `y=0`。
- `M`、`N` 在同一条边上，射线打到 `y` 轴墙面，且 `MN` 间距已知。
- `P` 在相邻边上，射线打到 `x` 轴墙面。
- 三颗测距仪、odin 点 `S` 与底盘刚性固定，但安装偏置未知。

## 数学模型与算法选择

设 `M/N` 间距为 `d`，两颗测距读数为 `m/n`。同一边上的两条平行射线会因为车体旋转产生距离差：

```text
tan(yaw_laser) = (m - n) / d
yaw_laser = atan2(m - n, d)
```

如果实际接线、M/N 顺序或坐标正方向相反，可在拟合时使用 `--yaw-sign -1`。

将激光读数投影回墙面法向坐标：

```text
h_x = 0.5 * (m + n) * cos(yaw_laser)
h_y = p * cos(yaw_laser)
```

上面的 `h_x/h_y` 是旧的低维近似模型。现在默认使用更严格的 `explicit_geometry`，显式标定三颗激光相对 `S` 的安装位置，适合 `M` 接近 `A`、`N` 接近 `B`、`P` 接近 `C`，且传感器间距达到几十厘米的情况。

设 `S` 是输出位姿点，车体坐标系下：

```text
M = (ab_sensor_x, ab_sensor_mid_y - d/2)
N = (ab_sensor_x, ab_sensor_mid_y + d/2)
P = (p_sensor_x, p_sensor_y)
```

其中 `M/N` 在同一条 AB 边上，`d` 是已知的 `MN` 间距。算法要求解的纠错函数参数是：

```text
yaw_offset
ab_sensor_x
ab_sensor_mid_y
p_sensor_x
p_sensor_y
```

对单帧数据，先由 `M/N` 求：

```text
corrected_yaw = yaw_laser + yaw_offset
```

然后根据每颗激光的独立安装位置反解 `S`：

```text
c = cos(corrected_yaw)
s = sin(corrected_yaw)

corrected_x = 0.5*(m+n)*c - ab_sensor_x*c + ab_sensor_mid_y*s
corrected_y = p*c - p_sensor_x*s - p_sensor_y*c
```

这几行来自刚体变换。以 `M/N` 为例：

```text
sensor_world = S_world + R(yaw) * sensor_body
measured_distance = sensor_world.x / cos(yaw)
```

整理后就能从测距值反解 `S_world.x`。`P` 同理反解 `S_world.y`。

默认标定算法是 `explicit_geometry`：

```text
minimize soft_l1([
  (S_corrected.x - odin.s_x) / position_scale,
  (S_corrected.y - odin.s_y) / position_scale,
  wrap(S_corrected.yaw - odin.s_yaw) / yaw_scale
])
```

这个选择的原因：

- `M/N` 的距离差对 yaw 约束很强，直接给出高精度 `yaw_laser`。
- `M/N/P` 的距离读数比 odin 更准，能稳定约束靠墙方向的位置。
- 三颗激光的位置差异被显式建模，不再假定它们共享一个虚拟参考点。
- 至少 20 组不同位置、不同 yaw 的数据可以让这些参数可观测。
- `soft_l1` 鲁棒损失可以降低少量异常帧对最终纠错函数的影响。

在线阶段不再做优化，只需输入单组 `{m,n,p,s_x,s_y,s_yaw}`，直接返回纠正后的 `{x,y,yaw}`。

## 安装

```bash
cd ~/pose-correct
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

## CSV 输入格式

CSV 必须包含这些列，单位保持一致即可，角度使用弧度：

```csv
m,n,p,s_x,s_y,s_yaw
1.3100,1.3505,0.8700,1.3000,0.8200,-0.0800
```

## 命令行接口

拟合模型：

```bash
pose-correct fit data.csv --mn-distance 0.40 --model model.json
```

如果 yaw 方向反了：

```bash
pose-correct fit data.csv --mn-distance 0.40 --yaw-sign -1 --model model.json
```

如果你明确想退回普通线性最小二乘：

```bash
pose-correct fit data.csv --mn-distance 0.40 --method linear --model model.json
```

旧的虚拟参考点鲁棒模型也保留了：

```bash
pose-correct fit data.csv --mn-distance 0.40 --method robust_geometric --model model.json
```

`--position-scale` 和 `--yaw-scale` 用来表达 odin 数据的大致可信度。默认值分别是 `0.02` 米和 `0.02` 弧度。odin 误差更大时可以调大，例如：

```bash
pose-correct fit data.csv --mn-distance 0.40 --position-scale 0.05 --yaw-scale 0.05
```

应用模型：

```bash
pose-correct correct new_data.csv --model model.json --output corrected.csv
```

## Python 接口

```python
from pose_correct import Observation, PoseCorrector

samples = [
    Observation(m=1.31, n=1.35, p=0.87, s_x=1.30, s_y=0.82, s_yaw=-0.08),
    Observation(m=1.28, n=1.26, p=0.90, s_x=1.28, s_y=0.85, s_yaw=0.05),
]

corrector = PoseCorrector.fit(samples, mn_distance=0.40)
pose = corrector.correct(samples[0])

print(pose.x, pose.y, pose.yaw)
```

## 项目结构

```text
pose-correct/
├── README.md
├── pyproject.toml
├── examples/
│   └── sample_data.csv
├── src/
│   └── pose_correct/
│       ├── __init__.py
│       ├── cli.py
│       ├── corrector.py
│       ├── io.py
│       ├── math_utils.py
│       └── models.py
└── tests/
    └── test_corrector.py
```

## 注意

这个模型不需要提前知道 `M/N/P/S` 在底盘上的具体安装位置，但需要采集多组不同 yaw、不同位置的数据来把固定偏置拟合出来。建议至少 20 组，并尽量覆盖：

- 不同的 `x/y` 距离。
- 正负两个方向的小 yaw。
- 不同角落距离，不要所有样本都挤在同一个位置。
- 激光和 odin 时间戳尽量同步。

如果所有样本几乎同位置、同角度，最小二乘会退化，模型残差可能看起来很小，但泛化到新位置会不稳定。
