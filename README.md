# pose-correct

`pose-correct` 用两颗固定在正方形小车底盘上的激光测距仪，对 odin 输出的 `S.x / S.y` 做标定和纠错。这里假定 odin 的 `S.yaw` 是准确的，所以模型不会修正 yaw，只会原样返回输入 yaw。

场景假设：

- 小车在第一象限角落，两面墙分别是 `x=0` 和 `y=0`。
- `M` 在 `AB` 边上，射线垂直于 `AB`，测到 `y` 轴墙面的距离。
- `N` 在 `BC` 边上，射线垂直于 `BC`，测到 `x` 轴墙面的距离。
- `M`、`N` 和 odin 点 `S` 都固定在底盘上，但相对位置未知。

## 数学模型

把 `S` 作为车体坐标系原点。两颗激光在车体坐标系中的固定位置分别为：

```text
M = (m_sensor_x, m_sensor_y)
N = (n_sensor_x, n_sensor_y)
```

对单帧数据，已知 `m / n / s_yaw`。设：

```text
c = cos(s_yaw)
s = sin(s_yaw)
```

根据刚体变换，可以得到纠错函数：

```text
x = m*c - m_sensor_x*c + m_sensor_y*s
y = n*c - n_sensor_x*s - n_sensor_y*c
yaw = s_yaw
```

标定阶段用多组 `{m,n,s_x,s_y,s_yaw}` 数据，通过线性最小二乘拟合四个未知安装参数：

```text
m_sensor_x
m_sensor_y
n_sensor_x
n_sensor_y
```

在线阶段不再优化，只需输入单组数据，直接返回 `{x,y,yaw}`。

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
m,n,s_x,s_y,s_yaw
0.8264,0.4666,1.0000,0.7000,-0.2400
```

## 数据采集

`scripts/collect_sensor_odin_data.py` 可以交互式采集多路测距传感器和
Odin 定位数据。脚本直接读取 CH9344 USB 串口数据，并订阅
`/odin1/relocation` 的 `geometry_msgs/PoseStamped`。

运行前需要 ROS 2 环境和 `pyserial`：

```bash
source /opt/ros/jazzy/setup.bash
pip install -e .
```

采集：

```bash
python3 scripts/collect_sensor_odin_data.py -o sensor_odin_samples.csv
```

脚本启动后，输入 `y` 追加一行 CSV，输入 `q` 退出。

默认输入：

| 来源 | 默认值 |
|---|---|
| 测距传感器 | `/dev/ttyCH9344USB0` 到 `/dev/ttyCH9344USB6` |
| 波特率 | `230400` |
| Odin 位姿话题 | `/odin1/relocation` |
| 输出 CSV | `sensor_odin_samples.csv` |

每行 CSV 字段：

```text
sample_index, ros_time_s, unix_time_s,
sensor_0_mm ... sensor_6_mm,
odin_frame_id, odin_stamp_s,
odin_x_m, odin_y_m, odin_z_m,
odin_qx, odin_qy, odin_qz, odin_qw,
odin_yaw_rad, odin_pose_age_s
```

常用覆盖参数：

```bash
# 改 Odin 话题
python3 scripts/collect_sensor_odin_data.py \
  -o calibration.csv \
  --pose-topic /odin1/relocation

# 指定串口列表
python3 scripts/collect_sensor_odin_data.py \
  --port-names /dev/ttyCH9344USB0,/dev/ttyCH9344USB1,/dev/ttyCH9344USB2
```

## 命令行接口

拟合模型：

```bash
pose-correct fit examples/sample_data.csv --model model.json
```

应用模型：

```bash
pose-correct correct examples/sample_data.csv --model model.json --output corrected.csv
```

## Python 接口

```python
from pose_correct import Observation, PoseCorrector

samples = [
    Observation(m=0.8264, n=0.4666, s_x=1.0, s_y=0.7, s_yaw=-0.24),
    Observation(m=0.8471, n=0.4942, s_x=1.04, s_y=0.725, s_yaw=-0.18),
]

corrector = PoseCorrector.fit(samples)
pose = corrector.correct(samples[0])

print(pose.x, pose.y, pose.yaw)
```

## 注意

这个模型不需要提前知道 `M/N/S` 在底盘上的具体安装位置，但需要采集多组不同位置、不同 yaw 的数据来拟合固定偏置。建议至少 10 组，并尽量覆盖：

- 不同的 `x/y` 距离。
- 正负两个方向的小 yaw。
- 不要所有样本都挤在同一个位置或同一个角度。
- 激光和 odin 时间戳尽量同步。

如果所有样本几乎同位置、同角度，最小二乘会退化，模型残差可能看起来很小，但泛化到新位置会不稳定。
