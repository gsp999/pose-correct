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

如果 Odin 原点不在场地边缘，先用对应队伍配置里的
`field_origin_in_odin` 做平移：

```text
field_x = odin_x - teams.<red_or_blue>.field_origin_in_odin.x_m
field_y = odin_y - teams.<red_or_blue>.field_origin_in_odin.y_m
field_yaw = odin_yaw
```

也就是说，新世界坐标系的原点只需要提供“它在 Odin 坐标系下的
位置”。两个坐标系的 x/y 轴方向保持一致，不需要旋转参数。

## 固定几何配置

`config/pick_geometry.yaml` 保存运行时固定信息：

```yaml
gripper:
  forward_m: 0.0
  left_m: 0.0
  yaw_rad: 0.0

teams:
  red:
    field_origin_in_odin:
      x_m: 0.0
      y_m: 0.0
    targets:
      default:
        x_m: 0.0
        y_m: 0.0
        yaw_rad: 0.0
  blue:
    field_origin_in_odin:
      x_m: 0.0
      y_m: 0.0
    targets:
      default:
        x_m: 0.0
        y_m: 0.0
        yaw_rad: 0.0
```

约定：

- `teams.red.field_origin_in_odin`：红方场地边缘世界坐标系原点，在 Odin 坐标系下的位置。
- `teams.blue.field_origin_in_odin`：蓝方场地边缘世界坐标系原点，在 Odin 坐标系下的位置。
- `teams.*.targets.*`：红/蓝各自需要夹取的目标，坐标在各自场地边缘世界坐标系下。
- `gripper.forward_m`：夹爪夹取点相对 Odin 点 `S` 的前向偏移。
- `gripper.left_m`：夹爪夹取点相对 Odin 点 `S` 的左向偏移。
- `gripper.yaw_rad`：夹爪坐标系相对车体朝向的角度偏移。

## 目标到夹爪坐标的变换逻辑

`pose-correct` 最终要给 `pick_action` 的不是 Odin 全局坐标，而是目标
相对夹爪的局部坐标：

```text
pick.x_m = 目标相对夹爪的左右偏差，左为正
pick.y_m = 目标相对夹爪的前后距离，前为正
```

完整链路分三步。

第一步，把修正后的 Odin 位姿平移到红/蓝自己的场地边缘世界坐标系。
红蓝是两个完全独立的场地，所以各自使用自己的 `field_origin_in_odin`：

```text
robot_field_x = corrected_odin_x - team.field_origin_in_odin.x_m
robot_field_y = corrected_odin_y - team.field_origin_in_odin.y_m
robot_field_yaw = corrected_odin_yaw
```

这里没有旋转，因为场地坐标系和 Odin 坐标系的 x/y 轴方向相同。

第二步，在场地边缘世界坐标系下，计算目标相对机器人 Odin 点 `S`
的向量：

```text
dx = target_field_x - robot_field_x
dy = target_field_y - robot_field_y
```

然后把这个世界向量旋转到机器人自身坐标系。机器人坐标系约定：

```text
+forward = 机器人当前 yaw 朝向
+left    = 机器人左侧
```

公式：

```text
target_forward =  dx*cos(yaw) + dy*sin(yaw)
target_left    = -dx*sin(yaw) + dy*cos(yaw)
```

第三步，减去夹爪相对 Odin 点 `S` 的安装偏移：

```text
delta_forward = target_forward - gripper.forward_m
delta_left    = target_left    - gripper.left_m
```

如果夹爪坐标系和车体坐标系还有一个固定角度差 `gripper.yaw_rad`，
再把这个偏差旋到夹爪坐标系：

```text
pick_y =  delta_forward*cos(gripper_yaw) + delta_left*sin(gripper_yaw)
pick_x = -delta_forward*sin(gripper_yaw) + delta_left*cos(gripper_yaw)
```

最后输出给 `pick_action`：

```text
x_m = pick_x
y_m = pick_y
```

得到修正后的机器人场地坐标后，可以这样调用：

```python
from pose_correct import (
    PoseEstimate,
    load_pick_geometry_config,
    target_to_pick_coordinates,
)

cfg = load_pick_geometry_config("config/pick_geometry.yaml")
team = cfg.teams["red"]
robot = PoseEstimate(x=1.0, y=0.7, yaw=0.0)  # corrected field/world pose
target = team.targets["default"]
pick = target_to_pick_coordinates(
    robot_pose_field=robot,
    target_pose_field=target,
    gripper=cfg.gripper,
)

print(pick.x_m, pick.y_m, pick.yaw_rad)
```

其中 `pick.x_m` 是目标相对夹爪的左右偏差，`pick.y_m` 是目标相对
夹爪的前后距离。这个结果可以发布成和 2D 雷达识别相同语义的
`/spear_recognition/result`，让 `pick_action_server` 继续使用原来的
`ALIGN_X -> FORWARD -> GRASP` 流程。

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
| 测距传感器 | `/dev/ttyCH9344USB0` 到 `/dev/ttyCH9344USB7` |
| 波特率 | `230400` |
| Odin 位姿话题 | `/odin1/relocation` |
| 输出 CSV | `sensor_odin_samples.csv` |

每行 CSV 字段：

```text
sample_index, ros_time_s, unix_time_s,
sensor_0_mm ... sensor_7_mm,
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
